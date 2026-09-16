#!/usr/bin/env python3
"""
app.py

Flask + Flask-SocketIO web app for the vc2 pneumatic station. It drives BOTH
serial devices of the station over two separate USB ports:

  - Arduino Giga R1 (valve regulator) — valve control, via ValveController
    (vc2/vc2.ino protocol). Enumerates as e.g. /dev/ttyACM0 "Arduino Giga".
    Setpoints are entered in kPa; ValveController converts to the mV the
    sketch expects.
  - EKU081 8-port USB↔RS-485 adapter (GJW encoders) — position/turns/speed
    readout, via EncoderController (Modbus RTU). Its WCH CH9344 driver creates
    one node per port: /dev/ttyCH9344USB0 … USB7. The encoder bus defaults to
    the lowest-numbered node; use --rs485-port to pick another.

Both ports are opened at once, so the browser can command valves and watch the
encoders live side by side. Either device may be absent — the app runs with
whichever it can open.

Usage:
    python app.py                                   # auto-detect both ports
    python app.py --valve-port /dev/ttyACM0 --rs485-port /dev/ttyCH9344USB0
    python app.py --rs485-port /dev/ttyCH9344USB3 --no-valves
    python app.py --ids 50-80 --baud 115200 --parity N
"""

import argparse
import time
from types import SimpleNamespace

import serial.tools.list_ports
from flask import Flask, render_template, jsonify
from flask_socketio import SocketIO

from valve_controller import ValveController, MAX_INPUT_KPA, NUM_VALVES
from encoder_controller import (
    DEFAULT_COUNTS_PER_TURN,
    DEFAULT_DRUM_DIAMETER_MM,
    EncoderController,
    is_ch9344_port,
    list_ch9344_ports,
    parse_slave_ids,
    port_index,
)
from sensor_mapping import DEFAULT_MAPPING_PATH, load_mapping


app = Flask(__name__)
app.config['SECRET_KEY'] = 'vc2-pneumatic-station'
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")

# Length-bar scaling (mm). Configurable here in code, not via CLI. Each valve's
# bar plots its mapped sensor's length within this band.
LENGTH_BAR_MIN_MM = -10.0
LENGTH_BAR_MAX_MM = 2.0

# Two shared controllers, created in main(). Either may stay None if its device
# is absent or disabled.
valve_controller = None
encoder_controller = None
_broadcast_started = False

# regulator (valve id) -> {"muscle", "sensor"}; loaded in main(), reloaded on
# RESCAN so edits to sensor_mapping.json apply without restarting the server.
sensor_map = {}
mapping_path = DEFAULT_MAPPING_PATH


# ============================================================
# Port resolution — tell the Giga R1 from the RS-485 adapter
# ============================================================

def _usb_ports():
    """Serial ports that look like real USB/CDC devices (skip ttyS*).

    Includes the EKU081's /dev/ttyCH9344USB* nodes, merged in from a /dev glob
    because the out-of-tree CH9344 driver is not always visible to pyserial.
    """
    out = []
    for p in serial.tools.list_ports.comports():
        dev = p.device or ""
        if ("ttyACM" in dev or "ttyUSB" in dev or is_ch9344_port(dev)
                or dev.upper().startswith("COM")):
            out.append(p)
    seen = {p.device for p in out}
    out += [SimpleNamespace(device=d, description="CH9344 USB-RS485 (EKU081)",
                            hwid="USB VID:PID=1a86:CH9344")
            for d in list_ch9344_ports() if d not in seen]
    return out


def _blob(p):
    return " ".join(str(x) for x in (
        p.description, getattr(p, "product", None),
        getattr(p, "manufacturer", None), p.hwid)).lower()


def _is_giga(p):
    """Arduino Giga R1 (valve regulator): VID 2341 / 'giga' / 'arduino'."""
    b = _blob(p)
    return "giga" in b or "arduino" in b or "2341:" in b


def _is_rs485_bridge(p):
    """USB↔RS-485 adapters: the EKU081 first, then common bridge chips."""
    if is_ch9344_port(p.device):
        return True
    b = _blob(p)
    return any(k in b for k in (
        "1a86", "ch340", "ch343", "ch9344", "single serial",  # WCH
        "0403", "ftdi",                               # FTDI
        "10c4", "cp210",                              # Silicon Labs
        "rs485", "rs-485", "usb serial", "usb-serial",
    ))


def resolve_ports(args):
    """Pick distinct ports for the valve regulator and the RS-485 adapter."""
    ports = _usb_ports()

    valve_port = args.valve_port
    if not valve_port and not args.no_valves:
        # A CH9344 node is never the regulator, whatever its description says.
        valve_port = next((p.device for p in ports
                           if _is_giga(p) and not is_ch9344_port(p.device)), None)

    rs485_port = args.rs485_port
    if not rs485_port and not args.no_encoders:
        others = [p for p in ports if p.device != valve_port]
        # The EKU081 (CH9344) is the station's adapter — its lowest-numbered
        # port carries the encoder bus; pass --rs485-port to use another.
        ch9344 = sorted((p.device for p in others if is_ch9344_port(p.device)),
                        key=port_index)
        rs485_port = ch9344[0] if ch9344 else None
        if not rs485_port:
            rs485_port = next(
                (p.device for p in others if _is_rs485_bridge(p)), None)
        if not rs485_port:  # fall back to any other USB port
            rs485_port = next((p.device for p in others), None)

    if valve_port and rs485_port and valve_port == rs485_port:
        raise SystemExit(
            f"[ERR] valve and RS-485 ports are the same ({valve_port}); "
            "pass distinct --valve-port / --rs485-port")
    return valve_port, rs485_port


# ============================================================
# Combined state (valves + encoders + sensor mapping)
# ============================================================

def _muscles_state(enc_state):
    """One entry per regulator, joined with its mapped sensor's length.

    Kept separate from the valve `active` map (which only lists valves with a
    live setpoint) so all length bars render whether or not the valve is on.
    """
    positions = {
        e["slave"]: (e.get("position_mm"), bool(e.get("online")))
        for e in (enc_state or {}).get("encoders", [])
    }
    muscles = []
    for reg in range(NUM_VALVES):
        m = sensor_map.get(reg, {})
        sensor = m.get("sensor")
        pos, online = positions.get(sensor, (None, False))
        muscles.append({
            "regulator": reg,
            "muscle": m.get("muscle", f"muscle_{reg:02d}"),
            "sensor": sensor,
            "position_mm": pos,
            "sensor_online": online,
        })
    return muscles


def build_state():
    """Full snapshot pushed to the browser: valves, encoders, joined muscles."""
    enc_state = encoder_controller.get_state() if encoder_controller else None
    return {
        "valves": valve_controller.get_state() if valve_controller else None,
        "encoders": enc_state,
        "muscles": _muscles_state(enc_state),
        "length_bar_min_mm": LENGTH_BAR_MIN_MM,
        "length_bar_max_mm": LENGTH_BAR_MAX_MM,
    }


# ============================================================
# Background broadcaster: push combined state + messages
# ============================================================

def _broadcast_loop():
    """Emit valve + encoder state and drained messages ~5x/sec."""
    while True:
        socketio.emit('state', build_state())

        lines = []
        if valve_controller:
            lines += [f"[VC2] {m}" for m in valve_controller.drain_messages()]
        if encoder_controller:
            lines += [f"[ENC] {m}" for m in encoder_controller.drain_messages()]
        if lines:
            socketio.emit('messages', {'lines': lines})
        socketio.sleep(0.2)


def _ensure_broadcaster():
    global _broadcast_started
    if not _broadcast_started:
        socketio.start_background_task(_broadcast_loop)
        _broadcast_started = True


# ============================================================
# HTTP routes
# ============================================================

@app.route('/')
def index():
    return render_template('index.html')


@app.route('/api/state')
def api_state():
    return jsonify(build_state())


@app.route('/api/ports')
def api_ports():
    return jsonify({"ports": ValveController.list_ports()})


# ============================================================
# SocketIO events
# ============================================================

@socketio.on('connect')
def on_connect():
    _ensure_broadcaster()
    socketio.emit('state', build_state())


def _parse_ramp(data):
    """Ramp duration in seconds from a payload; 0 (instant) if blank/invalid."""
    try:
        ramp = float(data.get('ramp', 0) or 0)
    except (ValueError, TypeError):
        return 0.0
    return ramp if ramp > 0 else 0.0


# ---- Valve events (Giga R1) --------------------------------

@socketio.on('set_valve')
def on_set_valve(data):
    """data = {valve: int, value: kPa (number) | 'off', ramp: float}"""
    if valve_controller is None:
        return
    valve = int(data.get('valve'))
    value = data.get('value')
    ramp = _parse_ramp(data)
    if isinstance(value, str) and value.strip().lower() in ('off', 'o', 'x'):
        valve_controller.valve_off(valve, ramp=ramp)
        return
    try:
        valve_controller.set_valve(valve, float(value), ramp=ramp)
    except (ValueError, TypeError):
        valve_controller.message_queue.append(f"[ERR] V{valve}: invalid value '{value}'")


@socketio.on('apply_all')
def on_apply_all(data):
    """data = {entries: [{valve:int, value:kPa|'off'}, ...], ramp: float}

    Validates everything first; if anything is invalid the whole batch is
    aborted, mirroring vc2_gui.py's APPLY ALL behaviour.
    """
    if valve_controller is None:
        return
    entries = data.get('entries', [])
    pairs = []
    invalid = []
    for e in entries:
        valve = int(e.get('valve'))
        raw = e.get('value')
        if isinstance(raw, str) and raw.strip().lower() in ('off', 'o', 'x'):
            pairs.append((valve, 'off'))
            continue
        try:
            val = float(raw)
        except (ValueError, TypeError):
            invalid.append((valve, raw))
            continue
        if val > MAX_INPUT_KPA:
            invalid.append((valve, raw))
            continue
        pairs.append((valve, val))

    if invalid:
        for v, txt in invalid:
            valve_controller.message_queue.append(
                f"[ERR] V{v}: invalid value '{txt}' (APPLY ALL aborted)"
            )
        return

    if not pairs:
        valve_controller.message_queue.append("[INFO] APPLY ALL: no values to apply")
        return

    ramp = _parse_ramp(data)
    if valve_controller.set_multiple_valves(pairs, ramp=ramp):
        suffix = f" over {ramp:g}s" if ramp else ""
        valve_controller.message_queue.append(f"APPLY ALL: {len(pairs)} valves{suffix}")


@socketio.on('valve_off')
def on_valve_off(data):
    if valve_controller is not None:
        valve_controller.valve_off(int(data.get('valve')), ramp=_parse_ramp(data))


@socketio.on('stop')
def on_stop():
    if valve_controller is not None:
        valve_controller.emergency_stop()
        valve_controller.message_queue.append('** EMERGENCY STOP (button) **')


@socketio.on('status')
def on_status():
    if valve_controller is not None:
        valve_controller.query_status()


@socketio.on('ping_arduino')
def on_ping():
    if valve_controller is not None:
        valve_controller.ping()


# ---- Encoder events (RS-485) -------------------------------

@socketio.on('zero')
def on_zero():
    """Global ZERO ALL — zero every online sensor."""
    if encoder_controller is not None:
        encoder_controller.zero_all()


@socketio.on('zero_sensor')
def on_zero_sensor(data):
    """Individual zero — zero one sensor. data = {slave: int}."""
    if encoder_controller is None:
        return
    try:
        slave = int(data.get('slave'))
    except (ValueError, TypeError):
        return
    encoder_controller.zero(slave)


@socketio.on('clear_zero')
def on_clear_zero():
    """Global CLEAR ZERO — drop every zero offset (back to absolute mm)."""
    if encoder_controller is not None:
        encoder_controller.clear_zero()


@socketio.on('rescan')
def on_rescan():
    """Re-sweep the RS-485 bus and re-read the sensor mapping."""
    global sensor_map
    if encoder_controller is not None:
        encoder_controller.rescan()
    sensor_map = load_mapping(mapping_path)


# ============================================================
# Entry point
# ============================================================

def main():
    global valve_controller, encoder_controller, sensor_map, mapping_path

    parser = argparse.ArgumentParser(
        description="vc2 pneumatic station web app (valves + RS-485 encoders)")
    # Ports
    parser.add_argument('--valve-port', default=None,
                        help='Serial port of the Giga R1 regulator (e.g. /dev/ttyACM0)')
    parser.add_argument('--rs485-port', default=None,
                        help='Serial port of the EKU081 USB↔RS-485 adapter '
                             '(e.g. /dev/ttyCH9344USB0)')
    parser.add_argument('--no-valves', action='store_true',
                        help='Do not open the valve regulator')
    parser.add_argument('--no-encoders', '--ne', action='store_true',
                        help='Do not open the RS-485 encoder bus')
    # Encoder / Modbus options
    parser.add_argument('--ids', type=parse_slave_ids, default=parse_slave_ids("50-80"),
                        help='Encoder slave ids to scan, e.g. 50-80 or 1,2,10')
    parser.add_argument('--baud', type=int, default=115200, help='RS-485 baud rate')
    parser.add_argument('--parity', default='N', choices=['N', 'E', 'O'],
                        help='RS-485 parity')
    parser.add_argument('--counts-per-turn', type=lambda v: int(v, 0),
                        default=DEFAULT_COUNTS_PER_TURN,
                        help='Encoder single-turn resolution for absolute position')
    parser.add_argument('--drum-diameter', type=float,
                        default=DEFAULT_DRUM_DIAMETER_MM,
                        help='Draw-wire drum diameter in mm (default 14.0)')
    parser.add_argument('--timeout', type=float, default=0.06,
                        help='RS-485 per-read timeout (s)')
    parser.add_argument('--interval', type=float, default=0.2,
                        help='Seconds between encoder poll cycles')
    # Sensor mapping
    parser.add_argument('--mapping', default=DEFAULT_MAPPING_PATH,
                        help='Path to the sensor↔regulator mapping JSON')
    # Web server
    parser.add_argument('--host', default='0.0.0.0', help='Web server host')
    parser.add_argument('--http-port', type=int, default=5000,
                        help='Web server port (default 5000)')
    args = parser.parse_args()

    print("=" * 65)
    print("   VC2 PNEUMATIC STATION (Web App) — valves + RS-485 encoders")
    print("=" * 65)

    mapping_path = args.mapping
    sensor_map = load_mapping(mapping_path)

    valve_port, rs485_port = resolve_ports(args)
    print(f"Valve regulator : {valve_port or '(none / disabled)'}")
    print(f"RS-485 encoders : {rs485_port or '(none / disabled)'}")
    print(f"Sensor mapping  : {mapping_path} ({len(sensor_map)} regulator(s) mapped)")

    if not args.no_valves and valve_port:
        vc = ValveController(port=valve_port)
        valve_controller = vc if vc.connected else None
        if valve_controller is None:
            print(f"[WARN] Could not open valve regulator on {valve_port}")

    if not args.no_encoders and rs485_port:
        ec = EncoderController(
            port=rs485_port,
            baudrate=args.baud,
            parity=args.parity,
            slave_ids=args.ids,
            counts_per_turn=args.counts_per_turn,
            drum_diameter_mm=args.drum_diameter,
            timeout=args.timeout,
            interval=args.interval,
        )
        encoder_controller = ec if ec.connected else None
        if encoder_controller is None:
            print(f"[WARN] Could not open RS-485 adapter on {rs485_port}")

    if valve_controller is None and encoder_controller is None:
        print("[ERR] Opened neither device. Check connections / --valve-port / --rs485-port.")
        return

    print(f"\n[OK] Serving on http://{args.host}:{args.http_port}")
    try:
        socketio.run(app, host=args.host, port=args.http_port,
                     allow_unsafe_werkzeug=True)
    except KeyboardInterrupt:
        print("\n[STOP] Shutting down...")
    finally:
        if valve_controller is not None:
            try:
                valve_controller.emergency_stop()
                time.sleep(0.2)
            except Exception:
                pass
            valve_controller.close()
        if encoder_controller is not None:
            encoder_controller.close()


if __name__ == '__main__':
    main()
