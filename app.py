#!/usr/bin/env python3
"""
app.py

Flask + Flask-SocketIO web app for the vc2 pneumatic station. It drives BOTH
serial devices of the station over two separate USB ports:

  - Arduino Giga R1 (valve regulator) — valve control, via ValveController
    (vc2/vc2.ino protocol). Enumerates as e.g. /dev/ttyACM0 "Arduino Giga".
    Setpoints are entered in kPa; ValveController converts to the mV the
    sketch expects.
  - EKU081 8-port USB↔RS-485 adapter (GJW draw-wire sensors) — one poll
    thread per channel, merged by SensorHub. Linux: /dev/ttyCH9344USB0 … USB7;
    Windows: "WCH USB-SERIAL Ch A (COMn)" … "Ch H". Every channel is opened by
    default; --rs485-ports restricts it.

Both devices are opened at once, so the browser can command valves and watch
all sensors live side by side. Either device may be absent — the app runs with
whichever it can open.

Usage:
    python app.py                                   # auto-detect everything
    python app.py --max-kpa 50                      # lower the pressure cap
    python app.py --valve-port /dev/ttyACM0 --rs485-ports /dev/ttyCH9344USB0
    python app.py --no-valves                       # sensors only
    python app.py --presets my_presets.json         # presets for 'ps <name>'

The command box under the activity log takes one line: v,kPa pairs, s, ?, p, a,
i1, i0, i or 'ps <name>' (see presets.py).
"""

import argparse
import os
import sys
import threading
import time
from types import SimpleNamespace
from typing import List, Optional, Tuple

import serial.tools.list_ports
from flask import Flask, render_template, jsonify
from flask_socketio import SocketIO

from valve_controller import ValveController, MAX_INPUT_KPA, NUM_VALVES, check_max_kpa
from encoder_controller import (
    DEFAULT_COUNTS_PER_TURN,
    DEFAULT_DRUM_DIAMETER_MM,
    DEFAULT_SLAVE_IDS,
    list_ch9344_ports,
    parse_slave_ids,
)
from sensor_hub import SensorHub
from sensor_mapping import DEFAULT_MAPPING_PATH, load_all
from presets import CommandError, PresetRunner, load_presets, parse_command, parse_preset


app = Flask(__name__)
app.config['SECRET_KEY'] = 'vc2-pneumatic-station'
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")

# Length-bar scaling (mm). Configurable here in code, not via CLI. Each valve's
# bar plots its mapped sensor's length within the band of its muscle's region
# (sensor_mapping.json "region"); palmar muscles travel less than forearm ones.
LENGTH_BAR_BANDS_MM = {"palm": (-4.0, 4.0)}
LENGTH_BAR_DEFAULT_MM = (-10.0, 10.0)


def bar_band(region):
    """(min_mm, max_mm) of the length bar for a muscle region."""
    return LENGTH_BAR_BANDS_MM.get(region, LENGTH_BAR_DEFAULT_MM)

# Two shared devices, created in main(). Either may stay None if absent or
# disabled. The sensors are one SensorHub over every RS-485 channel in use.
valve_controller = None
sensor_hub = None
_broadcast_started = False

# regulator (valve id) -> {"muscle", "sensor"} and sensor id -> {"muscle",
# "regulator"}; loaded in main(), reloaded on RESCAN so edits to
# sensor_mapping.json apply without restarting the server.
sensor_map = {}
sensor_index = {}
mapping_path = DEFAULT_MAPPING_PATH
ids_from_mapping = True     # False when --ids fixes the scanned ids

# Presets for the command box ('ps <name>'); the file is re-read on every use.
DEFAULT_PRESETS_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "presets.json")
presets_path = DEFAULT_PRESETS_PATH
preset_runner = None        # created with the valve controller


# ============================================================
# Port resolution — tell the Giga R1 from the RS-485 adapter
# ============================================================

def _usb_ports(eku):
    """Serial ports that look like real USB/CDC devices, plus the EKU081
    channels in `eku` (globbed on Linux, so maybe unseen by pyserial).
    Bluetooth COM ports are skipped: opening one can stall on a connect."""
    out = []
    for p in serial.tools.list_ports.comports():
        dev = p.device or ""
        if "BTHENUM" in (p.hwid or "").upper():
            continue
        if ("ttyACM" in dev or "ttyUSB" in dev or dev in eku
                or dev.upper().startswith("COM")):
            out.append(p)
    seen = {p.device for p in out}
    out += [SimpleNamespace(device=d, description="CH9344 USB-RS485 (EKU081)",
                            hwid="", product=None, manufacturer=None)
            for d in eku if d not in seen]
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
    """Common USB↔RS-485 bridge chips (EKU081 channels are resolved earlier)."""
    b = _blob(p)
    return any(k in b for k in (
        "1a86", "ch340", "ch343", "ch9344", "single serial",  # WCH
        "0403", "ftdi",                               # FTDI
        "10c4", "cp210",                              # Silicon Labs
        "rs485", "rs-485", "usb serial", "usb-serial",
    ))


def resolve_ports(args) -> Tuple[Optional[str], List[str]]:
    """Pick the valve regulator port and the RS-485 sensor ports, all distinct."""
    eku = list_ch9344_ports()
    ports = _usb_ports(set(eku))

    valve_port = args.valve_port
    if not valve_port and not args.no_valves:
        # An EKU081 channel is never the regulator, whatever its description says.
        valve_port = next((p.device for p in ports
                           if _is_giga(p) and p.device not in eku), None)

    rs485_ports: List[str] = []
    if not args.no_encoders:
        if args.rs485_ports:
            rs485_ports = list(args.rs485_ports)
        else:
            # Every EKU081 channel: channels without sensors go idle after their
            # first scan. --rs485-ports restricts the set.
            rs485_ports = [d for d in eku if d != valve_port]
            if not rs485_ports:
                # No EKU081: fall back to a generic bridge, never the Giga.
                others = [p for p in ports if p.device != valve_port and not _is_giga(p)]
                fallback = (next((p.device for p in others if _is_rs485_bridge(p)), None)
                            or next((p.device for p in others), None))
                rs485_ports = [fallback] if fallback else []

    if valve_port and valve_port in rs485_ports:
        raise SystemExit(
            f"[ERR] {valve_port} is both the valve port and an RS-485 port; "
            "pass distinct --valve-port / --rs485-ports")
    return valve_port, rs485_ports


def default_slave_ids(index) -> List[int]:
    """Sensor ids to scan without --ids: every sensor in the mapping, or the old
    default range when the mapping names none."""
    return sorted(index) or list(DEFAULT_SLAVE_IDS)


def parse_port_list(value: str) -> List[str]:
    """'COM7,COM8' -> ['COM7', 'COM8']."""
    ports = [p.strip() for p in value.split(",") if p.strip()]
    if not ports:
        raise argparse.ArgumentTypeError("expected at least one port")
    return ports


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
        lo, hi = bar_band(m.get("region"))
        muscles.append({
            "regulator": reg,
            "muscle": m.get("muscle"),          # None: regulator not in the mapping
            "sensor": sensor,
            "position_mm": pos,
            "sensor_online": online,
            "bar_min_mm": lo,
            "bar_max_mm": hi,
        })
    return muscles


def build_state():
    """Full snapshot pushed to the browser: valves, sensors, joined muscles."""
    enc_state = sensor_hub.get_state() if sensor_hub else None
    return {
        "valves": valve_controller.get_state() if valve_controller else None,
        "encoders": enc_state,
        "muscles": _muscles_state(enc_state),
        "sensor_index": {str(s): v for s, v in sensor_index.items()},
        "length_bar_min_mm": LENGTH_BAR_DEFAULT_MM[0],
        "length_bar_max_mm": LENGTH_BAR_DEFAULT_MM[1],
        "length_bar_bands_mm": {k: list(v) for k, v in LENGTH_BAR_BANDS_MM.items()},
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
        if sensor_hub:
            lines += [f"[ENC] {m}" for m in sensor_hub.drain_messages()]
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
        if val > valve_controller.max_kpa:
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


def stop_everything(source):
    """End any preset first (so no later step goes out), then vent every valve."""
    if preset_runner is not None:
        preset_runner.stop()
    if valve_controller is not None:
        valve_controller.emergency_stop()
        valve_controller.message_queue.append(f'** EMERGENCY STOP ({source}) **')


@socketio.on('stop')
def on_stop():
    stop_everything('button')


@socketio.on('status')
def on_status():
    if valve_controller is not None:
        valve_controller.query_status()


@socketio.on('ping_arduino')
def on_ping():
    if valve_controller is not None:
        valve_controller.ping()


def _log_adc_reply(vc, since, timeout=1.0):
    """The 'a' reply is not echoed to the log by the reader; show it once here."""
    end = time.time() + timeout
    while time.time() < end:
        if vc.adc_time > since:
            volts = " ".join(f"{ch}:{v:.3f}" for ch, v in sorted(vc.adc_volts.items()))
            vc.message_queue.append(f"ADC (V): {volts}")
            return
        time.sleep(0.02)
    vc.message_queue.append("[ERR] no ADC reply")


@socketio.on('command')
def on_command(data):
    """data = {text: one command line, ramp: TIME box seconds}. See presets.py."""
    text = str(data.get('text', '')).strip()
    vc = valve_controller
    if vc is None:
        socketio.emit('messages', {'lines': ['[ERR] valves not connected']})
        return
    vc.message_queue.append(f"> {text}")
    try:
        cmd = parse_command(text, vc.max_kpa)
        if cmd[0] == "preset":
            name = cmd[1]
            presets = load_presets(presets_path)
            if name not in presets:
                raise CommandError(f"no preset '{name}' in {os.path.basename(presets_path)} "
                                   f"(have: {', '.join(presets) or 'none'})")
            steps = parse_preset(presets[name], vc.max_kpa)
    except CommandError as exc:
        vc.message_queue.append(f"[ERR] {exc}")
        return

    if cmd[0] == "stop":
        stop_everything('command')
    elif cmd[0] == "raw":
        since = vc.adc_time
        vc.send(cmd[1])
        if cmd[1] == "a":
            threading.Thread(target=_log_adc_reply, args=(vc, since), daemon=True).start()
    elif cmd[0] == "set":
        vc.set_multiple_valves(cmd[1], ramp=_parse_ramp(data))
    elif cmd[0] == "preset":
        preset_runner.start(name, steps)


# ---- Encoder events (RS-485) -------------------------------

@socketio.on('zero')
def on_zero():
    """Global ZERO ALL — zero every online sensor."""
    if sensor_hub is not None:
        sensor_hub.zero_all()


@socketio.on('zero_sensor')
def on_zero_sensor(data):
    """Individual zero — zero one sensor. data = {slave: int}."""
    if sensor_hub is None:
        return
    try:
        slave = int(data.get('slave'))
    except (ValueError, TypeError):
        return
    sensor_hub.zero(slave)


@socketio.on('clear_zero')
def on_clear_zero():
    """Global CLEAR ZERO — drop every zero offset (back to absolute mm)."""
    if sensor_hub is not None:
        sensor_hub.clear_zero()


@socketio.on('rescan')
def on_rescan():
    """Reload the mapping and re-sweep every RS-485 channel (on its own thread)."""
    global sensor_map, sensor_index
    sensor_map, sensor_index = load_all(mapping_path)
    if sensor_hub is not None:
        sensor_hub.request_rescan(
            default_slave_ids(sensor_index) if ids_from_mapping else None)


# ============================================================
# Entry point
# ============================================================

def main():
    global valve_controller, sensor_hub, sensor_map, sensor_index
    global mapping_path, ids_from_mapping, presets_path, preset_runner

    # Piped output on Windows is cp1252, which cannot print e.g. the help's
    # arrows; replace such characters instead of crashing.
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(errors="replace")

    parser = argparse.ArgumentParser(
        description="vc2 pneumatic station web app (valves + RS-485 sensors)")
    # Ports
    parser.add_argument('--valve-port', default=None,
                        help='Serial port of the Giga R1 regulator (e.g. COM8, /dev/ttyACM0)')
    parser.add_argument('--rs485-ports', type=parse_port_list, default=None,
                        help='Comma list of EKU081 channels (default: all of them)')
    parser.add_argument('--rs485-port', default=None,
                        help='A single RS-485 port (same as --rs485-ports with one port)')
    parser.add_argument('--no-valves', action='store_true',
                        help='Do not open the valve regulator')
    parser.add_argument('--no-encoders', '--ne', action='store_true',
                        help='Do not open the RS-485 sensor bus')
    parser.add_argument('--max-kpa', type=float, default=MAX_INPUT_KPA,
                        help=f'Session pressure cap in kPa, 0 < cap <= {MAX_INPUT_KPA:g} '
                             f'(default {MAX_INPUT_KPA:g})')
    # Encoder / Modbus options
    parser.add_argument('--ids', type=parse_slave_ids, default=None,
                        help='Sensor ids to scan (default: the ids in the mapping), '
                             'e.g. 50-86,173-175')
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
    parser.add_argument('--presets', default=DEFAULT_PRESETS_PATH,
                        help="Presets JSON for the command box's 'ps <name>' "
                             '(default presets.json next to app.py)')
    # Web server
    parser.add_argument('--host', default='0.0.0.0', help='Web server host')
    parser.add_argument('--http-port', type=int, default=5000,
                        help='Web server port (default 5000)')
    args = parser.parse_args()
    try:
        max_kpa = check_max_kpa(args.max_kpa)
    except ValueError as exc:
        parser.error(str(exc))
    if args.rs485_port and not args.rs485_ports:
        args.rs485_ports = [args.rs485_port]

    print("=" * 65)
    print("   VC2 PNEUMATIC STATION (Web App) — valves + RS-485 sensors")
    print("=" * 65)

    mapping_path = args.mapping
    presets_path = args.presets
    sensor_map, sensor_index = load_all(mapping_path)
    ids_from_mapping = args.ids is None
    slave_ids = args.ids or default_slave_ids(sensor_index)

    valve_port, rs485_ports = resolve_ports(args)
    print(f"Valve regulator : {valve_port or '(none / disabled)'}")
    print(f"RS-485 sensors  : {', '.join(rs485_ports) or '(none / disabled)'}")
    print(f"Sensor mapping  : {mapping_path} ({len(sensor_map)} regulator(s), "
          f"{len(sensor_index)} sensor(s))")
    print(f"Scanned ids     : {len(slave_ids)} ({slave_ids[0]}..{slave_ids[-1]}, "
          f"{'from the mapping' if ids_from_mapping else '--ids'})")
    print(f"Pressure cap    : {max_kpa:g} kPa")
    print(f"Presets         : {presets_path}"
          f"{'' if os.path.exists(presets_path) else ' (not found yet)'}")

    if not args.no_valves and valve_port:
        vc = ValveController(port=valve_port, max_kpa=max_kpa)
        valve_controller = vc if vc.connected else None
        if valve_controller is None:
            print(f"[WARN] Could not open valve regulator on {valve_port}")
        else:
            preset_runner = PresetRunner(vc, vc.message_queue.append)

    if not args.no_encoders and rs485_ports:
        hub = SensorHub(
            rs485_ports, slave_ids,
            baudrate=args.baud,
            parity=args.parity,
            counts_per_turn=args.counts_per_turn,
            drum_diameter_mm=args.drum_diameter,
            timeout=args.timeout,
            interval=args.interval,
        )
        if hub.connected:
            sensor_hub = hub
        else:
            for m in hub.drain_messages():
                print(m)
            print(f"[WARN] Could not open any RS-485 port of {', '.join(rs485_ports)}")

    if valve_controller is None and sensor_hub is None:
        print("[ERR] Opened neither device. Check connections / --valve-port / --rs485-ports.")
        return

    print(f"\n[OK] Serving on http://{args.host}:{args.http_port}")
    try:
        socketio.run(app, host=args.host, port=args.http_port,
                     allow_unsafe_werkzeug=True)
    except KeyboardInterrupt:
        print("\n[STOP] Shutting down...")
    finally:
        if preset_runner is not None:
            preset_runner.stop()
        if valve_controller is not None:
            try:
                valve_controller.emergency_stop()
                time.sleep(0.2)
            except Exception:
                pass
            valve_controller.close()
        if sensor_hub is not None:
            sensor_hub.close()


if __name__ == '__main__':
    main()
