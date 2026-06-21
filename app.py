#!/usr/bin/env python3
"""
app.py

Flask + Flask-SocketIO web app for the vc2 16-valve controller. Provides the
same functionality as vc2/vc2_gui.py (the matplotlib GUI) in a browser:

  - Per-valve input box: set a single valve
  - APPLY ALL: fire every non-empty box in one batched command
  - STOP: emergency stop (all valves off)
  - STATUS: query all valve states
  - PING: ping the Arduino
  - Live bar display of all 16 valves with mV + bar pressure

The serial layer lives in valve_controller.ValveController (Giga R1 / vc2.ino).

Usage:
    python app.py [port]
    python app.py /dev/ttyACM0
    python app.py --mock            # run without hardware
    python app.py --mock --port 8000
"""

import argparse
import threading
import time

from flask import Flask, render_template, jsonify
from flask_socketio import SocketIO

from valve_controller import ValveController, NUM_VALVES, MAX_INPUT_VALUE


app = Flask(__name__)
app.config['SECRET_KEY'] = 'vc2-valve-controller'
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")

# Single shared controller instance, created in main().
controller = None
_broadcast_started = False
_broadcast_lock = threading.Lock()


# ============================================================
# Background broadcaster: push state + messages to all clients
# ============================================================

def _broadcast_loop():
    """Emit valve state and drained messages ~10x/sec."""
    while True:
        if controller is not None:
            state = controller.get_state()
            msgs = controller.drain_messages()
            socketio.emit('state', state)
            if msgs:
                socketio.emit('messages', {'lines': msgs})
        socketio.sleep(0.1)


def _ensure_broadcaster():
    global _broadcast_started
    with _broadcast_lock:
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
    if controller is None:
        return jsonify({"connected": False})
    return jsonify(controller.get_state())


@app.route('/api/ports')
def api_ports():
    return jsonify({"ports": ValveController.list_ports()})


# ============================================================
# SocketIO events (one per GUI action)
# ============================================================

@socketio.on('connect')
def on_connect():
    _ensure_broadcaster()
    if controller is not None:
        socketio.emit('state', controller.get_state())


@socketio.on('set_valve')
def on_set_valve(data):
    """data = {valve: int, value: int | 'off'}"""
    if controller is None:
        return
    valve = int(data.get('valve'))
    value = data.get('value')
    if isinstance(value, str) and value.strip().lower() in ('off', 'o', 'x'):
        controller.valve_off(valve)
        return
    try:
        controller.set_valve(valve, int(value))
    except (ValueError, TypeError):
        controller.message_queue.append(f"[ERR] V{valve}: invalid value '{value}'")


@socketio.on('apply_all')
def on_apply_all(data):
    """data = {entries: [{valve:int, value:int|'off'}, ...]}

    Validates everything first; if anything is invalid the whole batch is
    aborted, mirroring vc2_gui.py's APPLY ALL behaviour.
    """
    if controller is None:
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
            val = int(raw)
        except (ValueError, TypeError):
            invalid.append((valve, raw))
            continue
        if val > MAX_INPUT_VALUE:
            invalid.append((valve, raw))
            continue
        pairs.append((valve, val))

    if invalid:
        for v, txt in invalid:
            controller.message_queue.append(
                f"[ERR] V{v}: invalid value '{txt}' (APPLY ALL aborted)"
            )
        return

    if not pairs:
        controller.message_queue.append("[INFO] APPLY ALL: no values to apply")
        return

    if controller.set_multiple_valves(pairs):
        controller.message_queue.append(f"APPLY ALL: {len(pairs)} valves")


@socketio.on('valve_off')
def on_valve_off(data):
    if controller is not None:
        controller.valve_off(int(data.get('valve')))


@socketio.on('stop')
def on_stop():
    if controller is not None:
        controller.emergency_stop()
        controller.message_queue.append('** EMERGENCY STOP (button) **')


@socketio.on('status')
def on_status():
    if controller is not None:
        controller.query_status()


@socketio.on('ping_arduino')
def on_ping():
    if controller is not None:
        controller.ping()


# ============================================================
# Entry point
# ============================================================

def main():
    global controller

    parser = argparse.ArgumentParser(description="vc2 16-valve web controller")
    parser.add_argument('port', nargs='?', default=None,
                        help='Serial port (e.g. /dev/ttyACM0)')
    parser.add_argument('--mock', action='store_true',
                        help='Run without hardware (simulated valves)')
    parser.add_argument('--host', default='0.0.0.0', help='Web server host')
    parser.add_argument('--http-port', type=int, default=5000,
                        help='Web server port (default 5000)')
    args = parser.parse_args()

    print("=" * 65)
    print("   16-VALVE CONTROLLER (Web App)")
    print("=" * 65)

    controller = ValveController(port=args.port, mock=args.mock)
    if not controller.connected:
        print("[ERR] Failed to connect. Use --mock to run without hardware.")
        return

    print(f"\n[OK] Serving on http://{args.host}:{args.http_port}")
    try:
        socketio.run(app, host=args.host, port=args.http_port,
                     allow_unsafe_werkzeug=True)
    except KeyboardInterrupt:
        print("\n[STOP] Shutting down...")
    finally:
        try:
            controller.emergency_stop()
            time.sleep(0.2)
        except Exception:
            pass
        controller.close()


if __name__ == '__main__':
    main()
