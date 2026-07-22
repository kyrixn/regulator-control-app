#!/usr/bin/env python3
"""
valve_controller.py

Serial layer for the vc2 16-valve controller, adapted from vc2/vc2_control.py
for use in the web app. The matplotlib display has been removed; this module
only owns the serial connection, a background read thread, and the parsed
valve state.

Hardware target: Arduino Giga R1 running vc2/vc2.ino.
  - Valves  0..7  on Wire   (SDA/SCL)
  - Valves  8..15 on Wire1  (SDA1/SCL1)

Arduino command set (Serial @ 115200):
  valve,value          Set single valve: 0,3000 or 9,2100
  v1,val1,v2,val2,...  Set multiple valves: 0,3000,9,2500
  valve,off            Turn off a valve: 9,off
  s                    Emergency stop (all valves off)
  ?                    Query status of all valves
  p                    Ping test
"""

import re
import threading
import time
from collections import deque

import serial
import serial.tools.list_ports

from encoder_controller import is_ch9344_port


NUM_VALVES = 16
ROW_SIZE = 8
MAX_INPUT_VALUE = 4000  # mV or kPa — never send values above this
RAMP_STEP = 0.04        # seconds between setpoints while ramping (~25 Hz)


def mv_to_bar(mV):
    """Convert commanded mV to output pressure in bar.

    Device maps 0-10 V to -100..500 kPa, i.e. kPa = (mV/1000)*60 - 100.
    Then bar = kPa / 100. Clamped to >= 0 (no vacuum hardware).
    """
    bar = ((mV / 1000.0) * 60.0 - 100.0) / 100.0
    return max(0.0, bar)


class ValveController:
    """16-valve serial interface (headless)."""

    def __init__(self, port=None, baudrate=115200):
        self.ser = None
        self.baudrate = baudrate
        self.running = False
        self.read_thread = None
        self.port_name = None

        # Valve readings (commanded mV): {valve_id: mV}
        self.valve_data = {}

        # Protects valve_data against concurrent access.
        self.display_lock = threading.Lock()

        # Recent Arduino / app messages for the activity log.
        self.message_queue = deque(maxlen=50)

        # Active linear ramps: valve -> dict(start_val, target, off, start_t, end_t).
        # A background ticker thread streams interpolated setpoints over serial.
        self.ramps = {}
        self.ramp_lock = threading.Lock()
        self.ramp_thread = None

        if port:
            self.connect(port)
        else:
            self.auto_connect()

    # ------------------------------------------------------------------
    # Connection
    # ------------------------------------------------------------------

    @staticmethod
    def list_ports():
        """Return a list of (device, description) tuples for available ports."""
        return [(p.device, p.description)
                for p in serial.tools.list_ports.comports()]

    def auto_connect(self):
        """Try to auto-detect and connect to the Arduino."""
        ports = serial.tools.list_ports.comports()

        print("\nAvailable serial ports:")
        for i, port in enumerate(ports):
            print(f"  [{i}] {port.device} - {port.description}")

        if not ports:
            print("[ERR] No serial ports found!")
            return False

        # Skip the EKU081 RS-485 adapter's nodes (ttyCH9344USB*): they belong to
        # the encoder bus, and their names would otherwise match 'USB' below.
        ports = [p for p in ports if not is_ch9344_port(p.device)]
        if not ports:
            print("[ERR] No non-RS485 serial ports found!")
            return False

        for port in ports:
            if ('ACM' in port.device or 'USB' in port.device
                    or 'Arduino' in port.description):
                try:
                    print(f"\nTrying {port.device}...")
                    self.connect(port.device)
                    return True
                except Exception as e:
                    print(f"  Failed: {e}")

        # Fall back to the first port rather than blocking on input(),
        # since this may run inside a web server process.
        try:
            self.connect(ports[0].device)
            return True
        except Exception as e:
            print(f"[ERR] Connection failed: {e}")
            return False

    def connect(self, port):
        """Connect to the specified serial port."""
        self.ser = serial.Serial(port, self.baudrate, timeout=0.1)
        self.port_name = port
        time.sleep(2)  # Wait for Arduino reset
        print(f"[OK] Connected to {port}")

        self.running = True
        self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.read_thread.start()

        self.ramp_thread = threading.Thread(target=self._ramp_loop, daemon=True)
        self.ramp_thread.start()

    @property
    def connected(self):
        return self.running and self.ser is not None

    # ------------------------------------------------------------------
    # Serial read / parse
    # ------------------------------------------------------------------

    def _read_loop(self):
        """Background thread to read serial responses."""
        while self.running:
            try:
                if self.ser and self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        self._handle_response(line)
            except Exception as e:
                if self.running:
                    self.message_queue.append(f"[ERR] Read error: {e}")
            time.sleep(0.005)

    def _handle_response(self, line):
        """Process Arduino responses and keep valve_data in sync."""
        # Set/off acknowledgments
        if line.startswith("OK:"):
            for m in re.findall(r'V(\d+)=(-?\d+)', line):
                v, mV = int(m[0]), int(m[1])
                with self.display_lock:
                    self.valve_data[v] = mV

            for m in re.findall(r'V(\d+)\s+OFF', line):
                v = int(m)
                with self.display_lock:
                    self.valve_data.pop(v, None)

            self.message_queue.append(line)
            return

        # Status query response: "V0=val | V1=val | ... | V15=val"
        if line.startswith("V") and "=" in line and "|" in line:
            matches = re.findall(r'V(\d+)=(-?\d+)', line)
            with self.display_lock:
                self.valve_data.clear()
                for m in matches:
                    v, val = int(m[0]), int(m[1])
                    if val != 0:
                        self.valve_data[v] = val
            self.message_queue.append(line)
            return

        if line.startswith("EMERGENCY"):
            with self.display_lock:
                self.valve_data.clear()
            self.message_queue.append(f"** {line} **")
            return

        if line.startswith("PONG"):
            self.message_queue.append(line)
            return

        if line.startswith("===") or line.startswith("---"):
            self.message_queue.append(line)
            return

        if "ERROR" in line:
            self.message_queue.append(line)
            return

        # Boot banner / info messages
        if (line.startswith("Mode:")
                or line.startswith("Range:")
                or line.startswith("Device")
                or line.startswith("Layout:")
                or line.startswith("Commands:")
                or line.startswith("Safety")
                or line.startswith("DAC initialized")
                or line.startswith("All DACs")
                or line.startswith("WARNING")):
            self.message_queue.append(line)

    # ------------------------------------------------------------------
    # Sending
    # ------------------------------------------------------------------

    def send(self, command):
        """Send a raw command line to the Arduino."""
        if not self.ser:
            self.message_queue.append("[ERR] Not connected!")
            return False

        try:
            self.ser.write(f"{command}\n".encode())
            return True
        except Exception as e:
            self.message_queue.append(f"[ERR] Send error: {e}")
            return False

    # ------------------------------------------------------------------
    # Ramping (linear setpoint streaming)
    # ------------------------------------------------------------------

    def _current_mV(self, valve):
        """Last commanded mV for a valve (0 if currently off/unknown)."""
        with self.display_lock:
            return self.valve_data.get(valve, 0)

    def _start_ramp(self, valve, target, off, duration):
        """Begin (or restart) a linear ramp on a valve over `duration` seconds."""
        now = time.time()
        with self.ramp_lock:
            self.ramps[valve] = {
                'start_val': self._current_mV(valve),
                'target': int(target),
                'off': bool(off),
                'start_t': now,
                'end_t': now + max(0.0, float(duration)),
            }

    def _cancel_ramp(self, valve):
        with self.ramp_lock:
            self.ramps.pop(valve, None)

    def _cancel_all_ramps(self):
        with self.ramp_lock:
            self.ramps.clear()

    def _ramp_loop(self):
        """Stream interpolated setpoints for all active ramps until they finish."""
        while self.running:
            time.sleep(RAMP_STEP)
            now = time.time()
            batch = []        # (valve, value-or-'off')
            finished = []

            with self.ramp_lock:
                if not self.ramps:
                    continue
                for valve, r in self.ramps.items():
                    dur = r['end_t'] - r['start_t']
                    frac = 1.0 if dur <= 0 else (now - r['start_t']) / dur
                    if frac >= 1.0:
                        frac = 1.0
                        finished.append(valve)
                        batch.append((valve, 'off' if r['off'] else r['target']))
                    else:
                        cur = int(round(
                            r['start_val'] + (r['target'] - r['start_val']) * frac
                        ))
                        batch.append((valve, cur))
                for valve in finished:
                    self.ramps.pop(valve, None)

            if batch:
                cmd = ",".join(f"{v},{val}" for v, val in batch)
                self.send(cmd)
                for v, val in batch:
                    if val == 'off':
                        with self.display_lock:
                            self.valve_data.pop(v, None)

    # ------------------------------------------------------------------
    # Control commands
    # ------------------------------------------------------------------

    def _value_within_limit(self, value):
        """Return True if value is allowed to be sent to the hardware."""
        if value > MAX_INPUT_VALUE:
            self.message_queue.append(
                f"[ERR] Value {value} exceeds limit (max {MAX_INPUT_VALUE})"
            )
            return False
        return True

    def set_valve(self, valve, value, ramp=0.0):
        """Set a single valve to value (mV or kPa depending on Arduino mode).

        If ramp > 0, linearly ramp from the current value to `value` over
        that many seconds instead of jumping immediately.
        """
        if not self._value_within_limit(value):
            return False
        if ramp and ramp > 0:
            self._start_ramp(valve, int(value), off=False, duration=ramp)
            return True
        self._cancel_ramp(valve)
        return self.send(f"{valve},{value}")

    def set_multiple_valves(self, valve_value_pairs, ramp=0.0):
        """Set multiple valves in one batched command.

        valve_value_pairs: list of (valve, value) where value is an int
        (mV / kPa) or the string 'off'.

        If ramp > 0, every valve in the batch linearly ramps from its current
        value to its target over that many seconds, simultaneously.
        """
        for _, val in valve_value_pairs:
            if val == 'off':
                continue
            if not self._value_within_limit(val):
                return False
        if ramp and ramp > 0:
            for v, val in valve_value_pairs:
                if val == 'off':
                    self._start_ramp(v, 0, off=True, duration=ramp)
                else:
                    self._start_ramp(v, int(val), off=False, duration=ramp)
            return True
        for v, _ in valve_value_pairs:
            self._cancel_ramp(v)
        cmd = ",".join(f"{v},{val}" for v, val in valve_value_pairs)
        return self.send(cmd)

    def valve_off(self, valve, ramp=0.0):
        """Turn off a specific valve (optionally ramping down to 0 first)."""
        if ramp and ramp > 0:
            self._start_ramp(valve, 0, off=True, duration=ramp)
            return True
        self._cancel_ramp(valve)
        with self.display_lock:
            self.valve_data.pop(valve, None)
        return self.send(f"{valve},off")

    def emergency_stop(self):
        """Emergency stop - all valves off (cancels any ramps in progress)."""
        self._cancel_all_ramps()
        with self.display_lock:
            self.valve_data.clear()
        return self.send("s")

    def query_status(self):
        """Query status of all valves."""
        return self.send("?")

    def ping(self):
        """Ping test."""
        return self.send("p")

    # ------------------------------------------------------------------
    # State accessors (used by the web layer)
    # ------------------------------------------------------------------

    def get_state(self):
        """Snapshot of valve state for serialization to the browser."""
        with self.display_lock:
            valves = {
                str(v): {"mV": mV, "bar": round(mv_to_bar(mV), 3)}
                for v, mV in self.valve_data.items()
            }
        return {
            "connected": self.connected,
            "port": self.port_name,
            "num_valves": NUM_VALVES,
            "max_value": MAX_INPUT_VALUE,
            "valves": valves,
            "active": len(valves),
        }

    def drain_messages(self):
        """Pop and return all queued messages (oldest first)."""
        msgs = []
        while self.message_queue:
            msgs.append(self.message_queue.popleft())
        return msgs

    def close(self):
        """Close the connection."""
        self.running = False
        self._cancel_all_ramps()
        if self.read_thread:
            self.read_thread.join(timeout=1)
        if self.ramp_thread:
            self.ramp_thread.join(timeout=1)
        if self.ser:
            self.ser.close()
            print("\n[OK] Connection closed")
