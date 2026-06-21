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


NUM_VALVES = 16
ROW_SIZE = 8
MAX_INPUT_VALUE = 4000  # mV or kPa — never send values above this


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

    def set_valve(self, valve, value):
        """Set a single valve to value (mV or kPa depending on Arduino mode)."""
        if not self._value_within_limit(value):
            return False
        return self.send(f"{valve},{value}")

    def set_multiple_valves(self, valve_value_pairs):
        """Set multiple valves in one batched command.

        valve_value_pairs: list of (valve, value) where value is an int
        (mV / kPa) or the string 'off'.
        """
        for _, val in valve_value_pairs:
            if val == 'off':
                continue
            if not self._value_within_limit(val):
                return False
        cmd = ",".join(f"{v},{val}" for v, val in valve_value_pairs)
        return self.send(cmd)

    def valve_off(self, valve):
        """Turn off a specific valve."""
        with self.display_lock:
            self.valve_data.pop(valve, None)
        return self.send(f"{valve},off")

    def emergency_stop(self):
        """Emergency stop - all valves off."""
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
        if self.read_thread:
            self.read_thread.join(timeout=1)
        if self.ser:
            self.ser.close()
            print("\n[OK] Connection closed")
