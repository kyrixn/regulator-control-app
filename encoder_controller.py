#!/usr/bin/env python3
"""
encoder_controller.py

Serial layer for the GJW RS-485 encoder dashboard, adapted from the
rs485-reader project (gjw_encoder_dashboard.py) for use in the web app.

The original was a Windows terminal dashboard (msvcrt keyboard handling, COM
ports, one serial open/close per read). This module instead:

  - keeps a single persistent serial connection open,
  - polls every configured slave id in a background thread (mirroring the
    read-thread model in valve_controller.py),
  - exposes a JSON-serialisable snapshot for the browser,
  - handles "zero" / "clear zero" as method calls driven by the web UI
    instead of keystrokes.

Hardware: GJW absolute encoders on one RS-485 bus, reached through a
USB↔RS-485 adapter (typically /dev/ttyUSB0 on Linux, COMx on Windows).
"""

from __future__ import annotations

import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence

import serial
import serial.tools.list_ports

from modbus_rtu import (
    GJW_STATE_COUNT,
    GJW_STATE_REGISTER,
    READ_FUNCTIONS,
    SerialTimeoutError,
    build_read_request,
    compute_absolute_position,
    decode_gjw_state_registers,
    parse_read_registers_response,
)


DEFAULT_COUNTS_PER_TURN = 2_097_152  # 21-bit single-turn resolution
DEFAULT_SLAVE_IDS = tuple(range(50, 81))  # matches rs485-reader's 50-80 default


@dataclass
class EncoderReading:
    slave: int
    online: bool = False
    absolute_position: Optional[int] = None
    single_turn_position: Optional[int] = None
    turns: Optional[int] = None
    speed: Optional[int] = None
    status_code: Optional[int] = None
    error_count: Optional[int] = None
    message: str = ""
    timestamp: float = 0.0


def parse_slave_ids(value: str) -> List[int]:
    """Parse comma-separated ids and inclusive ranges like '50-80,90'."""
    ids: List[int] = []
    seen = set()
    for part in value.split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part:
            start_text, end_text = part.split("-", 1)
            start = int(start_text.strip(), 0)
            end = int(end_text.strip(), 0)
            if end < start:
                raise ValueError(f"range end before start: {part}")
            candidates = range(start, end + 1)
        else:
            candidates = [int(part, 0)]
        for slave in candidates:
            if not 1 <= slave <= 247:
                raise ValueError(f"slave id out of Modbus range: {slave}")
            if slave not in seen:
                ids.append(slave)
                seen.add(slave)
    if not ids:
        raise ValueError("expected at least one slave id")
    return ids


class EncoderController:
    """GJW RS-485 encoder poller (headless)."""

    def __init__(
        self,
        port: Optional[str] = None,
        baudrate: int = 115200,
        parity: str = "N",
        slave_ids: Sequence[int] = DEFAULT_SLAVE_IDS,
        counts_per_turn: int = DEFAULT_COUNTS_PER_TURN,
        timeout: float = 0.06,
        interval: float = 0.2,
    ):
        self.ser = None
        self.baudrate = baudrate
        self.parity = parity
        self.slave_ids = list(slave_ids)
        self.counts_per_turn = counts_per_turn
        self.timeout = timeout
        self.interval = interval

        self.running = False
        self.poll_thread = None
        self.port_name = None

        # Which slave ids actually answered during the initial scan; only these
        # are polled in the live loop so absent ids don't slow every cycle.
        self.active_slaves: List[int] = []

        # slave_id -> EncoderReading
        self.readings: Dict[int, EncoderReading] = {}
        # slave_id -> absolute_position captured when the user pressed "zero".
        self.zero_offsets: Dict[int, int] = {}

        self.state_lock = threading.Lock()
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

    def auto_connect(self) -> bool:
        """Auto-detect and connect to a USB↔RS-485 adapter."""
        ports = serial.tools.list_ports.comports()

        print("\nAvailable serial ports:")
        for i, port in enumerate(ports):
            print(f"  [{i}] {port.device} - {port.description}")

        if not ports:
            print("[ERR] No serial ports found!")
            return False

        # RS-485 adapters usually enumerate as USB serial (ttyUSB* / ttyACM* on
        # Linux, or by a FTDI/CH340/CP210x description).
        for port in ports:
            desc = port.description or ""
            if ("USB" in port.device or "ACM" in port.device
                    or any(chip in desc for chip in ("FTDI", "CH340", "CP210",
                                                     "RS485", "RS-485", "Serial"))):
                try:
                    print(f"\nTrying {port.device}...")
                    self.connect(port.device)
                    return True
                except Exception as e:
                    print(f"  Failed: {e}")

        try:
            self.connect(ports[0].device)
            return True
        except Exception as e:
            print(f"[ERR] Connection failed: {e}")
            return False

    def connect(self, port: str) -> None:
        """Open the RS-485 adapter and start scanning + polling."""
        self.ser = serial.Serial(
            port=port,
            baudrate=self.baudrate,
            bytesize=8,
            parity=self.parity,
            stopbits=1,
            timeout=self.timeout,
        )
        self.port_name = port
        print(f"[OK] Connected to {port} @ {self.baudrate} 8{self.parity}1")
        self.message_queue.append(f"[OK] Connected to {port}")

        self.running = True
        self.poll_thread = threading.Thread(target=self._poll_loop, daemon=True)
        self.poll_thread.start()

    @property
    def connected(self) -> bool:
        return self.running and self.ser is not None

    # ------------------------------------------------------------------
    # Serial transaction (one Modbus read on the open connection)
    # ------------------------------------------------------------------

    def _read_state_block(self, slave: int) -> Dict[str, int]:
        """Send one GJW state read and return the decoded state dict."""
        request = build_read_request(
            slave, READ_FUNCTIONS["holding"], GJW_STATE_REGISTER, GJW_STATE_COUNT
        )
        self.ser.reset_input_buffer()
        self.ser.write(request)
        self.ser.flush()

        header = self.ser.read(3)
        if len(header) != 3:
            raise SerialTimeoutError(
                f"timeout: got {len(header)} of 3 header byte(s)"
            )
        if header[1] & 0x80:  # exception frame
            body = self.ser.read(2)
        else:
            body = self.ser.read(header[2] + 2)
        response = header + body

        registers = parse_read_registers_response(
            response, slave, READ_FUNCTIONS["holding"], GJW_STATE_COUNT
        )
        return decode_gjw_state_registers(registers)

    def _read_encoder(self, slave: int) -> EncoderReading:
        state = self._read_state_block(slave)
        return EncoderReading(
            slave=slave,
            online=True,
            single_turn_position=state["single_turn_position"],
            turns=state["turns"],
            absolute_position=compute_absolute_position(
                single_turn_position=state["single_turn_position"],
                turns=state["turns"],
                counts_per_turn=self.counts_per_turn,
            ),
            speed=state["angular_velocity"],
            status_code=state["status_code"],
            error_count=state["error_count"],
            message="ok",
            timestamp=time.time(),
        )

    # ------------------------------------------------------------------
    # Background polling
    # ------------------------------------------------------------------

    def _scan(self) -> None:
        """One pass over every configured id to find the encoders present."""
        found = []
        for slave in self.slave_ids:
            try:
                reading = self._read_encoder(slave)
            except Exception:
                continue
            found.append(slave)
            with self.state_lock:
                self.readings[slave] = reading
        self.active_slaves = found
        msg = (f"Scan complete: {len(found)} encoder(s) online "
               f"in ids {self.slave_ids[0]}-{self.slave_ids[-1]}"
               if found else
               f"No encoders found in ids {self.slave_ids[0]}-{self.slave_ids[-1]}")
        self.message_queue.append(msg)

    def _poll_loop(self) -> None:
        """Scan once, then continuously refresh the encoders that answered."""
        self._scan()
        while self.running:
            if not self.active_slaves:
                # Nothing responded; retry a full scan periodically.
                time.sleep(max(1.0, self.interval))
                if self.running:
                    self._scan()
                continue

            for slave in self.active_slaves:
                if not self.running:
                    break
                try:
                    reading = self._read_encoder(slave)
                except Exception as exc:
                    reading = EncoderReading(
                        slave=slave,
                        online=False,
                        message=str(exc).replace("\n", " ")[:40],
                        timestamp=time.time(),
                    )
                with self.state_lock:
                    self.readings[slave] = reading
            time.sleep(self.interval)

    # ------------------------------------------------------------------
    # Web-driven actions (replace the terminal 'z' / 'c' keystrokes)
    # ------------------------------------------------------------------

    def zero_all(self) -> None:
        """Capture current absolute positions as the display zero."""
        with self.state_lock:
            self.zero_offsets = {
                slave: r.absolute_position
                for slave, r in self.readings.items()
                if r.online and r.absolute_position is not None
            }
        self.message_queue.append(f"Zeroed {len(self.zero_offsets)} online encoder(s)")

    def clear_zero(self) -> None:
        """Drop all zero offsets (display absolute position again)."""
        with self.state_lock:
            self.zero_offsets = {}
        self.message_queue.append("Cleared all zero offsets")

    def rescan(self) -> None:
        """Force a fresh scan across the full slave-id range."""
        self.message_queue.append("Rescanning...")
        self._scan()

    # ------------------------------------------------------------------
    # State accessors (used by the web layer)
    # ------------------------------------------------------------------

    def get_state(self) -> dict:
        """Snapshot of encoder state for serialization to the browser."""
        with self.state_lock:
            zero_offsets = dict(self.zero_offsets)
            encoders = []
            for slave in sorted(self.readings):
                r = self.readings[slave]
                offset = zero_offsets.get(slave, 0)
                display = (None if r.absolute_position is None
                           else r.absolute_position - offset)
                encoders.append({
                    "slave": r.slave,
                    "online": r.online,
                    "display_position": display,
                    "absolute_position": r.absolute_position,
                    "single_turn_position": r.single_turn_position,
                    "turns": r.turns,
                    "speed": r.speed,
                    "status_code": r.status_code,
                    "error_count": r.error_count,
                    "zeroed": slave in zero_offsets,
                    "message": r.message,
                })
            online = sum(1 for e in encoders if e["online"])
        return {
            "connected": self.connected,
            "port": self.port_name,
            "baudrate": self.baudrate,
            "parity": self.parity,
            "counts_per_turn": self.counts_per_turn,
            "scanned_range": [self.slave_ids[0], self.slave_ids[-1]],
            "encoders": encoders,
            "online": online,
            "zeroed": len(zero_offsets),
        }

    def drain_messages(self) -> List[str]:
        """Pop and return all queued messages (oldest first)."""
        msgs = []
        while self.message_queue:
            msgs.append(self.message_queue.popleft())
        return msgs

    def close(self) -> None:
        """Stop polling and close the connection."""
        self.running = False
        if self.poll_thread:
            self.poll_thread.join(timeout=1)
        if self.ser:
            self.ser.close()
            print("\n[OK] Connection closed")
