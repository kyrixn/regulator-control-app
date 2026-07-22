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

Hardware: GJW absolute encoders on one RS-485 bus, reached through the EKU081
8-port USB↔RS-485 adapter (WCH CH9344), which enumerates as /dev/ttyCH9344USB0
… USB7 on Linux (COMx on Windows).
"""

from __future__ import annotations

import glob
import math
import re
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


# EKU081 8-port USB↔RS-485 adapter (WCH CH9344). Its vendor driver creates one
# node per port — /dev/ttyCH9344USB0 … USB7 — and, being out of tree, those
# nodes are not always enumerated by pyserial, so we glob /dev as well.
# ttyCH343USB* covers the 1/2-port CH343 sibling driver.
CH9344_PREFIXES = ("ttyCH9344", "ttyCH343")
CH9344_GLOBS = ("/dev/ttyCH9344USB*", "/dev/ttyCH343USB*")


def is_ch9344_port(device: str) -> bool:
    """True for a device node of the CH9344/CH343 USB↔RS-485 adapter."""
    return any(pre in (device or "") for pre in CH9344_PREFIXES)


def port_index(device: str) -> int:
    """Trailing number of a device node, for lowest-port-first ordering."""
    m = re.search(r"(\d+)$", device or "")
    return int(m.group(1)) if m else 0


def list_ch9344_ports() -> List[str]:
    """CH9344 device nodes present in /dev, lowest port number first."""
    return sorted({d for pat in CH9344_GLOBS for d in glob.glob(pat)},
                  key=port_index)


DEFAULT_COUNTS_PER_TURN = 2_097_152  # 21-bit single-turn resolution
DEFAULT_SLAVE_IDS = (1,) + tuple(range(50, 81))  # id 1 plus the 50-80 range
DEFAULT_DRUM_DIAMETER_MM = 14.0  # draw-wire drum: thread displacement = arc length


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
        drum_diameter_mm: float = DEFAULT_DRUM_DIAMETER_MM,
        timeout: float = 0.06,
        interval: float = 0.2,
    ):
        self.ser = None
        self.baudrate = baudrate
        self.parity = parity
        self.slave_ids = list(slave_ids)
        self.counts_per_turn = counts_per_turn
        self.drum_diameter_mm = drum_diameter_mm
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
        found = [(p.device, p.description)
                 for p in serial.tools.list_ports.comports()]
        seen = {dev for dev, _ in found}
        found += [(d, "CH9344 USB-RS485 (EKU081)")
                  for d in list_ch9344_ports() if d not in seen]
        return found

    def auto_connect(self) -> bool:
        """Auto-detect and connect to a USB↔RS-485 adapter."""
        ports = serial.tools.list_ports.comports()

        print("\nAvailable serial ports:")
        for i, port in enumerate(ports):
            print(f"  [{i}] {port.device} - {port.description}")

        # The EKU081 (CH9344) is the station's adapter: try its 8 nodes first,
        # lowest port number first. Globbed, since the out-of-tree driver's
        # nodes may be missing from comports() above.
        ch9344 = list_ch9344_ports()
        for dev in ch9344:
            print(f"  [ch9344] {dev}")
        for dev in ch9344:
            try:
                print(f"\nTrying {dev}...")
                self.connect(dev)
                return True
            except Exception as e:
                print(f"  Failed: {e}")

        if not ports:
            if not ch9344:
                print("[ERR] No serial ports found!")
            return False

        # Otherwise fall back to a generic USB serial bridge (ttyUSB* / ttyACM*
        # on Linux, or a FTDI/CH340/CP210x description).
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

    def zero(self, slave: int) -> None:
        """Capture one sensor's current absolute position as its display zero.

        Individual and global (`zero_all`) zeroing share `zero_offsets`, so they
        overwrite each other per sensor — last write wins.
        """
        with self.state_lock:
            r = self.readings.get(slave)
            if r is not None and r.online and r.absolute_position is not None:
                self.zero_offsets[slave] = r.absolute_position
                ok = True
            else:
                ok = False
        self.message_queue.append(
            f"Zeroed encoder {slave}" if ok
            else f"[WARN] cannot zero encoder {slave} (offline/unknown)"
        )

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

    def _counts_to_mm(self, counts: Optional[int]) -> Optional[float]:
        """Convert encoder counts to thread displacement in mm.

        The magnet rides a drum of diameter `drum_diameter_mm`; one full turn
        pays out one circumference of thread, so displacement is the arc length
        counts / counts_per_turn * pi * diameter.
        """
        if counts is None:
            return None
        return counts / self.counts_per_turn * math.pi * self.drum_diameter_mm

    def get_state(self) -> dict:
        """Snapshot of encoder state for serialization to the browser.

        The POSITION field is thread displacement in mm (relative to the last
        zero); every other field keeps its original raw value.
        """
        with self.state_lock:
            zero_offsets = dict(self.zero_offsets)
            encoders = []
            for slave in sorted(self.readings):
                r = self.readings[slave]
                offset = zero_offsets.get(slave, 0)
                display = (None if r.absolute_position is None
                           else r.absolute_position - offset)
                position_mm = self._counts_to_mm(display)
                encoders.append({
                    "slave": r.slave,
                    "online": r.online,
                    "position_mm": (None if position_mm is None
                                    else round(position_mm, 2)),
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
            "drum_diameter_mm": self.drum_diameter_mm,
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
