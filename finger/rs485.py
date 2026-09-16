#!/usr/bin/env python3
"""
finger/rs485.py

Minimal synchronous RS-485 access to the finger's GJW draw-wire encoders,
shared by the timing scan and the calibration tool. One Modbus read per call,
no background thread, full-precision counts and a monotonic timestamp.

Only the transport lives here; framing and decoding come from modbus_rtu.
"""

from __future__ import annotations

import os
import sys
import time
from dataclasses import dataclass
from typing import Dict, Iterable, Optional

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import serial  # noqa: E402

from modbus_rtu import (  # noqa: E402
    GJW_STATE_COUNT,
    GJW_STATE_REGISTER,
    READ_FUNCTIONS,
    build_read_request,
    compute_absolute_position,
    decode_gjw_state_registers,
    parse_read_registers_response,
)

DEFAULT_TIMEOUT = 0.06


@dataclass
class Reading:
    slave: int
    outcome: str                      # "ok" | "timeout" | "error"
    counts: Optional[int] = None      # absolute multi-turn counts
    status: Optional[int] = None
    velocity: Optional[int] = None
    error_count: Optional[int] = None
    t: float = 0.0                    # time.monotonic() after the read

    @property
    def ok(self) -> bool:
        return self.outcome == "ok"


def open_port(port: str, baud: int = 115200, timeout: float = DEFAULT_TIMEOUT) -> serial.Serial:
    return serial.Serial(port=port, baudrate=baud, bytesize=8, parity="N",
                         stopbits=1, timeout=timeout)


def read_sensor(ser: serial.Serial, slave: int, counts_per_turn: int) -> Reading:
    """One GJW state read. Never raises on bus problems; see Reading.outcome."""
    request = build_read_request(slave, READ_FUNCTIONS["holding"],
                                 GJW_STATE_REGISTER, GJW_STATE_COUNT)
    ser.reset_input_buffer()
    ser.write(request)
    header = ser.read(3)
    if len(header) != 3:
        return Reading(slave, "timeout", t=time.monotonic())
    body = ser.read(2 if header[1] & 0x80 else header[2] + 2)
    try:
        regs = parse_read_registers_response(header + body, slave,
                                             READ_FUNCTIONS["holding"], GJW_STATE_COUNT)
        state = decode_gjw_state_registers(regs)
    except Exception:
        return Reading(slave, "error", t=time.monotonic())
    return Reading(
        slave, "ok",
        counts=compute_absolute_position(state["single_turn_position"],
                                         state["turns"], counts_per_turn),
        status=state["status_code"],
        velocity=state["angular_velocity"],
        error_count=state["error_count"],
        t=time.monotonic(),
    )


class Bus:
    """Thin wrapper: one open port, sequential reads of a fixed sensor set."""

    def __init__(self, ser: serial.Serial, counts_per_turn: int):
        self.ser = ser
        self.counts_per_turn = counts_per_turn

    def read(self, slave: int) -> Reading:
        return read_sensor(self.ser, slave, self.counts_per_turn)

    def read_all(self, slaves: Iterable[int]) -> Dict[int, Reading]:
        return {s: self.read(s) for s in slaves}

    def close(self) -> None:
        self.ser.close()
