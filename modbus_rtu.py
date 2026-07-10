#!/usr/bin/env python3
"""
modbus_rtu.py

Pure Modbus-RTU + GJW-encoder protocol helpers, adapted from the
rs485-reader project (rs485_encoder_test.py). No hardware or pyserial
dependency lives here so the framing/decoding can be unit-tested on its own;
the serial transport is owned by encoder_controller.EncoderController.

The GJW absolute encoders speak Modbus RTU over RS-485. Their live state block
sits at holding register 0x0380 and packs single-turn position, multi-turn
count, status and speed into 16 consecutive registers.
"""

from __future__ import annotations

import struct
from typing import Dict, List, Optional, Sequence


READ_FUNCTIONS = {
    "holding": 0x03,
    "input": 0x04,
}

# GJW group-14 live state block.
GJW_STATE_REGISTER = 0x0380
GJW_STATE_COUNT = 16

MODBUS_EXCEPTION_CODES = {
    1: "illegal function",
    2: "illegal data address",
    3: "illegal data value",
    4: "server device failure",
    5: "acknowledge",
    6: "server device busy",
    8: "memory parity error",
    10: "gateway path unavailable",
    11: "gateway target failed to respond",
}


class ModbusProtocolError(Exception):
    """Raised when a Modbus RTU frame is malformed or reports an exception."""


class SerialTimeoutError(TimeoutError):
    """Raised when the serial device does not return a full Modbus response."""


def modbus_crc(data: bytes) -> int:
    """Return the Modbus RTU CRC-16 value for data."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def format_hex(data: bytes) -> str:
    return " ".join(f"{byte:02X}" for byte in data)


def _require_range(name: str, value: int, minimum: int, maximum: int) -> None:
    if not minimum <= value <= maximum:
        raise ValueError(f"{name} must be between {minimum} and {maximum}, got {value}")


def build_read_request(slave: int, function: int, address: int, count: int) -> bytes:
    """Build a Modbus RTU read holding/input registers request."""
    _require_range("slave", slave, 1, 247)
    if function not in READ_FUNCTIONS.values():
        raise ValueError("function must be 0x03/read holding or 0x04/read input")
    _require_range("address", address, 0, 0xFFFF)
    _require_range("count", count, 1, 125)

    payload = struct.pack(">BBHH", slave, function, address, count)
    return payload + struct.pack("<H", modbus_crc(payload))


def _validate_crc(frame: bytes) -> None:
    if len(frame) < 4:
        raise ModbusProtocolError(f"frame too short: {format_hex(frame)}")
    expected = modbus_crc(frame[:-2])
    actual = struct.unpack("<H", frame[-2:])[0]
    if actual != expected:
        raise ModbusProtocolError(
            f"bad CRC: expected {expected:04X}, got {actual:04X}; frame={format_hex(frame)}"
        )


def parse_read_registers_response(
    frame: bytes,
    slave: int,
    function: int,
    expected_count: Optional[int] = None,
) -> List[int]:
    """Validate and decode a Modbus RTU read registers response frame."""
    _validate_crc(frame)

    frame_slave = frame[0]
    frame_function = frame[1]
    if frame_slave != slave:
        raise ModbusProtocolError(f"wrong slave id: expected {slave}, got {frame_slave}")

    if frame_function & 0x80:
        if len(frame) != 5:
            raise ModbusProtocolError(f"malformed exception frame: {format_hex(frame)}")
        code = frame[2]
        text = MODBUS_EXCEPTION_CODES.get(code, "unknown exception")
        raise ModbusProtocolError(f"modbus exception {code}: {text}")

    if frame_function != function:
        raise ModbusProtocolError(
            f"wrong function: expected 0x{function:02X}, got 0x{frame_function:02X}"
        )

    byte_count = frame[2]
    expected_length = 3 + byte_count + 2
    if len(frame) != expected_length:
        raise ModbusProtocolError(
            f"wrong response length: expected {expected_length}, got {len(frame)}; "
            f"frame={format_hex(frame)}"
        )
    if byte_count % 2:
        raise ModbusProtocolError(f"odd byte count in register response: {byte_count}")

    registers = [
        struct.unpack(">H", frame[offset : offset + 2])[0]
        for offset in range(3, 3 + byte_count, 2)
    ]
    if expected_count is not None and len(registers) != expected_count:
        raise ModbusProtocolError(
            f"wrong register count: expected {expected_count}, got {len(registers)}"
        )
    return registers


def _signed16(value: int) -> int:
    return value - 0x10000 if value & 0x8000 else value


def _signed32(value: int) -> int:
    return value - 0x100000000 if value & 0x80000000 else value


def _u32_from_registers(high: int, low: int) -> int:
    return ((high & 0xFFFF) << 16) | (low & 0xFFFF)


def decode_gjw_state_registers(registers: Sequence[int]) -> Dict[str, int]:
    """Decode the GJW group 14 state block read from holding register 0x0380."""
    if len(registers) < 13:
        raise ValueError(f"GJW state block needs at least 13 registers, got {len(registers)}")

    turns_u32 = _u32_from_registers(registers[2], registers[3])
    return {
        "single_turn_position": _u32_from_registers(registers[0], registers[1]),
        "turns": _signed32(turns_u32),
        "status_code": registers[4],
        "angular_velocity": _signed16(registers[5]),
        "original_single_turn_position": _u32_from_registers(registers[8], registers[9]),
        "error_count": registers[12],
    }


def compute_absolute_position(
    single_turn_position: int,
    turns: int,
    counts_per_turn: int,
) -> int:
    """Absolute multi-turn position in raw encoder counts."""
    return turns * counts_per_turn + single_turn_position
