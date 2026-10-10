#!/usr/bin/env python3
"""
imu_check.py

Bring-up check of the Giga's BNO055 stream. It sends only the IMU commands
(i, i1, i0) and never a valve command. Stop app.py and close the Arduino
Serial Monitor first: on Windows a COM port can be open in only one program.

Prints one 'i' reading, runs the 'i1' stream for --seconds and reports the
rate (from the Giga's own timestamps), the frame spacing, missing frames (seq
gaps), the calibration status and the quaternion norm, then stops it with
'i0', whose reply carries the Giga's sent/dropped/read-error counts. The run
fails (exit 1) unless the rate is inside 40-50 Hz with no frame missing.

--stall S then stops reading for S seconds with the stream still on, as a
paused script or a sleeping PC would. Touch the screen during the stall: it
must keep responding, because the Giga drops frames instead of waiting for
the PC. Frames missing around the stall are expected and not counted.

    python imu_check.py
    python imu_check.py --seconds 60
    python imu_check.py --stall 10
"""

from __future__ import annotations

import argparse
import math
import statistics
import sys
import time
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import serial

from valve_controller import ImuFrame, find_giga_port, parse_imu_line

RATE_BAND = (40.0, 50.0)


@dataclass
class Summary:
    frames: int
    rate_hz: float               # from the Giga's clock, not the PC's
    dt_min_ms: float
    dt_mean_ms: float
    dt_max_ms: float
    missing: int                 # seq gaps
    pc_gap_max_ms: float         # longest wait between frames as the PC saw them
    quat_norm: float             # mean |q|: ~1 while fusion runs, 0 if it does not
    calib: Tuple[int, int, int, int]


def summarize(samples: Sequence[Tuple[float, ImuFrame]]) -> Summary:
    """samples: (PC receive time, frame), in arrival order."""
    frames = [f for _, f in samples]
    dts = [(b.giga_ms - a.giga_ms) % 2**32 for a, b in zip(frames, frames[1:])]
    missing = sum((b.seq - a.seq) % 2**16 - 1 for a, b in zip(frames, frames[1:]))
    pc = [t for t, _ in samples]
    pc_gaps = [(b - a) * 1000 for a, b in zip(pc, pc[1:])]
    span = sum(dts)
    steps = sum((b.seq - a.seq) % 2**16 for a, b in zip(frames, frames[1:]))
    return Summary(
        frames=len(frames),
        rate_hz=steps * 1000.0 / span if span else 0.0,
        dt_min_ms=min(dts, default=0.0), dt_mean_ms=statistics.fmean(dts) if dts else 0.0,
        dt_max_ms=max(dts, default=0.0),
        missing=missing,
        pc_gap_max_ms=max(pc_gaps, default=0.0),
        quat_norm=statistics.fmean(math.sqrt(sum(q * q for q in f.quat)) for f in frames)
        if frames else 0.0,
        calib=frames[-1].calib if frames else (0, 0, 0, 0))


def problems(s: Summary, count_missing: bool = True) -> List[str]:
    if s.frames < 2:
        return [f"only {s.frames} frame(s) received"]
    out = []
    lo, hi = RATE_BAND
    if not lo <= s.rate_hz <= hi:
        out.append(f"rate {s.rate_hz:.1f} Hz outside {lo:g}-{hi:g} Hz")
    if count_missing and s.missing:
        out.append(f"{s.missing} frame(s) missing (seq gaps)")
    if not 0.9 <= s.quat_norm <= 1.1:
        out.append(f"|q| = {s.quat_norm:.2f}: fusion not running (NDOF)?")
    return out


def report(s: Summary) -> str:
    sy, g, a, m = s.calib
    return (f"frames {s.frames}, rate {s.rate_hz:.2f} Hz, spacing {s.dt_min_ms:.0f}/"
            f"{s.dt_mean_ms:.1f}/{s.dt_max_ms:.0f} ms (min/mean/max), missing {s.missing}, "
            f"longest PC wait {s.pc_gap_max_ms:.0f} ms, |q| {s.quat_norm:.3f}, "
            f"cal S{sy}G{g}A{a}M{m}")


# ----------------------------------------------------------------------------
# Serial
# ----------------------------------------------------------------------------

def read_for(ser, seconds: float, samples: list, until=None) -> Optional[str]:
    """Read lines for `seconds`, collecting frames; returns the first non-frame
    line starting with `until` (a prefix or a tuple of them) and stops there."""
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        line = ser.readline().decode("utf-8", errors="ignore").strip()
        if not line:
            continue
        frame = parse_imu_line(line)
        if frame is not None:
            samples.append((time.time(), frame))
        elif until is not None and line.startswith(until):
            return line
    return None


def command(ser, cmd: str, reply, samples: list, timeout: float = 2.0) -> Optional[str]:
    ser.write(f"{cmd}\n".encode())
    return read_for(ser, timeout, samples, until=reply)


def main(argv: Optional[Sequence[str]] = None) -> int:
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(errors="replace")

    ap = argparse.ArgumentParser(description="Check the Giga's BNO055 stream (no valve I/O).")
    ap.add_argument("--port", default=None, help="Giga port (default: found by USB VID 2341)")
    ap.add_argument("--seconds", type=float, default=10.0, help="stream time (default 10)")
    ap.add_argument("--stall", type=float, default=0.0,
                    help="then stop reading for this long with the stream on (s)")
    args = ap.parse_args(argv)

    port = args.port or find_giga_port()
    if port is None:
        print("[ERR] Giga not found; pass --port")
        return 1
    try:
        ser = serial.Serial(port, 115200, timeout=0.1)
    except serial.SerialException as exc:
        print(f"[ERR] {port}: {exc}\n      (app.py or the Serial Monitor holding the port?)")
        return 1

    samples: list = []
    with ser:
        time.sleep(0.5)
        ser.reset_input_buffer()

        line = command(ser, "i", ("IMU", "ERROR"), samples)
        print(f"i  -> {line or '(no reply)'}")
        if line is None or line.startswith("ERROR"):
            print("[ERR] the Giga does not know 'i': is the new vc2 flashed?")
            return 1
        if line.startswith("IMU ERROR"):
            return 1

        line = command(ser, "i1", "IMU", samples)
        print(f"i1 -> {line or '(no reply)'}")
        if line is None or not line.startswith("IMU: stream on"):
            return 1
        samples.clear()
        read_for(ser, args.seconds, samples)
        s = summarize(samples)

        if args.stall > 0:
            print(f"Not reading for {args.stall:g} s, stream still on: touch the screen now, "
                  f"it must keep responding.")
            time.sleep(args.stall)
            ser.reset_input_buffer()            # what piled up in the driver meanwhile
            after: list = []
            read_for(ser, 2.0, after)
            print(f"after the stall: {len(after)} frame(s) in 2 s"
                  f"{'' if len(after) >= 70 else '  [ERR] stream did not recover'}")

        line = command(ser, "i0", "IMU: stream off", samples)
        print(f"i0 -> {line or '(no reply)'}")

    print(report(s))
    found = problems(s)
    for p in found:
        print(f"[ERR] {p}")
    if not found:
        print(f"PASS: {RATE_BAND[0]:g}-{RATE_BAND[1]:g} Hz, no frame missing")
    return 1 if found else 0


if __name__ == "__main__":
    sys.exit(main())
