#!/usr/bin/env python3
"""
sensor_hub.py

Reads the station's draw-wire sensors on several RS-485 ports at once: one
EncoderController, and so one poll thread, per EKU081 channel, merged into a
single table keyed by sensor id. Sensor ids are unique across the station, so
the hub only learns which port each id answers on. An id that answers on two
ports is reported as a duplicate, and the first port's reading is shown.
"""

from __future__ import annotations

from collections import deque
from typing import Callable, Dict, List, Optional, Sequence

from encoder_controller import EncoderController


def _default_factory(**encoder_kwargs) -> Callable:
    def make(port: str, slave_ids: Sequence[int]) -> EncoderController:
        return EncoderController(port=port, slave_ids=slave_ids, **encoder_kwargs)
    return make


class SensorHub:
    """Several EncoderControllers behind the single-controller interface."""

    def __init__(self, ports: Sequence[str], slave_ids: Sequence[int],
                 factory: Optional[Callable] = None, **encoder_kwargs):
        make = factory or _default_factory(**encoder_kwargs)
        self.message_queue = deque(maxlen=50)
        self.controllers: Dict[str, EncoderController] = {}
        for port in ports:
            try:
                self.controllers[port] = make(port, list(slave_ids))
            except Exception as exc:
                # One bad channel must not keep the others from running.
                self.message_queue.append(f"[ERR] {port}: {exc}")
        self._reported_duplicates: Dict[int, List[str]] = {}

    @property
    def connected(self) -> bool:
        return any(c.connected for c in self.controllers.values())

    @property
    def ports(self) -> List[str]:
        return list(self.controllers)

    def owners(self) -> Dict[int, List[str]]:
        """Sensor id -> the ports it answered on, in port order."""
        out: Dict[int, List[str]] = {}
        for port, c in self.controllers.items():
            for slave in list(c.active_slaves):
                out.setdefault(slave, []).append(port)
        return out

    def add_listener(self, fn: Callable) -> None:
        """Call fn(EncoderReading) for every reading polled on any port."""
        for c in self.controllers.values():
            c.listeners.append(fn)

    def duplicates(self) -> Dict[int, List[str]]:
        return {s: ps for s, ps in self.owners().items() if len(ps) > 1}

    def get_state(self) -> dict:
        """Merged snapshot: one entry per sensor id (tagged with its port),
        plus a per-port summary."""
        encoders: Dict[int, dict] = {}
        ports = []
        for port, c in self.controllers.items():
            st = c.get_state()
            ports.append({"port": port, "connected": st["connected"],
                          "found": len(c.active_slaves), "online": st["online"]})
            for e in st["encoders"]:
                encoders.setdefault(e["slave"], dict(e, port=port))
        merged = [encoders[s] for s in sorted(encoders)]
        return {
            "connected": self.connected,
            "ports": ports,
            "encoders": merged,
            "online": sum(1 for e in merged if e["online"]),
            "zeroed": sum(1 for e in merged if e["zeroed"]),
            "duplicates": sorted(self.duplicates()),
        }

    # ------------------------------------------------------------------
    # Web-driven actions
    # ------------------------------------------------------------------

    def zero(self, slave: int) -> None:
        ports = self.owners().get(slave)
        if not ports:
            self.message_queue.append(f"[WARN] cannot zero sensor {slave}: not found on any port")
            return
        self.controllers[ports[0]].zero(slave)

    def zero_all(self) -> None:
        for c in self.controllers.values():
            c.zero_all()

    def clear_zero(self) -> None:
        for c in self.controllers.values():
            c.clear_zero()

    def request_rescan(self, slave_ids: Optional[Sequence[int]] = None) -> None:
        for c in self.controllers.values():
            c.request_rescan(slave_ids)

    def drain_messages(self) -> List[str]:
        """Hub messages, then each port's messages prefixed with the port, then
        a warning for each duplicate id not reported before."""
        msgs = list(self.message_queue)
        self.message_queue.clear()
        for port, c in self.controllers.items():
            msgs += [f"{port}: {m}" for m in c.drain_messages()]
        dups = self.duplicates()
        for slave, ports in sorted(dups.items()):
            if self._reported_duplicates.get(slave) != ports:
                msgs.append(f"[WARN] sensor {slave} answers on {' and '.join(ports)}; "
                            f"showing {ports[0]}")
        self._reported_duplicates = dups
        return msgs

    def close(self) -> None:
        for c in self.controllers.values():
            c.close()
