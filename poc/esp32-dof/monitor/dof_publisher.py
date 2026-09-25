#!/usr/bin/env python3
"""Localhost UDP publisher: monitor -> Webots follower (contract: README "Webots IPC contract", #232).

Datagram: JSON {"seq": int, "roll": float, "pitch": float, "yaw": float}, angles in RADIANS.
UDP here is localhost IPC to Webots only; the ESP32 transport stays serial.
publish() never raises: OSError (Webots not running, buffer full, closed socket) is swallowed
and counted; non-finite angles or a non-int seq are refused (counted in `skipped`).
Counters: sent (successful sendto only), errors (swallowed OSError), skipped (refused).
Use an IPv4 literal for `host` (a hostname would be resolved on every send).
"""
import json
import math
import socket

_TIMEOUT_S = 0.05


def _is_real(v) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool)


class OrientationPublisher:
    def __init__(self, host: str = "127.0.0.1", port: int = 5005):
        if not isinstance(host, str) or not host:
            raise ValueError(f"host must be a non-empty str, got {host!r}")
        if isinstance(port, bool) or not isinstance(port, int) or not (1 <= port <= 65535):
            raise ValueError(f"port must be int 1..65535, got {port!r}")
        self._addr = (host, port)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)   # OSError propagates
        self._sock.settimeout(_TIMEOUT_S)
        self.sent = 0
        self.errors = 0
        self.skipped = 0

    def publish(self, seq: int, roll: float, pitch: float, yaw: float) -> None:
        if self._sock is None:                       # closed: silent no-op
            return
        if isinstance(seq, bool) or not isinstance(seq, int) \
                or not all(_is_real(a) for a in (roll, pitch, yaw)):
            self.skipped += 1
            return
        try:
            r, p, y = float(roll), float(pitch), float(yaw)
        except (OverflowError, ValueError):
            self.skipped += 1
            return
        if not all(math.isfinite(a) for a in (r, p, y)):
            self.skipped += 1
            return
        data = json.dumps({"seq": seq, "roll": r, "pitch": p, "yaw": y}).encode()
        try:
            self._sock.sendto(data, self._addr)
        except OSError:
            self.errors += 1
            return
        self.sent += 1

    def close(self) -> None:
        sock, self._sock = self._sock, None
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass
