#!/usr/bin/env python3
"""Stream statistics for the ESP32 6-DOF IMU frame stream (pure, no I/O).

seq wraps at 65536 and t_us at 2**32, so every difference is modular.

Rules (documented, deterministic):
  d = (seq - prev_seq) % 65536
    d == 1            in order
    2 <= d <= 32768   forward gap: dropped += d - 1
    d == 0            duplicate            -> out_of_order += 1 (prev_seq unchanged)
    d  > 32768        backward jump by b = 65536 - d
        b <= RESET_BACK (50)  -> out_of_order += 1 (prev_seq unchanged; high-water mark)
        b >  RESET_BACK       -> device reset: resets += 1, stream rebased (no drops charged)
  For an in-order/forward frame, dt = (t_us - prev_t_us) & 0xFFFFFFFF; dt >= 2**31 means
  device time went backwards -> also a reset.
  Known limitation: a reset that restarts within 50 of the old seq shows up as up to 50
  out_of_order frames before the stream resumes.

received counts every parsed frame (including out-of-order and reset frames).
rate_hz: last <=50 accepted frames, (n-1) / (sum of adjacent wrapped dt in s); 0.0 if n < 2.
avg_rate_hz: accepted intervals / total accepted device time (NOT wall clock).
"""
from collections import deque
from typing import Optional

SEQ_MOD = 65536
SEQ_HALF = 32768
T_MASK = 0xFFFFFFFF
T_HALF = 2 ** 31
RESET_BACK = 50
WINDOW = 50


class StreamStats:
    def __init__(self, window: int = WINDOW):
        if isinstance(window, bool) or not isinstance(window, int) or window < 2:
            raise ValueError(f"window must be int >= 2, got {window!r}")
        self.received = 0
        self.dropped = 0
        self.out_of_order = 0
        self.resets = 0
        self.bad_frames = 0
        self.last_dt_us: Optional[int] = None   # dt of the most recent update, None if first/OOO/reset
        self._prev_seq: Optional[int] = None
        self._prev_t: int = 0
        self._elapsed_us = 0                    # unwrapped device time since last rebase
        self._window = deque(maxlen=window)     # unwrapped elapsed_us of accepted frames
        self._intervals = 0
        self._total_us = 0

    def note_bad(self) -> None:
        self.bad_frames += 1

    def _rebase(self, seq: int, t_us: int) -> None:
        self._prev_seq = seq
        self._prev_t = t_us
        self._elapsed_us = 0
        self._window.clear()
        self._window.append(0)

    def update(self, seq: int, t_us: int) -> bool:
        """Record one parsed frame. Returns True if it advances the stream (first frame,
        in-order, forward gap, or reset); False if it is out_of_order (dup/backward)."""
        self.received += 1
        self.last_dt_us = None
        if self._prev_seq is None:
            self._rebase(seq, t_us)
            return True
        d = (seq - self._prev_seq) % SEQ_MOD
        if d == 0 or d > SEQ_HALF:
            back = (self._prev_seq - seq) % SEQ_MOD      # 0 for duplicate
            if back <= RESET_BACK:
                self.out_of_order += 1
                return False
            self.resets += 1
            self._rebase(seq, t_us)
            return True
        dt = (t_us - self._prev_t) & T_MASK
        if dt >= T_HALF:                                 # device time went backwards
            self.resets += 1
            self._rebase(seq, t_us)
            return True
        self.dropped += d - 1
        self._prev_seq = seq
        self._prev_t = t_us
        self._elapsed_us += dt
        self._window.append(self._elapsed_us)
        self._intervals += 1
        self._total_us += dt
        self.last_dt_us = dt
        return True

    @property
    def rate_hz(self) -> float:
        n = len(self._window)
        if n < 2:
            return 0.0
        span = self._window[-1] - self._window[0]
        if span <= 0:
            return 0.0
        return (n - 1) * 1_000_000 / span

    @property
    def elapsed_s(self) -> float:
        return self._total_us / 1e6

    @property
    def avg_rate_hz(self) -> float:
        if self._total_us <= 0:
            return 0.0
        return self._intervals * 1_000_000 / self._total_us
