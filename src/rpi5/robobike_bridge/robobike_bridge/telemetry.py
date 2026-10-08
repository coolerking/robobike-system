"""ROS-independent parsing and timing helpers for ROBOBIKE GET /get_acc."""

import dataclasses
import math
from collections import deque

CONTROL_RATE_HZ = 250
SAMPLE_PERIOD_MS = 1000 // CONTROL_RATE_HZ
GAP_THRESHOLD_MS = 1.5 * SAMPLE_PERIOD_MS
COLUMNS_LEGACY = 8
COLUMNS_CURRENT = 13
RECORD_TYPE = "a"
# Column order of the current firmware (v1033+); the legacy format omits SWP_SG, SV_POS and ACC_*.
CURRENT_FIELDS = (
    "sv_drv", "sv_str", "sv_std", "swp_sg", "sv_pos", "gy_roll", "gy_yaw", "gy_pitch", "acc_x", "acc_y", "acc_z",
)
LEGACY_FIELDS = ("sv_drv", "sv_str", "sv_std", "gy_roll", "gy_yaw", "gy_pitch")


@dataclasses.dataclass
class Sample:
    record_type: str
    time_ms: int
    sv_drv: float
    sv_str: float
    sv_std: float
    gy_roll: float
    gy_yaw: float
    gy_pitch: float
    swp_sg: float = math.nan
    sv_pos: float = math.nan
    acc_x: float = math.nan
    acc_y: float = math.nan
    acc_z: float = math.nan
    columns: int = COLUMNS_CURRENT


def parse_line(line):
    """Parse one CSV row; raise ValueError for anything that is not a monitor record."""
    fields = [field.strip() for field in line.strip().split(",")]
    if len(fields) == COLUMNS_CURRENT:
        names = CURRENT_FIELDS
    elif len(fields) == COLUMNS_LEGACY:
        names = LEGACY_FIELDS
    else:
        raise ValueError(f"unexpected column count {len(fields)}")
    if fields[0] != RECORD_TYPE:
        raise ValueError(f"unexpected record type {fields[0]!r}")
    time_ms = int(fields[1])
    if time_ms < 0:
        raise ValueError("negative TIME_MS")
    values = {name: float(value) for name, value in zip(names, fields[2:])}
    return Sample(record_type=fields[0], time_ms=time_ms, columns=len(fields), **values)


def parse_body(text):
    """Return (samples, malformed_count) for a /get_acc response body; blank lines are ignored."""
    samples, malformed = [], 0
    for line in text.split("\n"):
        if not line.strip():
            continue
        try:
            samples.append(parse_line(line))
        except ValueError:
            malformed += 1
    return samples, malformed


class ClockMapper:
    """Map ROBOBIKE TIME_MS to local time using the smallest recent (receive time - TIME_MS) offset."""

    def __init__(self, window_ns=10_000_000_000):
        self.window_ns = window_ns
        self.offsets = deque()
        self.last_time_ms = None
        self.rewinds = 0

    def reset(self):
        self.offsets.clear()
        self.last_time_ms = None

    def observe(self, recv_ns, time_ms):
        """Record the newest sample of a response; return True if TIME_MS went backwards."""
        rewound = self.last_time_ms is not None and time_ms < self.last_time_ms
        if rewound:
            self.reset()
            self.rewinds += 1
        self.last_time_ms = time_ms
        self.offsets.append((recv_ns, recv_ns - time_ms * 1_000_000))
        while self.offsets and recv_ns - self.offsets[0][0] > self.window_ns:
            self.offsets.popleft()
        return rewound

    def offset_ns(self):
        return min(offset for _, offset in self.offsets) if self.offsets else None

    def stamp_ns(self, time_ms):
        offset = self.offset_ns()
        return None if offset is None else time_ms * 1_000_000 + offset


class GapDetector:
    """Estimate samples lost between consecutive TIME_MS values (ring-buffer overflow, link loss)."""

    def __init__(self):
        self.last_time_ms = None
        self.missing = 0

    def feed(self, time_ms):
        last, self.last_time_ms = self.last_time_ms, time_ms
        if last is None or time_ms < last:
            return 0
        delta = time_ms - last
        if delta <= GAP_THRESHOLD_MS:
            return 0
        lost = max(0, round(delta / SAMPLE_PERIOD_MS) - 1)
        self.missing += lost
        return lost
