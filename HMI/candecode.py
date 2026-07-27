"""
candump log decoder for RemoteRudder.

Emits a time-ordered list of Observation records. Each observation is one
scalar (or small tuple) measurement with the source address preserved, so
that duplicate sensors (two GPS units) stay distinguishable downstream.

Format handled:  (timestamp) iface ID#HEXDATA
Timestamps may be absolute epoch or relative to zero; we normalise to
seconds-since-first-message.

NMEA2000 "not available" sentinels are 0xFFFF / 0xFFFE (and 0xFF for bytes).
These are rejected here rather than being passed to the filter as real data --
this is the single most important thing this module does.
"""

import math
import struct
from dataclasses import dataclass, field

# ---------------------------------------------------------------------------
# PGN constants (extracted from the actual capture files, not from spec guesses)
# ---------------------------------------------------------------------------
PGN_VESSEL_HEADING = 0x1F112   # 127250, magnetic/true heading + deviation/variation
PGN_COG_SOG        = 0x1F802   # 129026, course & speed over ground, rapid update
PGN_POSITION_RAPID = 0x1F801   # 129025, lat/lon rapid update
PGN_RATE_OF_TURN   = 0x1F113   # 127251
PGN_ATTITUDE       = 0x1F119   # 127257, yaw/pitch/roll
PGN_VEHICLE_DIR    = 0x0FEE8   # 65256, J1939 vehicle direction/speed

# N2K angular resolution: 1e-4 rad/bit, unsigned 16-bit
RAD_PER_BIT = 1e-4
# N2K speed resolution: 0.01 m/s per bit
MPS_PER_BIT = 0.01
MPS_TO_MPH = 2.2369362920544

# Sentinels meaning "data not available" / "out of range"
U16_INVALID = (0xFFFF, 0xFFFE)
I32_INVALID = (0x7FFFFFFF, -0x80000000)


def _u16(b, off):
    return struct.unpack_from("<H", b, off)[0]


def _i16(b, off):
    return struct.unpack_from("<h", b, off)[0]


def _i32(b, off):
    return struct.unpack_from("<i", b, off)[0]


def wrap_pi(a):
    """Wrap angle to (-pi, pi]."""
    a = (a + math.pi) % (2.0 * math.pi) - math.pi
    if a <= -math.pi:
        a += 2.0 * math.pi
    return a


def wrap_2pi(a):
    """Wrap angle to [0, 2pi)."""
    return a % (2.0 * math.pi)


@dataclass
class Observation:
    """One decoded measurement.

    kind:  'heading' | 'cog' | 'sog' | 'position' | 'rot' | 'attitude'
    src:   J1939 source address -- distinguishes duplicate sensors
    value: primary scalar, in SI (radians / m/s / degrees for lat-lon)
    extra: dict of secondary fields (e.g. sog alongside cog)
    """
    t: float
    kind: str
    src: int
    value: float
    extra: dict = field(default_factory=dict)


def parse_can_id(cid):
    """Return (priority, pgn, source_address) from a 29-bit J1939 ID."""
    pf = (cid >> 16) & 0xFF
    pgn = (cid >> 8) & 0x3FFFF
    if pf < 240:                 # PDU1: destination-specific, mask off PS
        pgn &= 0x3FF00
    return (cid >> 26) & 0x7, pgn, cid & 0xFF


def decode_frame(t, cid, data):
    """Decode one CAN frame into zero or more Observations."""
    _, pgn, src = parse_can_id(cid)
    out = []

    if pgn == PGN_VESSEL_HEADING and len(data) >= 8:
        raw = _u16(data, 1)
        if raw not in U16_INVALID:
            heading = raw * RAD_PER_BIT
            # Bytes 3-4 deviation, 5-6 variation, byte 7 low 2 bits = reference
            # reference: 0 = true, 1 = magnetic
            ref = data[7] & 0x03
            dev_raw = _u16(data, 3)
            var_raw = _u16(data, 5)
            extra = {
                "reference": "true" if ref == 0 else "magnetic",
                "deviation": None if dev_raw in U16_INVALID else _i16(data, 3) * RAD_PER_BIT,
                "variation": None if var_raw in U16_INVALID else _i16(data, 5) * RAD_PER_BIT,
            }
            out.append(Observation(t, "heading", src, wrap_2pi(heading), extra))

    elif pgn == PGN_COG_SOG and len(data) >= 6:
        cog_raw = _u16(data, 2)
        sog_raw = _u16(data, 4)
        ref = data[1] & 0x03      # 0 = true, 1 = magnetic
        sog = None
        if sog_raw not in U16_INVALID:
            sog = sog_raw * MPS_PER_BIT
            out.append(Observation(t, "sog", src, sog))
        if cog_raw not in U16_INVALID:
            out.append(Observation(
                t, "cog", src, wrap_2pi(cog_raw * RAD_PER_BIT),
                {"sog": sog, "reference": "true" if ref == 0 else "magnetic"},
            ))

    elif pgn == PGN_POSITION_RAPID and len(data) >= 8:
        lat_raw = _i32(data, 0)
        lon_raw = _i32(data, 4)
        if lat_raw not in I32_INVALID and lon_raw not in I32_INVALID:
            out.append(Observation(
                t, "position", src, lat_raw * 1e-7,
                {"lon": lon_raw * 1e-7},
            ))

    elif pgn == PGN_RATE_OF_TURN and len(data) >= 5:
        rot_raw = _i32(data, 1)
        if rot_raw not in I32_INVALID:
            # N2K 127251 resolution is 3.125e-8 rad/s per bit
            out.append(Observation(t, "rot", src, rot_raw * 3.125e-8))

    elif pgn == PGN_ATTITUDE and len(data) >= 7:
        yaw_raw = _i16(data, 1)
        if yaw_raw not in (0x7FFF, -0x8000):
            out.append(Observation(t, "attitude", src, yaw_raw * RAD_PER_BIT))

    elif pgn == PGN_VEHICLE_DIR and len(data) >= 4:
        d_raw = _u16(data, 0)
        if d_raw not in U16_INVALID:
            direction = math.radians(d_raw * 0.0078125)
            speed = _u16(data, 2) * 0.00390625 / 3.6   # km/h -> m/s
            out.append(Observation(t, "vehicle_dir", src,
                                   wrap_2pi(direction), {"speed": speed}))

    return out


def load_log(path, limit=None):
    """Parse a candump log into a time-sorted list of Observations."""
    obs = []
    t0 = None
    with open(path, "r", errors="replace") as fh:
        for n, line in enumerate(fh):
            if limit and n >= limit:
                break
            parts = line.split()
            if len(parts) < 3 or "#" not in parts[2]:
                continue
            try:
                t = float(parts[0].strip("()"))
                cid_s, data_s = parts[2].split("#", 1)
                cid = int(cid_s, 16)
                data = bytes.fromhex(data_s.strip())
            except ValueError:
                continue
            if t0 is None:
                t0 = t
            obs.extend(decode_frame(t - t0, cid, data))
    obs.sort(key=lambda o: o.t)
    return obs


def summarise(obs):
    """Print per-(kind, src) counts, rates and ranges. Sanity check helper."""
    groups = {}
    for o in obs:
        groups.setdefault((o.kind, o.src), []).append(o)
    print(f"{'kind':14s} {'src':>4s} {'n':>7s} {'period':>8s} {'min':>9s} {'max':>9s}")
    for (kind, src), lst in sorted(groups.items()):
        span = lst[-1].t - lst[0].t
        period = span / max(len(lst) - 1, 1)
        vals = [o.value for o in lst]
        if kind in ("heading", "cog", "vehicle_dir", "attitude"):
            lo, hi = math.degrees(min(vals)), math.degrees(max(vals))
        else:
            lo, hi = min(vals), max(vals)
        print(f"{kind:14s} {src:4d} {len(lst):7d} {period*1000:7.1f}ms "
              f"{lo:9.3f} {hi:9.3f}")
