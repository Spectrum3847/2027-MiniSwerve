"""Reads AdvantageKit .wpilog files with no dependencies. Ported from 2026-FM-SystemCore.

The WPILOG format: https://github.com/wpilibsuite/allwpilib/blob/main/wpiutil/doc/datalog.adoc

    records, types = read_wpilog(path)
    for t, value in series(records, types, "Swerve/Odometry/Pose"): ...
"""

import collections
import math
import statistics
import struct
import sys


# types[TICKS_KEY]: /Timestamp's value units per second. Record times stay microseconds (the
# WPILOG format); AdvantageKit 27.0.0-alpha-7 and later write the values in nanoseconds.
TICKS_KEY = "#ticks"


def read_wpilog(path):
    with open(path, "rb") as f:
        data = f.read()
    if data[:6] != b"WPILOG":
        raise ValueError("not a wpilog")
    extra_len = struct.unpack_from("<I", data, 8)[0]
    pos = 12 + extra_len
    ticks = 1e6  # /Timestamp value units per second
    entries = {}  # id to (name, type)
    records = collections.defaultdict(list)  # name to list of (t, payload)
    n = len(data)
    while pos < n:
        hdr = data[pos]
        pos += 1
        id_len = (hdr & 0x3) + 1
        size_len = ((hdr >> 2) & 0x3) + 1
        ts_len = ((hdr >> 4) & 0x7) + 1
        eid = int.from_bytes(data[pos : pos + id_len], "little")
        pos += id_len
        size = int.from_bytes(data[pos : pos + size_len], "little")
        pos += size_len
        ts = int.from_bytes(data[pos : pos + ts_len], "little")
        pos += ts_len
        payload = data[pos : pos + size]
        pos += size
        if eid == 0:
            if payload and payload[0] == 0:
                (entry,) = struct.unpack_from("<I", payload, 1)
                o = 5
                (ln,) = struct.unpack_from("<I", payload, o)
                name = payload[o + 4 : o + 4 + ln].decode()
                o += 4 + ln
                (lt,) = struct.unpack_from("<I", payload, o)
                typ = payload[o + 4 : o + 4 + lt].decode()
                entries[entry] = (name, typ)
                if name == "/Timestamp" and b'"unit":"nanoseconds"' in payload:
                    ticks = 1e9  # AdvantageKit 27.0.0-alpha-7 and later
            continue
        if eid in entries:
            records[entries[eid][0]].append((ts / 1e6, payload))
    types = {name: typ for name, typ in entries.values()}
    types[TICKS_KEY] = ticks
    return records, types


def decode(typ, payload):
    try:
        return _decode(typ, payload)
    except (struct.error, IndexError, UnicodeDecodeError):
        return None  # a record cut off at the end of a log that was not closed cleanly


def _decode(typ, payload):
    if typ == "double":
        return struct.unpack("<d", payload)[0]
    if typ == "boolean":
        return payload[0] != 0
    if typ == "int64":
        return struct.unpack("<q", payload)[0]
    if typ == "string":
        return payload.decode(errors="replace")
    if typ == "boolean[]":
        return [b != 0 for b in payload]
    if typ == "double[]":
        return list(struct.unpack("<%dd" % (len(payload) // 8), payload))
    if typ == "int64[]":
        return list(struct.unpack("<%dq" % (len(payload) // 8), payload))
    if typ == "string[]":
        (count,) = struct.unpack_from("<I", payload, 0)
        out, o = [], 4
        for _ in range(count):
            (ln,) = struct.unpack_from("<I", payload, o)
            out.append(payload[o + 4 : o + 4 + ln].decode(errors="replace"))
            o += 4 + ln
        return out
    if typ == "struct:Pose2d":
        return struct.unpack("<3d", payload)
    if typ == "struct:Pose2d[]":
        return [struct.unpack_from("<3d", payload, i) for i in range(0, len(payload), 24)]
    if typ == "struct:Pose3d":
        return struct.unpack("<7d", payload)  # x, y, z, qw, qx, qy, qz
    if typ == "struct:Pose3d[]":
        return [struct.unpack_from("<7d", payload, i) for i in range(0, len(payload), 56)]
    return None




def series(records, types, key):
    """(time s, value) pairs for a key, looked up under outputs, replay outputs and inputs."""
    for prefix in ("/RealOutputs/", "/ReplayOutputs/", "/", ""):
        name = prefix + key
        if name in records:
            return [(t, decode(types[name], p)) for t, p in records[name]]
    return []


def at(points, t):
    """The value a key held at time t: the last record at or before t.

    The Spectrum AdvantageKit fork logs a value only when it changes, so keys must be aligned by
    time, never by index.
    """
    import bisect

    i = bisect.bisect_right([p[0] for p in points], t) - 1
    return points[i][1] if i >= 0 else None
