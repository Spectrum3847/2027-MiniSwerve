"""Pulls a window of a real FM match into a CSV the sim can replay and compare against.

    python tools/fm_extract.py <log.wpilog> <out.csv> [--start S] [--seconds N]

Columns, on a 100 Hz grid from the first enabled moment (+ --start seconds):
  t, tv0..tv3, ta0..ta3   commanded wheel speed (m/s) and angle (rad), /Swerve/ModuleTargets
  mv0..mv3, ma0..ma3      measured wheel speed and angle, /Swerve/ModuleVelocities
  ax, ay                  Pigeon acceleration, robot frame (g, gravity leaks in on tilt)
  i0..i3                  drive stator current (A), /RealOutputs/Swerve/Currents/<module>
  vbat                    battery voltage
Module order FL, FR, BL, BR.
"""

import argparse
import struct
import sys

from wpilog_read import at, read_wpilog, series

MODULES = ["FrontLeft", "FrontRight", "BackLeft", "BackRight"]


def module_states(records, key):
    """SwerveModuleVelocity[4] structs: (velocity, angle) pairs."""
    for prefix in ("/", "/RealOutputs/", ""):
        name = prefix + key
        if name in records:
            out = []
            for t, p in records[name]:
                if len(p) >= 64:
                    out.append((t, struct.unpack_from("<8d", p, 0)))
            return out
    return []


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log")
    ap.add_argument("out")
    ap.add_argument("--start", type=float, default=0.0)
    ap.add_argument("--seconds", type=float, default=20.0)
    a = ap.parse_args()
    r, t = read_wpilog(a.log)
    enabled = series(r, t, "DriverStation/Enabled")
    t_on = next((tm for tm, v in enabled if v), None)
    if t_on is None:
        print("never enabled")
        return 1
    t0 = t_on + a.start
    targets = module_states(r, "Swerve/ModuleTargets")
    measured = module_states(r, "Swerve/ModuleVelocities")
    ax = series(r, t, "Swerve/AccelXG")
    ay = series(r, t, "Swerve/AccelYG")
    cur = [series(r, t, "Swerve/Currents/%s/DriveStatorCurrent" % m) for m in MODULES]
    vbat = series(r, t, "Runtime/BatteryVoltage") or series(r, t, "SystemStats/BatteryVoltage")
    if not targets or not measured:
        print("missing ModuleTargets/ModuleVelocities")
        return 1
    rows = 0
    with open(a.out, "w", encoding="utf-8", newline="\n") as f:
        cols = ["t"] + ["tv%d" % i for i in range(4)] + ["ta%d" % i for i in range(4)]
        cols += ["mv%d" % i for i in range(4)] + ["ma%d" % i for i in range(4)]
        cols += ["ax", "ay"] + ["i%d" % i for i in range(4)] + ["vbat"]
        f.write(",".join(cols) + "\n")
        k = 0
        while k * 0.01 < a.seconds:
            tm = t0 + k * 0.01
            tg = at(targets, tm)
            ms = at(measured, tm)
            if tg is None or ms is None:
                k += 1
                continue
            row = [round(k * 0.01, 3)]
            row += [tg[2 * i] for i in range(4)] + [tg[2 * i + 1] for i in range(4)]
            row += [ms[2 * i] for i in range(4)] + [ms[2 * i + 1] for i in range(4)]
            row += [at(ax, tm) or 0.0, at(ay, tm) or 0.0]
            row += [at(c, tm) or 0.0 for c in cur]
            row += [at(vbat, tm) or 12.0]
            f.write(",".join("%.5g" % x for x in row) + "\n")
            rows += 1
            k += 1
    print("wrote %d rows from t=%.2f (enable %.2f + %.1f s)" % (rows, t0, t_on, a.start))
    return 0


if __name__ == "__main__":
    sys.exit(main())
