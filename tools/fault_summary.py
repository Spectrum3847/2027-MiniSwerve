"""Summarises a fault-scenario sim log (MINISWERVE_SIM_SCRIPT=faults): per script step, which
devices were faulted, each module's mode, how far the true robot moved (it must keep driving), and
odometry error. Ends with whether every module recovered to NORMAL.

    python tools/fault_summary.py logs/<log>.wpilog
"""

import math
import sys

from wpilog_read import at, read_wpilog, series


def main(path):
    r, t = read_wpilog(path)
    steps = series(r, t, "Sim/ScriptStep")
    faults = series(r, t, "Sim/Faults")
    modes = series(r, t, "Swerve/Modules/Mode")
    truth = series(r, t, "Sim/TruthPose")
    odo = series(r, t, "Swerve/Odometry/Pose")
    state = series(r, t, "Swerve/State/Current")
    end = truth[-1][0] if truth else 0
    ok = True
    print("%-38s %-24s %-40s %7s %8s" % ("step", "faults", "modes (mid-step)", "moved", "odo err"))
    for k, (t0, name) in enumerate(steps):
        t1 = steps[k + 1][0] if k + 1 < len(steps) else end
        mid = (t0 + t1) / 2
        a, b = at(truth, t0 + 0.05), at(truth, t1 - 0.01)
        moved = math.hypot(b[0] - a[0], b[1] - a[1]) + abs(b[2] - a[2]) * 0.2 if a and b else 0
        p, q = at(odo, t1 - 0.01), at(truth, t1 - 0.01)
        err = math.hypot(p[0] - q[0], p[1] - q[1]) if p and q else float("nan")
        f = at(faults, mid) or []
        m = at(modes, mid) or []
        print("%-38s %-24s %-40s %6.2fm %7.3fm" % (name[:38], ",".join(f)[:24], ",".join(m), moved, err))
        if at(state, mid) not in (None, "TELEOP_DRIVE", "IDLE"):
            ok = False
    final = modes[-1][1] if modes else []
    resets = series(r, t, "Swerve/Health/DeviceResets")
    print("final modes:", final, " device resets seen:", resets[-1][1] if resets else 0)
    recovered = all(m == "NORMAL" for m in final)
    print("RECOVERED" if recovered else "NOT RECOVERED")
    return 0 if recovered and ok else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1]))
