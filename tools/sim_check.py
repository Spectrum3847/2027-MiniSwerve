"""Checks a sim script's log against what the robot must do in it ("Finish the match" as a test).

    python tools/sim_check.py <script> logs/<log>.wpilog

Scripts: guards (our own code throwing, the swerve thread ending), driver (controller unplugged,
Driver Station lost), red (red alliance, never re-zeroed), faults (devices failing; uses
fault_summary.py). Exits 1 with the failed checks listed. tools/sim_check.sh runs them all.
"""

import math
import sys

import fault_summary
from wpilog_read import at, read_wpilog, series

# Per script: step name -> (least, most) the true robot may move in that step (m).
MOVES = {
    "guards": {
        "forward": (0.6, 9),
        "vision stage throws; forward": (0.8, 9),
        "localization stage throws too; forward": (0.8, 9),
        "both back; forward": (0.8, 9),
        "wanted-state stage throws: limp path, strafe": (0.8, 9),
        "wanted-state stage back": (0.8, 9),
        "swerve thread ends; forward": (0.8, 9),
        "commands stage throws; forward": (0.8, 9),
    },
    "driver": {
        "hands off: stick drift only": (0, 0.05),
        "forward": (1.5, 9),
        "controller unplugged, stick still pushed": (0, 1.0),
        "controller back: forward": (1.5, 9),
        "Driver Station lost, stick pushed": (0, 1.0),
        "Driver Station back: forward": (1.5, 9),
    },
    "red": {
        "forward (away from the red driver)": (1.5, 9),
    },
}

# Per script: counters that must end above zero (the script really did inject the fault).
MUST_COUNT = {
    "guards": [
        "Stages/Vision/Faults",
        "Stages/Localization/Faults",
        "Stages/SwerveApply/Faults",
        "Stages/Commands/Faults",
        "Swerve/Thread/Restarts",
    ],
}

# Counters that must stay at zero in every script: guards that should not fire on their own.
MUST_STAY_ZERO = ["Swerve/Thread/StaleRequestCycles", "Swerve/Thread/NonFiniteDropped"]


def last(r, t, key):
    points = series(r, t, key)
    return points[-1][1] if points else None


def main(script, path):
    if script == "faults":
        return fault_summary.main(path)
    if script not in MOVES:
        print("sim_check: no checks for script", script)
        return 1
    r, t = read_wpilog(path)
    steps = series(r, t, "Sim/ScriptStep")
    truth = series(r, t, "Sim/FieldPose")
    if not steps or not truth:
        print("sim_check: no Sim/ScriptStep or Sim/FieldPose in", path)
        return 1
    end = truth[-1][0]
    failed = []
    seen = set()
    for k, (t0, name) in enumerate(steps):
        t1 = steps[k + 1][0] if k + 1 < len(steps) else end
        a, b = at(truth, t0), at(truth, t1)
        moved = math.hypot(b[0] - a[0], b[1] - a[1])
        want = MOVES[script].get(name)
        seen.add(name)
        verdict = ""
        if want and not want[0] <= moved <= want[1]:
            verdict = "  FAILED: wanted %.2f to %.2f m" % want
            failed.append("%s: moved %.2f m" % (name, moved))
        print("%-48s %6.2f m%s" % (name[:48], moved, verdict))
        if script == "red" and want and b[0] >= a[0]:
            failed.append("%s: drove toward the red wall (x %.2f to %.2f)" % (name, a[0], b[0]))
    for name in MOVES[script]:
        if name not in seen:
            failed.append("step missing from the log (renamed?): " + name)
    for key in MUST_COUNT.get(script, []):
        value = last(r, t, key)
        print("%-48s %s" % (key, value))
        if not value:
            failed.append(key + " never counted: the fault was not injected")
    for key in MUST_STAY_ZERO:
        value = last(r, t, key)
        if value:
            failed.append("%s is %s, should be 0" % (key, value))
    if script == "red":
        odo = series(r, t, "Swerve/Odometry/Pose")
        heading = at(odo, steps[1][0])[2] if len(steps) > 1 and odo else 0
        if abs(abs(heading) - math.pi) > 0.05:
            failed.append("odometry heading %.2f rad at first enable, should be pi" % heading)
    for f in failed:
        print("FAILED:", f)
    print("sim_check %s: %s" % (script, "FAILED" if failed else "passed"))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1], sys.argv[2]))
