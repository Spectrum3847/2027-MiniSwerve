"""Summarises a sim drive log: odometry against the sim's true pose, and swerve thread health.

    python tools/swerve_summary.py logs/<log>.wpilog
"""

import math
import sys

from wpilog_read import read_wpilog, series


def main(path):
    records, types = read_wpilog(path)
    odo = dict(series(records, types, "Swerve/Odometry/Pose"))
    truth = dict(series(records, types, "Sim/TruthPose"))
    errs = []
    heading_errs = []
    for t, p in odo.items():
        q = truth.get(t)
        if p is None or q is None:
            continue
        errs.append(math.hypot(p[0] - q[0], p[1] - q[1]))
        heading_errs.append(abs(math.remainder(p[2] - q[2], 2 * math.pi)))
    samples = series(records, types, "Swerve/Samples/Timestamp")
    counts = [len(v) for _, v in samples if v is not None]
    dropped = series(records, types, "Swerve/Thread/DroppedSamples")
    faults = series(records, types, "Swerve/Thread/Faults")
    steps = [v for _, v in series(records, types, "Sim/ScriptStep")]
    final_truth = list(truth.values())[-1] if truth else None
    print("script steps:", steps)
    print("main loops:", len(counts), " samples:", sum(counts),
          " per loop mean %.2f max %d" % (sum(counts) / max(1, len(counts)), max(counts or [0])))
    if len(counts) > 1:
        print("samples per second: %.1f" % (sum(counts) / ((samples[-1][0] - samples[0][0]) or 1)))
    print("dropped samples:", dropped[-1][1] if dropped else "n/a",
          " thread faults:", faults[-1][1] if faults else "n/a")
    if final_truth:
        print("final truth pose: x %.2f y %.2f heading %.1f deg"
              % (final_truth[0], final_truth[1], math.degrees(final_truth[2])))
    if errs:
        print("odometry error vs truth: mean %.4f m, max %.4f m, final %.4f m"
              % (sum(errs) / len(errs), max(errs), errs[-1]))
        print("heading error: max %.3f deg" % math.degrees(max(heading_errs)))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1]))
