"""Scores how closely the physics sim matches a real robot on the same commands.

    python tools/fm_extract.py <log.wpilog> build/simreal/x.csv --seconds 30
    ./gradlew simReplay -Pin=build/simreal/x.csv -Pout=build/simreal/x_sim.csv
    python tools/sim_vs_real.py build/simreal/x.csv build/simreal/x_sim.csv

For each measure it prints real, sim and the gap, so a physics change can be judged by whether the
gaps shrink:
  - wheel speed: how far measured speed is from the command (RMS), and sim vs real (RMS)
  - steer: settle time to within 2 degrees and overshoot after each 30+ degree command step
  - speed rise: time from a sprint command (0 to 3+ m/s) to 90% of it
  - acceleration: 50/90/99th percentile of horizontal acceleration (g)
  - current: share of time a drive motor is near its 80 A limit
"""

import csv
import math
import statistics
import sys


def load(path):
    with open(path, encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    return {k: [float(r[k]) for r in rows] for k in rows[0]}


def at(ts, values, t):
    """Value at time t (last sample at or before t)."""
    lo, hi = 0, len(ts) - 1
    if t <= ts[0]:
        return values[0]
    while lo < hi:
        mid = (lo + hi + 1) // 2
        if ts[mid] <= t:
            lo = mid
        else:
            hi = mid - 1
    return values[lo]


def wrap(a):
    return math.remainder(a, 2 * math.pi)


def axis_err(a, b):
    """Angle error, treating a wheel pointed backward as the same axis."""
    e = abs(wrap(a - b))
    return min(e, math.pi - e)


def smooth(xs, n=5):
    """Moving average over n samples (50 ms at 100 Hz)."""
    out, acc = [], 0.0
    for i, x in enumerate(xs):
        acc += x
        if i >= n:
            acc -= xs[i - n]
        out.append(acc / min(i + 1, n))
    return out


def pct(xs, p):
    xs = sorted(xs)
    return xs[min(len(xs) - 1, int(p / 100 * len(xs)))] if xs else float("nan")


def steer_steps(real, sim):
    """Settle time (s) and overshoot (deg) after each target step of 30+ degrees, real and sim."""
    t = real["t"]
    out = {"real": [], "sim": []}
    for m in range(4):
        ta = real["ta%d" % m]
        for k in range(1, len(t)):
            step = axis_err(ta[k], ta[k - 1])
            if step < math.radians(30):
                continue
            target = ta[k]
            # Held for at least 0.3 s.
            hold = [j for j in range(k, len(t)) if t[j] - t[k] <= 0.3]
            if any(axis_err(ta[j], target) > math.radians(2) for j in hold):
                continue
            for name, angle_key, times in (
                ("real", "ma%d" % m, t),
                ("sim", "sa%d" % m, sim["t"]),
            ):
                src = real if name == "real" else sim
                settle = None
                worst = 0.0
                for j in range(len(times)):
                    tj = times[j]
                    if tj < t[k] or tj > t[k] + 0.3:
                        continue
                    e = axis_err(src[angle_key][j], target)
                    if settle is None and e < math.radians(2):
                        settle = tj - t[k]
                    if settle is not None:
                        worst = max(worst, e)
                if settle is not None:
                    out[name].append((settle, math.degrees(worst)))
    return out


def speed_rises(real, sim):
    """Time from a wheel at rest being commanded 3+ m/s to it reaching 90% of the command."""
    t = real["t"]
    out = {"real": [], "sim": []}
    for m in range(4):
        tv, mv = real["tv%d" % m], real["mv%d" % m]
        k = 1
        while k < len(t):
            if abs(mv[k]) < 0.3 and abs(tv[k]) > 3.0:
                goal = 0.9 * abs(tv[k])
                for name, key, src in (("real", "mv%d" % m, real), ("sim", "sv%d" % m, sim)):
                    times = src["t"]
                    for j in range(len(times)):
                        if times[j] < t[k] or times[j] > t[k] + 2.0:
                            continue
                        if abs(src[key][j]) >= goal:
                            out[name].append(times[j] - t[k])
                            break
                # Skip ahead past this sprint.
                while k < len(t) and abs(mv[k]) < goal:
                    k += 1
            k += 1
    return out


def main(real_path, sim_path):
    real = load(real_path)
    sim = load(sim_path)
    t = real["t"]
    print("%-34s %10s %10s %10s" % ("measure", "real", "sim", "gap"))

    def line(name, a, b, fmt="%.3f"):
        gap = b - a if not (math.isnan(a) or math.isnan(b)) else float("nan")
        print("%-34s %10s %10s %10s" % (name, fmt % a, fmt % b, fmt % gap))

    # Wheel speed against the command, and sim against real.
    track_real, track_sim, sim_real = [], [], []
    for k, tm in enumerate(t):
        for m in range(4):
            target = real["tv%d" % m][k]
            mv = real["mv%d" % m][k]
            sv = at(sim["t"], sim["sv%d" % m], tm)
            # Compare speeds along the wheel's axis (sign depends on which way it points).
            track_real.append((abs(mv) - abs(target)) ** 2)
            track_sim.append((abs(sv) - abs(target)) ** 2)
            sim_real.append((abs(sv) - abs(mv)) ** 2)
    line("wheel speed err vs command (m/s)", math.sqrt(statistics.mean(track_real)),
         math.sqrt(statistics.mean(track_sim)))
    print("%-34s %10s %10.3f" % ("wheel speed sim vs real RMS (m/s)", "", math.sqrt(statistics.mean(sim_real))))

    steps = steer_steps(real, sim)
    for name in ("real", "sim"):
        steps[name] = steps[name] or [(float("nan"), float("nan"))]
    line("steer settle median (ms)", 1000 * statistics.median(s for s, _ in steps["real"]),
         1000 * statistics.median(s for s, _ in steps["sim"]), "%.0f")
    line("steer settle p90 (ms)", 1000 * pct([s for s, _ in steps["real"]], 90),
         1000 * pct([s for s, _ in steps["sim"]], 90), "%.0f")
    line("steer overshoot median (deg)", statistics.median(o for _, o in steps["real"]),
         statistics.median(o for _, o in steps["sim"]), "%.1f")
    print("%-34s %10d %10d" % ("steer steps scored", len(steps["real"]), len(steps["sim"])))

    rises = speed_rises(real, sim)
    line("speed rise to 90%, median (ms)", 1000 * statistics.median(rises["real"] or [float("nan")]),
         1000 * statistics.median(rises["sim"] or [float("nan")]), "%.0f")
    print("%-34s %10d %10d" % ("sprints scored", len(rises["real"]), len(rises["sim"])))

    # Both smoothed the same way (50 ms moving average): the Pigeon filters its accelerometer,
    # and the sim's is a raw velocity difference, so unsmoothed medians are not comparable.
    ar = [math.hypot(x, y) for x, y in zip(smooth(real["ax"]), smooth(real["ay"]))]
    asim = [math.hypot(x, y) for x, y in zip(smooth(sim["sax"]), smooth(sim["say"]))]
    for p in (50, 90, 99):
        line("accel p%d (g)" % p, pct(ar, p), pct(asim, p))

    near_r = statistics.mean(1.0 if real["i%d" % m][k] >= 75 else 0.0 for k in range(len(t)) for m in range(4))
    near_s = statistics.mean(1.0 if sim["si%d" % m][k] >= 75 else 0.0 for k in range(len(sim["t"])) for m in range(4))
    line("time at current limit (share)", near_r, near_s)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1], sys.argv[2]))
