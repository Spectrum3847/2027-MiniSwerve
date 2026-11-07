"""Fits sim parameters to real match logs: robot mass, turning inertia, command latency (and any
other SimReplay param) by coordinate descent.

    python tools/fit_sim.py <log.wpilog> [<log.wpilog> ...] [--seconds 40] [--start 20]
        [--fit mass=40:80:10 inertia=0.3:1.2:0.15 latency=0:0.04:0.005] [--fixed "friction=0.9"]

Each log's window is extracted once (fm_extract.py), then every candidate value runs through
SimReplay in one JVM per log (`-Psweep`). The score is the mean over logs of:
  - wheel speed, sim vs real (m/s RMS, speeds along the wheel),
  - yaw rate, sim vs real (rad/s RMS),
  - steer angle, sim vs real (rad RMS, modulo half a turn since a module may flip),
scored only on free driving: samples where every real wheel tracks its command within 0.5 m/s
(pushing matches and wall hits are not in the sim). Prints each pass and the best values.
"""

import argparse
import math
import os
import statistics
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
OUT = os.path.join(ROOT, "build", "simfit")
GRADLE = os.path.join(ROOT, "gradlew.bat" if os.name == "nt" else "gradlew")


def load(path):
    with open(path, encoding="utf-8") as f:
        head = f.readline().strip().split(",")
        rows = [list(map(float, line.split(","))) for line in f if line.strip()]
    return {k: [r[i] for r in rows] for i, k in enumerate(head)}


def extract(log, name, start, seconds):
    out = os.path.join(OUT, name + ".csv")
    if not os.path.exists(out):
        subprocess.run([sys.executable, os.path.join(HERE, "fm_extract.py"), log, out,
                        "--start", str(start), "--seconds", str(seconds)], check=True)
    return out


def sweep(real_csv, name, candidates, fixed):
    """Runs every candidate {label: params} through SimReplay; returns {label: csv path}."""
    sweep_file = os.path.join(OUT, name + "_sweep.txt")
    with open(sweep_file, "w", encoding="utf-8", newline="\n") as f:
        for label, params in candidates.items():
            f.write(label + " " + fixed + " " + " ".join("%s=%g" % kv for kv in params.items()) + "\n")
    base = os.path.join(OUT, name + "_sim.csv")
    subprocess.run([GRADLE, "simReplay", "-q", "-Pin=" + real_csv, "-Pout=" + base,
                    "-Psweep=" + sweep_file], cwd=ROOT, check=True, stdout=subprocess.DEVNULL)
    return {label: base[:-4] + "_" + label + ".csv" for label in candidates}


def score(real, sim):
    n = min(len(real["t"]), len(sim["t"]))
    speed, yaw, steer = [], [], []
    for k in range(n):
        free = all(abs(abs(real["mv%d" % m][k]) - abs(real["tv%d" % m][k])) < 0.5 for m in range(4))
        if not free:
            continue
        for m in range(4):
            speed.append((abs(sim["sv%d" % m][k]) - abs(real["mv%d" % m][k])) ** 2)
            d = math.remainder(sim["sa%d" % m][k] - real["ma%d" % m][k], math.pi)
            steer.append(d * d)
        if "yawrate" in real:
            yaw.append((sim["syaw"][k] - real["yawrate"][k]) ** 2)
    if not speed:
        return float("inf"), {}
    parts = {
        "speed": math.sqrt(statistics.mean(speed)),
        "yaw": math.sqrt(statistics.mean(yaw)) if yaw else 0.0,
        "steer": math.sqrt(statistics.mean(steer)),
        "free": len(speed) / 4 / n,
    }
    return parts["speed"] + parts["yaw"] + parts["steer"], parts


def grid(spec):
    lo, hi, step = (float(x) for x in spec.split(":"))
    vals, v = [], lo
    while v <= hi + 1e-9:
        vals.append(round(v, 6))
        v += step
    return vals


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("logs", nargs="+")
    ap.add_argument("--start", type=float, default=20.0, help="seconds after enable (skip auto)")
    ap.add_argument("--seconds", type=float, default=40.0)
    ap.add_argument("--fit", nargs="+", default=["mass=40:80:10", "inertia=0.3:1.2:0.15",
                                                 "latency=0:0.04:0.005"])
    ap.add_argument("--fixed", default="")
    ap.add_argument("--passes", type=int, default=2)
    a = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)
    grids = {s.split("=")[0]: grid(s.split("=")[1]) for s in a.fit}
    best = {k: v[len(v) // 2] for k, v in grids.items()}
    reals = {}
    for log in a.logs:
        name = os.path.basename(log).replace(".wpilog", "")
        reals[name] = (extract(log, name, a.start, a.seconds), None)
    reals = {k: (p, load(p)) for k, (p, _) in reals.items()}
    for pas in range(a.passes):
        for key, values in grids.items():
            candidates = {}
            for i, v in enumerate(values):
                c = dict(best)
                c[key] = v
                candidates["%s%d" % (key, i)] = c
            totals = {label: [] for label in candidates}
            details = {label: [] for label in candidates}
            for name, (path, real) in reals.items():
                outs = sweep(path, name, candidates, a.fixed)
                for label, sim_path in outs.items():
                    s, parts = score(real, load(sim_path))
                    totals[label].append(s)
                    details[label].append(parts)
            print("pass %d, %s:" % (pas + 1, key))
            for label, c in candidates.items():
                mean = statistics.mean(totals[label])
                parts = details[label]
                print("  %s=%-8g score %.4f  speed %.3f m/s  yaw %.3f rad/s  steer %.3f rad  free %.0f%%" % (
                    key, c[key], mean,
                    statistics.mean(p.get("speed", float("nan")) for p in parts),
                    statistics.mean(p.get("yaw", float("nan")) for p in parts),
                    statistics.mean(p.get("steer", float("nan")) for p in parts),
                    100 * statistics.mean(p.get("free", 0) for p in parts)))
            winner = min(candidates, key=lambda lb: statistics.mean(totals[lb]))
            best[key] = candidates[winner][key]
            print("  -> %s = %g" % (key, best[key]))
    print("best:", " ".join("%s=%g" % kv for kv in best.items()))
    return 0


if __name__ == "__main__":
    sys.exit(main())
