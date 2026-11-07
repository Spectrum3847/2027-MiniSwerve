"""Fits the battery model (open-circuit voltage and resistance) from real match logs.

    python tools/battery_fit.py <log.wpilog> [<log.wpilog> ...]

A battery under load reads V = Voc - R * I. Each log's enabled samples of battery voltage and total
current (BatteryLogger) are fit to that line; R is the battery's internal resistance plus wiring
and breaker, the number SwervePhysicsSim's battery uses. Prints each log's fit and the median.
Needs /RealOutputs/BatteryLogger/BatteryVoltage and /RealOutputs/BatteryLogger/Current.
"""

import statistics
import sys

from wpilog_read import at, read_wpilog, series


def fit(path):
    r, t = read_wpilog(path)
    volts = series(r, t, "/RealOutputs/BatteryLogger/BatteryVoltage")
    amps = series(r, t, "/RealOutputs/BatteryLogger/Current")
    enabled = series(r, t, "/DriverStation/Enabled")
    if not volts or not amps:
        return None
    xs, ys = [], []
    for tm, i in amps:
        on = at(enabled, tm)
        v = at(volts, tm)
        if on and v is not None and i is not None:
            xs.append(i)
            ys.append(v)
    if len(xs) < 50 or max(xs) - min(xs) < 20:
        return None  # too little load range to fit a slope
    mx, my = statistics.mean(xs), statistics.mean(ys)
    sxx = sum((x - mx) ** 2 for x in xs)
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    slope = sxy / sxx
    voc = my - slope * mx
    resid = [y - (voc + slope * x) for x, y in zip(xs, ys)]
    rms = (sum(e * e for e in resid) / len(resid)) ** 0.5
    return -slope, voc, rms, len(xs), max(xs)


def main(paths):
    rs, vs = [], []
    for p in paths:
        f = fit(p)
        name = p.replace("\\", "/").split("/")[-1]
        if f is None:
            print("%-50s no battery current data (or too little load)" % name)
            continue
        r, voc, rms, n, peak = f
        rs.append(r)
        vs.append(voc)
        print("%-50s R %.4f ohm  Voc %.2f V  fit RMS %.2f V  (%d samples, peak %.0f A)"
              % (name, r, voc, rms, n, peak))
    if rs:
        print("median: R %.4f ohm, Voc %.2f V over %d logs" % (statistics.median(rs), statistics.median(vs), len(rs)))
    return 0 if rs else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
