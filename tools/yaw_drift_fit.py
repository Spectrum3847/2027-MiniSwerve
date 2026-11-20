"""Finds steering misalignment (wheels toed against each other) from how the robot rotates while
driving straight, and prints the CANcoder offset changes that cancel it.

    python tools/yaw_drift_fit.py logs/<carpet log>.wpilog [more logs with the same offsets]

Drive straight on carpet at a steady speed in several directions (forward, back, left, right, a
second or two each) without touching the turn stick, then run this on the log.

The idea: a wheel pointing d radians counterclockwise of where the code believes pushes the robot
sideways there. With the wheels toed against each other the robot turns, in proportion to speed:

    yaw rate / speed = (A cos(dir) + B sin(dir)) / (8 h)

    dir = travel direction in the robot frame (0 forward, 90 deg left), h = half the track
    A = dFL + dFR - dBL - dBR   front wheels against back wheels
    B = dFL - dFR + dBL - dBR   left wheels against right wheels

Only A and B can be seen this way, not each wheel alone, so the trim printed is the smallest one
that cancels them: FL (A+B)/4, FR (A-B)/4, BL (-A+B)/4, BR (-A-B)/4, divided by RESPONSE.

Direction (verified on MiniSwerve, 2026-11-19: 18 deg/s of drift fell to 8 deg/s, overshooting):
  - magnetOffset is added to the CANcoder's reading, so RAISING a module's offset makes the code
    believe the wheel is further counterclockwise, and the wheel ends up further CLOCKWISE (seen
    from above) for the same command.
  - Robot turns counterclockwise while driving FORWARD (A > 0): raise the front offsets, lower the
    back ones. Robot turns counterclockwise while driving LEFT (B > 0): raise the left offsets,
    lower the right ones.

What this cannot see: all four wheels off the same way (the robot crabs), and the diagonal pattern
(FL and BR one way, FR and BL the other), which scrubs without turning the robot. A straightedge
you trust across both wheels of a side is still the real alignment.
"""

import bisect
import math
import sys

from wpilog_read import read_wpilog, series

# How much more the robot really turned than the formula above predicts, measured on MiniSwerve:
# a trim sized for A = 9.6 deg moved the measured A by 13.2 deg (1.38), and B by 1.47.
RESPONSE = 1.4

MIN_SPEED = 2.0  # m/s
MODULES = ("FL", "FR", "BL", "BR")


def flat(records, types, key, width):
    """One (record time, values) per swerve sample, or None where the arrays do not line up."""
    stamps = series(records, types, "Swerve/Samples/Timestamp")
    points = [p for p in series(records, types, "Swerve/Samples/" + key) if p[1]]
    times = [p[0] for p in points]
    out = []
    for t, st in stamps:
        if not st:
            continue
        i = bisect.bisect_right(times, t) - 1
        if i < 0 or len(points[i][1]) != width * len(st):
            out += [None] * len(st)
            continue
        out += [(t, points[i][1][width * j : width * (j + 1)]) for j in range(len(st))]
    return out


def cruise_samples(path):
    """(travel direction rad, yaw rate per metre) for each sample of steady straight driving."""
    records, types = read_wpilog(path)
    target_v = flat(records, types, "TargetVelocity", 4)
    target_a = flat(records, types, "TargetAngle", 4)
    velocity = flat(records, types, "Velocity", 4)
    yaw_rate = flat(records, types, "AngularVelocityZWorld", 1)
    request = series(records, types, "Swerve/Request/Velocity")
    request_t = [p[0] for p in request]
    out = []
    last_turn = -1e9
    for i in range(10, min(len(target_v), len(target_a), len(velocity), len(yaw_rate))):
        rows = (target_v[i], target_a[i], velocity[i], yaw_rate[i], target_v[i - 10])
        if any(r is None for r in rows):
            continue
        t = target_v[i][0]
        asked = request[max(0, bisect.bisect_right(request_t, t) - 1)][1]
        if abs(asked[2]) > 0.05:
            last_turn = t
        if t - last_turn < 0.5:
            continue  # the driver was turning, or just had been
        v, a = target_v[i][1], target_a[i][1]
        vectors = [(v[m] * math.cos(a[m]), v[m] * math.sin(a[m])) for m in range(4)]
        vx = sum(x for x, _ in vectors) / 4
        vy = sum(y for _, y in vectors) / 4
        speed = math.hypot(vx, vy)
        if speed < MIN_SPEED:
            continue
        if max(math.hypot(x - vx, y - vy) for x, y in vectors) > 0.15:
            continue  # not a pure translation
        if abs(speed - sum(abs(x) for x in target_v[i - 10][1]) / 4) > 0.15:
            continue  # speeding up or slowing down: slip, not alignment
        if abs(sum(abs(x) for x in velocity[i][1]) / 4 - speed) > 0.4:
            continue
        out.append((math.atan2(vy, vx), yaw_rate[i][1][0] / speed))
    settings = {}
    for name in ("track",) + tuple("magnetOffset" + m for m in MODULES):
        s = series(records, types, "RobotSettings/Values/swerve/" + name)
        settings[name] = s[-1][1] if s else None
    return out, settings


def solve3(m, y):
    """Solves a 3x3 linear system by elimination."""
    m = [row[:] for row in m]
    y = y[:]
    for c in range(3):
        p = max(range(c, 3), key=lambda r: abs(m[r][c]))
        m[c], m[p] = m[p], m[c]
        y[c], y[p] = y[p], y[c]
        for r in range(c + 1, 3):
            f = m[r][c] / m[c][c]
            for j in range(c, 3):
                m[r][j] -= f * m[c][j]
            y[r] -= f * y[c]
    x = [0.0] * 3
    for c in (2, 1, 0):
        x[c] = (y[c] - sum(m[c][j] * x[j] for j in range(c + 1, 3))) / m[c][c]
    return x


def main(paths):
    points = []
    settings = {}
    for path in paths:
        got, settings = cruise_samples(path)
        print("%s: %d samples of steady straight driving" % (path, len(got)))
        points += got
    if len(points) < 100:
        print("not enough straight driving to fit (need a few seconds in several directions)")
        return 1
    bins = {}
    for direction, k in points:
        bins.setdefault(int(round(math.degrees(direction) / 45)) * 45, []).append(k)
    print("travel direction (deg, 0 forward, 90 left) | samples | yaw per metre (rad/m)")
    for b in sorted(bins):
        print("  %5d | %5d | %+.3f" % (b, len(bins[b]), sum(bins[b]) / len(bins[b])))
    if not any(abs(abs(b) - 90) <= 45 for b in bins if len(bins[b]) > 50):
        print("WARNING: almost no sideways driving: the left-against-right number (B) is a guess")
    normal = [[0.0] * 3 for _ in range(3)]
    rhs = [0.0] * 3
    for direction, k in points:
        basis = (math.cos(direction), math.sin(direction), 1.0)
        for i in range(3):
            rhs[i] += basis[i] * k
            for j in range(3):
                normal[i][j] += basis[i] * basis[j]
    c, s, constant = solve3(normal, rhs)
    half = (settings["track"] or 0.0) / 2
    if half <= 0:
        print("no swerve/track setting in the log")
        return 1
    a_deg = math.degrees(c * 8 * half)
    b_deg = math.degrees(s * 8 * half)
    rms = math.sqrt(sum(k * k for _, k in points) / len(points))
    left = math.sqrt(
        sum((k - (c * math.cos(d) + s * math.sin(d) + constant)) ** 2 for d, k in points)
        / len(points)
    )
    print("yaw per metre = %.3f cos(dir) + %.3f sin(dir) + %.3f" % (c, s, constant))
    print("  rms %.3f rad/m, %.3f left after the fit" % (rms, left))
    print("front against back A = %+.1f deg, left against right B = %+.1f deg" % (a_deg, b_deg))
    trims = {
        "FL": (a_deg + b_deg) / 4,
        "FR": (a_deg - b_deg) / 4,
        "BL": (-a_deg + b_deg) / 4,
        "BR": (-a_deg - b_deg) / 4,
    }
    print("offset changes that cancel it (already divided by the response of %.1f):" % RESPONSE)
    for m in MODULES:
        change = trims[m] / RESPONSE
        old = settings["magnetOffset" + m]
        new = "" if old is None else "  magnetOffset%s %.5f -> %.5f" % (m, old, old + change / 360)
        print("  %s %+.2f deg (%+.5f rot)%s" % (m, change, change / 360, new))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
