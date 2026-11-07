package frc.spectrumSim.field;

/**
 * Whether two convex outlines overlap and, if they do, how: the separating-axis test. The result is
 * written into this object (no allocation per step).
 *
 * <p>{@link #normal} points from {@code a} (the obstacle) into {@code b} (the moving body): pushing
 * {@code b} along it by {@link #depth} separates them. {@link #pointX}/{@link #pointY} is where
 * they touch: the deepest corner, or the middle of a flat face pressed flat against another.
 */
public final class Contact {
    public boolean touching;
    public double normalX;
    public double normalY;
    public double depth;
    public double pointX;
    public double pointY;

    /** Corners within this much (m) of the deepest one count as touching too (flat faces). */
    private static final double FLAT = 1e-3;

    /** Tests {@code a} against {@code b}; returns {@link #touching}. */
    public boolean test(ConvexPolygon a, ConvexPolygon b) {
        touching = false;
        double best = Double.POSITIVE_INFINITY;
        double nx = 0, ny = 0;
        boolean fromA = true;
        for (int pass = 0; pass < 2; pass++) {
            ConvexPolygon p = pass == 0 ? a : b;
            int n = p.size();
            for (int i = 0; i < n; i++) {
                int j = (i + 1) % n;
                // Outward normal of a counter-clockwise edge.
                double ex = p.x[j] - p.x[i], ey = p.y[j] - p.y[i];
                double len = Math.hypot(ex, ey);
                if (len < 1e-12) {
                    continue;
                }
                double ax = ey / len, ay = -ex / len;
                double overlap = overlap(a, b, ax, ay);
                if (overlap <= 0) {
                    return false; // a separating axis: no contact
                }
                if (overlap < best) {
                    best = overlap;
                    nx = ax;
                    ny = ay;
                    fromA = pass == 0;
                }
            }
        }
        // Point the normal from a toward b.
        double dcx = center(b.x) - center(a.x), dcy = center(b.y) - center(a.y);
        if (nx * dcx + ny * dcy < 0) {
            nx = -nx;
            ny = -ny;
        }
        touching = true;
        normalX = nx;
        normalY = ny;
        depth = best;
        // The contact point: b's corners deepest into a's face, or a's corners deepest into b's.
        if (fromA) {
            deepest(b, nx, ny, -1);
        } else {
            deepest(a, nx, ny, 1);
        }
        return true;
    }

    /** Overlap of the two shadows on axis (ax, ay); negative or zero when apart. */
    private static double overlap(ConvexPolygon a, ConvexPolygon b, double ax, double ay) {
        double aMin = Double.POSITIVE_INFINITY, aMax = Double.NEGATIVE_INFINITY;
        for (int i = 0; i < a.size(); i++) {
            double d = a.x[i] * ax + a.y[i] * ay;
            aMin = Math.min(aMin, d);
            aMax = Math.max(aMax, d);
        }
        double bMin = Double.POSITIVE_INFINITY, bMax = Double.NEGATIVE_INFINITY;
        for (int i = 0; i < b.size(); i++) {
            double d = b.x[i] * ax + b.y[i] * ay;
            bMin = Math.min(bMin, d);
            bMax = Math.max(bMax, d);
        }
        return Math.min(aMax - bMin, bMax - aMin);
    }

    /** Averages p's corners lying furthest along {@code sign} * normal (within FLAT). */
    private void deepest(ConvexPolygon p, double nx, double ny, int sign) {
        double most = Double.NEGATIVE_INFINITY;
        for (int i = 0; i < p.size(); i++) {
            most = Math.max(most, sign * (p.x[i] * nx + p.y[i] * ny));
        }
        double sx = 0, sy = 0;
        int count = 0;
        for (int i = 0; i < p.size(); i++) {
            if (sign * (p.x[i] * nx + p.y[i] * ny) >= most - FLAT) {
                sx += p.x[i];
                sy += p.y[i];
                count++;
            }
        }
        pointX = sx / count;
        pointY = sy / count;
    }

    private static double center(double[] v) {
        double s = 0;
        for (double d : v) {
            s += d;
        }
        return s / v.length;
    }
}
