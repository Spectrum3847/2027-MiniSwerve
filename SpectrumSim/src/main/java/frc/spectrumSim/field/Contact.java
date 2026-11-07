package frc.spectrumSim.field;

/**
 * Whether two convex outlines overlap and, if they do, how: the separating-axis test. The result is
 * written into this object (no allocation per step).
 *
 * <p>{@link #normalX}/{@link #normalY} points from {@code a} into {@code b}: pushing {@code b}
 * along it by {@link #depth} separates them. Up to two touch points ({@link #count}): every corner
 * of one outline inside the other, deepest first. A bumper face pressed flat or nearly flat against
 * a wall gives two points, so the hit is shared along the face instead of landing on one corner and
 * spinning the robot.
 */
public final class Contact {
    public boolean touching;
    public double normalX;
    public double normalY;
    public double depth;

    /** Touch points: how many, where, and how deep each is along the normal. */
    public int count;

    public final double[] pointX = new double[2];
    public final double[] pointY = new double[2];
    public final double[] pointDepth = new double[2];

    /** Tests {@code a} against {@code b}; returns {@link #touching}. */
    public boolean test(ConvexPolygon a, ConvexPolygon b) {
        touching = false;
        double best = Double.POSITIVE_INFINITY;
        double nx = 0, ny = 0;
        for (int pass = 0; pass < 2; pass++) {
            ConvexPolygon p = pass == 0 ? a : b;
            int n = p.size();
            for (int i = 0; i < n; i++) {
                int j = (i + 1) % n;
                double ex = p.x[j] - p.x[i], ey = p.y[j] - p.y[i];
                double len = Math.hypot(ex, ey);
                if (len < 1e-12) {
                    continue;
                }
                double ax = ey / len, ay = -ex / len;
                // Shadows on this axis; b sits on the + side if pushing it + is the shorter way.
                double aMin = Double.POSITIVE_INFINITY, aMax = Double.NEGATIVE_INFINITY;
                for (int k = 0; k < a.size(); k++) {
                    double d = a.x[k] * ax + a.y[k] * ay;
                    aMin = Math.min(aMin, d);
                    aMax = Math.max(aMax, d);
                }
                double bMin = Double.POSITIVE_INFINITY, bMax = Double.NEGATIVE_INFINITY;
                for (int k = 0; k < b.size(); k++) {
                    double d = b.x[k] * ax + b.y[k] * ay;
                    bMin = Math.min(bMin, d);
                    bMax = Math.max(bMax, d);
                }
                double plus = aMax - bMin, minus = bMax - aMin;
                double overlap = Math.min(plus, minus);
                if (overlap <= 0) {
                    return false; // a separating axis: no contact
                }
                if (overlap < best) {
                    best = overlap;
                    nx = plus <= minus ? ax : -ax;
                    ny = plus <= minus ? ay : -ay;
                }
            }
        }
        touching = true;
        normalX = nx;
        normalY = ny;
        depth = best;
        count = 0;
        // Corners of b inside a (depth past a's face) and corners of a inside b.
        double aFace = Double.NEGATIVE_INFINITY, bFace = Double.POSITIVE_INFINITY;
        for (int k = 0; k < a.size(); k++) {
            aFace = Math.max(aFace, a.x[k] * nx + a.y[k] * ny);
        }
        for (int k = 0; k < b.size(); k++) {
            bFace = Math.min(bFace, b.x[k] * nx + b.y[k] * ny);
        }
        for (int k = 0; k < b.size(); k++) {
            if (inside(a, b.x[k], b.y[k])) {
                offer(b.x[k], b.y[k], aFace - (b.x[k] * nx + b.y[k] * ny));
            }
        }
        for (int k = 0; k < a.size(); k++) {
            if (inside(b, a.x[k], a.y[k])) {
                offer(a.x[k], a.y[k], (a.x[k] * nx + a.y[k] * ny) - bFace);
            }
        }
        if (count == 0) {
            // Edges crossing with no corner inside: the deepest corner of b along -n.
            int deepest = 0;
            for (int k = 1; k < b.size(); k++) {
                if (b.x[k] * nx + b.y[k] * ny < b.x[deepest] * nx + b.y[deepest] * ny) {
                    deepest = k;
                }
            }
            offer(b.x[deepest], b.y[deepest], best);
        }
        return true;
    }

    /** Keeps the two deepest points. */
    private void offer(double x, double y, double d) {
        if (count < 2) {
            pointX[count] = x;
            pointY[count] = y;
            pointDepth[count] = d;
            count++;
            return;
        }
        int shallow = pointDepth[0] <= pointDepth[1] ? 0 : 1;
        if (d > pointDepth[shallow]) {
            pointX[shallow] = x;
            pointY[shallow] = y;
            pointDepth[shallow] = d;
        }
    }

    /** Whether (x, y) is inside a counter-clockwise convex outline (on an edge counts). */
    private static boolean inside(ConvexPolygon p, double x, double y) {
        int n = p.size();
        for (int i = 0; i < n; i++) {
            int j = (i + 1) % n;
            double cross = (p.x[j] - p.x[i]) * (y - p.y[i]) - (p.y[j] - p.y[i]) * (x - p.x[i]);
            if (cross < -1e-12) {
                return false;
            }
        }
        return true;
    }
}
