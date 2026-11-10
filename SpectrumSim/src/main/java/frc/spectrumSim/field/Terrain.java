package frc.spectrumSim.field;

/**
 * The floor's height across the field, so the sim robot rides over bumps and ramps. Each bump is a
 * box footprint (center, length along its heading, width across) whose top starts at {@code edge}
 * above the carpet at both ends, rises at its ramp angle, and is capped at its height: a 2026 BUMP
 * (a ridge with a small lip at each ramp's foot), a flat platform with ramps, or a single step.
 * Loaded from a field file by {@link FieldMap}.
 *
 * <p>Field frame (m). Steps too steep to drive up belong in {@link Arena} as walls, not here.
 * Allocation-free lookups; immutable, so safe from any thread.
 */
public final class Terrain {
    /** One bump: center, unit heading, half sizes, height, rise per meter of ramp, lip height. */
    private final double[] cx, cy, cos, sin, halfLength, halfWidth, height, slope, edge;

    /** A flat floor. */
    public static final Terrain FLAT = new Terrain(new double[0][]);

    /**
     * @param bumps each {x, y, length, width, height, rampDeg, headingDeg, edge} (m, deg); edge is
     *     the height of each ramp's foot above the carpet
     */
    public Terrain(double[][] bumps) {
        int n = bumps.length;
        cx = new double[n];
        cy = new double[n];
        cos = new double[n];
        sin = new double[n];
        halfLength = new double[n];
        halfWidth = new double[n];
        height = new double[n];
        slope = new double[n];
        edge = new double[n];
        for (int i = 0; i < n; i++) {
            double[] b = bumps[i];
            cx[i] = b[0];
            cy[i] = b[1];
            halfLength[i] = b[2] / 2;
            halfWidth[i] = b[3] / 2;
            height[i] = b[4];
            slope[i] = Math.tan(Math.toRadians(b[5]));
            cos[i] = Math.cos(Math.toRadians(b[6]));
            sin[i] = Math.sin(Math.toRadians(b[6]));
            edge[i] = b.length > 7 ? b[7] : 0;
        }
    }

    /** Floor height at a field point (m). Overlapping bumps take the higher. */
    public double height(double x, double y) {
        double z = 0;
        for (int i = 0; i < cx.length; i++) {
            double dx = x - cx[i], dy = y - cy[i];
            double along = Math.abs(dx * cos[i] + dy * sin[i]);
            double across = Math.abs(-dx * sin[i] + dy * cos[i]);
            if (along < halfLength[i] && across < halfWidth[i]) {
                z = Math.max(z, Math.min(height[i], edge[i] + (halfLength[i] - along) * slope[i]));
            }
        }
        return z;
    }

    /**
     * The floor's slope at a field point (rise per meter along field x and y) into {@code out}.
     * Exact, not differenced: the lip at a ramp's foot and a bump's side faces are steps of no
     * length, so they add no slope (the plane fit under the wheels still lifts the robot over
     * them).
     */
    public void slope(double x, double y, double[] out) {
        double z = 0, gx = 0, gy = 0;
        for (int i = 0; i < cx.length; i++) {
            double dx = x - cx[i], dy = y - cy[i];
            double signedAlong = dx * cos[i] + dy * sin[i];
            double along = Math.abs(signedAlong);
            double across = Math.abs(-dx * sin[i] + dy * cos[i]);
            if (along < halfLength[i] && across < halfWidth[i]) {
                double ramp = edge[i] + (halfLength[i] - along) * slope[i];
                double h = Math.min(height[i], ramp);
                if (h > z) {
                    z = h;
                    // On a ramp the floor falls away from the middle; on the cap it is level.
                    double s = ramp < height[i] ? -Math.signum(signedAlong) * slope[i] : 0;
                    gx = s * cos[i];
                    gy = s * sin[i];
                }
            }
        }
        out[0] = gx;
        out[1] = gy;
    }

    /** Bumps in this terrain. */
    public int bumps() {
        return cx.length;
    }
}
