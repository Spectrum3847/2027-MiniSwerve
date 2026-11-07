package frc.spectrumSim.swerve;

/**
 * How the robot's weight splits across its wheels as it accelerates: weight transfer. Speeding up
 * presses the back wheels down and lightens the front ones; turning or strafing loads the outside
 * wheels. A lighter wheel grips less, so a high center of gravity loses traction and, past a point,
 * lifts wheels and tips.
 *
 * <p>Quasi-static and rigid: the chassis does not flex or tilt, so loads follow the acceleration at
 * once. With wheels in a symmetric layout (any rectangle) the four-wheel split is exact; a wheel
 * whose load would go negative is lifted (load 0) and the robot is starting to tip. Tipping itself
 * (the body rotating up) is not modeled yet.
 */
final class WheelLoads {
    private static final double GRAVITY = 9.81;

    private final double[] mx;
    private final double[] my;
    private final double sumX2;
    private final double sumY2;
    private final double[] load;
    private final boolean[] lifted;

    WheelLoads(double[] mx, double[] my) {
        this.mx = mx.clone();
        this.my = my.clone();
        double sx = 0, sy = 0;
        for (int i = 0; i < mx.length; i++) {
            sx += mx[i] * mx[i];
            sy += my[i] * my[i];
        }
        sumX2 = sx;
        sumY2 = sy;
        load = new double[mx.length];
        lifted = new boolean[mx.length];
    }

    /**
     * Splits the weight for an acceleration.
     *
     * @param massKg robot mass
     * @param cogHeight center of gravity above the floor (m)
     * @param ax robot-frame acceleration forward (m/s^2), as the net tire force over mass
     * @param ay robot-frame acceleration left (m/s^2)
     */
    void update(double massKg, double cogHeight, double ax, double ay) {
        int n = mx.length;
        // Moments about the floor: the weight's share stays mg / n per wheel, and the tire forces
        // acting below the center of gravity shift m a h of load from the front (or left) wheels
        // to the back (or right) ones, in proportion to each wheel's distance out.
        double total = 0;
        for (int i = 0; i < n; i++) {
            double shift =
                    massKg
                            * cogHeight
                            * (ax * mx[i] / (sumX2 > 0 ? sumX2 : 1)
                                    + ay * my[i] / (sumY2 > 0 ? sumY2 : 1));
            double l = massKg * GRAVITY / n - shift;
            lifted[i] = l < 0;
            load[i] = Math.max(0, l);
            total += load[i];
        }
        // A lifted wheel's (negative) share is carried by the rest: the weight is still all there.
        if (total > 0) {
            double scale = massKg * GRAVITY / total;
            for (int i = 0; i < n; i++) {
                load[i] *= scale;
            }
        }
    }

    /** Wheel {@code i}'s load on the floor (N). */
    double load(int i) {
        return load[i];
    }

    /** Wheel {@code i} is off the floor: the robot is starting to tip. */
    boolean lifted(int i) {
        return lifted[i];
    }
}
