package frc.spectrumSim.swerve;

/**
 * How the robot's weight splits across its wheels as it accelerates: weight transfer. Speeding up
 * presses the back wheels down and lightens the front ones; turning or strafing loads the outside
 * wheels. A lighter wheel grips less, so a high center of gravity loses traction and, past a point,
 * lifts wheels and tips.
 *
 * <p>Quasi-static and rigid: the chassis does not flex or tilt, so loads follow the acceleration at
 * once. Exact for wheels laid out symmetrically about the center of gravity (any centered
 * rectangle, every swerve we build). A wheel whose load would go negative is lifted (load 0) and
 * the robot is starting to tip: with one wheel up the other three are solved exactly, with more up
 * the rest share the weight. Tipping itself (the body rotating up) is not modeled yet.
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
        int up = 0, liftedWheel = -1;
        for (int i = 0; i < n; i++) {
            if (lifted[i]) {
                up++;
                liftedWheel = i;
            }
        }
        if (up == 1 && n == 4 && solveThree(liftedWheel, massKg, cogHeight, ax, ay)) {
            return;
        }
        // Several wheels up: the rest share the weight, in proportion to what they carried.
        if (total > 0) {
            double scale = massKg * GRAVITY / total;
            for (int i = 0; i < n; i++) {
                load[i] *= scale;
            }
        }
    }

    /**
     * One wheel up: the three still down carry the weight and both tipping moments exactly (three
     * equations, three loads). False if that would need a negative load (a second wheel lifts).
     */
    private boolean solveThree(int up, double massKg, double cogHeight, double ax, double ay) {
        int[] k = three;
        int c = 0;
        for (int i = 0; i < 4; i++) {
            if (i != up) {
                k[c++] = i;
            }
        }
        // Rows: sum N = W, sum N mx = -m h ax, sum N my = -m h ay. Cramer's rule.
        double w = massKg * GRAVITY, px = -massKg * cogHeight * ax, py = -massKg * cogHeight * ay;
        double a1 = mx[k[0]], a2 = mx[k[1]], a3 = mx[k[2]];
        double b1 = my[k[0]], b2 = my[k[1]], b3 = my[k[2]];
        double det = (a2 * b3 - a3 * b2) - (a1 * b3 - a3 * b1) + (a1 * b2 - a2 * b1);
        if (Math.abs(det) < 1e-12) {
            return false;
        }
        double n1 = (w * (a2 * b3 - a3 * b2) - px * (b3 - b2) + py * (a3 - a2)) / det;
        double n2 = (-w * (a1 * b3 - a3 * b1) + px * (b3 - b1) - py * (a3 - a1)) / det;
        double n3 = (w * (a1 * b2 - a2 * b1) - px * (b2 - b1) + py * (a2 - a1)) / det;
        if (n1 < 0 || n2 < 0 || n3 < 0) {
            return false;
        }
        load[k[0]] = n1;
        load[k[1]] = n2;
        load[k[2]] = n3;
        load[up] = 0;
        return true;
    }

    private final int[] three = new int[3];

    /** Wheel {@code i}'s load on the floor (N). */
    double load(int i) {
        return load[i];
    }

    /** Wheel {@code i} is off the floor: the robot is starting to tip. */
    boolean lifted(int i) {
        return lifted[i];
    }
}
