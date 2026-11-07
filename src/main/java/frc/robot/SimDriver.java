package frc.robot;

import java.util.Random;

/**
 * The person on the controller in a sim script, so scripted driving reaches the robot the way a
 * driver's does, not as perfect step changes:
 *
 * <ul>
 *   <li><b>Hands take time:</b> a stick moves toward where the step wants it over about 0.15 s
 *       (time constant {@link #REACH_SECONDS}), and falls back to center the same way.
 *   <li><b>Pushes are imperfect:</b> each step's push is scaled 95 to 100% and turned up to 3 deg
 *       off the axis the step names, picked once per step.
 *   <li><b>Sticks are worn:</b> each axis rests a little off center (up to 3% of travel, fixed per
 *       run) and jitters (0.4% of travel), so the deadband has to hold a still robot still.
 * </ul>
 *
 * <p>The radio and Driver Station delay are in {@link SimScript} ({@link #LINK_SECONDS}): it plays
 * the driver's inputs that much behind the robot's clock. Noise is seeded: runs repeat. {@code
 * MINISWERVE_SIM_DRIVER=perfect} turns all of this off (exact sticks, no delay).
 */
final class SimDriver {
    /** How long a hand takes to reach where it is going (time constant, s). */
    static final double REACH_SECONDS = 0.05;

    /** Controller to robot code: Driver Station laptop, radio, robot loop (s). */
    static final double LINK_SECONDS = 0.03;

    private static final double MAX_REST_OFFSET = 0.03;
    private static final double JITTER = 0.004;
    private static final double MAX_ANGLE_ERROR = Math.toRadians(3);

    private final boolean perfect;
    private final Random random = new Random(3847);
    private final double[] rest = new double[3];
    private final double[] now = new double[3];
    private int step = -1;
    private double scale = 1;
    private double angleError;

    SimDriver() {
        perfect = "perfect".equals(System.getenv("MINISWERVE_SIM_DRIVER"));
        for (int i = 0; i < 3; i++) {
            rest[i] = (random.nextDouble() * 2 - 1) * MAX_REST_OFFSET;
        }
    }

    /** Whether to model the link delay (off for a perfect driver). */
    boolean delayed() {
        return !perfect;
    }

    /**
     * Where the sticks are this loop, given where the step wants them.
     *
     * @param stepIndex the step being played (a new one picks a new imperfect push)
     * @param want left X, left Y, right X the step asks for (-1..1)
     * @param dt seconds since the last loop
     * @return left X, left Y, right X as the controller reads them (reused array)
     */
    double[] sticks(int stepIndex, double[] want, double dt) {
        if (perfect) {
            System.arraycopy(want, 0, now, 0, 3);
            return now;
        }
        if (stepIndex != step) {
            step = stepIndex;
            scale = 0.95 + 0.05 * random.nextDouble();
            angleError = (random.nextDouble() * 2 - 1) * MAX_ANGLE_ERROR;
        }
        // The left stick's push, scaled and turned a little off the named axis.
        double c = Math.cos(angleError), s = Math.sin(angleError);
        double lx = (want[0] * c - want[1] * s) * scale;
        double ly = (want[0] * s + want[1] * c) * scale;
        double rx = want[2] * scale;
        double k = 1 - Math.exp(-dt / REACH_SECONDS);
        now[0] += (lx - now[0]) * k;
        now[1] += (ly - now[1]) * k;
        now[2] += (rx - now[2]) * k;
        double[] out = read;
        for (int i = 0; i < 3; i++) {
            double v = now[i] + rest[i] + random.nextGaussian() * JITTER;
            out[i] = Math.max(-1, Math.min(1, v));
        }
        return out;
    }

    private final double[] read = new double[3];
}
