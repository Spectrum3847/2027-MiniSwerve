package frc.spectrumLib.swerve;

/**
 * Measures the real wheel radius by spinning in place: the gyro says how far the robot turned, each
 * wheel's travel says how far it rolled, and on a circle of the drive-base radius those must agree.
 * The same method as team 6328's routine; the analysis of other teams' code ranks wheel radius the
 * most important characterization, because it scales every odometry distance.
 *
 * <p>radius = configured radius * (gyro turn * drive-base radius) / mean wheel travel.
 *
 * <p>Main thread; reads only logged samples, so it reruns in replay.
 */
public final class WheelRadiusCharacterizer {
    private final double driveBaseRadius;
    private final int n;
    private final double[] startDistance;
    private double lastYaw;
    private double turned;
    private double estimate = Double.NaN;
    private boolean started;

    /**
     * @param mx module x positions (m)
     * @param my module y positions (m)
     */
    public WheelRadiusCharacterizer(double[] mx, double[] my) {
        n = mx.length;
        double sum = 0;
        for (int i = 0; i < n; i++) {
            sum += Math.hypot(mx[i], my[i]);
        }
        driveBaseRadius = sum / n;
        startDistance = new double[n];
    }

    /** Starts a measurement from this sample. */
    public void start(SwerveSample s) {
        System.arraycopy(s.distance, 0, startDistance, 0, n);
        lastYaw = s.yaw;
        turned = 0;
        estimate = Double.NaN;
        started = true;
    }

    /**
     * Adds a sample and updates the estimate.
     *
     * @param configuredRadius the wheel radius the samples' distances were computed with (m)
     */
    public void update(SwerveSample s, double configuredRadius) {
        if (!started) {
            start(s);
            return;
        }
        turned += Math.abs(SwerveMath.angleModulus(s.yaw - lastYaw));
        lastYaw = s.yaw;
        double travel = 0;
        for (int i = 0; i < n; i++) {
            travel += Math.abs(s.distance[i] - startDistance[i]);
        }
        travel /= n;
        // Too little motion gives a noisy ratio.
        if (turned > Math.toRadians(30) && travel > 1e-3) {
            estimate = configuredRadius * turned * driveBaseRadius / travel;
        }
    }

    /** Estimated wheel radius (m), NaN until the robot has turned enough. */
    public double estimate() {
        return estimate;
    }

    /** Total gyro turn since start (rad). */
    public double turned() {
        return turned;
    }

    public void stop() {
        started = false;
    }
}
