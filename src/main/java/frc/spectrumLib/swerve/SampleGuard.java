package frc.spectrumLib.swerve;

/**
 * Catches readings that jump farther in one sample than the robot can move, and marks them stale
 * for that sample, so odometry and field-relative driving never use them. A device that reboots
 * does this: a TalonFX forgets its config and reports rotor rotations (the steer reading jumps by
 * the gear ratio, and the wheel distance with it, through the coupling term), and a Pigeon 2 starts
 * its yaw again from zero. The main loop notices a reboot only on its next health check; this
 * catches the jump on the sample it happens.
 *
 * <p>A module marked stale is re-primed by odometry on its next fresh sample; a gyro marked stale
 * leaves heading to the wheels for that sample, which moves the gyro offset to the new yaw.
 *
 * <p>Driver and vision resets are untouched: seedFieldCentric and pose resets move odometry's
 * offsets and the pose estimator, never the raw device readings this checks. Keep it that way: code
 * that sets a device's position or yaw ({@code setYaw}, {@code setPosition}) would be rejected here
 * for one sample and must instead go through odometry's reset.
 *
 * <p>Swerve thread, on each raw sample before it is used or queued, so the logged sample (and
 * replay) already has the result. Allocation-free.
 */
public final class SampleGuard {
    private final double maxWheelSpeed;
    private final double maxSteerRate;
    private final double maxYawRate;
    private final double[] lastDistance;
    private final double[] lastAngle;
    private double lastYaw;
    private double lastTimestamp = Double.NaN;
    private long lastReadNanos;
    private final double minPeriod;
    private boolean yawJumped;
    private volatile long rejected;

    /** Extra room per sample over the physical limit (m, rad): jitter and latency compensation. */
    private static final double DISTANCE_SLACK = 0.01;

    private static final double ANGLE_SLACK = 0.1;
    private static final double YAW_SLACK = 0.1;

    /**
     * @param modules module count
     * @param maxWheelSpeed fastest a wheel can really move (m/s), with margin
     * @param maxSteerRate fastest a module can really turn (rad/s), with margin
     * @param maxYawRate fastest the robot can really spin (rad/s), with margin
     * @param minPeriod the thread's period (s): the least time a sample can span
     */
    public SampleGuard(
            int modules,
            double maxWheelSpeed,
            double maxSteerRate,
            double maxYawRate,
            double minPeriod) {
        this.minPeriod = minPeriod;
        this.maxWheelSpeed = maxWheelSpeed;
        this.maxSteerRate = maxSteerRate;
        this.maxYawRate = maxYawRate;
        lastDistance = new double[modules];
        lastAngle = new double[modules];
    }

    /**
     * Checks one raw sample against the last, clearing moduleOk or gyroOk where a reading jumped.
     */
    @SuppressWarnings("NonAtomicVolatileUpdate") // one writer: the swerve thread
    public void check(SwerveSample s) {
        boolean first = Double.isNaN(lastTimestamp);
        // Readings are latency-compensated to when they were read, so a late read moves them
        // further than the capture gap says: allow the larger of the capture and read gaps,
        // never less than one period (capture times can shift when a device goes quiet).
        long now = System.nanoTime();
        double dt =
                first
                        ? 0
                        : Math.max(
                                minPeriod,
                                Math.max(
                                        s.timestamp - lastTimestamp, (now - lastReadNanos) * 1e-9));
        lastReadNanos = now;
        yawJumped = false;
        for (int i = 0; i < lastDistance.length; i++) {
            if (!first
                    && s.moduleOk[i]
                    && (Math.abs(s.distance[i] - lastDistance[i])
                                    > maxWheelSpeed * dt + DISTANCE_SLACK
                            || Math.abs(SwerveMath.angleModulus(s.angle[i] - lastAngle[i]))
                                    > maxSteerRate * dt + ANGLE_SLACK)) {
                s.moduleOk[i] = false;
                rejected = rejected + 1; // one writer: this thread
            }
            // Compare the next sample with this raw reading: after a reset the new readings are
            // consistent with each other, so only the jump itself is rejected.
            lastDistance[i] = s.distance[i];
            lastAngle[i] = s.angle[i];
        }
        if (!first
                && s.gyroOk
                && Math.abs(SwerveMath.angleModulus(s.yaw - lastYaw))
                        > maxYawRate * dt + YAW_SLACK) {
            s.gyroOk = false;
            yawJumped = true;
            rejected = rejected + 1;
        }
        lastYaw = s.yaw;
        lastTimestamp = s.timestamp;
    }

    /** The gyro's yaw jumped on the last checked sample (a Pigeon reboot). */
    public boolean yawJumped() {
        return yawJumped;
    }

    /** Readings rejected as jumps, since boot. */
    public long rejected() {
        return rejected;
    }
}
