package frc.spectrumLib.swerve;

/**
 * What the main loop wants the drivetrain to do, read by the swerve thread every cycle. Immutable:
 * the main loop publishes a new one (at most 100 Hz) and the thread uses the latest.
 *
 * @param mode how to drive
 * @param vx forward (m/s), field or robot frame per {@code fieldRelative}
 * @param vy left (m/s)
 * @param omega CCW rotation (rad/s)
 * @param fieldRelative vx/vy are field-frame; the thread rotates them by the live gyro heading
 * @param headingOffset with {@code useGyro}: field heading minus gyro yaw (rad), so the thread can
 *     use fresh yaw; without: the field heading itself (rad)
 * @param useGyro the gyro is good; false makes the thread use {@code headingOffset} as the heading
 * @param wheelForceFeedforwardX per-module force (N) along x, in the same frame as vx/vy, or null;
 *     CTRE's {@code WheelForceFeedforwardX}, from a path follower such as Choreo
 * @param wheelForceFeedforwardY per-module force (N) along y, or null
 * @param maxWheelVelocity this cycle's top wheel speed (m/s), NaN for the configured one; e.g. a
 *     raised elevator lowering it to keep the robot from tipping
 * @param maxDriveAcceleration this cycle's wheel acceleration limit (m/s^2), NaN for configured
 */
public record DriveRequest(
        Mode mode,
        double vx,
        double vy,
        double omega,
        boolean fieldRelative,
        double headingOffset,
        boolean useGyro,
        double[] wheelForceFeedforwardX,
        double[] wheelForceFeedforwardY,
        double maxWheelVelocity,
        double maxDriveAcceleration) {

    public enum Mode {
        /** Motors neutral (brake or coast per config). */
        NEUTRAL,
        /** Drive at the given chassis velocity. */
        VELOCITY,
        /** Wheels in an X, not driving: hard to push. */
        X_LOCK
    }

    public static final DriveRequest NEUTRAL =
            new DriveRequest(
                    Mode.NEUTRAL, 0, 0, 0, false, 0, true, null, null, Double.NaN, Double.NaN);

    /** A velocity request with no wheel force feedforward. */
    public static DriveRequest velocity(
            double vx,
            double vy,
            double omega,
            boolean fieldRelative,
            double headingOffset,
            boolean useGyro) {
        return new DriveRequest(
                Mode.VELOCITY,
                vx,
                vy,
                omega,
                fieldRelative,
                headingOffset,
                useGyro,
                null,
                null,
                Double.NaN,
                Double.NaN);
    }

    /** This request with this cycle's speed and acceleration limits (NaN keeps the configured). */
    public DriveRequest withLimits(double maxWheelVelocity, double maxDriveAcceleration) {
        return new DriveRequest(
                mode,
                vx,
                vy,
                omega,
                fieldRelative,
                headingOffset,
                useGyro,
                wheelForceFeedforwardX,
                wheelForceFeedforwardY,
                maxWheelVelocity,
                maxDriveAcceleration);
    }

    public static final DriveRequest X_LOCK =
            new DriveRequest(
                    Mode.X_LOCK, 0, 0, 0, false, 0, true, null, null, Double.NaN, Double.NaN);
}
