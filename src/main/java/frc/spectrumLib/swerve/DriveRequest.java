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
 * @param headingOffset field heading minus gyro yaw (rad), so the thread can use fresh yaw
 */
public record DriveRequest(
        Mode mode,
        double vx,
        double vy,
        double omega,
        boolean fieldRelative,
        double headingOffset) {

    public enum Mode {
        /** Motors neutral (brake or coast per config). */
        NEUTRAL,
        /** Drive at the given chassis velocity. */
        VELOCITY,
        /** Wheels in an X, not driving: hard to push. */
        X_LOCK
    }

    public static final DriveRequest NEUTRAL = new DriveRequest(Mode.NEUTRAL, 0, 0, 0, false, 0);
    public static final DriveRequest X_LOCK = new DriveRequest(Mode.X_LOCK, 0, 0, 0, false, 0);
}
