package frc.spectrumLib.swerve;

/**
 * The drivetrain hardware, as the swerve thread and main loop see it. {@code SwerveIOPhoenix}
 * drives real (or Phoenix-simulated) devices; replay uses {@link #NONE} and gets everything from
 * the log.
 */
public interface SwerveIO {
    /**
     * Swerve thread: wait for the next set of device signals (or one period, in sim) and fill
     * {@code out}'s readings. Must not log.
     */
    void waitForSample(SwerveSample out);

    /**
     * Swerve thread: command every module. Called right after {@link #waitForSample}.
     *
     * @param angle module directions (rad)
     * @param velocity wheel speeds (m/s)
     * @param steerVelocity how fast each module is being turned (rad/s), for coupling compensation
     */
    void setModules(double[] angle, double[] velocity, double[] steerVelocity);

    /** Swerve thread: all motors neutral. */
    void setNeutral();

    /** Main thread, slow rate: currents, temperatures, connection. */
    void updateHealth(SwerveHealth out);

    /** Whether there is hardware (real or simulated) behind this IO. */
    default boolean hasHardware() {
        return true;
    }

    SwerveIO NONE =
            new SwerveIO() {
                @Override
                public void waitForSample(SwerveSample out) {}

                @Override
                public void setModules(double[] angle, double[] velocity, double[] steerVelocity) {}

                @Override
                public void setNeutral() {}

                @Override
                public void updateHealth(SwerveHealth out) {}

                @Override
                public boolean hasHardware() {
                    return false;
                }
            };
}
