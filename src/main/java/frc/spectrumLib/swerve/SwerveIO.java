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
     * @param steerVelocity planned module turn rate (rad/s), the steer velocity feedforward
     * @param wheelForce force each wheel should push along its direction (N), for feedforward
     * @param commands which motors get their real command, which coast, and which are disconnected
     *     and only get a command at the slow rate
     */
    void setModules(
            double[] angle,
            double[] velocity,
            double[] steerVelocity,
            double[] wheelForce,
            ModuleCommands commands);

    /**
     * Main thread: queue another config apply for any device that has not taken its config yet.
     * Non-blocking (the applies run in the background).
     */
    default void retryFailedConfigs() {}

    /** Swerve thread: all motors neutral. */
    void setNeutral();

    /**
     * Main thread: drive motors brake (true) or coast at neutral. Applying it is a blocking config
     * call, so implementations do it on their own thread; call only on a change.
     */
    default void setDriveBrake(boolean brake) {}

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
                public void setModules(
                        double[] angle,
                        double[] velocity,
                        double[] steerVelocity,
                        double[] wheelForce,
                        ModuleCommands commands) {}

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
