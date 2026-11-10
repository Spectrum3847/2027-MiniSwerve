package frc.spectrumSim.swerve;

/**
 * What {@link SwervePhysicsSim} moves: the simulated motors, encoders and gyro. Either Phoenix's
 * own simulator ({@link PhoenixSimDevices}, wall clock) or our TalonFX models ({@link SwerveIOSim},
 * sim clock, repeatable). All values are in the mechanism's direction (after inverts).
 */
interface SwerveSimDevices {
    /**
     * Drive motor stator current asked for now (A), before battery and supply limits.
     *
     * @param driveMotorOmega drive rotor speed (rad/s), including the steer coupling
     */
    double driveAmps(int i, double driveMotorOmega, double batteryVolts, double dt);

    /** Steer motor stator current asked for now (A), before battery and supply limits. */
    double steerAmps(int i, double steerMotorOmega, double batteryVolts, double dt);

    /**
     * Writes one module's motion to its sensors.
     *
     * @param steerRot module angle (rotations)
     * @param steerOmega module turn rate (rad/s)
     * @param driveRotorRot drive rotor position (rotations), including the steer coupling
     * @param driveMotorOmega drive rotor speed (rad/s)
     * @param encoderAlive the CANcoder has power (an unplugged one stops updating)
     */
    void sense(
            int i,
            double steerRot,
            double steerOmega,
            double driveRotorRot,
            double driveMotorOmega,
            double batteryVolts,
            boolean encoderAlive);

    /** Writes the robot's heading (rad) and turn rate (rad/s) to the gyro. */
    void senseGyro(double heading, double omega, double roll, double pitch, double batteryVolts);
}
