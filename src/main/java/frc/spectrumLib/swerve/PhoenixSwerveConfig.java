package frc.spectrumLib.swerve;

/**
 * Everything {@link SwerveIOPhoenix} needs to build a drivetrain of TalonFX drive and steer motors,
 * CANcoders and a Pigeon 2. Arrays are per module, FL FR BL BR. Units in each name's javadoc;
 * Phoenix's own config names are used where they exist.
 *
 * @param canBus Phoenix CAN bus name ("" for the controller's built-in bus)
 * @param pigeonId Pigeon 2 CAN ID
 * @param driveIds drive TalonFX CAN IDs
 * @param steerIds steer TalonFX CAN IDs
 * @param encoderIds CANcoder IDs
 * @param magnetOffsets CANcoder MagnetOffset per module (rotations)
 * @param driveInverted drive motor inverted (Clockwise_Positive) per module
 * @param steerInverted steer motors inverted (Clockwise_Positive)
 * @param driveRatio drive motor rotations per wheel rotation
 * @param steerRatio steer motor rotations per module rotation
 * @param coupleRatio drive motor rotations per module rotation, from the steer gearing
 * @param wheelRadius wheel radius (m)
 * @param driveStatorCurrentLimit amps
 * @param steerStatorCurrentLimit amps
 * @param drive drive Slot0 gains, TorqueCurrentFOC units (A per wheel rps, A)
 * @param steer steer Slot0 gains, TorqueCurrentFOC units (A per rotation of error, A)
 * @param signalHz update rate of the signals the swerve thread reads
 * @param pigeonMountPose Pigeon 2 MountPoseYaw, MountPosePitch, MountPoseRoll (deg)
 */
public record PhoenixSwerveConfig(
        String canBus,
        int pigeonId,
        int[] driveIds,
        int[] steerIds,
        int[] encoderIds,
        double[] magnetOffsets,
        boolean[] driveInverted,
        boolean steerInverted,
        double driveRatio,
        double steerRatio,
        double coupleRatio,
        double wheelRadius,
        double driveStatorCurrentLimit,
        double steerStatorCurrentLimit,
        Gains drive,
        Gains steer,
        double signalHz,
        double[] pigeonMountPose) {

    /** Phoenix Slot0 gains: kP, kI, kD, kS, kV, kA. */
    public record Gains(double kP, double kI, double kD, double kS, double kV, double kA) {}

    public int modules() {
        return driveIds.length;
    }
}
