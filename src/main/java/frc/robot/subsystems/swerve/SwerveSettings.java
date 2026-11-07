package frc.robot.subsystems.swerve;

import frc.robot.config.Hardware;
import frc.spectrumLib.robotsettings.BooleanSetting;
import frc.spectrumLib.robotsettings.DoubleSetting;
import frc.spectrumLib.robotsettings.RobotSettings;
import frc.spectrumLib.robotsettings.SettingGroup;

/**
 * The drivetrain's Robot Settings: calibration and gains, in {@code deploy/settings/swerve.json}.
 * Gains and calibration are read when the drivetrain is built (boot); a change needs a reboot.
 *
 * <p>Gains are TorqueCurrentFOC starting points for sim; tune on the robot.
 */
final class SwerveSettings {
    private SwerveSettings() {}

    private static final SettingGroup G = RobotSettings.group("swerve");

    /** CANcoder MagnetOffset per module (rotations). HARD STOP: calibrate on the robot. */
    static final DoubleSetting[] MAGNET_OFFSET = {
        G.number("magnetOffsetFL", 0.0, "rot", -1, 1),
        G.number("magnetOffsetFR", 0.0, "rot", -1, 1),
        G.number("magnetOffsetBL", 0.0, "rot", -1, 1),
        G.number("magnetOffsetBR", 0.0, "rot", -1, 1),
    };

    /**
     * Swerve thread and signal rate (Hz), read at boot. 250 on a CANivore; one CAN 2.0 bus with 8
     * Talons, 4 CANcoders and a Pigeon may need 100-200 (watch bus utilization).
     */
    static final DoubleSetting LOOP_HZ = G.number("loopHz", 250, "Hz", 50, 250);

    static final DoubleSetting WHEEL_RADIUS =
            G.number("wheelRadius", Hardware.MK4iL2.NOMINAL_WHEEL_RADIUS_METERS, "m", 0.04, 0.06);

    /** Module spacing, center to center, front to back and side to side (square robot). */
    static final DoubleSetting TRACK =
            G.number("track", Hardware.NOMINAL_TRACK_METERS, "m", 0.2, 1);

    /** Top wheel speed the thread will command (m/s). L2 Falcon FOC free speed is ~4.8. */
    static final DoubleSetting MAX_WHEEL_VELOCITY =
            G.number("maxWheelVelocity", 4.5, "m/s", 0.5, 5.0);

    /** Limit acceleration and steer rate with the setpoint generator (smoother, truer odometry). */
    static final BooleanSetting SETPOINT_GENERATOR = G.bool("setpointGenerator", true);

    /** Largest wheel speed change the setpoint generator allows (m/s^2). */
    static final DoubleSetting MAX_DRIVE_ACCELERATION =
            G.number("maxDriveAcceleration", 10.0, "m/s^2", 1, 30);

    /** Fastest module turn the setpoint generator plans (rad/s); MK4i free speed is about 29. */
    static final DoubleSetting MAX_STEER_VELOCITY =
            G.number("maxSteerVelocity", 20.0, "rad/s", 2, 30);

    /**
     * Field-relative lookahead (s): aim with heading + yaw rate * this, so driving while spinning
     * goes straight despite module lag. Tune on the robot by driving straight while spinning.
     */
    static final DoubleSetting ROTATION_LOOKAHEAD =
            G.number("rotationLookahead", 0.04, "s", 0.0, 0.15);

    /**
     * Hold heading while the driver is not turning (see HeadingHold: captures after the robot
     * settles, only corrects while driving, lets go after a hit). Off: drivers have disliked
     * heading hold that steers back after contact; try this one on the robot.
     */
    static final BooleanSetting HEADING_HOLD = G.bool("headingHold", false);

    /** Heading hold gain (rad/s per rad of error). */
    static final DoubleSetting HEADING_HOLD_KP = G.number("headingHoldKP", 3.0, "1/s", 0, 20);

    /** Heading hold's largest correction (rad/s): enough to trim drift, not to steer. */
    static final DoubleSetting HEADING_HOLD_MAX_CORRECTION =
            G.number("headingHoldMaxCorrection", 1.0, "rad/s", 0, 5);

    /** Error past which heading hold accepts the new heading instead of correcting (deg). */
    static final DoubleSetting HEADING_HOLD_RELEASE_DEG =
            G.number("headingHoldReleaseDeg", 10.0, "deg", 1, 90);

    /** Driver top speed and turn rate. */
    static final DoubleSetting MAX_DRIVE_VELOCITY =
            G.number("maxDriveVelocity", 4.0, "m/s", 0.5, 5.0);

    static final DoubleSetting MAX_ANGULAR_VELOCITY =
            G.number("maxAngularVelocity", 8.0, "rad/s", 1, 15);

    static final DoubleSetting DRIVE_KP = G.number("driveKP", 10.0, "A/rps", 0, 100);
    static final DoubleSetting DRIVE_KS = G.number("driveKS", 3.0, "A", 0, 20);
    static final DoubleSetting STEER_KP = G.number("steerKP", 1000.0, "A/rot", 0, 5000);
    static final DoubleSetting STEER_KD = G.number("steerKD", 30.0, "A/rps", 0, 500);
    static final DoubleSetting STEER_KS = G.number("steerKS", 2.0, "A", 0, 20);

    /**
     * Steer with MotionMagicExpoTorqueCurrentFOC (a smooth profile: less current and scrub) instead
     * of PositionTorqueCurrentFOC. Off until tested on the robot.
     */
    static final BooleanSetting STEER_MOTION_MAGIC_EXPO = G.bool("steerMotionMagicExpo", false);

    /** Steer MotionMagicExpo_kV (V per module rps); CTRE's default is 0.12 * steer ratio. */
    static final DoubleSetting STEER_EXPO_KV =
            G.number("steerExpoKv", 0.12 * Hardware.MK4iL2.STEER_RATIO, "V/rps", 0.1, 20);

    /** Steer MotionMagicExpo_kA (V per module rps/s); CTRE's default is 1.2 / steer ratio. */
    static final DoubleSetting STEER_EXPO_KA =
            G.number("steerExpoKa", 1.2 / Hardware.MK4iL2.STEER_RATIO, "V/(rps/s)", 0.001, 5);

    /** Robot mass for the physics sim (kg). */
    static final DoubleSetting SIM_MASS = G.number("simMass", 18.0, "kg", 5, 80);
}
