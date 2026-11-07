package frc.robot.subsystems.swerve;

import frc.robot.config.Hardware;
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

    static final DoubleSetting WHEEL_RADIUS =
            G.number("wheelRadius", Hardware.MK4iL2.NOMINAL_WHEEL_RADIUS_METERS, "m", 0.04, 0.06);

    /** Module spacing, center to center, front to back and side to side (square robot). */
    static final DoubleSetting TRACK =
            G.number("track", Hardware.NOMINAL_TRACK_METERS, "m", 0.2, 1);

    /** Top wheel speed the thread will command (m/s). L2 Falcon FOC free speed is ~4.8. */
    static final DoubleSetting MAX_WHEEL_VELOCITY =
            G.number("maxWheelVelocity", 4.5, "m/s", 0.5, 5.0);

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

    /** Robot mass for the physics sim (kg). */
    static final DoubleSetting SIM_MASS = G.number("simMass", 18.0, "kg", 5, 80);
}
