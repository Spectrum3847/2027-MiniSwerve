package frc.robot.subsystems.swerve;

import static org.wpilib.units.Units.Kilograms;
import static org.wpilib.units.Units.Pounds;

import frc.robot.config.Hardware;
import frc.spectrumLib.robotsettings.BooleanSetting;
import frc.spectrumLib.robotsettings.DoubleSetting;
import frc.spectrumLib.robotsettings.RobotSettings;
import frc.spectrumLib.robotsettings.SettingGroup;
import frc.spectrumLib.robotsettings.StringSetting;

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

    /**
     * Largest wheel speed change the setpoint generator plans (m/s^2). Default 30: above anything
     * the robot can do, so the motors always get full power. Traction is protected by the drive
     * slip current (stator and peak torque current at the point the wheels slip), not by this.
     * Lower it only for a gentler drive.
     */
    static final DoubleSetting MAX_DRIVE_ACCELERATION =
            G.number("maxDriveAcceleration", 30.0, "m/s^2", 1, 60);

    /**
     * Also hold each wheel's planned speed-up to what its motor can give at its speed and the
     * battery voltage (back-EMF, slip and supply current limits), so plans near top speed or on a
     * sagging battery are ones the motors can follow.
     */
    static final BooleanSetting MOTOR_ACCELERATION_LIMIT = G.bool("motorAccelerationLimit", true);

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

    /** Heading hold captures after the yaw rate stays below headingHoldSettleRate this long (s). */
    static final DoubleSetting HEADING_HOLD_SETTLE_SECONDS =
            G.number("headingHoldSettleSeconds", 0.25, "s", 0, 2);

    /** Yaw rate below which the robot counts as settled for heading hold (rad/s). */
    static final DoubleSetting HEADING_HOLD_SETTLE_RATE =
            G.number("headingHoldSettleRate", 0.1, "rad/s", 0, 2);

    /** Heading hold corrects only above this commanded translation (m/s). */
    static final DoubleSetting HEADING_HOLD_MIN_TRANSLATION =
            G.number("headingHoldMinTranslation", 0.2, "m/s", 0, 2);

    /**
     * Use Phoenix timesync when the bus is a CANivore. Off to compare latency; ignored on a plain
     * CAN bus.
     */
    static final BooleanSetting TIMESYNC = G.bool("timesync", true);

    /**
     * Coast the drive motors once the robot has been disabled and still for coastDelaySeconds, so
     * it can be pushed; brake while enabled and while still moving. HARD STOP (brake/coast).
     */
    static final BooleanSetting COAST_ON_DISABLE = G.bool("coastOnDisable", true);

    /** Seconds disabled and still before coasting. */
    static final DoubleSetting COAST_DELAY_SECONDS = G.number("coastDelaySeconds", 0.5, "s", 0, 10);

    /** Spin rate for the wheel-radius measurement (rad/s). */
    static final DoubleSetting WHEEL_RADIUS_SPIN =
            G.number("wheelRadiusSpin", 1.5, "rad/s", 0.3, 4);

    /**
     * How fast the setpoint generator changes a module's turn rate (rad/s^2): the steer plan speeds
     * up and slows down at this rate so the wheels land on their angle without overshooting.
     */
    static final DoubleSetting MAX_STEER_ACCELERATION =
            G.number("maxSteerAcceleration", 300, "rad/s^2", 20, 1000);

    /**
     * Fraction of the computed traction to use (just under 1): the drive slip current and the
     * planned acceleration are this much of the computed slip point. Raise toward 1 once the real
     * slip point is measured; lower if the wheels spin.
     */
    static final DoubleSetting TRACTION_USE = G.number("tractionUse", 0.97, "", 0.5, 1.0);

    /** Driver top speed and turn rate. */
    static final DoubleSetting MAX_DRIVE_VELOCITY =
            G.number("maxDriveVelocity", 4.0, "m/s", 0.5, 5.0);

    static final DoubleSetting MAX_ANGULAR_VELOCITY =
            G.number("maxAngularVelocity", 8.0, "rad/s", 1, 15);

    static final DoubleSetting DRIVE_KP = G.number("driveKP", 10.0, "A/rps", 0, 100);
    static final DoubleSetting DRIVE_KS = G.number("driveKS", 3.0, "A", 0, 20);

    /**
     * Steer closed-loop output, CTRE's ClosedLoopOutputType names: "Voltage" (PositionVoltage with
     * FOC; back-EMF damps it, CTRE's default and the most common choice) or "TorqueCurrentFOC".
     * Each has its own gains below.
     */
    static final StringSetting STEER_CLOSED_LOOP_OUTPUT =
            G.string("steerClosedLoopOutput", "Voltage");

    /** Steer gains for Voltage output: CTRE's generated MK4i defaults. */
    static final DoubleSetting STEER_VOLTAGE_KP =
            G.number("steerVoltageKP", 100.0, "V/rot", 0, 500);

    static final DoubleSetting STEER_VOLTAGE_KD = G.number("steerVoltageKD", 0.5, "V/rps", 0, 20);
    static final DoubleSetting STEER_VOLTAGE_KS = G.number("steerVoltageKS", 0.1, "V", 0, 2);
    static final DoubleSetting STEER_VOLTAGE_KV = G.number("steerVoltageKV", 2.66, "V/rps", 0, 10);

    /** Steer gains for TorqueCurrentFOC output. */
    static final DoubleSetting STEER_TORQUE_KP =
            G.number("steerTorqueKP", 1000.0, "A/rot", 0, 5000);

    static final DoubleSetting STEER_TORQUE_KD = G.number("steerTorqueKD", 30.0, "A/rps", 0, 500);
    static final DoubleSetting STEER_TORQUE_KS = G.number("steerTorqueKS", 2.0, "A", 0, 20);

    static boolean steerVoltage() {
        return !"TorqueCurrentFOC".equals(STEER_CLOSED_LOOP_OUTPUT.get());
    }

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

    /**
     * The robot's weight ready to play, battery and bumpers on (lb). Weigh it. Sets the drive slip
     * current and the sim's physics (with tireFriction). Placeholder until weighed.
     */
    static final DoubleSetting ROBOT_WEIGHT_LBS = G.number("robotWeightLbs", 55.0, "lb", 10, 175);

    /** {@link #ROBOT_WEIGHT_LBS} in kilograms. */
    static double robotMassKg() {
        return Pounds.of(ROBOT_WEIGHT_LBS.get()).in(Kilograms);
    }

    /**
     * Tire-to-carpet friction coefficient for this tread (about 1.1 for typical FRC tread on
     * carpet; lower for worn or hard tread). With robotMass it sets how hard the robot can
     * accelerate before the wheels slide.
     */
    static final DoubleSetting TIRE_FRICTION = G.number("tireFriction", 1.1, "", 0.3, 2.0);

    /**
     * Height of the robot's center of gravity above the floor, ready to play (in). Sets weight
     * transfer in the sim: how much grip the front wheels lose under hard acceleration and when the
     * robot would tip. Placeholder until measured (balance the robot on an edge, or tilt it until
     * it tips and use the angle).
     */
    static final DoubleSetting COG_HEIGHT_INCHES = G.number("cogHeightInches", 6.0, "in", 1, 40);

    /**
     * Bumper outline, front to back, bumpers on (in). Sets where the sim robot hits walls and other
     * robots. Placeholder until measured.
     */
    static final DoubleSetting BUMPER_LENGTH_INCHES =
            G.number("bumperLengthInches", 22.0, "in", 10, 40);

    /** Bumper outline, side to side, bumpers on (in). Placeholder until measured. */
    static final DoubleSetting BUMPER_WIDTH_INCHES =
            G.number("bumperWidthInches", 22.0, "in", 10, 40);
}
