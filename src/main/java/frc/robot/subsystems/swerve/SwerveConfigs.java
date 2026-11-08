package frc.robot.subsystems.swerve;

import frc.robot.config.Hardware;
import frc.robot.config.ImuSettings;
import frc.spectrumLib.swerve.DriveMotorLimit;
import frc.spectrumLib.swerve.PhoenixSwerveConfig;
import org.littletonrobotics.junction.Logger;

/** Builds the Phoenix drivetrain configuration from {@link Hardware} and the Robot Settings. */
final class SwerveConfigs {
    private SwerveConfigs() {}

    static PhoenixSwerveConfig phoenix(int modules) {
        int[] drive = new int[modules];
        int[] steer = new int[modules];
        int[] encoder = new int[modules];
        double[] offsets = new double[modules];
        for (Hardware.Module m : Hardware.Module.values()) {
            int i = m.ordinal();
            drive[i] = m.driveId;
            steer[i] = m.steerId;
            encoder[i] = m.encoderId;
            offsets[i] = SwerveSettings.MAGNET_OFFSET[i].get();
        }
        return new PhoenixSwerveConfig(
                Hardware.CAN_BUS,
                Hardware.PIGEON_ID,
                drive,
                steer,
                encoder,
                offsets,
                Hardware.driveInverted(),
                Hardware.STEER_INVERTED,
                Hardware.MK4iL2.DRIVE_RATIO,
                Hardware.MK4iL2.STEER_RATIO,
                Hardware.MK4iL2.COUPLE_RATIO,
                SwerveSettings.WHEEL_RADIUS.get(),
                driveSlipCurrent(),
                Hardware.STEER_STATOR_LIMIT_AMPS,
                new PhoenixSwerveConfig.Gains(
                        SwerveSettings.DRIVE_KP.get(), 0, 0, SwerveSettings.DRIVE_KS.get(), 0, 0),
                SwerveSettings.steerVoltage()
                        ? new PhoenixSwerveConfig.Gains(
                                SwerveSettings.STEER_VOLTAGE_KP.get(),
                                0,
                                SwerveSettings.STEER_VOLTAGE_KD.get(),
                                SwerveSettings.STEER_VOLTAGE_KS.get(),
                                SwerveSettings.STEER_VOLTAGE_KV.get(),
                                0)
                        : new PhoenixSwerveConfig.Gains(
                                SwerveSettings.STEER_TORQUE_KP.get(),
                                0,
                                SwerveSettings.STEER_TORQUE_KD.get(),
                                SwerveSettings.STEER_TORQUE_KS.get(),
                                0,
                                0),
                SwerveSettings.LOOP_HZ.get(),
                new double[] {
                    ImuSettings.PIGEON_MOUNT_YAW.get(),
                    ImuSettings.PIGEON_MOUNT_PITCH.get(),
                    ImuSettings.PIGEON_MOUNT_ROLL.get()
                },
                SwerveSettings.STEER_MOTION_MAGIC_EXPO.get(),
                SwerveSettings.STEER_EXPO_KV.get(),
                SwerveSettings.STEER_EXPO_KA.get(),
                Hardware.MK4iL2.DRIVE_KT,
                SwerveSettings.TIMESYNC.get(),
                SwerveSettings.steerVoltage(),
                Hardware.DRIVE_SUPPLY_LIMIT_AMPS,
                Hardware.STEER_SUPPLY_LIMIT_AMPS);
    }

    /**
     * Most acceleration the tires can give before sliding (m/s^2): friction times g. (The drive
     * rotors' inertia does not lower this; it costs extra motor current, in {@link
     * #driveSlipCurrent}.)
     */
    static double tractionLimit() {
        return SwerveSettings.TIRE_FRICTION.get() * GRAVITY;
    }

    /**
     * Drive current at which a wheel would start to slip while the robot accelerates at the
     * traction limit (A): the tire's share of friction times weight through the gearing, plus what
     * spins up the motor's own rotor at that acceleration, over the motor's torque constant.
     */
    static double slipCurrentAtLimit() {
        double m = SwerveSettings.robotMassKg();
        double r = SwerveSettings.WHEEL_RADIUS.get();
        double gear = Hardware.MK4iL2.DRIVE_RATIO;
        double tireForce = SwerveSettings.TIRE_FRICTION.get() * m * GRAVITY / 4;
        double accel = tractionLimit();
        double torque = tireForce * r / gear + FALCON_ROTOR_INERTIA * (accel / r) * gear;
        return torque / Hardware.MK4iL2.DRIVE_KT;
    }

    /**
     * The drive slip current used (A): tractionUse of the computed slip current, never above the
     * hardware cap. HARD STOP (current limit): set robotMass and tireFriction truthfully.
     */
    static double driveSlipCurrent() {
        return Math.min(
                Hardware.DRIVE_SLIP_CURRENT_AMPS,
                SwerveSettings.TRACTION_USE.get() * slipCurrentAtLimit());
    }

    /** Acceleration the setpoint generator may plan (m/s^2): tractionUse of the traction limit. */
    static double plannedAccelerationLimit() {
        return Math.min(
                SwerveSettings.MAX_DRIVE_ACCELERATION.get(),
                SwerveSettings.TRACTION_USE.get() * tractionLimit());
    }

    /** Each drive motor's speed-up limit for the setpoint generator, or null when turned off. */
    static DriveMotorLimit motorLimit(int wheels) {
        if (!SwerveSettings.MOTOR_ACCELERATION_LIMIT.get()) {
            return null;
        }
        return new DriveMotorLimit(
                Hardware.MK4iL2.DRIVE_STALL_TORQUE,
                Hardware.MK4iL2.DRIVE_STALL_CURRENT,
                Hardware.MK4iL2.DRIVE_FREE_RPM,
                Hardware.MK4iL2.DRIVE_RATIO,
                SwerveSettings.WHEEL_RADIUS.get(),
                driveSlipCurrent(),
                Hardware.DRIVE_SUPPLY_LIMIT_AMPS,
                SwerveSettings.robotMassKg(),
                wheels);
    }

    private static final double GRAVITY = 9.81;

    /** Falcon 500 rotor inertia (kg m^2). */
    private static final double FALCON_ROTOR_INERTIA = 0.0000565;

    /** Logs the traction numbers once at boot. */
    static void logTraction() {
        // Robot weight used (lb).
        Logger.recordOutput(
                "Swerve/Traction/RobotWeightLbs", SwerveSettings.ROBOT_WEIGHT_LBS.get());
        // Traction limit, friction x g (m/s^2), and the drive current at the slip point (A).
        Logger.recordOutput("Swerve/Traction/Limit", tractionLimit());
        Logger.recordOutput("Swerve/Traction/SlipCurrentAtLimit", slipCurrentAtLimit());
        // What is used: tractionUse of each (planned acceleration m/s^2, drive slip current A).
        Logger.recordOutput("Swerve/Traction/PlannedAcceleration", plannedAccelerationLimit());
        Logger.recordOutput("Swerve/Traction/DriveSlipCurrent", driveSlipCurrent());
    }
}
