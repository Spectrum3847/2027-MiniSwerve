package frc.robot.subsystems.swerve;

import frc.robot.config.Hardware;
import frc.robot.config.ImuSettings;
import frc.spectrumLib.swerve.PhoenixSwerveConfig;

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
                Hardware.DRIVE_SLIP_CURRENT_AMPS,
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
                SwerveSettings.steerVoltage());
    }
}
