package frc.robot.config;

import frc.spectrumLib.robotsettings.DoubleSetting;
import frc.spectrumLib.robotsettings.RobotSettings;
import frc.spectrumLib.robotsettings.SettingGroup;
import frc.spectrumLib.robotsettings.StringSetting;

/**
 * How each gyro is mounted, in {@code deploy/settings/imu.json}. Read at boot. Names follow the
 * devices' own config: Phoenix's {@code MountPoseYaw/Pitch/Roll} for the Pigeon 2, WPILib's {@code
 * OnboardIMU.MountOrientation} for SystemCore's IMU (imitated on this robot by a USB gyro).
 */
public final class ImuSettings {
    private ImuSettings() {}

    private static final SettingGroup G = RobotSettings.group("imu");

    /** Pigeon 2 MountPoseYaw (deg): rotation of the Pigeon from robot-forward, CCW positive. */
    public static final DoubleSetting PIGEON_MOUNT_YAW =
            G.number("pigeonMountPoseYaw", 0.0, "deg", -180, 180);

    /** Pigeon 2 MountPosePitch (deg). */
    public static final DoubleSetting PIGEON_MOUNT_PITCH =
            G.number("pigeonMountPosePitch", 0.0, "deg", -90, 90);

    /** Pigeon 2 MountPoseRoll (deg). */
    public static final DoubleSetting PIGEON_MOUNT_ROLL =
            G.number("pigeonMountPoseRoll", 0.0, "deg", -180, 180);

    /** SystemCore IMU mount: FLAT, LANDSCAPE or PORTRAIT ({@code OnboardIMU.MountOrientation}). */
    public static final StringSetting SYSTEMCORE_MOUNT_ORIENTATION =
            G.string("systemCoreMountOrientation", "FLAT");

    /** SystemCore IMU yaw offset from robot-forward after the mount orientation (deg). */
    public static final DoubleSetting SYSTEMCORE_MOUNT_YAW =
            G.number("systemCoreMountYaw", 0.0, "deg", -180, 180);
}
