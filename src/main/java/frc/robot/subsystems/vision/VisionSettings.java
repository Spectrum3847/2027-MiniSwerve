package frc.robot.subsystems.vision;

import frc.robot.RunMode;
import frc.spectrumLib.robotsettings.BooleanSetting;
import frc.spectrumLib.robotsettings.DoubleSetting;
import frc.spectrumLib.robotsettings.RobotSettings;
import frc.spectrumLib.robotsettings.SettingGroup;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;

/**
 * Cameras: which are on the robot and where each is mounted. Mounts are calibration (Robot
 * Settings, logged), measured from the robot's center on the floor: x forward, y left, z up, then
 * roll, pitch (up is negative in WPILib: nose-up is a negative pitch), yaw. MiniSwerve has no
 * cameras yet: they are off on the robot and on in sim, at placeholder mounts.
 */
public final class VisionSettings {
    private VisionSettings() {}

    /**
     * The field: where the AprilTags are. The 2026 AndyMark field (what the coprocessor uses by
     * default) until the 2027 field is released.
     */
    public static final org.wpilib.fields.Fields FIELD =
            org.wpilib.fields.Fields.FRC_2026_REBUILT_ANDY_MARK;

    /** Camera names: their NetworkTables topics and log keys. */
    private static final String[] CAMERAS = {"front", "back"};

    /** One camera's settings. */
    public record Camera(
            String name,
            BooleanSetting enabled,
            DoubleSetting x,
            DoubleSetting y,
            DoubleSetting z,
            DoubleSetting rollDeg,
            DoubleSetting pitchDeg,
            DoubleSetting yawDeg) {
        public Transform3d robotToCamera() {
            return new Transform3d(
                    x.get(),
                    y.get(),
                    z.get(),
                    new Rotation3d(
                            Math.toRadians(rollDeg.get()),
                            Math.toRadians(pitchDeg.get()),
                            Math.toRadians(yawDeg.get())));
        }
    }

    /** Builds each camera's settings (boot, after RobotSettings.start). */
    public static Camera[] cameras() {
        Camera[] out = new Camera[CAMERAS.length];
        double[][] mount = { // placeholders: x, y, z (m), roll, pitch, yaw (deg)
            {0.25, 0, 0.30, 0, -15, 0}, {-0.25, 0, 0.30, 0, -15, 180}
        };
        for (int i = 0; i < CAMERAS.length; i++) {
            SettingGroup g = RobotSettings.group("vision." + CAMERAS[i]);
            double[] m = mount[i];
            out[i] =
                    new Camera(
                            CAMERAS[i],
                            // The camera is mounted and sending frames (off on the robot until
                            // cameras are mounted; on in sim).
                            g.bool("enabled", RunMode.CURRENT != RunMode.Mode.REAL),
                            g.number("x", m[0], "m", -1, 1),
                            g.number("y", m[1], "m", -1, 1),
                            g.number("z", m[2], "m", 0, 1.5),
                            g.number("rollDeg", m[3], "deg", -180, 180),
                            g.number("pitchDeg", m[4], "deg", -90, 90),
                            g.number("yawDeg", m[5], "deg", -180, 180));
        }
        return out;
    }
}
