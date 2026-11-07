package frc.spectrumSim.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.spectrumLib.vision.VisionFrame;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.wpilib.fields.Field;
import org.wpilib.fields.FieldTag;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.networktables.NetworkTableInstance;

class CameraSimTest {
    @Test
    void aTagStraightAheadIsSeenAtItsRangeAndBearing() {
        HAL.initialize();
        // One tag 3 m ahead of the robot, 0.3 m up (camera height), facing the robot.
        var tag = new FieldTag(5, new Pose3d(5, 2, 0.3, new Rotation3d(0, 0, Math.PI)));
        var field = new Field("test", "", "", null, 10, 5, "FRC", List.of(tag));
        var model = new CameraSim.Model(1280, 800, 915, 915, 640, 400, 50, 0, 0);
        var cam =
                new CameraSim(
                        "simtest",
                        new Transform3d(0, 0, 0.3, new Rotation3d()),
                        model,
                        field,
                        () -> new Pose2d(2, 2, new org.wpilib.math.geometry.Rotation2d()));
        var sub =
                NetworkTableInstance.getDefault()
                        .getRawTopic(VisionFrame.topic("simtest"))
                        .subscribe(VisionFrame.TYPE, new byte[0]);
        cam.step(0.02);
        byte[] bytes = sub.get();
        var f = new VisionFrame();
        assertTrue(f.decode(bytes, 0, bytes.length) > 0);
        assertEquals(1, f.count);
        assertEquals(5, f.id[0]);
        assertEquals(0, f.yaw[0], 1e-6);
        assertEquals(0, f.pitch[0], 1e-6);
        assertEquals(3, f.range[0], 0.1); // range noise: 0.05 m at 3 m
        // Corners: a 0.1651 m tag at 3 m spans about 50 px, centered on the image.
        assertEquals(640 - 915 * 0.1651 / 2 / 3, f.corners[0], 1e-3);
    }
}
