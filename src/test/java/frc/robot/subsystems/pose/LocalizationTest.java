package frc.robot.subsystems.pose;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.spectrumLib.localization.TagSighting;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;

class LocalizationTest {
    /** What a camera on a robot at (x, y, heading) would measure to each tag. */
    private static List<TagSighting> seen(
            double x, double y, double heading, Transform3d mount, Translation3d... tags) {
        Pose3d camera = new Pose3d(x, y, 0, new Rotation3d(0, 0, heading)).transformBy(mount);
        List<TagSighting> out = new ArrayList<>();
        int id = 1;
        for (Translation3d tag : tags) {
            Translation3d inCamera =
                    tag.minus(camera.getTranslation()).rotateBy(camera.getRotation().inverse());
            out.add(new TagSighting("c", 0, id++, tag, mount, inCamera, 1));
        }
        return out;
    }

    @Test
    void twoTagsGiveThePose() {
        var mount = new Transform3d(0.25, 0.1, 0.3, new Rotation3d(0, Math.toRadians(-15), 0.2));
        var pose =
                Localization.seedFrom(
                        seen(
                                4,
                                3,
                                0.7,
                                mount,
                                new Translation3d(7, 5, 1.1),
                                new Translation3d(6, 7, 0.9),
                                new Translation3d(8, 3, 1.2)));
        assertEquals(4, pose.getX(), 1e-9);
        assertEquals(3, pose.getY(), 1e-9);
        assertEquals(0.7, pose.getRotation().getRadians(), 1e-9);
    }
}
