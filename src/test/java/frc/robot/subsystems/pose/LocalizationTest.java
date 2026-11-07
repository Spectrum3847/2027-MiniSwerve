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
    void tagsTooCloseOrMisidentifiedAreNotTrusted() {
        var mount = new Transform3d(0.25, 0, 0.3, new Rotation3d());
        // Two tags 0.3 m apart: too short a baseline to trust the heading.
        var close = seen(4, 3, 0, mount, new Translation3d(7, 3, 1), new Translation3d(7, 3.3, 1));
        assertEquals("tags closer than 1 m", Localization.solve(close).check());
        // One tag misidentified: the field puts it 0.5 m from where the camera saw it.
        var s =
                seen(
                        4,
                        3,
                        0,
                        mount,
                        new Translation3d(7, 1, 1),
                        new Translation3d(7, 5, 1),
                        new Translation3d(8, 3, 1));
        var bad = s.get(2);
        s.set(
                2,
                new TagSighting(
                        "c",
                        0,
                        9,
                        new Translation3d(8.5, 3, 1),
                        bad.robotToCamera(),
                        bad.cameraToTag(),
                        1));
        var r = Localization.solve(s);
        assertEquals(null, r.pose(), r.check());
    }

    @Test
    void twoTagsGiveThePose() {
        var mount = new Transform3d(0.25, 0.1, 0.3, new Rotation3d(0, Math.toRadians(-15), 0.2));
        var pose =
                Localization.solve(
                        seen(
                                4,
                                3,
                                0.7,
                                mount,
                                new Translation3d(7, 5, 1.1),
                                new Translation3d(6, 7, 0.9),
                                new Translation3d(8, 3, 1.2)));
        assertEquals("ok", pose.check());
        assertEquals(4, pose.pose().getX(), 1e-9);
        assertEquals(3, pose.pose().getY(), 1e-9);
        assertEquals(0.7, pose.pose().getRotation().getRadians(), 1e-9);
    }
}
