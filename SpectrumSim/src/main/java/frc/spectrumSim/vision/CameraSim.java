package frc.spectrumSim.vision;

import frc.spectrumLib.vision.VisionFrame;
import java.util.ArrayDeque;
import java.util.List;
import java.util.Random;
import java.util.function.Supplier;
import org.wpilib.fields.Field;
import org.wpilib.fields.FieldTag;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.RawPublisher;
import org.wpilib.system.RobotController;

/**
 * A camera on the sim robot, sending {@link VisionFrame}s as Spectrum's PhotonVision fork does, on
 * the same NetworkTables topic, so the robot's real vision IO reads them.
 *
 * <p>Each frame: the field's AprilTags seen from the robot's true pose through a pinhole camera
 * ({@link Model}); a tag counts if it faces the camera (under 75 deg off its face), is within 8 m,
 * and all four corners land on the image. Corners get pixel noise; the bearing comes from the noisy
 * corners' center, as the coprocessor computes it, and range gets noise growing with distance
 * (0.005 + 0.005 r^2 m, as measured on the Jetson). Frames are exposed at the camera's frame rate
 * and arrive after its latency, all on the sim clock, with a seeded noise source: runs repeat.
 *
 * <p>No lens distortion, motion blur or occlusion yet. The bearing uses the corners' mean, which
 * sits a little off the true center at oblique angles, as the fork's will if it does the same.
 */
public final class CameraSim {
    /** AprilTag side (m): 6.5 in, the FRC tag's black square. */
    private static final double TAG_SIZE = 0.1651;

    /**
     * Corners in the tag's frame, OpenCV order as seen from the front: bottom-left, bottom-right,
     * top-right, top-left.
     */
    private static final double[] CORNER_Y = {
        -TAG_SIZE / 2, TAG_SIZE / 2, TAG_SIZE / 2, -TAG_SIZE / 2
    };

    private static final double[] CORNER_Z = {
        -TAG_SIZE / 2, -TAG_SIZE / 2, TAG_SIZE / 2, TAG_SIZE / 2
    };

    private static final double MAX_RANGE = 8.0;
    private static final double MAX_FACE_ANGLE = Math.toRadians(75);

    /** A camera's lens and timing. */
    public record Model(
            int width,
            int height,
            double fx,
            double fy,
            double cx,
            double cy,
            double fps,
            double latencySeconds, // median capture-to-robot; the spread is FM's measured one
            double pixelNoise) {
        /**
         * Arducam/Thriftiest Cam at 1280x800, about 70 deg across, 60 fps; latency as measured on
         * FM's Orin (median 15.7 ms, spread from the logs).
         */
        public static Model thriftiestCam() {
            return new Model(1280, 800, 915, 915, 640, 400, 60, 0.0157, 0.3);
        }
    }

    private final String name;
    private final Transform3d robotToCamera;
    private final Model model;
    private final int[] tagId;
    private final Pose3d[] tagPose;
    private final Supplier<Pose2d> truePose;
    private final RawPublisher publisher;
    private final Random random;
    private final ArrayDeque<Pending> inFlight = new ArrayDeque<>();
    private final VisionFrame frame = new VisionFrame();
    private final double[] u = new double[4];
    private final double[] v = new double[4];
    private long sequence;
    private double untilCapture;

    private record Pending(long dueNanos, byte[] bytes) {}

    /**
     * @param truePose the robot's true pose on the field (sim truth), read on the sim clock's
     *     thread
     */
    public CameraSim(
            String name,
            Transform3d robotToCamera,
            Model model,
            Field field,
            Supplier<Pose2d> truePose) {
        this.name = name;
        this.robotToCamera = robotToCamera;
        this.model = model;
        List<FieldTag> tags = field.getTags();
        tagId = new int[tags.size()];
        tagPose = new Pose3d[tags.size()];
        for (int i = 0; i < tags.size(); i++) {
            tagId[i] = tags.get(i).getID();
            tagPose[i] = tags.get(i).getPose();
        }
        this.truePose = truePose;
        random = new Random(name.hashCode());
        publisher =
                NetworkTableInstance.getDefault()
                        .getRawTopic(VisionFrame.topic(name))
                        .publish(VisionFrame.TYPE);
        frame.kind = VisionFrame.KIND_TAGS;
    }

    public String name() {
        return name;
    }

    /**
     * Capture to robot latency on a real coprocessor (s, share of frames, 2 ms bins): FM's Orin
     * over 13 TXMAN1 matches, 33,396 frames, median 15.7 ms. The model's latency scales it.
     */
    private static final double[] LATENCY_BIN = {0.012, 0.014, 0.016, 0.018, 0.020, 0.022};

    private static final double[] LATENCY_SHARE = {0.022, 0.562, 0.229, 0.047, 0.049, 0.011};

    private double latency() {
        double total = 0;
        for (double s : LATENCY_SHARE) {
            total += s;
        }
        double u = random.nextDouble() * total;
        double sum = 0;
        double bin = LATENCY_BIN[1];
        for (int i = 0; i < LATENCY_BIN.length; i++) {
            sum += LATENCY_SHARE[i];
            if (u < sum) {
                bin = LATENCY_BIN[i];
                break;
            }
        }
        double measured = bin + random.nextDouble() * 0.002; // within the 2 ms bin
        return measured * model.latencySeconds() / 0.0157;
    }

    /** One sim-clock tick: expose a frame when one is due, send frames whose latency is up. */
    public void step(double dt) {
        long now = RobotController.getMonotonicTime();
        untilCapture -= dt;
        if (untilCapture <= 0) {
            untilCapture += 1.0 / model.fps();
            capture(now);
            byte[] bytes = new byte[frame.encodedLength()];
            frame.encode(bytes, 0);
            inFlight.add(new Pending(now + (long) (latency() * 1e9), bytes));
        }
        while (!inFlight.isEmpty() && inFlight.peek().dueNanos() <= now) {
            publisher.set(inFlight.poll().bytes());
        }
    }

    /** What the camera sees now. */
    private void capture(long nowNanos) {
        Pose3d camera = new Pose3d(truePose.get()).transformBy(robotToCamera);
        frame.sequence = sequence++;
        frame.captureNanos = nowNanos;
        frame.count = 0;
        for (int t = 0; t < tagId.length && frame.count < VisionFrame.MAX; t++) {
            see(camera, t);
        }
    }

    private void see(Pose3d camera, int t) {
        Pose3d tag = tagPose[t];
        Translation3d tagToCamera = camera.getTranslation().minus(tag.getTranslation());
        double range = tagToCamera.getNorm();
        if (range > MAX_RANGE || range < 0.1) {
            return;
        }
        // The tag's face points along its +x.
        Translation3d face = new Translation3d(1, 0, 0).rotateBy(tag.getRotation());
        double cosFace =
                (face.getX() * tagToCamera.getX()
                                + face.getY() * tagToCamera.getY()
                                + face.getZ() * tagToCamera.getZ())
                        / range;
        if (cosFace < Math.cos(MAX_FACE_ANGLE)) {
            return;
        }
        // Corners in the tag's frame, OpenCV order as seen from the front: bottom-left,
        // bottom-right, top-right, top-left.
        for (int k = 0; k < 4; k++) {
            Translation3d corner =
                    new Translation3d(0, CORNER_Y[k], CORNER_Z[k])
                            .rotateBy(tag.getRotation())
                            .plus(tag.getTranslation());
            Translation3d c =
                    corner.minus(camera.getTranslation()).rotateBy(camera.getRotation().inverse());
            if (c.getX() < 0.05) {
                return;
            }
            u[k] =
                    model.cx()
                            - model.fx() * c.getY() / c.getX()
                            + random.nextGaussian() * model.pixelNoise();
            v[k] =
                    model.cy()
                            - model.fy() * c.getZ() / c.getX()
                            + random.nextGaussian() * model.pixelNoise();
            if (u[k] < 0 || u[k] >= model.width() || v[k] < 0 || v[k] >= model.height()) {
                return;
            }
        }
        int i = frame.count++;
        double uc = (u[0] + u[1] + u[2] + u[3]) / 4, vc = (v[0] + v[1] + v[2] + v[3]) / 4;
        double dy = (model.cx() - uc) / model.fx(), dz = (model.cy() - vc) / model.fy();
        frame.id[i] = tagId[t];
        frame.yaw[i] = Math.atan(dy);
        frame.pitch[i] = Math.atan2(dz, Math.hypot(1, dy));
        frame.range[i] = range + random.nextGaussian() * (0.005 + 0.005 * range * range);
        // Stand-in decision margin: the tag's side in pixels (bigger reads cleaner). Not on a real
        // detector's scale: do not tune margin thresholds on sim.
        frame.margin[i] = Math.hypot(u[1] - u[0], v[1] - v[0]);
        for (int k = 0; k < 4; k++) {
            frame.corners[i * 8 + 2 * k] = u[k];
            frame.corners[i * 8 + 2 * k + 1] = v[k];
        }
    }
}
