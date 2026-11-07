package frc.robot.subsystems.pose;

import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.Vision;
import frc.spectrumLib.localization.TagEkf;
import frc.spectrumLib.localization.TagSighting;
import frc.spectrumLib.swerve.SwerveSample;
import frc.spectrumLib.telemetry.Alert;
import frc.spectrumLib.vision.VisionFrame;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.littletonrobotics.junction.Logger;
import org.wpilib.fields.Field;
import org.wpilib.fields.FieldTag;
import org.wpilib.math.estimator.SwerveDrivePoseEstimator;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.linalg.VecBuilder;

/**
 * Where the robot is on the field. TagEKF ({@link TagEkf}) drives it: every 250 Hz odometry sample
 * predicts, every camera's tags correct one at a time (bearing and range), at their capture time.
 * WPILib's {@link SwerveDrivePoseEstimator} runs beside it on the same samples as a logged shadow,
 * fed the same checked multi-tag poses; plain odometry is Swerve's own.
 *
 * <p>Seeding: a frame with two or more tags gives a whole pose ({@link #solve}), used only if it
 * passes checks (tags at least 1 m apart and within 8 m, every pair agreeing on heading within 3
 * deg, every tag within 15 cm of where the pose puts it) and the next such frame agrees with it; a
 * seed also needs every tag within 6 m. It goes in at its capture time. Until then TagEKF has no
 * pose ({@link #seeded} false), and nothing should drive to field positions. If tags keep being
 * rejected for a second while checked frames agree on a pose more than 0.5 m away, TagEKF is
 * re-seeded there: a wrong seed or a big hit cannot leave it lost for good.
 *
 * <p>Index: constructor | {@link #onSample} | {@link #periodic} | {@link #onFrame} | {@link #solve}
 * | logging
 */
public final class Localization {
    private static final double MIN_BASELINE = 1.0;

    /** Farthest tag a frame's pose may use (m): the cameras' useful range. */
    private static final double MAX_RANGE = 8.0;

    /** Farthest tag a seeding frame may have (m): seeds need the nearer, better-measured tags. */
    private static final double SEED_RANGE = 6.0;

    private static final double MAX_HEADING_SPREAD = Math.toRadians(3);
    private static final double MAX_RESIDUAL = 0.15;
    private static final double AGREE_XY = 0.2;
    private static final double AGREE_HEADING = Math.toRadians(3);
    private static final double RESEED_DISTANCE = 0.5;

    /** Frame age (newest sample minus capture time) outside this means a clock problem (s). */
    private static final double MIN_AGE = -0.05;

    private static final double MAX_AGE = 0.3;

    /** A multi-tag frame's pose, or null with why it was not trusted. */
    record Solution(Pose2d pose, String check) {}

    private final TagEkf ekf;
    private final SwerveDrivePoseEstimator shadow;
    private final Map<Integer, Translation3d> tags = new HashMap<>();
    private final SwerveModulePosition[] positions;
    private final String[] cameraNames;
    private final double[] cameraToTag = new double[3];
    private final List<TagSighting> sightings = new ArrayList<>();
    private final Alert clockAlert =
            new Alert(
                    "Camera and swerve clocks disagree: vision frames are not used",
                    Alert.Level.HIGH);
    private boolean shadowSeeded;
    private Pose2d lastCandidate;
    private double newestSample = Double.NEGATIVE_INFINITY;
    private boolean headingFromGyro = true;
    private String seedCheck = "no multi-tag frame yet";
    private long seeds;
    private long reseeds;
    private long unknownTags;
    private double frameAge;
    private long rejectedAtMark;
    private long acceptedAtMark;
    private double markTime;
    private long loops;

    public Localization(Swerve swerve, Vision vision, Field field) {
        var locations = swerve.moduleLocations();
        ekf = new TagEkf(locations, PoseSettings.noise());
        double[] adaptive = PoseSettings.adaptive();
        ekf.setAdaptive(adaptive[0], adaptive[1]);
        positions = new SwerveModulePosition[locations.length];
        for (int i = 0; i < positions.length; i++) {
            positions[i] = new SwerveModulePosition();
        }
        shadow =
                new SwerveDrivePoseEstimator(
                        new SwerveDriveKinematics(locations),
                        new Rotation2d(),
                        positions,
                        new Pose2d());
        for (FieldTag t : field.getTags()) {
            tags.put(t.getID(), t.getPose().getTranslation());
        }
        cameraNames = vision.names();
        swerve.setSampleListener(this::onSample);
    }

    /** Each odometry sample (main thread, from Swerve.updateInputs): predict both estimators. */
    private void onSample(SwerveSample s, double heading, boolean fromGyro) {
        for (int i = 0; i < positions.length; i++) {
            positions[i].distance = s.distance[i];
            positions[i].angle = Rotation2d.fromRadians(s.angle[i]);
        }
        Rotation2d gyro = Rotation2d.fromRadians(heading);
        ekf.addOdometry(s.timestamp, gyro, positions);
        shadow.updateWithTime(s.timestamp, gyro, positions);
        newestSample = s.timestamp;
        headingFromGyro = fromGyro;
    }

    /**
     * Main loop, after Swerve.updateInputs and Vision.periodic: correct with this loop's frames.
     *
     * @param nowSeconds the robot clock (replayed), for the once-a-second lost check
     */
    public void periodic(Vision vision, double nowSeconds) {
        ekf.clearLoopStats();
        vision.forEachFrame(this::onFrame);
        if (nowSeconds - markTime >= 1.0) {
            markTime = nowSeconds;
            rejectedAtMark = ekf.getRejectedCount();
            acceptedAtMark = ekf.getAcceptedCount();
        }
        log();
    }

    /** Lost: 20 or more tags rejected and none accepted since the last mark. */
    private boolean lost() {
        return ekf.getRejectedCount() - rejectedAtMark >= 20
                && ekf.getAcceptedCount() == acceptedAtMark;
    }

    private void onFrame(int camera, Transform3d robotToCamera, VisionFrame f) {
        if (f.kind != VisionFrame.KIND_TAGS) {
            return;
        }
        double time = f.captureNanos * 1e-9;
        frameAge = newestSample - time;
        boolean clockOk = frameAge >= MIN_AGE && frameAge <= MAX_AGE;
        clockAlert.set(!clockOk);
        if (!clockOk || f.count == 0) {
            return;
        }
        sightings.clear();
        for (int i = 0; i < f.count; i++) {
            Translation3d tag = tags.get(f.id[i]);
            if (tag == null) {
                unknownTags++; // not on this field: a wrong field file looks like this
                continue;
            }
            f.cameraToTarget(i, cameraToTag);
            sightings.add(
                    new TagSighting(
                            cameraNames[camera],
                            time,
                            f.id[i],
                            tag,
                            robotToCamera,
                            new Translation3d(cameraToTag[0], cameraToTag[1], cameraToTag[2]),
                            1.0));
        }
        if (sightings.size() >= 2) {
            Solution s = solve(sightings);
            seedCheck = s.check();
            if (s.pose() != null) {
                useChecked(s.pose(), time);
            }
        }
        ekf.addFrame(sightings);
    }

    /** A checked multi-tag pose: seed or re-seed TagEKF when two agree, and feed the shadow. */
    private void useChecked(Pose2d pose, double time) {
        boolean agrees =
                lastCandidate != null
                        && lastCandidate.getTranslation().getDistance(pose.getTranslation())
                                < AGREE_XY
                        && Math.abs(
                                        lastCandidate
                                                .getRotation()
                                                .minus(pose.getRotation())
                                                .getRadians())
                                < AGREE_HEADING;
        lastCandidate = pose;
        double range = Double.POSITIVE_INFINITY, farthest = 0;
        for (TagSighting t : sightings) {
            range = Math.min(range, t.cameraToTag().getNorm());
            farthest = Math.max(farthest, t.cameraToTag().getNorm());
        }
        agrees &= farthest <= SEED_RANGE || ekf.isSeeded();
        if (agrees && !ekf.isSeeded()) {
            ekf.addPose(pose, time, 0.1, Math.toRadians(3));
            seeds++;
        } else if (agrees
                && lost()
                && ekf.getPose().getTranslation().getDistance(pose.getTranslation())
                        > RESEED_DISTANCE) {
            ekf.reset(pose);
            reseeds++;
        }
        if (!shadowSeeded) {
            if (agrees) {
                shadow.resetPose(pose);
                shadowSeeded = true;
            }
        } else {
            double xy = 0.05 + 0.02 * range * range / sightings.size();
            shadow.addVisionMeasurement(pose, time, VecBuilder.fill(xy, xy, 0.1 + 0.05 * range));
        }
    }

    /**
     * The robot's pose from one frame's tags: heading from the farthest-apart pair, position the
     * mean over every tag; null with the failed check if the frame should not be trusted.
     */
    static Solution solve(List<TagSighting> s) {
        int n = s.size();
        double[] rx = new double[n], ry = new double[n];
        for (int i = 0; i < n; i++) {
            if (s.get(i).cameraToTag().getNorm() > MAX_RANGE) {
                return new Solution(null, "tag farther than 8 m");
            }
            Translation3d inRobot =
                    new Pose3d()
                            .transformBy(s.get(i).robotToCamera())
                            .transformBy(new Transform3d(s.get(i).cameraToTag(), new Rotation3d()))
                            .getTranslation();
            rx[i] = inRobot.getX();
            ry[i] = inRobot.getY();
        }
        int a = 0, b = 1;
        double far = -1;
        for (int i = 0; i < n; i++) {
            for (int j = i + 1; j < n; j++) {
                double d = baseline(s, i, j);
                if (d > far) {
                    far = d;
                    a = i;
                    b = j;
                }
            }
        }
        if (far < MIN_BASELINE) {
            return new Solution(null, "tags closer than 1 m");
        }
        double heading = pairHeading(s, rx, ry, a, b);
        for (int i = 0; i < n; i++) {
            for (int j = i + 1; j < n; j++) {
                if (baseline(s, i, j) >= MIN_BASELINE / 2
                        && Math.abs(
                                        Math.IEEEremainder(
                                                pairHeading(s, rx, ry, i, j) - heading,
                                                2 * Math.PI))
                                > MAX_HEADING_SPREAD) {
                    return new Solution(null, "tag pairs disagree on heading");
                }
            }
        }
        double c = Math.cos(heading), sn = Math.sin(heading);
        double x = 0, y = 0;
        for (int i = 0; i < n; i++) {
            x += s.get(i).tagField().getX() - (rx[i] * c - ry[i] * sn);
            y += s.get(i).tagField().getY() - (rx[i] * sn + ry[i] * c);
        }
        x /= n;
        y /= n;
        for (int i = 0; i < n; i++) {
            double ex = x + rx[i] * c - ry[i] * sn - s.get(i).tagField().getX();
            double ey = y + rx[i] * sn + ry[i] * c - s.get(i).tagField().getY();
            if (Math.hypot(ex, ey) > MAX_RESIDUAL) {
                return new Solution(null, "a tag is more than 15 cm off");
            }
        }
        return new Solution(new Pose2d(x, y, Rotation2d.fromRadians(heading)), "ok");
    }

    private static double baseline(List<TagSighting> s, int i, int j) {
        return Math.hypot(
                s.get(i).tagField().getX() - s.get(j).tagField().getX(),
                s.get(i).tagField().getY() - s.get(j).tagField().getY());
    }

    private static double pairHeading(List<TagSighting> s, double[] rx, double[] ry, int i, int j) {
        double field =
                Math.atan2(
                        s.get(j).tagField().getY() - s.get(i).tagField().getY(),
                        s.get(j).tagField().getX() - s.get(i).tagField().getX());
        return field - Math.atan2(ry[j] - ry[i], rx[j] - rx[i]);
    }

    /** The robot's pose on the field (TagEKF), meaningful only when {@link #seeded}. */
    public Pose2d pose() {
        return ekf.getPose();
    }

    /** TagEKF has a pose: two checked multi-tag frames agreed. */
    public boolean seeded() {
        return ekf.isSeeded();
    }

    private void log() {
        Pose2d robot = ekf.getPose();
        // The robot's pose on the field: TagEKF's, what the robot drives on.
        Logger.recordOutput("Pose/Robot", robot);
        // TagEKF has a pose (two checked multi-tag frames agreed).
        Logger.recordOutput("Pose/Seeded", ekf.isSeeded());
        // TagEKF's standard deviations: x, y (m), heading (rad).
        Logger.recordOutput("Pose/StdDevs", ekf.getStdDevs());
        // Tags TagEKF accepted since boot.
        Logger.recordOutput("Pose/Accepted", ekf.getAcceptedCount());
        // Tags TagEKF rejected since boot (reasons in Pose/RejectReasons).
        Logger.recordOutput("Pose/Rejected", ekf.getRejectedCount());
        // Times TagEKF widened its covariance after tags kept failing.
        Logger.recordOutput("Pose/Relocalizations", ekf.getRelocalizations());
        // Times TagEKF was seeded since boot.
        Logger.recordOutput("Pose/Seeds", seeds);
        // Times TagEKF was re-seeded after being lost, since boot.
        Logger.recordOutput("Pose/Reseeds", reseeds);
        // The last multi-tag frame's check ("ok", or why it was not trusted).
        Logger.recordOutput("Pose/SeedCheck", seedCheck);
        // Tags seen that are not on this field, since boot.
        Logger.recordOutput("Pose/UnknownTags", unknownTags);
        // Newest odometry sample minus the last frame's capture time (s): the clocks' health.
        Logger.recordOutput("Pose/FrameAgeSeconds", frameAge);
        // The heading TagEKF predicts with comes from the gyro (false: from the wheels).
        Logger.recordOutput("Pose/HeadingFromGyro", headingFromGyro);
        Pose2d shadowPose = shadow.getEstimatedPosition();
        // Shadow: WPILib's SwerveDrivePoseEstimator on the same samples and checked poses.
        Logger.recordOutput("Pose/Shadow/WPILib", shadowPose);
        // How far the shadow is from TagEKF (m).
        Logger.recordOutput(
                "Pose/Shadow/Delta",
                shadowPose.getTranslation().getDistance(robot.getTranslation()));
        if (loops++ % 50 == 0) {
            // TagEKF's rejection reasons with counts since boot ("reason: n"), every half second.
            Logger.recordOutput("Pose/RejectReasons", reasons());
        }
    }

    private String[] reasons() {
        var r = ekf.getRejectionCounts();
        String[] out = new String[r.size()];
        int i = 0;
        for (var e : r.entrySet()) {
            out[i++] = e.getKey() + ": " + e.getValue();
        }
        return out;
    }
}
