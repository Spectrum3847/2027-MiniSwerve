package frc.robot.subsystems.pose;

import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.Vision;
import frc.spectrumLib.localization.TagEkf;
import frc.spectrumLib.localization.TagSighting;
import frc.spectrumLib.swerve.SwerveSample;
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
 * WPILib's {@link SwerveDrivePoseEstimator} runs beside it on the same samples and frames as a
 * logged shadow, to compare against; plain odometry is Swerve's own.
 *
 * <p>Until TagEKF has a pose, the first frame with two or more tags seeds it (heading from the line
 * between two tags, position from them all). Before that it has none ({@link #seeded} false), and
 * nothing should drive to field positions.
 *
 * <p>Index: constructor | {@link #onSample} | {@link #periodic} | {@link #seedFrom} | logging
 */
public final class Localization {
    private final TagEkf ekf;
    private final SwerveDrivePoseEstimator shadow;
    private final Map<Integer, Translation3d> tags = new HashMap<>();
    private final SwerveModulePosition[] positions;
    private final String[] cameraNames;
    private final double[] cameraToTag = new double[3];
    private boolean shadowSeeded;

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
    private void onSample(SwerveSample s, double heading) {
        for (int i = 0; i < positions.length; i++) {
            positions[i] =
                    new SwerveModulePosition(s.distance[i], Rotation2d.fromRadians(s.angle[i]));
        }
        Rotation2d gyro = Rotation2d.fromRadians(heading);
        ekf.addOdometry(s.timestamp, gyro, positions);
        shadow.updateWithTime(s.timestamp, gyro, positions);
    }

    /**
     * Main loop, after Swerve.updateInputs and Vision.periodic: correct with this loop's frames.
     */
    public void periodic(Vision vision) {
        ekf.clearLoopStats();
        vision.forEachFrame(this::onFrame);
        log();
    }

    private void onFrame(int camera, Transform3d robotToCamera, VisionFrame f) {
        if (f.kind != VisionFrame.KIND_TAGS || f.count == 0) {
            return;
        }
        double time = f.captureNanos * 1e-9;
        List<TagSighting> sightings = new ArrayList<>(f.count);
        for (int i = 0; i < f.count; i++) {
            Translation3d tag = tags.get(f.id[i]);
            if (tag == null) {
                continue; // not on this field
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
            Pose2d pose = seedFrom(sightings);
            if (!ekf.isSeeded()) {
                ekf.reset(pose);
            }
            // The shadow takes the same frame as one pose, weighted by the nearest tag's range.
            double range = Double.POSITIVE_INFINITY;
            for (TagSighting t : sightings) {
                range = Math.min(range, t.cameraToTag().getNorm());
            }
            double xy = 0.05 + 0.02 * range * range / sightings.size();
            if (!shadowSeeded) {
                shadow.resetPose(pose);
                shadowSeeded = true;
            } else {
                shadow.addVisionMeasurement(
                        pose, time, VecBuilder.fill(xy, xy, 0.1 + 0.05 * range));
            }
        }
        ekf.addFrame(sightings);
    }

    /**
     * The robot's pose from one frame's tags: heading from the line between the two farthest-apart
     * tags (field vs robot frame), then position as the mean over every tag.
     */
    static Pose2d seedFrom(List<TagSighting> s) {
        int n = s.size();
        double[] rx = new double[n], ry = new double[n];
        for (int i = 0; i < n; i++) {
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
                double d =
                        Math.hypot(
                                s.get(i).tagField().getX() - s.get(j).tagField().getX(),
                                s.get(i).tagField().getY() - s.get(j).tagField().getY());
                if (d > far) {
                    far = d;
                    a = i;
                    b = j;
                }
            }
        }
        double fieldAngle =
                Math.atan2(
                        s.get(b).tagField().getY() - s.get(a).tagField().getY(),
                        s.get(b).tagField().getX() - s.get(a).tagField().getX());
        double robotAngle = Math.atan2(ry[b] - ry[a], rx[b] - rx[a]);
        double heading = fieldAngle - robotAngle;
        double c = Math.cos(heading), sn = Math.sin(heading);
        double x = 0, y = 0;
        for (int i = 0; i < n; i++) {
            x += s.get(i).tagField().getX() - (rx[i] * c - ry[i] * sn);
            y += s.get(i).tagField().getY() - (rx[i] * sn + ry[i] * c);
        }
        return new Pose2d(x / n, y / n, Rotation2d.fromRadians(heading));
    }

    /** The robot's pose on the field (TagEKF), if it has one. */
    public Pose2d pose() {
        return ekf.getPose();
    }

    /** TagEKF has a pose: something has seeded it. */
    public boolean seeded() {
        return ekf.isSeeded();
    }

    private void log() {
        // The robot's pose on the field: TagEKF's, what the robot drives on.
        Logger.recordOutput("Pose/Robot", ekf.getPose());
        // TagEKF has a pose (seeded by a frame with two or more tags).
        Logger.recordOutput("Pose/Seeded", ekf.isSeeded());
        // TagEKF's standard deviations: x, y (m), heading (rad).
        Logger.recordOutput("Pose/StdDevs", ekf.getStdDevs());
        // Tags TagEKF accepted and rejected since boot.
        Logger.recordOutput("Pose/Accepted", ekf.getAcceptedCount());
        Logger.recordOutput("Pose/Rejected", ekf.getRejectedCount());
        // Times TagEKF widened its covariance after tags kept failing (lost, then pulled back).
        Logger.recordOutput("Pose/Relocalizations", ekf.getRelocalizations());
        // Shadow: WPILib's SwerveDrivePoseEstimator on the same samples and frames.
        Logger.recordOutput("Pose/Shadow/WPILib", shadow.getEstimatedPosition());
    }
}
