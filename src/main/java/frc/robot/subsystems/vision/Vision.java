package frc.robot.subsystems.vision;

import frc.robot.RunMode;
import frc.spectrumLib.telemetry.Alert;
import frc.spectrumLib.vision.VisionFrame;
import frc.spectrumLib.vision.VisionIO;
import frc.spectrumLib.vision.VisionIONetworkTables;
import frc.spectrumLib.vision.VisionInputs;
import org.littletonrobotics.junction.Logger;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.system.Timer;

/**
 * The robot's cameras: each loop, every frame each camera sent ({@link VisionFrame}, from the
 * coprocessor or the desktop sim), logged as raw inputs, decoded, checked and handed to the pose
 * estimator ({@link #forEachFrame}). A camera that stops sending raises an alert and drops out; the
 * robot keeps driving on odometry and the other cameras.
 *
 * <p>Index: constructor | {@link #periodic} | {@link #forEachFrame} | health
 */
public final class Vision {
    /** A camera counts as gone after this long without a frame (s). */
    private static final double STALE_SECONDS = 0.5;

    /**
     * Frames kept per camera per loop: as many as the NetworkTables queue holds, so every frame
     * that arrives is used. (More than this in one loop cannot arrive.)
     */
    private static final int FRAMES_PER_LOOP = VisionIONetworkTables.QUEUE;

    private final VisionSettings.Camera[] cameras;
    private final Transform3d[] mounts;
    private final VisionIO io;
    private final VisionInputs inputs;
    private final VisionFrame[][] frames;
    private final int[] decoded;
    private final long[] lastSequence;
    private final long[] dropped;
    private final long[] badFrames;
    private final double[] lastFrameSeconds;
    private final Alert[] staleAlerts;
    private final long[] restarts;

    /** Log keys per camera, built once so the loop builds no strings. */
    private final String[] connectedKey;

    private final String[] droppedKey;
    private final String[] badKey;
    private final String[] latencyKey;
    private final String[] tagsKey;
    private final String[] restartsKey;

    /**
     * A sequence number this far below the last one means the camera's process restarted (its
     * counter began again), not a late frame.
     */
    private static final long RESTART_JUMP = 50;

    private boolean expectFrames = true;

    /** Receives one decoded frame: camera index, its mount, the frame. */
    public interface FrameConsumer {
        void accept(int camera, Transform3d robotToCamera, VisionFrame frame);
    }

    public Vision(VisionSettings.Camera[] cameras) {
        this.cameras = cameras;
        int n = cameras.length;
        String[] names = new String[n];
        mounts = new Transform3d[n];
        for (int i = 0; i < n; i++) {
            names[i] = cameras[i].name();
            mounts[i] = cameras[i].robotToCamera();
        }
        io = RunMode.hasHardware() ? new VisionIONetworkTables(names) : VisionIO.NONE;
        inputs = new VisionInputs(names);
        frames = new VisionFrame[n][FRAMES_PER_LOOP];
        for (int i = 0; i < n; i++) {
            for (int k = 0; k < FRAMES_PER_LOOP; k++) {
                frames[i][k] = new VisionFrame();
            }
        }
        decoded = new int[n];
        lastSequence = new long[n];
        java.util.Arrays.fill(lastSequence, -1);
        dropped = new long[n];
        badFrames = new long[n];
        lastFrameSeconds = new double[n];
        java.util.Arrays.fill(lastFrameSeconds, Double.NEGATIVE_INFINITY);
        restarts = new long[n];
        connectedKey = new String[n];
        droppedKey = new String[n];
        badKey = new String[n];
        latencyKey = new String[n];
        tagsKey = new String[n];
        restartsKey = new String[n];
        for (int i = 0; i < n; i++) {
            String key = "Vision/" + names[i];
            connectedKey[i] = key + "/Connected";
            droppedKey[i] = key + "/DroppedFrames";
            badKey[i] = key + "/BadFrames";
            latencyKey[i] = key + "/LatencyMs";
            tagsKey[i] = key + "/Tags";
            restartsKey[i] = key + "/Restarts";
        }
        staleAlerts = new Alert[n];
        for (int i = 0; i < n; i++) {
            staleAlerts[i] =
                    new Alert("Camera " + names[i] + " not sending frames", Alert.Level.MEDIUM);
        }
    }

    /**
     * Whether frames should be arriving: false in a sim with no simulated cameras, so a silent
     * camera is not an alert there.
     */
    public void expectFrames(boolean expect) {
        expectFrames = expect;
    }

    /** Camera names, in the order {@link #forEachFrame} numbers them. */
    public String[] names() {
        return inputs.cameras.clone();
    }

    /** Each camera's mount on the robot (from Robot Settings at boot). */
    public Transform3d mount(int camera) {
        return mounts[camera];
    }

    /** Main loop, before the pose estimator: read, log and decode this loop's frames. */
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Vision", inputs);
        // The robot clock as AdvantageKit replays it (not the HAL clock, which replay does not).
        double now = Timer.getTimestamp();
        for (int i = 0; i < cameras.length; i++) {
            decoded[i] = 0;
            badFrames[i] += inputs.malformed[i]; // dropped at the IO: not exactly one frame
            byte[] bytes = inputs.frames[i];
            int offset = 0;
            while (offset < bytes.length && decoded[i] < FRAMES_PER_LOOP) {
                VisionFrame f = frames[i][decoded[i]];
                int used = f.decode(bytes, offset, bytes.length - offset);
                if (used < 0) {
                    badFrames[i]++;
                    break; // the rest of this loop's bytes cannot be framed
                }
                offset += used;
                if (f.sequence < lastSequence[i] - RESTART_JUMP) {
                    // The camera restarted and counts from the start again: count gaps from
                    // here, or every drop would hide until the new count passed the old one.
                    restarts[i]++;
                    lastSequence[i] = -1;
                }
                if (lastSequence[i] >= 0 && f.sequence > lastSequence[i] + 1) {
                    dropped[i] += f.sequence - lastSequence[i] - 1;
                }
                // Only forward: a frame arriving out of order must not count as a gap later.
                lastSequence[i] = Math.max(lastSequence[i], f.sequence);
                lastFrameSeconds[i] = now;
                decoded[i]++;
            }
            log(i, now);
        }
    }

    private void log(int i, double now) {
        boolean enabled = cameras[i].enabled().get();
        boolean stale = now - lastFrameSeconds[i] > STALE_SECONDS;
        staleAlerts[i].set(enabled && stale && expectFrames);
        // The camera is enabled and sent a frame within the last 0.5 s.
        Logger.recordOutput(connectedKey[i], enabled && !stale);
        // Frames skipped by the camera's own counter, since boot.
        Logger.recordOutput(droppedKey[i], dropped[i]);
        // Frames that could not be decoded (wrong version or cut short), since boot.
        Logger.recordOutput(badKey[i], badFrames[i]);
        // Times the camera's frame counter started over (its process restarted), since boot.
        Logger.recordOutput(restartsKey[i], restarts[i]);
        if (decoded[i] > 0) {
            VisionFrame last = frames[i][decoded[i] - 1];
            // Newest frame's age when the loop read it: exposure to robot code (ms).
            Logger.recordOutput(latencyKey[i], (now * 1e9 - last.captureNanos) * 1e-6);
            // Tags in the newest frame.
            Logger.recordOutput(tagsKey[i], last.count);
        }
    }

    /** This loop's decoded frames from enabled cameras, oldest first per camera. */
    public void forEachFrame(FrameConsumer consumer) {
        for (int i = 0; i < cameras.length; i++) {
            if (!cameras[i].enabled().get()) {
                continue;
            }
            for (int k = 0; k < decoded[i]; k++) {
                consumer.accept(i, mounts[i], frames[i][k]);
            }
        }
    }
}
