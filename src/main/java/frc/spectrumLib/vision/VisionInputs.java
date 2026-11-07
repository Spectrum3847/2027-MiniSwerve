package frc.spectrumLib.vision;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * Logged vision inputs: per camera, the raw bytes of every frame that arrived this loop, back to
 * back ({@link VisionFrame} wire format), and when they arrived. Logging the bytes the coprocessor
 * sent, not decoded values, keeps the log small (50 bytes a tag) and makes replay decode exactly
 * what the robot decoded.
 */
public final class VisionInputs implements LoggableInputs {
    public final String[] cameras;

    /** Frames that arrived this loop, concatenated. */
    public final byte[][] frames;

    /** How many frames are in {@link #frames} (also readable from the bytes). */
    public final int[] frameCount;

    public VisionInputs(String... cameras) {
        this.cameras = cameras.clone();
        frames = new byte[cameras.length][];
        frameCount = new int[cameras.length];
        for (int i = 0; i < cameras.length; i++) {
            frames[i] = new byte[0];
        }
    }

    @Override
    public void toLog(LogTable table) {
        for (int i = 0; i < cameras.length; i++) {
            // Raw frames from this camera that arrived this loop (VisionFrame v1, concatenated).
            table.put(cameras[i] + "/Frames", frames[i]);
            // Number of frames in Frames.
            table.put(cameras[i] + "/FrameCount", frameCount[i]);
        }
    }

    @Override
    public void fromLog(LogTable table) {
        for (int i = 0; i < cameras.length; i++) {
            frames[i] = table.get(cameras[i] + "/Frames", new byte[0]);
            frameCount[i] = table.get(cameras[i] + "/FrameCount", 0);
        }
    }
}
