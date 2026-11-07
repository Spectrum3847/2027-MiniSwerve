package frc.spectrumLib.vision;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

/**
 * One camera frame as the coprocessor sends it: what the robot needs from each frame and nothing
 * more. Shared word for word with Spectrum's PhotonVision fork (copy this file there), which
 * publishes one frame per NetworkTables raw value on {@link #topic}; the desktop sim (SpectrumSim)
 * makes the same bytes.
 *
 * <p>Wire format, version 1, little-endian:
 *
 * <pre>
 * header (16 bytes):
 *   u8  version      = 1
 *   u8  kind         1 = AprilTags, 2 = objects
 *   u16 count        observations that follow
 *   u32 sequence     the camera's frame counter (gaps = dropped frames)
 *   i64 captureNanos when the image was exposed, on the robot's clock
 *                    (RobotController.getMonotonicTime, ns)
 * tag (50 bytes each):
 *   i16 id           signed (a tag id fits)
 *   f32 yaw          bearing to the tag's center, camera frame, +LEFT (rad)
 *   f32 pitch        bearing to the tag's center, +up (rad)
 *   f32 range        straight-line distance, camera to the tag's center (m), from the pose solve
 *   f32 margin       detector decision margin (quality; higher is better)
 *   f32 x8 corners   sub-pixel corners in the raw (distorted) image, u right, v down, OpenCV
 *                    order: bottom-left, bottom-right, top-right, top-left, as u0 v0 u1 v1 ...
 * object (22 bytes each):
 *   i16 classId      signed
 *   f32 confidence   0..1
 *   f32 yaw, pitch   bearing to the middle of the box's bottom edge (where the object meets the
 *                    floor), as for tags (rad)
 *   f32 width, height box size as angles (rad)
 * </pre>
 *
 * <p>Camera frame: x out of the lens, y left, z up (WPILib's). <b>Yaw is +left: PhotonVision's own
 * target yaw is +right, so negate it.</b> Bearings are undistorted (through the calibration). A
 * frame with no tags is still sent (count 0): it says the camera is alive and saw nothing.
 *
 * <p>Publishing (the coprocessor): one raw value per frame, type string {@link #TYPE}, on {@link
 * #topic}, with publisher options {@code PubSubOption.SEND_ALL} and {@code
 * PubSubOption.periodic(0.005)} (or {@code NetworkTableInstance.flush()} after each frame). Without
 * them NetworkTables merges frames sent faster than its 100 ms default flush, and the robot sees
 * one frame in six at 60 fps. {@code captureNanos} is the exposure time on the robot's clock: the
 * coprocessor's own time plus NetworkTables' server time offset ({@code
 * NetworkTableInstance.getServerTimeOffset}, microseconds; convert to nanoseconds).
 *
 * <p>Decoding fills this object's arrays (no allocation), up to {@link #MAX} observations.
 */
public final class VisionFrame {
    public static final int VERSION = 1;
    public static final int KIND_TAGS = 1;
    public static final int KIND_OBJECTS = 2;
    public static final int HEADER_BYTES = 16;
    public static final int TAG_BYTES = 50;
    public static final int OBJECT_BYTES = 22;

    /** Most observations kept from one frame. */
    public static final int MAX = 32;

    /** NetworkTables raw type string for frames. */
    public static final String TYPE = "spectrum.vision.frame.v1";

    /** The topic a camera's frames are published on. */
    public static String topic(String camera) {
        return "/spectrum/vision/" + camera + "/frames";
    }

    public int kind;
    public int count;
    public long sequence;
    public long captureNanos;

    public final int[] id = new int[MAX];
    public final double[] yaw = new double[MAX];
    public final double[] pitch = new double[MAX];
    public final double[] range = new double[MAX];
    public final double[] margin = new double[MAX];
    public final double[] corners = new double[MAX * 8];
    public final double[] width = new double[MAX];
    public final double[] height = new double[MAX];

    /**
     * Reads a frame at {@code offset}; returns the bytes it used, or -1 if it is not a version 1
     * frame or is cut short (the caller skips it and counts it).
     */
    public int decode(byte[] bytes, int offset, int length) {
        if (offset < 0 || length < HEADER_BYTES || offset > bytes.length - length) {
            return -1;
        }
        ByteBuffer b = ByteBuffer.wrap(bytes, offset, length).order(ByteOrder.LITTLE_ENDIAN);
        // Check the whole header before changing this frame: a rejected frame leaves it as it was.
        int version = b.get(offset) & 0xff;
        int k = b.get(offset + 1) & 0xff;
        int n = b.getShort(offset + 2) & 0xffff;
        int each = k == KIND_TAGS ? TAG_BYTES : k == KIND_OBJECTS ? OBJECT_BYTES : -1;
        if (version != VERSION || each < 0 || length < HEADER_BYTES + n * each) {
            return -1;
        }
        b.position(offset + 4);
        kind = k;
        sequence = b.getInt() & 0xffffffffL;
        captureNanos = b.getLong();
        count = Math.min(n, MAX);
        for (int i = 0; i < n; i++) {
            if (i >= MAX) {
                b.position(b.position() + each); // more than we keep: skip
                continue;
            }
            id[i] = b.getShort();
            if (kind == KIND_TAGS) {
                yaw[i] = b.getFloat();
                pitch[i] = b.getFloat();
                range[i] = b.getFloat();
                margin[i] = b.getFloat();
                for (int c = 0; c < 8; c++) {
                    corners[i * 8 + c] = b.getFloat();
                }
            } else {
                margin[i] = b.getFloat(); // confidence
                yaw[i] = b.getFloat();
                pitch[i] = b.getFloat();
                width[i] = b.getFloat();
                height[i] = b.getFloat();
            }
        }
        return HEADER_BYTES + n * each;
    }

    /** Bytes {@link #encode} writes for this frame. */
    public int encodedLength() {
        return HEADER_BYTES + count * (kind == KIND_TAGS ? TAG_BYTES : OBJECT_BYTES);
    }

    /**
     * Writes this frame into {@code out} at {@code offset}; returns the bytes written. At most
     * {@link #MAX} observations ({@link #count} is clamped).
     */
    public int encode(byte[] out, int offset) {
        count = Math.max(0, Math.min(count, MAX));
        ByteBuffer b = ByteBuffer.wrap(out, offset, encodedLength()).order(ByteOrder.LITTLE_ENDIAN);
        b.put((byte) VERSION);
        b.put((byte) kind);
        b.putShort((short) count);
        b.putInt((int) sequence);
        b.putLong(captureNanos);
        for (int i = 0; i < count; i++) {
            b.putShort((short) id[i]);
            if (kind == KIND_TAGS) {
                b.putFloat((float) yaw[i]);
                b.putFloat((float) pitch[i]);
                b.putFloat((float) range[i]);
                b.putFloat((float) margin[i]);
                for (int c = 0; c < 8; c++) {
                    b.putFloat((float) corners[i * 8 + c]);
                }
            } else {
                b.putFloat((float) margin[i]);
                b.putFloat((float) yaw[i]);
                b.putFloat((float) pitch[i]);
                b.putFloat((float) width[i]);
                b.putFloat((float) height[i]);
            }
        }
        return encodedLength();
    }

    /** Direction to observation {@code i}, camera frame, scaled to its range: x, y, z (m). */
    public void cameraToTarget(int i, double[] out) {
        double cy = Math.cos(yaw[i]), sy = Math.sin(yaw[i]);
        double cp = Math.cos(pitch[i]), sp = Math.sin(pitch[i]);
        out[0] = range[i] * cp * cy;
        out[1] = range[i] * cp * sy;
        out[2] = range[i] * sp;
    }
}
