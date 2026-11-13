package frc.spectrumLib.vision;

import java.io.ByteArrayOutputStream;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.PubSubOption;
import org.wpilib.networktables.RawSubscriber;

/**
 * Frames from the coprocessor (Spectrum's PhotonVision fork, or the desktop sim) on NetworkTables:
 * one raw value per frame on {@link VisionFrame#topic}. Every frame is kept (a queue, not just the
 * latest), so none is lost between loops.
 *
 * <p>The frames of one loop are logged joined end to end, and the decoder finds each frame's end
 * from its own header. So only whole frames are joined: one cut short would borrow the next frame's
 * bytes as its last tag, and one with bytes left over would put every frame after it out of step. A
 * value that is not exactly one frame is dropped here and counted.
 */
public final class VisionIONetworkTables implements VisionIO {
    /** Frames held between loops per camera: half a second at 60 fps. */
    public static final int QUEUE = 32;

    private final RawSubscriber[] subscribers;
    private final ByteArrayOutputStream buffer = new ByteArrayOutputStream(4096);

    public VisionIONetworkTables(String... cameras) {
        var nt = NetworkTableInstance.getDefault();
        subscribers = new RawSubscriber[cameras.length];
        for (int i = 0; i < cameras.length; i++) {
            subscribers[i] =
                    nt.getRawTopic(VisionFrame.topic(cameras[i]))
                            .subscribe(
                                    VisionFrame.TYPE,
                                    new byte[0],
                                    PubSubOption.pollStorage(QUEUE),
                                    PubSubOption.SEND_ALL,
                                    PubSubOption.KEEP_DUPLICATES);
        }
    }

    @Override
    public void updateInputs(VisionInputs inputs) {
        for (int i = 0; i < subscribers.length; i++) {
            byte[][] queued = subscribers[i].readQueueValues();
            buffer.reset();
            int kept = 0;
            for (byte[] frame : queued) {
                if (isOneFrame(frame)) {
                    buffer.write(frame, 0, frame.length);
                    kept++;
                }
            }
            inputs.frames[i] = buffer.toByteArray();
            inputs.frameCount[i] = kept;
            inputs.malformed[i] = queued.length - kept;
        }
    }

    /** Whether {@code bytes} is exactly one version 1 frame: header right, length to the byte. */
    static boolean isOneFrame(byte[] bytes) {
        if (bytes.length < VisionFrame.HEADER_BYTES) {
            return false;
        }
        int version = bytes[0] & 0xff;
        int kind = bytes[1] & 0xff;
        int count = (bytes[2] & 0xff) | ((bytes[3] & 0xff) << 8);
        int each =
                kind == VisionFrame.KIND_TAGS
                        ? VisionFrame.TAG_BYTES
                        : kind == VisionFrame.KIND_OBJECTS ? VisionFrame.OBJECT_BYTES : -1;
        return version == VisionFrame.VERSION
                && each > 0
                && bytes.length == VisionFrame.HEADER_BYTES + count * each;
    }
}
