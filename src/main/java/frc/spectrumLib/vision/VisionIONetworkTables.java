package frc.spectrumLib.vision;

import java.io.ByteArrayOutputStream;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.PubSubOption;
import org.wpilib.networktables.RawSubscriber;

/**
 * Frames from the coprocessor (Spectrum's PhotonVision fork, or the desktop sim) on NetworkTables:
 * one raw value per frame on {@link VisionFrame#topic}. Every frame is kept (a queue, not just the
 * latest), so none is lost between loops.
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
            for (byte[] frame : queued) {
                buffer.write(frame, 0, frame.length);
            }
            inputs.frames[i] = buffer.toByteArray();
            inputs.frameCount[i] = queued.length;
        }
    }
}
