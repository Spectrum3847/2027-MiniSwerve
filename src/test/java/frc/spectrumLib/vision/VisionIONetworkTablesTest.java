package frc.spectrumLib.vision;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

/** Only a value that is exactly one frame is joined into the loop's bytes. */
class VisionIONetworkTablesTest {
    private static byte[] frame(int kind, int count, int extraBytes) {
        int each = kind == VisionFrame.KIND_TAGS ? VisionFrame.TAG_BYTES : VisionFrame.OBJECT_BYTES;
        byte[] b = new byte[VisionFrame.HEADER_BYTES + count * each + extraBytes];
        b[0] = VisionFrame.VERSION;
        b[1] = (byte) kind;
        b[2] = (byte) count;
        return b;
    }

    @Test
    void wholeFramesPass() {
        assertTrue(VisionIONetworkTables.isOneFrame(frame(VisionFrame.KIND_TAGS, 0, 0)));
        assertTrue(VisionIONetworkTables.isOneFrame(frame(VisionFrame.KIND_TAGS, 3, 0)));
        assertTrue(VisionIONetworkTables.isOneFrame(frame(VisionFrame.KIND_OBJECTS, 2, 0)));
    }

    @Test
    void cutShortOrPaddedOrUnknownFramesDoNot() {
        // Says three tags, carries two: would borrow the next frame's bytes.
        assertFalse(
                VisionIONetworkTables.isOneFrame(
                        frame(VisionFrame.KIND_TAGS, 3, -VisionFrame.TAG_BYTES)));
        // Bytes left over: every frame after it would be out of step.
        assertFalse(VisionIONetworkTables.isOneFrame(frame(VisionFrame.KIND_TAGS, 1, 4)));
        assertFalse(VisionIONetworkTables.isOneFrame(new byte[0]));
        assertFalse(VisionIONetworkTables.isOneFrame(new byte[5]));
        byte[] wrongVersion = frame(VisionFrame.KIND_TAGS, 1, 0);
        wrongVersion[0] = 9;
        assertFalse(VisionIONetworkTables.isOneFrame(wrongVersion));
        assertFalse(VisionIONetworkTables.isOneFrame(frame(7, 1, 0)));
    }
}
