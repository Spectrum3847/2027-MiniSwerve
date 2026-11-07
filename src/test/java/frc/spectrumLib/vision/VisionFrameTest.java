package frc.spectrumLib.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class VisionFrameTest {
    @Test
    void tagsRoundTrip() {
        var f = new VisionFrame();
        f.kind = VisionFrame.KIND_TAGS;
        f.count = 2;
        f.sequence = 4_000_000_000L; // past a signed int: the counter is unsigned
        f.captureNanos = 123_456_789_012L;
        for (int i = 0; i < 2; i++) {
            f.id[i] = 7 + i;
            f.yaw[i] = 0.1 * (i + 1);
            f.pitch[i] = -0.05;
            f.range[i] = 3.25;
            f.margin[i] = 80;
            for (int c = 0; c < 8; c++) {
                f.corners[i * 8 + c] = 100 + c;
            }
        }
        byte[] bytes = new byte[f.encodedLength() + 3];
        assertEquals(16 + 2 * 50, f.encode(bytes, 3));
        var g = new VisionFrame();
        assertEquals(116, g.decode(bytes, 3, bytes.length - 3));
        assertEquals(2, g.count);
        assertEquals(f.sequence, g.sequence);
        assertEquals(f.captureNanos, g.captureNanos);
        assertEquals(8, g.id[1]);
        assertEquals(0.2, g.yaw[1], 1e-6);
        assertEquals(107, g.corners[15], 1e-6);
    }

    @Test
    void objectsRoundTrip() {
        var f = new VisionFrame();
        f.kind = VisionFrame.KIND_OBJECTS;
        f.count = 1;
        f.id[0] = 2;
        f.margin[0] = 0.9;
        f.yaw[0] = -0.3;
        f.pitch[0] = -0.2;
        f.width[0] = 0.1;
        f.height[0] = 0.05;
        byte[] bytes = new byte[f.encodedLength()];
        f.encode(bytes, 0);
        var g = new VisionFrame();
        assertEquals(16 + 22, g.decode(bytes, 0, bytes.length));
        assertEquals(VisionFrame.KIND_OBJECTS, g.kind);
        assertEquals(0.9, g.margin[0], 1e-6);
        assertEquals(0.05, g.height[0], 1e-6);
    }

    @Test
    void wrongVersionOrShortFramesAreRejected() {
        var f = new VisionFrame();
        f.kind = VisionFrame.KIND_TAGS;
        f.count = 1;
        byte[] bytes = new byte[f.encodedLength()];
        f.encode(bytes, 0);
        var g = new VisionFrame();
        assertEquals(-1, g.decode(bytes, 0, bytes.length - 1)); // cut short
        bytes[0] = 2;
        assertEquals(-1, g.decode(bytes, 0, bytes.length)); // a version we do not know
        assertEquals(-1, g.decode(bytes, 4, bytes.length)); // asks past the end: no exception
        assertEquals(-1, g.decode(bytes, -1, 16));
    }

    @Test
    void aRejectedFrameLeavesTheOldOneAsItWas() {
        var f = new VisionFrame();
        f.kind = VisionFrame.KIND_TAGS;
        f.count = 1;
        f.sequence = 9;
        f.captureNanos = 1234;
        byte[] good = new byte[f.encodedLength()];
        f.encode(good, 0);
        var g = new VisionFrame();
        g.decode(good, 0, good.length);
        byte[] bad = good.clone();
        bad[0] = 3; // a newer version
        bad[4] = 99; // and another sequence
        assertEquals(-1, g.decode(bad, 0, bad.length));
        assertEquals(9, g.sequence);
        assertEquals(1234, g.captureNanos);
    }
}
