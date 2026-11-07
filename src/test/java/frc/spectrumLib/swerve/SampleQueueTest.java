package frc.spectrumLib.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class SampleQueueTest {
    @Test
    void drainsOldestFirstAndCountsDrops() {
        SampleQueue q = new SampleQueue(4, 1);
        SwerveSample s = new SwerveSample(1);
        for (int i = 1; i <= 6; i++) {
            s.timestamp = i;
            q.offer(s);
        }
        SwerveSample[] out = {new SwerveSample(1), new SwerveSample(1), new SwerveSample(1)};
        int n = q.drainTo(out);
        assertEquals(3, n);
        assertEquals(4.0, out[0].timestamp);
        assertEquals(6.0, out[2].timestamp);
        assertEquals(3, q.dropped()); // 2 overwritten in the ring, 1 that did not fit in out
        assertEquals(0, q.drainTo(out));
    }
}
