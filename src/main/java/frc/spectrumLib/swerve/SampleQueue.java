package frc.spectrumLib.swerve;

/**
 * Hands samples from the swerve thread to the main loop without allocating or blocking either side.
 * A fixed ring of preallocated {@link SwerveSample}s guarded by one short lock; if the main loop
 * falls behind, the oldest samples are dropped and counted.
 */
public final class SampleQueue {
    private final SwerveSample[] ring;
    private int head; // next write
    private int size;
    private long dropped;

    public SampleQueue(int capacity, int modules) {
        ring = new SwerveSample[capacity];
        for (int i = 0; i < capacity; i++) {
            ring[i] = new SwerveSample(modules);
        }
    }

    /** Swerve thread: copies {@code s} in. */
    public synchronized void offer(SwerveSample s) {
        ring[head].copyFrom(s);
        head = (head + 1) % ring.length;
        if (size == ring.length) {
            dropped++;
        } else {
            size++;
        }
    }

    /**
     * Main thread: copies queued samples, oldest first, into {@code out} and empties the queue.
     *
     * @return how many were copied (at most {@code out.length}; extra oldest ones are dropped)
     */
    public synchronized int drainTo(SwerveSample[] out) {
        int n = Math.min(size, out.length);
        dropped += size - n;
        int start = (head - n + ring.length) % ring.length;
        for (int i = 0; i < n; i++) {
            out[i].copyFrom(ring[(start + i) % ring.length]);
        }
        size = 0;
        return n;
    }

    /** Samples lost because the main loop fell behind, since boot. */
    public synchronized long dropped() {
        return dropped;
    }
}
