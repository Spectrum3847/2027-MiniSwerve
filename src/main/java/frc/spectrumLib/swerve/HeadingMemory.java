package frc.spectrumLib.swerve;

import java.nio.ByteBuffer;
import java.nio.channels.FileChannel;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;

/**
 * Remembers the field heading across a robot <em>code</em> restart, and only that. The gyro keeps
 * counting yaw while the code is down, so "field heading = gyro yaw + offset" is still true when
 * the code comes back; without this the heading starts again at zero and field-centric driving
 * points the wrong way until the driver re-zeroes it.
 *
 * <p>It must never bring back a heading from another time or place, so a saved value is used only
 * if all of these hold:
 *
 * <ul>
 *   <li>the file is in memory-backed storage ({@code /dev/shm}), which is empty after the robot is
 *       powered off;
 *   <li>it was written less than {@link #MAX_AGE_SECONDS} ago on the computer's monotonic clock
 *       (which restarts when the computer does), so a redeploy in the pit minutes later starts
 *       fresh;
 *   <li>its checksum is right (a write cut short by the crash is ignored);
 *   <li>the caller's own check of the gyro passes ({@link Saved#yaw}: a gyro that lost power starts
 *       again near zero).
 * </ul>
 *
 * <p>The write is 40 bytes to memory, in place, with no allocation: cheap enough for the main loop.
 * Any I/O failure turns the memory off for the rest of the run; it never throws.
 */
public final class HeadingMemory {
    /** Oldest saved heading that is still used (s): a code restart takes well under this. */
    public static final double MAX_AGE_SECONDS = 30.0;

    private static final long MAGIC = 0x4844474D454D3031L; // "HDGMEM01"
    private static final int SIZE = 40;

    /**
     * A saved heading.
     *
     * @param yawOffset field heading minus gyro yaw (rad)
     * @param yaw the gyro's yaw when it was saved (rad, continuous)
     * @param ageSeconds how long ago it was saved
     */
    public record Saved(double yawOffset, double yaw, double ageSeconds) {}

    private final Path file;
    private final ByteBuffer buffer = ByteBuffer.allocate(SIZE);
    private FileChannel channel;
    private boolean failed;
    private String lastError = "";

    /**
     * @param file where to keep it; memory-backed storage (e.g. {@code /dev/shm/...}), or null for
     *     none (sim, replay)
     */
    public HeadingMemory(Path file) {
        this.file = file;
    }

    /** The heading saved by the code that ran before this one, or null if there is none to use. */
    public Saved load() {
        return load(System.nanoTime());
    }

    /** {@link #load()} at a given monotonic time (ns), for tests. */
    Saved load(long nowNanos) {
        if (file == null) {
            return null;
        }
        try {
            byte[] bytes = java.nio.file.Files.readAllBytes(file);
            if (bytes.length != SIZE) {
                return null;
            }
            ByteBuffer b = ByteBuffer.wrap(bytes);
            long magic = b.getLong();
            double yawOffset = b.getDouble();
            double yaw = b.getDouble();
            long savedNanos = b.getLong();
            long sum = b.getLong();
            if (magic != MAGIC || sum != checksum(yawOffset, yaw, savedNanos)) {
                return null;
            }
            double age = (nowNanos - savedNanos) * 1e-9;
            if (!(age >= 0 && age <= MAX_AGE_SECONDS)
                    || !Double.isFinite(yawOffset)
                    || !Double.isFinite(yaw)) {
                return null;
            }
            return new Saved(yawOffset, yaw, age);
        } catch (java.io.IOException | RuntimeException e) {
            return null; // no file is the usual case: a fresh boot
        }
    }

    /**
     * Saves the heading now. Main loop, a few times a second.
     *
     * @param yawOffset field heading minus gyro yaw (rad)
     * @param yaw the gyro's yaw now (rad)
     */
    public void save(double yawOffset, double yaw) {
        save(yawOffset, yaw, System.nanoTime());
    }

    /** {@link #save(double, double)} at a given monotonic time (ns), for tests. */
    void save(double yawOffset, double yaw, long nowNanos) {
        if (file == null || failed || !Double.isFinite(yawOffset) || !Double.isFinite(yaw)) {
            return;
        }
        try {
            if (channel == null) {
                channel =
                        FileChannel.open(file, StandardOpenOption.CREATE, StandardOpenOption.WRITE);
            }
            buffer.clear();
            buffer.putLong(MAGIC);
            buffer.putDouble(yawOffset);
            buffer.putDouble(yaw);
            buffer.putLong(nowNanos);
            buffer.putLong(checksum(yawOffset, yaw, nowNanos));
            buffer.flip();
            channel.write(buffer, 0);
        } catch (java.io.IOException | RuntimeException e) {
            failed = true;
            lastError = e.toString();
        }
    }

    /** Forgets the saved heading (the driver re-zeroed, or it was judged wrong). */
    public void clear() {
        if (file == null) {
            return;
        }
        try {
            if (channel != null) {
                channel.close();
                channel = null;
            }
            java.nio.file.Files.deleteIfExists(file);
        } catch (java.io.IOException | RuntimeException e) {
            lastError = e.toString();
        }
    }

    /** Saving has failed and is off for this run; why is in {@link #lastError}. */
    public boolean failed() {
        return failed;
    }

    public String lastError() {
        return lastError;
    }

    private static long checksum(double yawOffset, double yaw, long nanos) {
        long h = MAGIC;
        h = h * 31 + Double.doubleToLongBits(yawOffset);
        h = h * 31 + Double.doubleToLongBits(yaw);
        h = h * 31 + nanos;
        return h;
    }
}
