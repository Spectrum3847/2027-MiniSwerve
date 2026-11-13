package frc.spectrumLib.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;

import java.nio.file.Files;
import java.nio.file.Path;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

/** A heading comes back only after a quick code restart, never from another time. */
class HeadingMemoryTest {
    private static final long S = 1_000_000_000L;

    @Test
    void comesBackAfterAQuickRestart(@TempDir Path dir) {
        Path file = dir.resolve("heading");
        new HeadingMemory(file).save(1.25, -7.5, 100 * S);
        // A new program, 12 s later on the same clock.
        HeadingMemory.Saved saved = new HeadingMemory(file).load(112 * S);
        assertNotNull(saved);
        assertEquals(1.25, saved.yawOffset(), 0.0);
        assertEquals(-7.5, saved.yaw(), 0.0);
        assertEquals(12.0, saved.ageSeconds(), 1e-9);
    }

    @Test
    void theNewestSaveWins(@TempDir Path dir) {
        Path file = dir.resolve("heading");
        HeadingMemory m = new HeadingMemory(file);
        m.save(1.0, 0.0, 100 * S);
        m.save(2.0, 0.5, 101 * S);
        assertEquals(2.0, new HeadingMemory(file).load(102 * S).yawOffset(), 0.0);
    }

    @Test
    void tooOldIsIgnored(@TempDir Path dir) {
        Path file = dir.resolve("heading");
        new HeadingMemory(file).save(1.0, 0.0, 100 * S);
        assertNull(new HeadingMemory(file).load(100 * S + 31 * S));
    }

    @Test
    void aClockThatStartedAgainIsIgnored(@TempDir Path dir) {
        // The computer rebooted: its monotonic clock is now before the save.
        Path file = dir.resolve("heading");
        new HeadingMemory(file).save(1.0, 0.0, 5000 * S);
        assertNull(new HeadingMemory(file).load(20 * S));
    }

    @Test
    void noFileAndDamagedFilesAreIgnored(@TempDir Path dir) throws Exception {
        Path file = dir.resolve("heading");
        assertNull(new HeadingMemory(file).load(1 * S));
        new HeadingMemory(file).save(1.0, 0.0, 100 * S);
        byte[] bytes = Files.readAllBytes(file);
        bytes[10] ^= 0x40; // one flipped bit in the offset
        Files.write(file, bytes);
        assertNull(new HeadingMemory(file).load(101 * S));
        Files.write(file, new byte[7]); // cut short
        assertNull(new HeadingMemory(file).load(101 * S));
    }

    @Test
    void clearForgets(@TempDir Path dir) {
        Path file = dir.resolve("heading");
        HeadingMemory m = new HeadingMemory(file);
        m.save(1.0, 0.0, 100 * S);
        m.clear();
        assertNull(new HeadingMemory(file).load(101 * S));
        // And it can save again afterwards.
        m.save(3.0, 0.0, 102 * S);
        assertEquals(3.0, new HeadingMemory(file).load(103 * S).yawOffset(), 0.0);
    }

    @Test
    void noPathMeansNoMemory() {
        HeadingMemory m = new HeadingMemory(null);
        m.save(1.0, 0.0);
        assertNull(m.load());
    }
}
