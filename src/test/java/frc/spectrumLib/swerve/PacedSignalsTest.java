package frc.spectrumLib.swerve;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.BaseStatusSignal;
import org.junit.jupiter.api.Test;

class PacedSignalsTest {
    @Test
    void dropsASilentDeviceAfterAFewCyclesAndTakesItBack() {
        var p = new PacedSignals(new BaseStatusSignal[0], new int[][] {{}, {}});
        for (int k = 1; k < PacedSignals.DROP_AFTER; k++) {
            assertFalse(p.update(0, false), "still waited on after " + k + " misses");
        }
        assertTrue(p.update(0, false), "dropped");
        assertFalse(p.waiting(0));
        assertTrue(p.waiting(1), "others untouched");
        assertFalse(p.update(0, false), "no change while still silent");
        assertTrue(p.update(0, true), "rejoins on its first frame");
        assertTrue(p.waiting(0));
        // A single miss then a frame never drops it.
        assertFalse(p.update(1, false));
        assertFalse(p.update(1, true));
        assertTrue(p.waiting(1));
    }
}
