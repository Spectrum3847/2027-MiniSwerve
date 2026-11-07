package frc.spectrumLib.swerve;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class CoastOnDisableTest {
    @Test
    void brakesWhileEnabledAndMovingThenCoastsWhenStill() {
        CoastOnDisable c = new CoastOnDisable(0.05, 0.5);
        assertTrue(c.brake(true, 0, 0.01));
        // Disabled but still rolling: keep braking.
        for (int k = 0; k < 100; k++) {
            assertTrue(c.brake(false, 1.0, 0.01));
        }
        // Still for 0.4 s: still braking; past 0.5 s: coast.
        for (int k = 0; k < 40; k++) {
            assertTrue(c.brake(false, 0.0, 0.01));
        }
        for (int k = 0; k < 15; k++) {
            c.brake(false, 0.0, 0.01);
        }
        assertFalse(c.brake(false, 0.0, 0.01));
        // Enabling brakes at once.
        assertTrue(c.brake(true, 0.0, 0.01));
    }
}
