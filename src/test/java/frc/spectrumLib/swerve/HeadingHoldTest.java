package frc.spectrumLib.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class HeadingHoldTest {
    private static final HeadingHold.Config C =
            new HeadingHold.Config(3.0, 1.0, 0.25, 0.1, Math.toRadians(10), 0.2);
    private static final double DT = 0.01;

    /** Runs settled, not turning, driving at 2 m/s, at heading h, for some loops. */
    private static double run(HeadingHold hold, double h, int loops) {
        double out = 0;
        for (int k = 0; k < loops; k++) {
            out = hold.calculate(C, false, 2.0, h, 0.0, DT);
        }
        return out;
    }

    @Test
    void capturesAfterSettlingThenTrimsDrift() {
        HeadingHold hold = new HeadingHold("Test/HeadingHold");
        assertEquals(0.0, run(hold, 0.0, 10)); // not settled long enough yet
        run(hold, 0.0, 30); // captures 0
        double out = run(hold, Math.toRadians(-3), 1); // drifted 3 deg clockwise
        assertTrue(out > 0, "corrects back counterclockwise");
        assertTrue(out <= C.maxCorrection());
    }

    @Test
    void aHitIsAcceptedNotFought() {
        HeadingHold hold = new HeadingHold("Test/HeadingHold");
        run(hold, 0.0, 40);
        assertEquals(0.0, run(hold, Math.toRadians(30), 1), 1e-12); // pushed 30 deg: re-capture
        double out = run(hold, Math.toRadians(28), 1); // and now holds the new heading
        assertTrue(out > 0 && out < 0.2);
    }

    @Test
    void driverTurningAndStoppedNeverCorrect() {
        HeadingHold hold = new HeadingHold("Test/HeadingHold");
        run(hold, 0.0, 40);
        assertEquals(0.0, hold.calculate(C, true, 2.0, 0.1, 0.0, DT));
        run(hold, 0.0, 40);
        assertEquals(0.0, hold.calculate(C, false, 0.0, Math.toRadians(-5), 0.0, DT));
    }
}
