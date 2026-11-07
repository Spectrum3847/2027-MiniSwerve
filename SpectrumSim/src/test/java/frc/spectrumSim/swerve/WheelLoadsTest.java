package frc.spectrumSim.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class WheelLoadsTest {
    private static final double H = 0.25;
    private static final double[] MX = {H, H, -H, -H};
    private static final double[] MY = {H, -H, H, -H};
    private static final double MASS = 50;
    private static final double W = MASS * 9.81;

    @Test
    void standingStillSplitsEvenly() {
        var l = new WheelLoads(MX, MY);
        l.update(MASS, 0.3, 0, 0);
        for (int i = 0; i < 4; i++) {
            assertEquals(W / 4, l.load(i), 1e-9);
        }
    }

    @Test
    void speedingUpLoadsTheBackWheels() {
        var l = new WheelLoads(MX, MY);
        double a = 5;
        l.update(MASS, 0.3, a, 0);
        // Front pair loses m a h / wheelbase; each front wheel half of it.
        double shift = MASS * a * 0.3 / (2 * H) / 2;
        assertEquals(W / 4 - shift, l.load(0), 1e-9);
        assertEquals(W / 4 + shift, l.load(2), 1e-9);
        assertEquals(W, l.load(0) + l.load(1) + l.load(2) + l.load(3), 1e-9);
    }

    @Test
    void turningLeftLoadsTheRightWheels() {
        var l = new WheelLoads(MX, MY);
        l.update(MASS, 0.3, 0, 4); // centripetal acceleration to the left
        assertTrue(l.load(1) > l.load(0), "right wheels carry more");
    }

    @Test
    void oneCornerUpTheOtherThreeBalanceExactly() {
        var l = new WheelLoads(MX, MY);
        l.update(MASS, 0.3, 4.5, 4.5); // hard diagonal: front-left lifts
        assertTrue(l.lifted(0));
        assertEquals(0, l.load(0), 1e-9);
        assertEquals(110.25, l.load(1), 1e-6);
        assertEquals(110.25, l.load(2), 1e-6);
        assertEquals(270.0, l.load(3), 1e-6);
    }

    @Test
    void pastTheTippingPointTheFrontWheelsLift() {
        var l = new WheelLoads(MX, MY);
        // Tips when a / g > half wheelbase / height: 0.25 / 0.3 g is about 8.2 m/s^2.
        l.update(MASS, 0.3, 7.5, 0);
        assertFalse(l.lifted(0));
        l.update(MASS, 0.3, 9.0, 0);
        assertTrue(l.lifted(0) && l.lifted(1));
        assertEquals(0, l.load(0), 1e-9);
        assertEquals(W, l.load(2) + l.load(3), 1e-9);
    }
}
