package frc.spectrumLib.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class DriveMotorLimitTest {
    private static final double H = 0.1778;
    private static final double[] MX = {H, H, -H, -H};
    private static final double[] MY = {H, -H, H, -H};
    private static final double RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);

    /** Falcon FOC on MK4i L2, 2 in wheel, 60 A stator, 40 A supply, 25 kg robot. */
    private static DriveMotorLimit falcon() {
        return new DriveMotorLimit(5.84, 304, 6080, RATIO, 0.0508, 60, 40, 25, 4);
    }

    @Test
    void stallIsTheStatorLimitAndFallsToZeroAtFreeSpeed() {
        DriveMotorLimit m = falcon();
        double force = 60 * (5.84 / 304) * RATIO / 0.0508;
        assertEquals(force / 6.25, m.maxSpeedUp(0, 12), 1e-6);
        double free = 6080 / 60.0 / RATIO * 2 * Math.PI * 0.0508;
        assertEquals(0.0, m.maxSpeedUp(free, 12), 1e-6);
        assertTrue(m.maxSpeedUp(3, 12) < m.maxSpeedUp(1, 12), "less at speed");
        assertTrue(m.maxSpeedUp(3, 10) < m.maxSpeedUp(3, 12), "less on a sagging battery");
        assertEquals(m.maxSpeedUp(-2, 12), m.maxSpeedUp(2, 12), 1e-12);
    }

    @Test
    void generatorSpeedsUpSlowerNearTopSpeedButBrakesAtFullRate() {
        var g = new SwerveSetpointGenerator(MX, MY);
        g.reset(new double[4], new double[4]);
        g.setMotorLimit(falcon());
        g.setBusVoltage(12);
        double dt = 0.004;
        double last = 0;
        double firstStep = Double.NaN;
        double lateStep = Double.NaN;
        for (int k = 0; k < 400; k++) {
            g.generate(4.4, 0, 0, dt, 4.5, 30, 20, null);
            double v = g.velocity()[0];
            assertTrue(v - last <= falcon().maxSpeedUp(last, 12) * dt + 1e-9, "step " + k);
            if (k == 0) {
                firstStep = v - last;
            }
            if (v > 4.2 && Double.isNaN(lateStep)) {
                lateStep = v - last;
            }
            last = v;
        }
        assertTrue(lateStep < firstStep * 0.8, "speed-up falls off near top speed");
        // Braking is not held to the motor's speed-up limit.
        g.generate(0, 0, 0, dt, 4.5, 30, 20, null);
        assertEquals(30 * dt, last - g.velocity()[0], 1e-3); // bisection resolution
    }
}
