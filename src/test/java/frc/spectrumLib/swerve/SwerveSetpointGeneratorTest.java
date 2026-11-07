package frc.spectrumLib.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class SwerveSetpointGeneratorTest {
    private static final double H = 0.1778;
    private static final double[] MX = {H, H, -H, -H};
    private static final double[] MY = {H, -H, H, -H};
    private static final double DT = 0.004;
    private static final double ACCEL = 10.0;
    private static final double STEER = 20.0;

    private static SwerveSetpointGenerator stopped() {
        var g = new SwerveSetpointGenerator(MX, MY);
        g.reset(new double[4], new double[4]);
        return g;
    }

    @Test
    void accelerationAndSteerRateStayWithinLimits() {
        var g = stopped();
        double[] lastV = new double[4];
        double[] lastA = new double[4];
        for (int k = 0; k < 500; k++) {
            // Forward, then a hard request sideways while spinning, then full reverse.
            double vx = k < 150 ? 4 : k < 300 ? 0 : -4;
            double vy = k >= 150 && k < 300 ? 3 : 0;
            double w = k >= 150 && k < 300 ? 5 : 0;
            g.generate(vx, vy, w, DT, 4.5, ACCEL, STEER);
            for (int i = 0; i < 4; i++) {
                double dv = Math.abs(g.velocity()[i] - lastV[i]);
                double da = Math.abs(SwerveMath.angleModulus(g.angle()[i] - lastA[i]));
                assertTrue(dv <= ACCEL * DT + 1e-6, "accel at step " + k + " module " + i);
                assertTrue(da <= STEER * DT + 1e-9, "steer rate at step " + k + " module " + i);
                lastV[i] = g.velocity()[i];
                lastA[i] = g.angle()[i];
            }
        }
    }

    @Test
    void reachesTheRequest() {
        var g = stopped();
        for (int k = 0; k < 250; k++) {
            g.generate(2.0, 0, 0, DT, 4.5, ACCEL, STEER);
        }
        for (int i = 0; i < 4; i++) {
            assertEquals(2.0, g.velocity()[i], 1e-6);
            assertEquals(0.0, g.angle()[i], 1e-6);
        }
    }

    @Test
    void reversingBrakesThroughZeroWithoutTurningTheWheels() {
        var g = stopped();
        for (int k = 0; k < 250; k++) {
            g.generate(3.0, 0, 0, DT, 4.5, ACCEL, STEER);
        }
        for (int k = 0; k < 250; k++) {
            g.generate(-3.0, 0, 0, DT, 4.5, ACCEL, STEER);
            for (int i = 0; i < 4; i++) {
                assertEquals(0.0, g.angle()[i], 1e-9, "wheels stay pointed forward");
            }
        }
        assertEquals(-3.0, g.velocity()[0], 1e-6);
    }

    @Test
    void desaturatesTheWholeChassis() {
        var g = stopped();
        for (int k = 0; k < 500; k++) {
            g.generate(10.0, 0, 0, DT, 4.5, ACCEL, STEER);
        }
        assertEquals(4.5, g.velocity()[0], 1e-6);
    }
}
