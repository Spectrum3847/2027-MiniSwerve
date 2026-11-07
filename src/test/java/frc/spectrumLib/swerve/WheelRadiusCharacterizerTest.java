package frc.spectrumLib.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class WheelRadiusCharacterizerTest {
    private static final double H = 0.1778;
    private static final double[] MX = {H, H, -H, -H};
    private static final double[] MY = {H, -H, H, -H};

    @Test
    void findsTheTrueRadiusFromAWrongSetting() {
        double trueRadius = 0.0508;
        double setting = 0.053;
        var c = new WheelRadiusCharacterizer(MX, MY);
        var s = new SwerveSample(4);
        double base = Math.hypot(H, H);
        assertTrue(Double.isNaN(c.estimate()));
        for (int k = 0; k < 1000; k++) {
            double yaw = 1.5 * 0.004 * k;
            s.yaw = yaw;
            // Wheels roll yaw * base of ground; odometry reports it scaled by setting / true.
            double reported = yaw * base * setting / trueRadius;
            for (int i = 0; i < 4; i++) {
                s.distance[i] = (i % 2 == 0 ? -1 : 1) * reported;
            }
            c.update(s, setting);
        }
        assertEquals(trueRadius, c.estimate(), 1e-9);
    }
}
