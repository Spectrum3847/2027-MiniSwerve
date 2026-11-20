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
            g.generate(vx, vy, w, DT, 4.5, ACCEL, STEER, null);
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
            g.generate(2.0, 0, 0, DT, 4.5, ACCEL, STEER, null);
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
            g.generate(3.0, 0, 0, DT, 4.5, ACCEL, STEER, null);
        }
        for (int k = 0; k < 250; k++) {
            g.generate(-3.0, 0, 0, DT, 4.5, ACCEL, STEER, null);
            for (int i = 0; i < 4; i++) {
                assertEquals(0.0, g.angle()[i], 1e-9, "wheels stay pointed forward");
            }
        }
        assertEquals(-3.0, g.velocity()[0], 1e-6);
    }

    @Test
    void aStickSweptInACircleNeverStopsTheWheels() {
        // Full stick swept around at 1.6 rev/s, as drivers did on the stand: about 10 rad/s, well
        // under the steer limit. The wheels must turn to follow it, not stop: braking for each
        // cycle's own step once held turns to ~3.3 rad/s, and the lag that built up then looked
        // like a reversal.
        var g = stopped();
        double slowest = Double.MAX_VALUE;
        for (int k = 0; k < 2500; k++) {
            double dir = 2 * Math.PI * 1.6 * k * DT;
            g.generate(4 * Math.cos(dir), 4 * Math.sin(dir), 0, DT, 4.5, ACCEL, STEER, null);
            if (k > 250) {
                for (int i = 0; i < 4; i++) {
                    slowest = Math.min(slowest, Math.abs(g.velocity()[i]));
                }
            }
        }
        assertTrue(slowest > 3.5, "slowest wheel " + slowest + " m/s");
    }

    @Test
    void stoppingASweptStickSlowsTheWheelsWithinTheSteerLimit() {
        // A fast sweep, then the stick held still: the wheels were planned to stop turning in one
        // cycle from ~17 rad/s, and the steer motors slammed and overshot ~20 deg on the robot.
        var g = stopped();
        double maxRateStep = SwerveSetpointGenerator.DEFAULT_STEER_ACCELERATION * DT + 1e-6;
        double[] lastA = new double[4];
        double[] lastRate = new double[4];
        double dir = 0;
        for (int k = 0; k < 600; k++) {
            if (k < 300) {
                dir = 2 * Math.PI * 1.6 * k * DT; // then held where it stopped
            }
            g.generate(4 * Math.cos(dir), 4 * Math.sin(dir), 0, DT, 4.5, ACCEL, STEER, null);
            for (int i = 0; i < 4; i++) {
                double rate = SwerveMath.angleModulus(g.angle()[i] - lastA[i]) / DT;
                if (k > 1) {
                    assertTrue(
                            Math.abs(rate - lastRate[i]) <= maxRateStep,
                            "turn rate jumped "
                                    + Math.abs(rate - lastRate[i])
                                    + " rad/s at step "
                                    + k);
                }
                lastA[i] = g.angle()[i];
                lastRate[i] = rate;
            }
        }
        for (int i = 0; i < 4; i++) {
            double off = Math.abs(SwerveMath.angleModulus(g.angle()[i] - dir));
            assertTrue(Math.min(off, Math.PI - off) < Math.toRadians(1), "settled on the stick");
        }
    }

    @Test
    void reversingAFewDegreesOffStraightFlipsTheWheelsInsteadOfTurningThem() {
        // Field-relative driving is never exactly along the robot's x: the stick and heading put
        // a small sideways part in both requests, so a straight line between them misses zero.
        double off = Math.toRadians(3);
        var g = stopped();
        for (int k = 0; k < 250; k++) {
            g.generate(3.0 * Math.cos(off), 3.0 * Math.sin(off), 0, DT, 4.5, ACCEL, STEER, null);
        }
        double start = g.angle()[0];
        for (int k = 0; k < 250; k++) {
            g.generate(
                    -3.0 * Math.cos(-off), -3.0 * Math.sin(-off), 0, DT, 4.5, ACCEL, STEER, null);
            for (int i = 0; i < 4; i++) {
                double turned = Math.abs(SwerveMath.angleModulus(g.angle()[i] - start));
                assertTrue(turned < Math.toRadians(10), "step " + k + " module " + i + " turned");
            }
        }
        assertTrue(
                g.velocity()[0] < -2.9,
                "drives backward at the end: "
                        + g.velocity()[0]
                        + " at "
                        + Math.toDegrees(g.angle()[0]));
    }

    @Test
    void desaturatesTheWholeChassis() {
        var g = stopped();
        for (int k = 0; k < 500; k++) {
            g.generate(10.0, 0, 0, DT, 4.5, ACCEL, STEER, null);
        }
        assertEquals(4.5, g.velocity()[0], 1e-6);
    }

    @Test
    void forwardThenSidewaysNeverFreezesOrJumps() {
        var g = stopped();
        for (int k = 0; k < 250; k++) {
            g.generate(4.0, 0, 0, DT, 4.5, ACCEL, STEER, null);
        }
        double[] lastV = g.velocity().clone();
        double minFraction = 1;
        boolean progressed = false;
        for (int k = 0; k < 500; k++) {
            g.generate(0, 4.0, 0, DT, 4.5, ACCEL, STEER, null);
            for (int i = 0; i < 4; i++) {
                assertTrue(Math.abs(g.velocity()[i] - lastV[i]) <= ACCEL * DT + 1e-6);
                lastV[i] = g.velocity()[i];
            }
            minFraction = Math.min(minFraction, g.lastFraction());
            progressed |= g.lastFraction() > 0;
        }
        assertTrue(progressed);
        for (int i = 0; i < 4; i++) {
            assertEquals(4.0, Math.abs(g.velocity()[i]), 1e-6);
            assertEquals(Math.PI / 2, Math.abs(g.angle()[i]), 1e-6);
        }
    }

    @Test
    void stepZeroAlwaysFits() {
        var g = new SwerveSetpointGenerator(MX, MY);
        // Moving and turning when reset: it may first wait for the wheels to turn, but it must
        // get going within a few cycles (never freeze).
        g.reset(new double[] {2.1, 1.9, 2.0, 2.05}, new double[] {0.3, 0.25, 0.32, 0.28});
        boolean moved = false;
        for (int k = 0; k < 50 && !moved; k++) {
            g.generate(0, -3, 6, DT, 4.5, ACCEL, STEER, null);
            moved = g.lastFraction() > 0;
        }
        assertTrue(moved);
    }

    @Test
    void aWheelThatCannotPushDoesNotCapTheOthers() {
        var g = stopped();
        boolean[] pushing = {true, true, true, false};
        // Spin plus drive: the back-right wheel is fastest; with it passive, the rest reach 4.5.
        for (int k = 0; k < 1000; k++) {
            g.generate(3.5, 0, 6.0, DT, 4.5, ACCEL, STEER, pushing);
        }
        double topPushing = 0;
        for (int i = 0; i < 3; i++) {
            topPushing = Math.max(topPushing, Math.abs(g.velocity()[i]));
        }
        assertEquals(4.5, topPushing, 1e-3);
    }
}
