package frc.spectrumLib.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class RobotAccelerationLimiterTest {
    private static final double H = 0.1778;
    private static final double[] MX = {H, H, -H, -H};
    private static final double[] MY = {H, -H, H, -H};
    private static final double DT = 0.01;
    private static final double ACCEL = 10.0;
    private static final double TURN_ACCEL = 70.0;
    private static final double STEER = 20.0;
    private static final double STEER_ACCEL = 300.0;

    private static RobotAccelerationLimiter stopped() {
        var g = new RobotAccelerationLimiter(MX, MY);
        g.reset(new double[4], new double[4], 0);
        return g;
    }

    private static void step(RobotAccelerationLimiter g, double vx, double vy, double w, double h) {
        g.generate(vx, vy, w, h, DT, 4.5, ACCEL, TURN_ACCEL, STEER, STEER_ACCEL, null);
    }

    @Test
    void launchReachesTheRequestAtTheAccelerationLimit() {
        var g = stopped();
        int steps = 0;
        while (Math.abs(g.velocity()[0] - 4.0) > 1e-6 && steps < 200) {
            step(g, 4, 0, 0, 0);
            steps++;
            assertTrue(Math.hypot(g.accelerationX(), g.accelerationY()) <= ACCEL + 1e-6);
        }
        // 4 m/s at 10 m/s^2 is 0.4 s.
        assertEquals(40, steps, 1);
        for (int i = 0; i < 4; i++) {
            assertEquals(0.0, g.angle()[i], 1e-9);
            assertEquals(4.0, g.velocity()[i], 1e-6);
        }
    }

    @Test
    void drivingStraightWhileSpinningIsNotHeldBack() {
        // The case the wheel-acceleration limit could not follow: a straight line on the field
        // at a fast spin. Every wheel's speed swings each turn, but the robot does not accelerate.
        var g = stopped();
        double heading = 0;
        double spin = 0;
        // Slower than the corners' own speed from the spin (8 rad/s at 0.25 m): faster than
        // that, the wheels have to whip around each turn and the steer rate limits it.
        double speed = 1.0;
        double worstSpeedError = 0;
        for (int k = 0; k < 400; k++) {
            // Field +x, seen from the turning robot.
            step(g, speed * Math.cos(-heading), speed * Math.sin(-heading), 8.0, heading);
            spin += g.angularAcceleration() * DT;
            heading += spin * DT;
            if (k > 150) {
                assertEquals(1.0, g.lastFraction(), 1e-9, "limited at step " + k);
                assertTrue(
                        Math.hypot(g.accelerationX(), g.accelerationY()) < 1e-6,
                        "planned acceleration at step " + k);
                // The wheels together still carry the robot at the asked speed along field +x.
                double fx = 0;
                double fy = 0;
                for (int i = 0; i < 4; i++) {
                    fx += g.velocity()[i] * Math.cos(g.angle()[i] + heading) / 4;
                    fy += g.velocity()[i] * Math.sin(g.angle()[i] + heading) / 4;
                }
                worstSpeedError = Math.max(worstSpeedError, Math.hypot(fx - speed, fy));
            }
        }
        assertEquals(8.0, spin, 1e-6);
        assertTrue(worstSpeedError < 0.25, "field velocity error " + worstSpeedError);
    }

    @Test
    void accelerationAndTurnAccelerationShareOneBudget() {
        var g = stopped();
        for (int k = 0; k < 30; k++) {
            step(g, 4, 0, 8, 0);
            double used =
                    Math.hypot(g.accelerationX(), g.accelerationY()) / ACCEL
                            + Math.abs(g.angularAcceleration()) / TURN_ACCEL;
            assertTrue(used <= 1 + 1e-6, "traction used " + used + " at step " + k);
        }
    }

    @Test
    void aReversalFlipsTheWheelsInsteadOfTurningThem() {
        var g = stopped();
        for (int k = 0; k < 60; k++) {
            step(g, 3, 0, 0, 0);
        }
        assertEquals(3.0, g.velocity()[0], 1e-6);
        double worstAngle = 0;
        for (int k = 0; k < 100; k++) {
            // Not quite straight back: the line to it misses zero.
            step(g, -3, 0.3, 0, 0);
            if (k < 30) {
                worstAngle = Math.max(worstAngle, Math.abs(g.angle()[0]));
            }
        }
        // Braked with the wheels held, then drove backward (negative speed), a few degrees over.
        assertTrue(worstAngle < 0.02, "wheel turned " + worstAngle + " rad while braking");
        assertTrue(g.velocity()[0] < -2.9, "speed " + g.velocity()[0]);
        assertTrue(Math.abs(g.angle()[0]) < 0.2, "final angle " + g.angle()[0]);
    }

    @Test
    void steeringStaysWithinItsRateAndAccelerationLimits() {
        var g = stopped();
        double[] lastAngle = new double[4];
        double[] lastRate = new double[4];
        for (int k = 0; k < 600; k++) {
            // A stick swept in a fast circle, then stopped dead, then a spin.
            double a = k < 300 ? k * DT * 10 : 300 * DT * 10;
            double w = k > 450 ? 6 : 0;
            step(g, 3 * Math.cos(a), 3 * Math.sin(a), w, 0);
            for (int i = 0; i < 4; i++) {
                double turned = Math.abs(SwerveMath.angleModulus(g.angle()[i] - lastAngle[i]));
                assertTrue(turned <= STEER * DT + 1e-9, "steer rate at step " + k);
                assertTrue(
                        Math.abs(g.steerVelocity()[i] - lastRate[i]) <= STEER_ACCEL * DT + 1e-6,
                        "steer acceleration at step " + k);
                lastAngle[i] = g.angle()[i];
                lastRate[i] = g.steerVelocity()[i];
            }
        }
    }

    @Test
    void wheelsTurnBeforeTheRobotIsPushedSideways() {
        // Wheels point forward, the request is straight left: nearly nothing may be driven until
        // the wheels have come around.
        var g = stopped();
        for (int k = 0; k < 100; k++) {
            step(g, 0, 3, 0, 0);
            double off = Math.abs(Math.abs(g.angle()[0]) - Math.PI / 2);
            if (off > 0.5) {
                assertTrue(
                        Math.abs(g.velocity()[0]) < 0.3,
                        "driving " + g.velocity()[0] + " m/s with the wheel " + off + " rad off");
            }
        }
        assertEquals(3.0, Math.abs(g.velocity()[0]), 1e-3);
        assertEquals(Math.PI / 2, Math.abs(g.angle()[0]), 1e-3);
    }

    @Test
    void stopsCleanly() {
        var g = stopped();
        for (int k = 0; k < 60; k++) {
            step(g, 2, 2, 3, 0);
        }
        for (int k = 0; k < 80; k++) {
            step(g, 0, 0, 0, 0);
        }
        for (int i = 0; i < 4; i++) {
            assertEquals(0.0, g.velocity()[i], 1e-9);
            assertEquals(0.0, g.steerVelocity()[i], 1e-6);
        }
    }
}
