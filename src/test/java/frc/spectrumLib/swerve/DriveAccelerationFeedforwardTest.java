package frc.spectrumLib.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class DriveAccelerationFeedforwardTest {
    private static final double H = 0.1778;
    private static final double[] MX = {H, H, -H, -H};
    private static final double[] MY = {H, -H, H, -H};
    private static final double MASS = 40.0;
    private static final double INERTIA = 1.5;
    private static final double ROTOR = 1.0;

    private static DriveAccelerationFeedforward feedforward(double gain) {
        return new DriveAccelerationFeedforward(MX, MY, MASS, INERTIA, ROTOR, gain);
    }

    @Test
    void straightAccelerationSharesMassAmongTheWheels() {
        double[] out = new double[4];
        double[] angle = new double[4];
        double[] wheelAccel = {5, 5, 5, 5};
        feedforward(1).calculate(5, 0, 0, angle, wheelAccel, null, out);
        for (double f : out) {
            // A quarter of the robot, plus the wheel's own rotor.
            assertEquals(MASS * 5 / 4 + ROTOR * 5, f, 1e-9);
        }
    }

    @Test
    void turnAccelerationGivesTheTurningTorque() {
        double[] out = new double[4];
        double[] angle = new double[4];
        for (int i = 0; i < 4; i++) {
            angle[i] = Math.atan2(MX[i], -MY[i]); // tangent to the circle through the modules
        }
        new DriveAccelerationFeedforward(MX, MY, MASS, INERTIA, 0, 1)
                .calculate(0, 0, 8, angle, new double[4], null, out);
        double torque = 0;
        for (int i = 0; i < 4; i++) {
            torque += out[i] * Math.hypot(MX[i], MY[i]);
        }
        assertEquals(INERTIA * 8, torque, 1e-9);
    }

    @Test
    void sidewaysForceIsLeftToTheTire() {
        double[] out = new double[4];
        // Wheels point forward, the robot accelerates sideways (a turn at speed): the drive
        // motors cannot give that force.
        feedforward(1).calculate(0, 6, 0, new double[4], new double[4], null, out);
        for (double f : out) {
            assertEquals(0.0, f, 1e-9);
        }
    }

    @Test
    void aWheelThatCannotPushGivesItsShareToTheOthers() {
        double[] out = new double[4];
        boolean[] pushing = {true, true, true, false};
        new DriveAccelerationFeedforward(MX, MY, MASS, INERTIA, 0, 1)
                .calculate(4, 0, 0, new double[4], new double[4], pushing, out);
        assertEquals(0.0, out[3], 1e-12);
        assertEquals(MASS * 4, out[0] + out[1] + out[2], 1e-9);
    }

    @Test
    void gainScalesTheForce() {
        double[] full = new double[4];
        double[] half = new double[4];
        double[] wheelAccel = {3, 3, 3, 3};
        feedforward(1).calculate(3, 0, 0, new double[4], wheelAccel, null, full);
        feedforward(0.5).calculate(3, 0, 0, new double[4], wheelAccel, null, half);
        assertEquals(full[0] / 2, half[0], 1e-9);
    }

    @Test
    void aNonFiniteInputSendsNoForce() {
        double[] out = {1, 1, 1, 1};
        feedforward(1).calculate(Double.NaN, 0, 0, new double[4], new double[4], null, out);
        for (double f : out) {
            assertEquals(0.0, f, 0.0);
        }
    }

    @Test
    void generatorReportsTheAccelerationItPlans() {
        var g = new SwerveSetpointGenerator(MX, MY);
        g.reset(new double[4], new double[4]);
        g.generate(4, 0, 0, 0.004, 4.5, 10, 20, null);
        // The bisection finds the step to 1/4096 of the request, so a little under the limit.
        assertEquals(10.0, g.accelerationX(), 0.3);
        assertEquals(0.0, g.accelerationY(), 1e-9);
        assertEquals(0.0, g.angularAcceleration(), 1e-9);
        for (double a : g.wheelAcceleration()) {
            assertEquals(g.accelerationX(), a, 1e-9);
        }
    }

    @Test
    void drivingStraightWhileSpinningPlansNoAcceleration() {
        // A straight line on the field at a steady spin: the robot-frame velocity turns at the
        // spin rate, but the floor sees no acceleration, so no force is needed.
        var g = new SwerveSetpointGenerator(MX, MY);
        g.reset(new double[4], new double[4]);
        double dt = 0.004;
        double speed = 2.0;
        double spin = 3.0;
        double worst = 0;
        for (int k = 0; k < 1500; k++) {
            double heading = spin * k * dt;
            g.generate(
                    speed * Math.cos(-heading),
                    speed * Math.sin(-heading),
                    spin,
                    dt,
                    4.5,
                    10,
                    20,
                    null);
            if (k > 1000) {
                worst = Math.max(worst, Math.hypot(g.accelerationX(), g.accelerationY()));
            }
        }
        // The robot-frame velocity changes at speed * spin = 6 m/s^2; almost none of it is real.
        assertTrue(worst < 0.3, "floor acceleration " + worst);
    }
}
