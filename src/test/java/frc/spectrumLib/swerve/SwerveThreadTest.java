package frc.spectrumLib.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

/** Target math in the thread, against a fake IO that records what it was sent. */
class SwerveThreadTest {
    private static final double H = 0.1778;
    private static final double[] MX = {H, H, -H, -H};
    private static final double[] MY = {H, -H, H, -H};

    private static final class RecordingIO implements SwerveIO {
        double[] angle = new double[4];
        double[] velocity = new double[4];
        double[] force = new double[4];
        boolean neutral;
        ModuleCommands commands;

        @Override
        public void waitForSample(SwerveSample out) {}

        @Override
        public void setModules(double[] a, double[] v, double[] sv, double[] f, ModuleCommands c) {
            commands = c;
            angle = a.clone();
            velocity = v.clone();
            force = f.clone();
            neutral = false;
        }

        @Override
        public void setNeutral() {
            neutral = true;
        }

        @Override
        public void updateHealth(SwerveHealth out) {}
    }

    private static SwerveThread thread(RecordingIO io) {
        return new SwerveThread(io, new SampleQueue(4, 4), MX, MY, 4.5, 0.004, 0);
    }

    @Test
    void wheelForceProjectsOntoEachWheel() {
        RecordingIO io = new RecordingIO();
        SwerveThread t = thread(io);
        double[] fx = {10, 10, 10, 10};
        double[] fy = {0, 0, 0, 0};
        t.computeTargets(
                new DriveRequest(
                        DriveRequest.Mode.VELOCITY,
                        1,
                        0,
                        0,
                        false,
                        0,
                        true,
                        fx,
                        fy,
                        Double.NaN,
                        Double.NaN,
                        Double.NaN,
                        Double.NaN));
        for (int i = 0; i < 4; i++) {
            assertEquals(0.0, io.angle[i], 1e-9);
            assertEquals(10.0, io.force[i], 1e-9);
        }
    }

    @Test
    void wheelForceFollowsAFlippedWheel() {
        RecordingIO io = new RecordingIO();
        SwerveThread t = thread(io);
        // Wheels point forward; driving backward flips the speed, not the wheel.
        double[] fx = {-10, -10, -10, -10};
        double[] fy = {0, 0, 0, 0};
        t.computeTargets(
                new DriveRequest(
                        DriveRequest.Mode.VELOCITY,
                        -1,
                        0,
                        0,
                        false,
                        0,
                        true,
                        fx,
                        fy,
                        Double.NaN,
                        Double.NaN,
                        Double.NaN,
                        Double.NaN));
        for (int i = 0; i < 4; i++) {
            assertEquals(0.0, io.angle[i], 1e-9);
            assertEquals(-1.0, io.velocity[i], 1e-9);
            assertEquals(-10.0, io.force[i], 1e-9);
        }
    }

    @Test
    void fieldRelativeRotatesByHeading() {
        RecordingIO io = new RecordingIO();
        SwerveThread t = thread(io);
        t.sampleForTest().yaw = Math.PI / 2; // robot faces field +y
        t.sampleForTest().gyroOk = true;
        java.util.Arrays.fill(t.sampleForTest().angle, -Math.PI / 2);
        t.computeTargets(DriveRequest.velocity(1, 0, 0, true, 0, true));
        for (int i = 0; i < 4; i++) {
            // Field +x is the robot's right.
            assertEquals(-Math.PI / 2, io.angle[i], 1e-6);
            assertEquals(1.0, io.velocity[i], 1e-6);
        }
    }

    @Test
    void aStaleGyroSampleHoldsTheLastHeadingInsteadOfItsYaw() {
        RecordingIO io = new RecordingIO();
        SwerveThread t = thread(io);
        t.sampleForTest().yaw = Math.PI / 2;
        t.sampleForTest().gyroOk = true;
        java.util.Arrays.fill(t.sampleForTest().angle, -Math.PI / 2);
        t.computeTargets(DriveRequest.velocity(1, 0, 0, true, 0, true));
        // The gyro drops before the main loop notices: its yaw keeps extrapolating, wrongly.
        t.sampleForTest().gyroOk = false;
        t.sampleForTest().yaw = Math.PI;
        t.computeTargets(DriveRequest.velocity(1, 0, 0, true, 0, true));
        for (int i = 0; i < 4; i++) {
            assertEquals(-Math.PI / 2, io.angle[i], 1e-6);
        }
    }

    @Test
    void neutralSendsNeutral() {
        RecordingIO io = new RecordingIO();
        thread(io).computeTargets(DriveRequest.NEUTRAL);
        assertEquals(true, io.neutral);
    }

    @Test
    void aRequestCanLowerTopSpeedButNotRaiseIt() {
        RecordingIO io = new RecordingIO();
        SwerveThread t = thread(io); // configured top wheel speed 4.5 m/s
        t.computeTargets(
                DriveRequest.velocity(4.0, 0, 0, false, 0, true)
                        .withLimits(2.0, Double.NaN, Double.NaN, Double.NaN));
        assertEquals(2.0, io.velocity[0], 1e-9);
        t.computeTargets(
                DriveRequest.velocity(6.0, 0, 0, false, 0, true)
                        .withLimits(9.0, Double.NaN, Double.NaN, Double.NaN));
        assertEquals(4.5, io.velocity[0], 1e-9);
    }

    @Test
    void zeroOrNegativeLimitsAreClamped() {
        RecordingIO io = new RecordingIO();
        SwerveThread t = thread(io);
        t.computeTargets(
                DriveRequest.velocity(3.0, 0, 0, false, 0, true)
                        .withLimits(-1, -1, Double.NaN, Double.NaN));
        assertEquals(0.0, io.velocity[0], 1e-9);
    }

    @Test
    void turnRateLimit() {
        RecordingIO io = new RecordingIO();
        SwerveThread t = thread(io);
        // Spin in place at 5 rad/s requested, limited to 1 rad/s: wheel speed = 1 * module radius.
        for (int i = 0; i < 4; i++) {
            t.sampleForTest().angle[i] = Math.atan2(MX[i], -MY[i]); // already on the spin angles
        }
        t.computeTargets(
                DriveRequest.velocity(0, 0, 5.0, false, 0, true)
                        .withLimits(Double.NaN, Double.NaN, 1.0, Double.NaN));
        assertEquals(Math.hypot(H, H), Math.abs(io.velocity[0]), 1e-3);
    }

    @Test
    void turnAccelerationLimit() {
        RecordingIO io = new RecordingIO();
        SwerveThread t = thread(io);
        // From still, ask for 5 rad/s with turn acceleration limited to 10 rad/s^2: after one
        // 4 ms cycle the turn rate is at most 0.04 rad/s.
        t.computeTargets(
                DriveRequest.velocity(0, 0, 5.0, false, 0, true)
                        .withLimits(Double.NaN, Double.NaN, Double.NaN, 10.0));
        assertTrue(Math.abs(io.velocity[0]) <= 0.04 * Math.hypot(H, H) + 1e-6);
    }

    @Test
    void degradedAndStaleModulesNeverStopTheOthers() {
        RecordingIO io = new RecordingIO();
        SwerveThread t = thread(io);
        ModuleCommands c =
                new ModuleCommands(
                        new boolean[] {false, false, true, true},
                        new boolean[] {true, false, true, true},
                        new boolean[4],
                        new boolean[4]);
        t.setModuleCommands(c);
        java.util.Arrays.fill(t.sampleForTest().moduleOk, true);
        t.sampleForTest().moduleOk[3] = false; // stale reading on a healthy module
        t.sampleForTest().allGood = false;
        t.computeTargets(DriveRequest.velocity(2.0, 0, 0, false, 0, true));
        assertEquals(false, io.neutral);
        assertEquals(c, io.commands);
        assertEquals(2.0, io.velocity[2], 1e-9);
        assertEquals(2.0, io.velocity[3], 1e-9);
    }

    @Test
    void aDeadWheelsForceShareGoesToTheOthers() {
        RecordingIO io = new RecordingIO();
        SwerveThread t = thread(io);
        t.setModuleCommands(
                new ModuleCommands(
                        new boolean[] {true, true, true, false},
                        new boolean[] {true, true, true, true},
                        new boolean[4],
                        new boolean[4]));
        double[] fx = {10, 10, 10, 10};
        double[] fy = {0, 0, 0, 0};
        t.computeTargets(
                new DriveRequest(
                        DriveRequest.Mode.VELOCITY,
                        1,
                        0,
                        0,
                        false,
                        0,
                        true,
                        fx,
                        fy,
                        Double.NaN,
                        Double.NaN,
                        Double.NaN,
                        Double.NaN));
        double total = 0;
        for (int i = 0; i < 4; i++) {
            total += io.force[i];
        }
        assertEquals(40.0, total, 1e-6);
        assertEquals(0.0, io.force[3], 1e-9);
    }
}
