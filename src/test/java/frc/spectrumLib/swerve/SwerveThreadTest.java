package frc.spectrumLib.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;

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

        @Override
        public void waitForSample(SwerveSample out) {}

        @Override
        public void setModules(double[] a, double[] v, double[] sv, double[] f) {
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
        java.util.Arrays.fill(t.sampleForTest().angle, -Math.PI / 2);
        t.computeTargets(DriveRequest.velocity(1, 0, 0, true, 0, true));
        for (int i = 0; i < 4; i++) {
            // Field +x is the robot's right.
            assertEquals(-Math.PI / 2, io.angle[i], 1e-6);
            assertEquals(1.0, io.velocity[i], 1e-6);
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
                DriveRequest.velocity(4.0, 0, 0, false, 0, true).withLimits(2.0, Double.NaN));
        assertEquals(2.0, io.velocity[0], 1e-9);
        t.computeTargets(
                DriveRequest.velocity(6.0, 0, 0, false, 0, true).withLimits(9.0, Double.NaN));
        assertEquals(4.5, io.velocity[0], 1e-9);
    }
}
