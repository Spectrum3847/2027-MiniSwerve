package frc.spectrumLib.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class SwerveOdometryTest {
    private static final double H = 0.1778;
    private static final double[] MX = {H, H, -H, -H};
    private static final double[] MY = {H, -H, H, -H};

    /** Drives a chassis velocity for {@code seconds} as 250 Hz samples. */
    /** A sample with every module reporting fresh readings. */
    private static SwerveSample fresh() {
        SwerveSample s = new SwerveSample(4);
        java.util.Arrays.fill(s.moduleOk, true);
        return s;
    }

    @Test
    void aModuleBackFromADropoutDoesNotMoveThePose() {
        SwerveOdometry odo = new SwerveOdometry(MX, MY);
        SwerveSample s = fresh();
        odo.update(s, true);
        s.moduleOk[0] = false;
        odo.update(s, true);
        // Unheard for a while; when it comes back its reading carries 0.6 m of travel at once.
        s.distance[0] = 0.6;
        s.moduleOk[0] = true;
        odo.update(s, true);
        assertEquals(0.0, odo.x(), 1e-12);
        assertEquals(0.0, odo.y(), 1e-12);
        // From then on it counts again.
        for (int i = 0; i < 4; i++) {
            s.distance[i] += 0.01;
        }
        odo.update(s, true);
        assertEquals(0.01, odo.x(), 1e-9);
    }

    @Test
    void headingSurvivesEverythingRebootingAtOnce() {
        SwerveOdometry odo = new SwerveOdometry(MX, MY);
        odo.resetPose(0, 0, 0.5);
        SwerveSample s = fresh();
        s.yaw = 1.0;
        odo.update(s, true);
        double before = odo.heading();
        // Brownout: every reading rejected for one sample, and the gyro's yaw restarts at zero.
        java.util.Arrays.fill(s.moduleOk, false);
        s.yaw = 0.0;
        odo.update(s, false);
        java.util.Arrays.fill(s.moduleOk, true);
        odo.update(s, true);
        assertEquals(0.5, before, 1e-12);
        assertEquals(before, odo.heading(), 1e-12);
    }

    private static void drive(
            SwerveOdometry odo, SwerveSample s, double vx, double vy, double w, double seconds) {
        double[] speed = new double[4];
        double[] angle = new double[4];
        SwerveMath.toSwerveModuleVelocities(vx, vy, w, MX, MY, speed, angle);
        for (int k = 0; k < Math.round(seconds * 250); k++) {
            for (int i = 0; i < 4; i++) {
                s.distance[i] += speed[i] * 0.004;
                s.angle[i] = angle[i];
            }
            s.yaw += w * 0.004;
            s.timestamp += 0.004;
            odo.update(s, true);
        }
    }

    @Test
    void straightLine() {
        SwerveOdometry odo = new SwerveOdometry(MX, MY);
        SwerveSample s = fresh();
        odo.update(s, true);
        drive(odo, s, 1.0, 0, 0, 2.0);
        assertEquals(2.0, odo.x(), 1e-9);
        assertEquals(0.0, odo.y(), 1e-9);
    }

    @Test
    void headingResetRotatesTheFrame() {
        SwerveOdometry odo = new SwerveOdometry(MX, MY);
        SwerveSample s = fresh();
        odo.update(s, true);
        odo.resetPose(0, 0, Math.PI / 2);
        drive(odo, s, 1.0, 0, 0, 1.0);
        assertEquals(0.0, odo.x(), 1e-9);
        assertEquals(1.0, odo.y(), 1e-9);
    }

    @Test
    void driveWhileSpinningStaysOnTheLine() {
        SwerveOdometry odo = new SwerveOdometry(MX, MY);
        SwerveSample s = fresh();
        odo.update(s, true);
        // Field-relative 1 m/s along x while spinning 2 rad/s, via robot-frame velocities.
        double[] speed = new double[4];
        double[] angle = new double[4];
        for (int k = 0; k < 500; k++) {
            double heading = s.yaw;
            double rx = Math.cos(heading);
            double ry = -Math.sin(heading);
            double[] d = new double[3];
            SwerveMath.discretize(rx, ry, 2.0, 0.004, d);
            SwerveMath.toSwerveModuleVelocities(d[0], d[1], d[2], MX, MY, speed, angle);
            for (int i = 0; i < 4; i++) {
                s.distance[i] += speed[i] * 0.004;
                s.angle[i] = angle[i];
            }
            s.yaw += 2.0 * 0.004;
            odo.update(s, true);
        }
        assertEquals(2.0, odo.x(), 0.01);
        assertEquals(0.0, odo.y(), 0.01);
    }

    @Test
    void aBadModuleIsLeftOut() {
        SwerveOdometry odo = new SwerveOdometry(MX, MY);
        SwerveSample s = fresh();
        odo.update(s, true);
        double[] weight = {1, 1, 1, 0};
        for (int k = 0; k < 250; k++) {
            for (int i = 0; i < 3; i++) {
                s.distance[i] += 1.0 * 0.004;
            }
            s.distance[3] += 5.0 * 0.004; // a slipping or broken module reads nonsense
            s.timestamp += 0.004;
            odo.update(s, true, weight);
        }
        assertEquals(1.0, odo.x(), 1e-9);
        assertEquals(0.0, odo.y(), 1e-9);
        assertEquals(3, odo.modulesUsed());
        assertEquals(0.016, odo.moduleSlip()[3], 1e-9); // still reported
    }
}
