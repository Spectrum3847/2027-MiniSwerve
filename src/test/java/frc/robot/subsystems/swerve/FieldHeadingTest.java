package frc.robot.subsystems.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.spectrumLib.swerve.HeadingMemory;
import frc.spectrumLib.swerve.SwerveInputs;
import frc.spectrumLib.swerve.SwerveOdometry;
import org.junit.jupiter.api.Test;

/** The saved heading comes back after a code restart, unless the gyro looks restarted. */
class FieldHeadingTest {
    private static final double H = 0.1778;
    private static final double[] MX = {H, H, -H, -H};
    private static final double[] MY = {H, -H, H, -H};

    private static SwerveInputs sampleAt(double yaw) {
        SwerveInputs in = new SwerveInputs(4, 4);
        in.count = 1;
        in.samples[0].yaw = yaw;
        in.samples[0].gyroOk = true;
        java.util.Arrays.fill(in.samples[0].moduleOk, true);
        return in;
    }

    private static double run(SwerveInputs in, int[] jumps) {
        SwerveOdometry odometry = new SwerveOdometry(MX, MY);
        FieldHeading heading =
                new FieldHeading(odometry, new HeadingMemory(null), () -> jumps[0]++);
        // Two loops: the first sample primes odometry, as on the robot.
        for (int i = 0; i < 2; i++) {
            odometry.update(in.samples[0], true);
            heading.update(in, true, false, false);
        }
        return odometry.heading();
    }

    @Test
    void restoresTheSavedHeading() {
        // Saved: heading = yaw + 1.0, at yaw 5.0. The robot coasted a little while the code was
        // down: yaw is now 5.3, so the heading is 6.3.
        SwerveInputs in = sampleAt(5.3);
        in.restoreYawOffset = 1.0;
        in.restoreYaw = 5.0;
        in.restoreAgeSeconds = 8.0;
        int[] jumps = {0};
        assertEquals(6.3, run(in, jumps), 1e-9);
        assertEquals(1, jumps[0]);
    }

    @Test
    void aGyroThatStartedAgainIsNotBelieved() {
        // Saved at yaw 40 rad (many turns in a match); the gyro now reads 0.1: it lost power.
        SwerveInputs in = sampleAt(0.1);
        in.restoreYawOffset = 1.0;
        in.restoreYaw = 40.0;
        in.restoreAgeSeconds = 8.0;
        int[] jumps = {0};
        assertEquals(0.1, run(in, jumps), 1e-9);
        assertEquals(0, jumps[0]);
    }

    @Test
    void nothingSavedLeavesTheHeadingAlone() {
        int[] jumps = {0};
        assertEquals(2.0, run(sampleAt(2.0), jumps), 1e-9);
        assertEquals(0, jumps[0]);
    }
}
