package frc.spectrumLib.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class SampleGuardTest {
    private static SwerveSample sample(double t) {
        SwerveSample s = new SwerveSample(4);
        s.timestamp = t;
        java.util.Arrays.fill(s.moduleOk, true);
        s.gyroOk = true;
        return s;
    }

    @Test
    void normalMotionPassesAndARebootJumpIsRejectedOnce() {
        SampleGuard g = new SampleGuard(4, 10, 40, 25, 0.004);
        SwerveSample s = sample(0);
        g.check(s);
        // 4 m/s and a fast turn: fine.
        s = sample(0.004);
        s.distance[0] = 0.016;
        s.angle[0] = 0.1;
        s.yaw = 0.05;
        g.check(s);
        assertTrue(s.moduleOk[0]);
        assertTrue(s.gyroOk);
        // Module 1's steer reboots: its reading jumps by the gear ratio.
        s = sample(0.008);
        s.distance[0] = 0.032;
        s.angle[0] = 0.1;
        s.distance[1] = -0.8;
        s.angle[1] = 2.5;
        s.yaw = 0.05;
        g.check(s);
        assertTrue(s.moduleOk[0]);
        assertFalse(s.moduleOk[1]);
        // The next reading continues from the new one: accepted.
        s = sample(0.012);
        s.distance[0] = 0.048;
        s.angle[0] = 0.1;
        s.distance[1] = -0.79;
        s.angle[1] = 2.5;
        s.yaw = 0.05;
        g.check(s);
        assertTrue(s.moduleOk[1]);
        assertEquals(1, g.rejected());
    }

    @Test
    void aPigeonRebootYawJumpIsRejected() {
        SampleGuard g = new SampleGuard(4, 10, 40, 25, 0.004);
        SwerveSample s = sample(0);
        s.yaw = 7.0;
        g.check(s);
        s = sample(0.004);
        s.yaw = 0.0; // rebooted: yaw starts from zero
        g.check(s);
        assertFalse(s.gyroOk);
        assertTrue(g.yawJumped());
        s = sample(0.008);
        s.yaw = 0.01;
        g.check(s);
        assertTrue(s.gyroOk);
        assertFalse(g.yawJumped());
    }
}
