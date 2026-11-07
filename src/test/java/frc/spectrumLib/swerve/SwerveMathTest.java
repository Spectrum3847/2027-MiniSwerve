package frc.spectrumLib.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModuleVelocity;

/** Our allocation-free math must match WPILib's to the last digit that matters. */
class SwerveMathTest {
    private static final double H = 0.1778;
    private static final double[] MX = {H, H, -H, -H};
    private static final double[] MY = {H, -H, H, -H};
    private static final SwerveDriveKinematics WPI =
            new SwerveDriveKinematics(
                    new Translation2d(H, H),
                    new Translation2d(H, -H),
                    new Translation2d(-H, H),
                    new Translation2d(-H, -H));

    @Test
    void discretizeMatchesWpilib() {
        double[] out = new double[3];
        SwerveMath.discretize(2.0, -1.0, 3.0, 0.004, out);
        ChassisVelocities w = new ChassisVelocities(2.0, -1.0, 3.0).discretize(0.004);
        assertEquals(w.vx, out[0], 1e-9);
        assertEquals(w.vy, out[1], 1e-9);
        assertEquals(w.omega, out[2], 1e-9);
    }

    @Test
    void moduleVelocitiesMatchWpilib() {
        double[] speed = new double[4];
        double[] angle = new double[4];
        SwerveMath.toSwerveModuleVelocities(1.5, 0.7, -2.0, MX, MY, speed, angle);
        SwerveModuleVelocity[] w =
                WPI.toSwerveModuleVelocities(new ChassisVelocities(1.5, 0.7, -2.0));
        for (int i = 0; i < 4; i++) {
            assertEquals(w[i].velocity, speed[i], 1e-9);
            assertEquals(w[i].angle.getRadians(), angle[i], 1e-9);
        }
    }

    @Test
    void chassisFromModulesInvertsModules() {
        double[] speed = new double[4];
        double[] angle = new double[4];
        SwerveMath.toSwerveModuleVelocities(1.5, 0.7, -2.0, MX, MY, speed, angle);
        double[] dx = new double[4];
        double[] dy = new double[4];
        for (int i = 0; i < 4; i++) {
            dx[i] = speed[i] * Math.cos(angle[i]);
            dy[i] = speed[i] * Math.sin(angle[i]);
        }
        double[] out = new double[3];
        double slip = SwerveMath.toChassisVelocities(dx, dy, MX, MY, out);
        assertEquals(1.5, out[0], 1e-9);
        assertEquals(0.7, out[1], 1e-9);
        assertEquals(-2.0, out[2], 1e-9);
        assertEquals(0.0, slip, 1e-9);
    }

    @Test
    void optimizeTurnsTheShortWay() {
        double[] v = {1.0};
        double[] a = {Math.PI};
        SwerveMath.optimize(v, a, new double[] {0.1});
        assertEquals(-1.0, v[0], 1e-12);
        assertEquals(0.0, a[0], 1e-12);
    }

    @Test
    void desaturateKeepsRatios() {
        double[] v = {2.0, 4.0, 6.0, 8.0};
        SwerveMath.desaturateWheelVelocities(v, 4.0);
        assertEquals(1.0, v[0], 1e-12);
        assertEquals(4.0, v[3], 1e-12);
    }

    @Test
    void angleModulusMatchesWpilib() {
        for (double a = -10; a < 10; a += 0.37) {
            assertEquals(
                    org.wpilib.math.util.MathUtil.angleModulus(a),
                    SwerveMath.angleModulus(a),
                    1e-12);
        }
    }
}
