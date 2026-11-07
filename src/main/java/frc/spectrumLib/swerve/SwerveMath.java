package frc.spectrumLib.swerve;

/**
 * Allocation-free swerve kinematics on plain arrays, for the 250 Hz swerve thread.
 *
 * <p>WPILib's kinematics returns new arrays and objects on every call; at 250 Hz that is garbage
 * the robot does not need. Everything here writes into arrays the caller owns. Angles are radians,
 * counterclockwise positive; module positions are meters from the robot center, +x forward, +y
 * left.
 *
 * <p>Names match WPILib's so nothing has to be relearned; the difference is arrays in, arrays out.
 *
 * <p>Index: {@link #discretize} | {@link #toSwerveModuleVelocities} | {@link
 * #desaturateWheelVelocities} | {@link #optimize} | {@link #cosineScale} | {@link
 * #toChassisVelocities} | {@link #angleModulus}
 */
public final class SwerveMath {
    private SwerveMath() {}

    /**
     * Corrects a chassis velocity for the robot rotating during one control period, so driving and
     * turning at once does not drift sideways. Same math as WPILib's {@code
     * ChassisVelocities.discretize}.
     *
     * @param out receives {vx, vy, omega}
     */
    public static void discretize(double vx, double vy, double omega, double dt, double[] out) {
        double dTheta = omega * dt;
        double half = dTheta / 2.0;
        double cosMinusOne = Math.cos(dTheta) - 1.0;
        double halfByTanHalf =
                Math.abs(cosMinusOne) < 1e-9
                        ? 1.0 - dTheta * dTheta / 12.0
                        : -(half * Math.sin(dTheta)) / cosMinusOne;
        double dx = vx * dt;
        double dy = vy * dt;
        out[0] = (dx * halfByTanHalf + dy * half) / dt;
        out[1] = (-dx * half + dy * halfByTanHalf) / dt;
        out[2] = omega;
    }

    /**
     * WPILib's {@code SwerveDriveKinematics.toSwerveModuleVelocities}: robot-relative chassis
     * velocity to each module's wheel speed and direction.
     *
     * @param mx module x positions (m)
     * @param my module y positions (m)
     * @param speed receives each wheel's speed (m/s, never negative)
     * @param angle receives each wheel's direction (rad); unchanged where the speed is zero
     */
    public static void toSwerveModuleVelocities(
            double vx,
            double vy,
            double omega,
            double[] mx,
            double[] my,
            double[] speed,
            double[] angle) {
        for (int i = 0; i < mx.length; i++) {
            double mvx = vx - omega * my[i];
            double mvy = vy + omega * mx[i];
            speed[i] = Math.hypot(mvx, mvy);
            if (speed[i] > 1e-9) {
                angle[i] = Math.atan2(mvy, mvx);
            }
        }
    }

    /**
     * WPILib's {@code desaturateWheelVelocities}: scales every wheel speed down together so none
     * exceeds {@code max}, keeping the direction of travel.
     *
     * @return the scale applied, 1 when nothing was over
     */
    public static double desaturateWheelVelocities(double[] speed, double max) {
        double top = 0;
        for (double s : speed) {
            top = Math.max(top, Math.abs(s));
        }
        if (top <= max || top < 1e-9) {
            return 1.0;
        }
        double k = max / top;
        for (int i = 0; i < speed.length; i++) {
            speed[i] *= k;
        }
        return k;
    }

    /**
     * WPILib's {@code SwerveModuleVelocity.optimize}: turn each module the short way. If the target
     * is more than 90 degrees from where the module points, aim the opposite way and drive
     * backwards.
     *
     * @param velocity wheel velocities, rewritten (may become negative)
     * @param angle wheel directions, rewritten
     * @param current where each module points now (rad)
     */
    public static void optimize(double[] velocity, double[] angle, double[] current) {
        for (int i = 0; i < velocity.length; i++) {
            if (Math.abs(angleModulus(angle[i] - current[i])) > Math.PI / 2) {
                angle[i] = angleModulus(angle[i] + Math.PI);
                velocity[i] = -velocity[i];
            }
        }
    }

    /**
     * WPILib's {@code SwerveModuleVelocity.cosineScale}: scale each wheel velocity by the cosine of
     * how far the module still has to turn, so a module mid-turn does not push the robot sideways.
     */
    public static void cosineScale(double[] velocity, double[] angle, double[] current) {
        for (int i = 0; i < velocity.length; i++) {
            velocity[i] *= Math.max(0.0, Math.cos(angle[i] - current[i]));
        }
    }

    /**
     * WPILib's {@code toChassisVelocities}, least squares: chassis motion from module motions, the
     * inverse of {@link #toSwerveModuleVelocities} when the wheels disagree slightly (slip, noise).
     * Works for velocities or for displacements over one sample.
     *
     * @param dx each module's motion along robot x
     * @param dy each module's motion along robot y
     * @param out receives {vx, vy, omega} (or {dx, dy, dTheta})
     * @return RMS disagreement of the modules with the fitted motion, a slip signal
     */
    public static double toChassisVelocities(
            double[] dx, double[] dy, double[] mx, double[] my, double[] out) {
        return toChassisVelocities(dx, dy, mx, my, null, out, null);
    }

    /**
     * {@link #toChassisVelocities(double[], double[], double[], double[], double[])} using only
     * some modules: a module with weight 0 (a dead drive motor, an untrusted angle) is left out.
     * Two good modules still determine x, y and rotation.
     *
     * @param weight per-module weight (1 use, 0 skip), or null for all
     * @param residual receives each module's disagreement with the fit (m), or null
     * @return RMS disagreement of the used modules, or NaN if fewer than two were used
     */
    public static double toChassisVelocities(
            double[] dx,
            double[] dy,
            double[] mx,
            double[] my,
            double[] weight,
            double[] out,
            double[] residual) {
        int n = mx.length;
        double used = 0;
        for (int i = 0; i < n; i++) {
            used += weight == null ? 1 : weight[i];
        }
        if (used < 2) {
            out[0] = 0;
            out[1] = 0;
            out[2] = 0;
            return Double.NaN;
        }
        // Normal equations for rows [1, 0, -y], [0, 1, x]; positions are centered, so for a
        // symmetric robot the sums vanish, but solving the 3x3 keeps any layout correct.
        double sx = 0, sy = 0, sr = 0, bx = 0, by = 0, bw = 0;
        for (int i = 0; i < n; i++) {
            double w = weight == null ? 1 : weight[i];
            sx += w * mx[i];
            sy += w * my[i];
            sr += w * (mx[i] * mx[i] + my[i] * my[i]);
            bx += w * dx[i];
            by += w * dy[i];
            bw += w * (-my[i] * dx[i] + mx[i] * dy[i]);
        }
        double a00 = used, a02 = -sy, a11 = used, a12 = sx, a22 = sr;
        // Solve [[a00,0,a02],[0,a11,a12],[a02,a12,a22]] * v = [bx,by,bw] by elimination.
        double det = a00 * (a11 * a22 - a12 * a12) - a02 * (a02 * a11);
        double w = (a00 * a11 * bw - a02 * a11 * bx - a00 * a12 * by) / det;
        double vx = (bx - a02 * w) / a00;
        double vy = (by - a12 * w) / a11;
        out[0] = vx;
        out[1] = vy;
        out[2] = w;
        double sq = 0;
        for (int i = 0; i < n; i++) {
            double ex = dx[i] - (vx - w * my[i]);
            double ey = dy[i] - (vy + w * mx[i]);
            double e2 = ex * ex + ey * ey;
            if (residual != null) {
                residual[i] = Math.sqrt(e2);
            }
            sq += (weight == null ? 1 : weight[i]) * e2;
        }
        return Math.sqrt(sq / used);
    }

    /** WPILib's {@code MathUtil.angleModulus}: an angle wrapped to (-pi, pi]. */
    public static double angleModulus(double radians) {
        double a = Math.IEEEremainder(radians, 2 * Math.PI);
        return a <= -Math.PI ? a + 2 * Math.PI : a;
    }
}
