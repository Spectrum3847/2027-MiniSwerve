package frc.spectrumLib.swerve;

/**
 * Turns the requested chassis velocity into module setpoints the modules can actually follow, one
 * swerve-thread cycle at a time. The idea of team 254's SwerveSetpointGenerator, simplified and
 * allocation-free:
 *
 * <ol>
 *   <li>Desaturate the request so no wheel would exceed top speed.
 *   <li>Move the chassis setpoint from last cycle's toward the request by the largest fraction
 *       {@code s} that keeps every wheel's speed change within {@code maxDriveAcceleration * dt}
 *       (bisection; s = 0 always fits). Acceleration, braking and reversing through zero are all
 *       limited the same way.
 *   <li>Turn each module toward its new direction (the short way) by at most {@code
 *       maxSteerVelocity * dt}; that turn rate is the steer velocity feedforward.
 * </ol>
 *
 * <p>Because setpoints move smoothly, the steer rate sent as feedforward is real (not a spike), and
 * the modules stay where the odometry thinks they are, which is most of the gain.
 *
 * <p>Index: {@link #reset} | {@link #generate} | outputs
 */
public final class SwerveSetpointGenerator {
    private static final int BISECTION_STEPS = 12;
    private static final double STOPPED = 1e-3; // m/s

    private final double[] mx;
    private final double[] my;
    private final int n;

    // Last cycle's setpoint.
    private double pvx;
    private double pvy;
    private double pw;
    private final double[] lastVelocity;
    private final double[] lastAngle;

    // Scratch (no allocation per cycle).
    private final double[] speed;
    private final double[] dir;
    private final double[] dx;
    private final double[] dy;
    private final double[] chassis = new double[3];

    // Outputs.
    private final double[] velocity;
    private final double[] angle;
    private final double[] steerVelocity;

    public SwerveSetpointGenerator(double[] mx, double[] my) {
        this.mx = mx.clone();
        this.my = my.clone();
        n = mx.length;
        lastVelocity = new double[n];
        lastAngle = new double[n];
        speed = new double[n];
        dir = new double[n];
        dx = new double[n];
        dy = new double[n];
        velocity = new double[n];
        angle = new double[n];
        steerVelocity = new double[n];
    }

    /**
     * Starts from what the modules are doing now (after neutral, X lock, or at boot), so the first
     * setpoint continues the robot's real motion instead of assuming it is stopped.
     *
     * @param measuredVelocity wheel speeds (m/s)
     * @param measuredAngle module directions (rad)
     */
    public void reset(double[] measuredVelocity, double[] measuredAngle) {
        for (int i = 0; i < n; i++) {
            lastVelocity[i] = measuredVelocity[i];
            lastAngle[i] = measuredAngle[i];
            dx[i] = measuredVelocity[i] * Math.cos(measuredAngle[i]);
            dy[i] = measuredVelocity[i] * Math.sin(measuredAngle[i]);
        }
        SwerveMath.toChassisVelocities(dx, dy, mx, my, chassis);
        pvx = chassis[0];
        pvy = chassis[1];
        pw = chassis[2];
    }

    /**
     * One cycle. Outputs are in {@link #velocity()}, {@link #angle()} and {@link #steerVelocity()}.
     *
     * @param vx requested robot-frame velocity (m/s), already discretized
     * @param vy requested robot-frame velocity (m/s)
     * @param omega requested rotation (rad/s)
     * @param dt cycle period (s)
     * @param maxWheelVelocity top wheel speed (m/s)
     * @param maxDriveAcceleration largest wheel speed change (m/s^2)
     * @param maxSteerVelocity fastest module turn (rad/s)
     */
    public void generate(
            double vx,
            double vy,
            double omega,
            double dt,
            double maxWheelVelocity,
            double maxDriveAcceleration,
            double maxSteerVelocity) {
        // 1. Desaturate the request as a whole chassis velocity.
        SwerveMath.toSwerveModuleVelocities(vx, vy, omega, mx, my, speed, dir);
        double top = 0;
        for (int i = 0; i < n; i++) {
            top = Math.max(top, speed[i]);
        }
        if (top > maxWheelVelocity && top > 1e-9) {
            double k = maxWheelVelocity / top;
            vx *= k;
            vy *= k;
            omega *= k;
        }

        // 2. Largest step toward the request that every wheel's acceleration allows.
        double maxDelta = maxDriveAcceleration * dt;
        double s = 1.0;
        if (!fits(vx, vy, omega, 1.0, maxDelta)) {
            double lo = 0.0;
            double hi = 1.0;
            for (int k = 0; k < BISECTION_STEPS; k++) {
                double mid = (lo + hi) / 2;
                if (fits(vx, vy, omega, mid, maxDelta)) {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }
            s = lo;
        }
        pvx += s * (vx - pvx);
        pvy += s * (vy - pvy);
        pw += s * (omega - pw);

        // 3. Module targets for the new chassis setpoint, with the steer rate limited.
        SwerveMath.toSwerveModuleVelocities(pvx, pvy, pw, mx, my, speed, dir);
        double maxTurn = maxSteerVelocity * dt;
        for (int i = 0; i < n; i++) {
            double v = speed[i];
            double target = speed[i] > STOPPED ? dir[i] : lastAngle[i];
            double err = SwerveMath.angleModulus(target - lastAngle[i]);
            if (Math.abs(err) > Math.PI / 2) {
                err = SwerveMath.angleModulus(err + Math.PI);
                v = -v;
            }
            double turn = Math.max(-maxTurn, Math.min(maxTurn, err));
            angle[i] = SwerveMath.angleModulus(lastAngle[i] + turn);
            steerVelocity[i] = turn / dt;
            // A module still turning toward its direction only pushes along where it points.
            velocity[i] = v * Math.max(0.0, Math.cos(err - turn));
            lastAngle[i] = angle[i];
            lastVelocity[i] = velocity[i];
        }
    }

    /**
     * Whether moving the fraction {@code s} toward the request keeps every wheel within maxDelta.
     */
    private boolean fits(double vx, double vy, double omega, double s, double maxDelta) {
        double cx = pvx + s * (vx - pvx);
        double cy = pvy + s * (vy - pvy);
        double cw = pw + s * (omega - pw);
        for (int i = 0; i < n; i++) {
            double mvx = cx - cw * my[i];
            double mvy = cy + cw * mx[i];
            // Wheel speed along where the module points now (the short way, so it may be < 0).
            double along = mvx * Math.cos(lastAngle[i]) + mvy * Math.sin(lastAngle[i]);
            double v = Math.hypot(mvx, mvy) * (along >= 0 ? 1 : -1);
            if (Math.abs(v - lastVelocity[i]) > maxDelta + 1e-9) {
                return false;
            }
        }
        return true;
    }

    /** Wheel speeds (m/s), signed: negative drives the wheel backward. */
    public double[] velocity() {
        return velocity;
    }

    /** Module directions (rad). */
    public double[] angle() {
        return angle;
    }

    /** Module turn rates this cycle (rad/s), the steer velocity feedforward. */
    public double[] steerVelocity() {
        return steerVelocity;
    }
}
