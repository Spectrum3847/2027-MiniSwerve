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

    private double lastFraction = 1.0;

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
            lastAngle[i] = measuredAngle[i];
            steerVelocity[i] = 0;
            dx[i] = measuredVelocity[i] * Math.cos(measuredAngle[i]);
            dy[i] = measuredVelocity[i] * Math.sin(measuredAngle[i]);
        }
        SwerveMath.toChassisVelocities(dx, dy, mx, my, chassis);
        pvx = chassis[0];
        pvy = chassis[1];
        pw = chassis[2];
        // Wheel speeds consistent with the fitted chassis velocity, so the first step is not an
        // unlimited jump by each wheel's slip residual.
        for (int i = 0; i < n; i++) {
            lastVelocity[i] = along(pvx, pvy, pw, i, lastAngle[i]);
        }
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
     * @param pushing which wheels can push; others are not limited or counted (null: all)
     */
    public void generate(
            double vx,
            double vy,
            double omega,
            double dt,
            double maxWheelVelocity,
            double maxDriveAcceleration,
            double maxSteerVelocity,
            boolean[] pushing) {
        generate(
                vx,
                vy,
                omega,
                dt,
                maxWheelVelocity,
                maxDriveAcceleration,
                maxSteerVelocity,
                DEFAULT_STEER_ACCELERATION,
                pushing);
    }

    /** Default steer acceleration (rad/s^2) when not given: about half what an MK4i can do. */
    public static final double DEFAULT_STEER_ACCELERATION = 300;

    /**
     * One cycle, with the steer acceleration limit.
     *
     * @param maxSteerAcceleration how fast a module's turn rate may change (rad/s^2)
     */
    public void generate(
            double vx,
            double vy,
            double omega,
            double dt,
            double maxWheelVelocity,
            double maxDriveAcceleration,
            double maxSteerVelocity,
            double maxSteerAcceleration,
            boolean[] pushing) {
        this.pushing = pushing;
        // 1. Desaturate the request as a whole chassis velocity.
        SwerveMath.toSwerveModuleVelocities(vx, vy, omega, mx, my, speed, dir);
        // Where each wheel will need to point for the request, so a stopped robot turns its
        // wheels toward it before moving (otherwise the steer limit below would never let it go).
        for (int i = 0; i < n; i++) {
            requestDir[i] = speed[i] > STOPPED ? dir[i] : Double.NaN;
        }
        // Only wheels that push count: a free-rolling wheel may spin past top speed.
        double top = 0;
        for (int i = 0; i < n; i++) {
            if (pushes(i)) {
                top = Math.max(top, speed[i]);
            }
        }
        if (top > maxWheelVelocity && top > 1e-9) {
            double k = maxWheelVelocity / top;
            vx *= k;
            vy *= k;
            omega *= k;
        }

        // 2. Largest step toward the request that every wheel's acceleration allows.
        double maxDelta = maxDriveAcceleration * dt;
        maxTurnStep = maxSteerVelocity * dt;
        // How far each wheel is from the direction it already has to point (s = 0): the step may
        // not make any wheel turn further than it can this cycle, unless it is already behind, and
        // then only not worse. So the robot's direction never outruns its wheels.
        for (int i = 0; i < n; i++) {
            turnAtZero[i] = turnNeeded(pvx, pvy, pw, i);
        }
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
        lastFraction = s;
        pvx += s * (vx - pvx);
        pvy += s * (vy - pvy);
        pw += s * (omega - pw);

        // 3. Module targets for the new chassis setpoint, with the steer rate limited.
        SwerveMath.toSwerveModuleVelocities(pvx, pvy, pw, mx, my, speed, dir);
        double maxTurn = maxSteerVelocity * dt;
        for (int i = 0; i < n; i++) {
            double target =
                    speed[i] > STOPPED
                            ? dir[i]
                            : Double.isNaN(requestDir[i]) ? lastAngle[i] : requestDir[i];
            // The short way: within 90 degrees, driving backward if that is closer.
            double err = SwerveMath.angleModulus(target - lastAngle[i]);
            if (Math.abs(err) > Math.PI / 2) {
                err = SwerveMath.angleModulus(err + Math.PI);
            }
            // A trapezoid in turn rate: speed up and slow down at maxSteerAcceleration, so the
            // steer feedforward ends at zero on the target instead of overshooting it.
            double stopRate = Math.sqrt(2 * maxSteerAcceleration * Math.abs(err));
            double rate = Math.copySign(Math.min(maxSteerVelocity, stopRate), err);
            double dv = maxSteerAcceleration * dt;
            rate = Math.max(steerVelocity[i] - dv, Math.min(steerVelocity[i] + dv, rate));
            double turn = rate * dt;
            if (Math.abs(turn) > Math.abs(err)) {
                turn = err; // land exactly
                rate = turn / dt;
            }
            turn = Math.max(-maxTurn, Math.min(maxTurn, turn));
            angle[i] = SwerveMath.angleModulus(lastAngle[i] + turn);
            steerVelocity[i] = rate;
            // The wheel pushes only along where it points: the module velocity projected onto
            // that direction (cosine scaling, signed). fits() measures acceleration the same way.
            double v = along(pvx, pvy, pw, i, angle[i]);
            // Turning the wheel also changes its speed along where it points; keep that change
            // within the acceleration limit too. What is stored is exactly what is sent.
            velocity[i] =
                    Math.max(lastVelocity[i] - maxDelta, Math.min(lastVelocity[i] + maxDelta, v));
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
            if (!pushes(i)) {
                continue; // a passive wheel's speed is not ours to limit
            }
            // Same quantity as the stored speed, so s = 0 always fits and the check is linear in s.
            if (Math.abs(along(cx, cy, cw, i, lastAngle[i]) - lastVelocity[i]) > maxDelta + 1e-9) {
                return false;
            }
            if (turnNeeded(cx, cy, cw, i) > Math.max(maxTurnStep, turnAtZero[i]) + 1e-9) {
                return false;
            }
        }
        return true;
    }

    private boolean[] pushing;
    private double maxTurnStep;
    private final double[] turnAtZero = new double[8];
    private final double[] requestDir = new double[8];

    /**
     * How far (rad, the short way) wheel {@code i} must turn from where it points to follow chassis
     * motion (vx, vy, omega); 0 when that wheel would be nearly still.
     */
    private double turnNeeded(double vx, double vy, double omega, int i) {
        double mvx = vx - omega * my[i];
        double mvy = vy + omega * mx[i];
        if (Math.hypot(mvx, mvy) <= STOPPED) {
            if (Double.isNaN(requestDir[i])) {
                return 0;
            }
            double e = Math.abs(SwerveMath.angleModulus(requestDir[i] - lastAngle[i]));
            return Math.min(e, Math.PI - e);
        }
        double err = Math.abs(SwerveMath.angleModulus(Math.atan2(mvy, mvx) - lastAngle[i]));
        return Math.min(err, Math.PI - err); // driving backward is as good as turning around
    }

    /** Whether wheel {@code i} can push this cycle (all, when not told otherwise). */
    private boolean pushes(int i) {
        return pushing == null || pushing[i];
    }

    /** Module {@code i}'s velocity for chassis (vx, vy, omega), projected onto direction a. */
    private double along(double vx, double vy, double omega, int i, double a) {
        double mvx = vx - omega * my[i];
        double mvy = vy + omega * mx[i];
        return mvx * Math.cos(a) + mvy * Math.sin(a);
    }

    /**
     * Fraction of the way from last setpoint to the request taken this cycle: 1 when not limited,
     * below 1 when acceleration limited the step.
     */
    public double lastFraction() {
        return lastFraction;
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
