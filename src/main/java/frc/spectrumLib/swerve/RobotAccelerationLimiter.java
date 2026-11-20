package frc.spectrumLib.swerve;

/**
 * Turns the requested robot velocity into module setpoints by limiting the robot's own acceleration
 * (what the tires must deliver), one swerve-thread cycle at a time. The alternative to {@link
 * SwerveSetpointGenerator}, which limits each wheel's speed change instead: a robot driving
 * straight while spinning changes every wheel's speed constantly without accelerating at all, and
 * that limit held it back.
 *
 * <ol>
 *   <li>Desaturate the request so no wheel would exceed top speed.
 *   <li>In a frame that does not turn with the robot, step the velocity toward the request by at
 *       most {@code maxAcceleration * dt}, and the turn rate by at most {@code
 *       maxAngularAcceleration * dt}; the two share one traction budget. A near-reversal brakes
 *       straight to zero first, so the wheels flip their drive direction instead of turning around.
 *       With a {@link DriveMotorLimit}, speeding up is also held to what the motors can give.
 *   <li>Take less of the step while a wheel is still far from where the step needs it to point, so
 *       the robot is not pushed in a direction its wheels do not face.
 *   <li>Turn each module toward its direction with a turn-rate profile that follows a moving target
 *       (its own rate plus what closes the error), within the steer rate and acceleration limits;
 *       the wheel speed is the module's velocity along where the wheel really points.
 * </ol>
 *
 * <p>The planned acceleration is reported in the robot frame for {@link
 * DriveAccelerationFeedforward}. Allocation-free.
 *
 * <p>Index: {@link #reset} | {@link #generate} | steering | outputs
 */
public final class RobotAccelerationLimiter {
    private static final double STOPPED = 1e-3; // m/s

    /** A request over 150 deg from the robot's motion is a reversal: brake to zero first. */
    private static final double REVERSAL_COS = Math.cos(Math.toRadians(150));

    /** A target direction that moves more than this in one cycle is a jump, not a rate (rad). */
    private static final double TARGET_JUMP = 0.5;

    private final double[] mx;
    private final double[] my;
    private final int n;

    // Last cycle's setpoint: velocity in the frame that does not turn, and turn rate.
    private double fvx;
    private double fvy;
    private double w;
    private final double[] lastVelocity;
    private final double[] lastAngle;
    private final double[] lastTarget;

    // Scratch.
    private final double[] speed;
    private final double[] dir;
    private final double[] stepSpeed;
    private final double[] stepDir;
    private final double[] dx;
    private final double[] dy;
    private final double[] chassis = new double[3];

    private DriveMotorLimit motorLimit;
    private double busVolts = 12.0;
    private double lastFraction = 1.0;

    // Outputs.
    private final double[] velocity;
    private final double[] angle;
    private final double[] steerVelocity;
    private final double[] wheelAcceleration;
    private double accelerationX;
    private double accelerationY;
    private double angularAcceleration;

    public RobotAccelerationLimiter(double[] mx, double[] my) {
        this.mx = mx.clone();
        this.my = my.clone();
        n = mx.length;
        lastVelocity = new double[n];
        lastAngle = new double[n];
        lastTarget = new double[n];
        speed = new double[n];
        dir = new double[n];
        stepSpeed = new double[n];
        stepDir = new double[n];
        dx = new double[n];
        dy = new double[n];
        velocity = new double[n];
        angle = new double[n];
        steerVelocity = new double[n];
        wheelAcceleration = new double[n];
    }

    /** Also holds speeding up to what the drive motors can give (null: only the limits given). */
    public void setMotorLimit(DriveMotorLimit limit) {
        motorLimit = limit;
    }

    /** Battery voltage the motor limit uses this cycle (V). Swerve thread, before generate. */
    public void setBusVoltage(double volts) {
        busVolts = volts;
    }

    /**
     * Starts from what the modules are doing now, so the first setpoint continues the robot's real
     * motion.
     *
     * @param measuredVelocity wheel speeds (m/s)
     * @param measuredAngle module directions (rad)
     * @param frameHeading the robot's heading in the frame that does not turn (rad)
     */
    public void reset(double[] measuredVelocity, double[] measuredAngle, double frameHeading) {
        for (int i = 0; i < n; i++) {
            lastAngle[i] = measuredAngle[i];
            lastTarget[i] = measuredAngle[i];
            steerVelocity[i] = 0;
            wheelAcceleration[i] = 0;
            dx[i] = measuredVelocity[i] * Math.cos(measuredAngle[i]);
            dy[i] = measuredVelocity[i] * Math.sin(measuredAngle[i]);
        }
        SwerveMath.toChassisVelocities(dx, dy, mx, my, chassis);
        double c = Math.cos(frameHeading);
        double s = Math.sin(frameHeading);
        fvx = chassis[0] * c - chassis[1] * s;
        fvy = chassis[0] * s + chassis[1] * c;
        w = chassis[2];
        accelerationX = accelerationY = angularAcceleration = 0;
        for (int i = 0; i < n; i++) {
            lastVelocity[i] = along(chassis[0], chassis[1], chassis[2], i, lastAngle[i]);
        }
    }

    /**
     * One cycle. Outputs are in {@link #velocity()}, {@link #angle()} and {@link #steerVelocity()}.
     *
     * @param vx requested robot-frame velocity (m/s), not discretized
     * @param vy requested robot-frame velocity (m/s)
     * @param omega requested rotation (rad/s)
     * @param frameHeading the robot's heading in a frame that does not turn (rad); only its changes
     *     matter, so any steady offset or slow drift is fine
     * @param dt cycle period (s)
     * @param maxWheelVelocity top wheel speed (m/s)
     * @param maxAcceleration largest robot acceleration (m/s^2)
     * @param maxAngularAcceleration largest turn acceleration (rad/s^2)
     * @param maxSteerVelocity fastest module turn (rad/s)
     * @param maxSteerAcceleration how fast a module's turn rate may change (rad/s^2)
     * @param pushing which wheels can push; others are not counted (null: all)
     */
    public void generate(
            double vx,
            double vy,
            double omega,
            double frameHeading,
            double dt,
            double maxWheelVelocity,
            double maxAcceleration,
            double maxAngularAcceleration,
            double maxSteerVelocity,
            double maxSteerAcceleration,
            boolean[] pushing) {
        // 1. Desaturate the request as a whole (wheels that cannot push may spin faster).
        SwerveMath.toSwerveModuleVelocities(vx, vy, omega, mx, my, speed, dir);
        double top = 0;
        for (int i = 0; i < n; i++) {
            if (pushing == null || pushing[i]) {
                top = Math.max(top, speed[i]);
            }
        }
        if (top > maxWheelVelocity && top > 1e-9) {
            double k = maxWheelVelocity / top;
            vx *= k;
            vy *= k;
            omega *= k;
        }

        // 2. The request in the frame that does not turn, and the step toward it.
        double c = Math.cos(frameHeading);
        double s = Math.sin(frameHeading);
        double tx = vx * c - vy * s;
        double ty = vx * s + vy * c;
        double now = Math.hypot(fvx, fvy);
        double want = Math.hypot(tx, ty);
        if (now > STOPPED && want > STOPPED && fvx * tx + fvy * ty < REVERSAL_COS * now * want) {
            // The straight line to a near-reversal passes close to zero without reaching it, and
            // the wheels would swing around 180 deg; from zero they flip instead.
            tx = ty = 0;
        }
        double wantDvx = tx - fvx;
        double wantDvy = ty - fvy;
        double wantDw = omega - w;
        double dvx = wantDvx;
        double dvy = wantDvy;
        double dw = wantDw;
        // Pushing the robot along and turning it use the same tires: together they may use the
        // traction once, so each is scaled by the sum of their shares.
        double maxDv = maxAcceleration * dt;
        double maxDw = maxAngularAcceleration * dt;
        double share = Math.hypot(dvx, dvy) / maxDv + Math.abs(dw) / maxDw;
        if (share > 1) {
            dvx /= share;
            dvy /= share;
            dw /= share;
        }
        if (motorLimit != null) {
            // Only the part of the step that speeds the robot up is the motors' to give.
            double fastest = 0;
            for (int i = 0; i < n; i++) {
                fastest = Math.max(fastest, Math.abs(lastVelocity[i]));
            }
            double cap = motorLimit.maxSpeedUp(fastest, busVolts) * dt;
            if (now > STOPPED) {
                double alongStep = (dvx * fvx + dvy * fvy) / now;
                if (alongStep > cap) {
                    dvx -= (alongStep - cap) * fvx / now;
                    dvy -= (alongStep - cap) * fvy / now;
                }
            } else {
                double step = Math.hypot(dvx, dvy);
                if (step > cap && step > 1e-12) {
                    dvx *= cap / step;
                    dvy *= cap / step;
                }
            }
        }

        // 3. Where the whole step needs each wheel to point, and how far the worst wheel is from
        // it beyond what it can turn this cycle: take that much less of the step.
        robotFrame(fvx + dvx, fvy + dvy, w + dw, c, s, dt);
        SwerveMath.toSwerveModuleVelocities(
                chassis[0], chassis[1], chassis[2], mx, my, stepSpeed, stepDir);
        double maxTurn = maxSteerVelocity * dt;
        double steerDv = maxSteerAcceleration * dt;
        double behind = 0;
        for (int i = 0; i < n; i++) {
            if ((pushing != null && !pushing[i]) || stepSpeed[i] <= STOPPED) {
                continue;
            }
            double reach =
                    Math.min(
                            maxTurn,
                            Math.min(maxSteerVelocity, Math.abs(steerVelocity[i]) + steerDv) * dt);
            behind = Math.max(behind, Math.abs(shortWay(stepDir[i] - lastAngle[i])) - reach);
        }
        double take = behind <= 0 ? 1.0 : Math.max(0.0, Math.cos(Math.min(behind, Math.PI / 2)));
        dvx *= take;
        dvy *= take;
        dw *= take;
        fvx += dvx;
        fvy += dvy;
        w += dw;
        double asked = Math.hypot(wantDvx, wantDvy) / maxDv + Math.abs(wantDw) / maxDw;
        double given = Math.hypot(dvx, dvy) / maxDv + Math.abs(dw) / maxDw;
        lastFraction = asked > 1e-9 ? Math.min(1.0, given / asked) : 1.0;
        // The step is the planned acceleration, here in the robot frame.
        accelerationX = (dvx * c + dvy * s) / dt;
        accelerationY = (-dvx * s + dvy * c) / dt;
        angularAcceleration = dw / dt;

        // 4. Module targets for the new setpoint.
        robotFrame(fvx, fvy, w, c, s, dt);
        SwerveMath.toSwerveModuleVelocities(chassis[0], chassis[1], chassis[2], mx, my, speed, dir);
        for (int i = 0; i < n; i++) {
            // A wheel that would be still aims where the full step needs it, so a stopped robot
            // turns its wheels before it moves.
            double target =
                    speed[i] > STOPPED
                            ? dir[i]
                            : stepSpeed[i] > STOPPED ? stepDir[i] : lastAngle[i];
            steer(i, target, dt, maxSteerVelocity, maxSteerAcceleration);
            velocity[i] = along(chassis[0], chassis[1], chassis[2], i, angle[i]);
            wheelAcceleration[i] = (velocity[i] - lastVelocity[i]) / dt;
            lastAngle[i] = angle[i];
            lastVelocity[i] = velocity[i];
        }
    }

    /**
     * Turns module {@code i} toward {@code target} (the short way: within 90 degrees, driving
     * backward if that is closer). The turn rate is the target's own rate plus what closes the
     * error while still able to stop on it, changed by at most the steer acceleration each cycle.
     */
    private void steer(int i, double target, double dt, double maxRate, double maxAcceleration) {
        double err = shortWay(target - lastAngle[i]);
        double aim = lastAngle[i] + err;
        // How fast the target itself is moving; a flip or a new request is a jump, not a rate.
        double moved = shortWay(aim - lastTarget[i]);
        double targetRate = Math.abs(moved) < TARGET_JUMP ? moved / dt : 0.0;
        lastTarget[i] = aim;
        double close = Math.copySign(Math.sqrt(2 * maxAcceleration * Math.abs(err)), err);
        double rate = Math.max(-maxRate, Math.min(maxRate, targetRate + close));
        double dv = maxAcceleration * dt;
        rate = Math.max(steerVelocity[i] - dv, Math.min(steerVelocity[i] + dv, rate));
        double turn = rate * dt;
        // Land on the target when the turn acceleration limit allows both getting there this
        // cycle and matching the target's rate the next; otherwise keep slowing at the limit and
        // come back, instead of asking the motor to stop dead.
        double landRate = err / dt;
        if (Math.abs(landRate - steerVelocity[i]) <= dv
                && Math.abs(landRate - targetRate) <= dv
                && Math.abs(landRate) <= maxRate) {
            turn = err;
            rate = landRate;
        }
        angle[i] = SwerveMath.angleModulus(lastAngle[i] + turn);
        steerVelocity[i] = rate;
    }

    /** Fills {@link #chassis} with the discretized robot-frame velocity for a setpoint. */
    private void robotFrame(
            double vxField, double vyField, double omega, double c, double s, double dt) {
        SwerveMath.discretize(
                vxField * c + vyField * s, -vxField * s + vyField * c, omega, dt, chassis);
    }

    /** The angle difference the short way for a wheel: within 90 degrees either side. */
    private static double shortWay(double radians) {
        double a = SwerveMath.angleModulus(radians);
        return Math.abs(a) > Math.PI / 2 ? SwerveMath.angleModulus(a + Math.PI) : a;
    }

    /** Module {@code i}'s velocity for chassis (vx, vy, omega), projected onto direction a. */
    private double along(double vx, double vy, double omega, int i, double a) {
        double mvx = vx - omega * my[i];
        double mvy = vy + omega * mx[i];
        return mvx * Math.cos(a) + mvy * Math.sin(a);
    }

    /** Share of the wanted step taken this cycle: 1 when not limited. */
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

    /** Each wheel's planned speed change along where it points this cycle (m/s^2). */
    public double[] wheelAcceleration() {
        return wheelAcceleration;
    }

    /** Planned robot acceleration along robot x this cycle, as the floor sees it (m/s^2). */
    public double accelerationX() {
        return accelerationX;
    }

    /** Planned robot acceleration along robot y this cycle, as the floor sees it (m/s^2). */
    public double accelerationY() {
        return accelerationY;
    }

    /** Planned turn acceleration this cycle (rad/s^2). */
    public double angularAcceleration() {
        return angularAcceleration;
    }
}
