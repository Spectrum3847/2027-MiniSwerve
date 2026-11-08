package frc.spectrumLib.swerve;

/**
 * The 250 Hz drivetrain loop: read every device, turn the latest {@link DriveRequest} into module
 * commands using that fresh reading, send them, and queue the sample for the main loop.
 *
 * <p>Runs only when there is hardware (real or Phoenix sim); in replay it never starts and the
 * samples come from the log. It never logs (AdvantageKit is main-thread only) and allocates nothing
 * per cycle. Faults are counted and the last message kept, for the main loop to log.
 *
 * <p>Over 400 lines: one cycle's path (read, guard, plan, send) is kept in one file so the 250 Hz
 * code can be read top to bottom.
 *
 * <p>Index: {@link #start} | {@link #setRequest} | {@link #run} | {@link #computeTargets}
 */
public final class SwerveThread {
    private final SwerveIO io;
    private final SampleQueue queue;
    private final double[] mx;
    private final double[] my;
    private final double periodSeconds;
    private final int priority;

    private volatile DriveRequest request = DriveRequest.NEUTRAL;
    private volatile double maxWheelVelocity;
    private volatile boolean useSetpointGenerator;
    private volatile double rotationLookahead;
    private volatile double maxDriveAcceleration = 10.0;
    private volatile double maxSteerVelocity = 20.0;
    private volatile double busVoltage = 12.0;

    /** Last field heading used (rad), held and advanced while the gyro's samples are stale. */
    private double heldHeading = Double.NaN;

    private volatile double maxSteerAcceleration =
            SwerveSetpointGenerator.DEFAULT_STEER_ACCELERATION;
    private final SwerveSetpointGenerator generator;
    private final double[] steerVelocity;
    private final double[] current;
    private final boolean[] pushing;
    private boolean allPushing;
    private final double[] robotFx;
    private final double[] robotFy;

    /** How each motor is commanded; replaced (never mutated) by the main thread. */
    private volatile ModuleCommands commands;

    /** Turn rate sent last cycle (rad/s), for the turn-acceleration limit. */
    private double lastOmega;

    private volatile boolean running;
    private volatile long faults;
    private volatile String lastFault = "";
    private volatile long cycles;
    private volatile long staleCycles;
    private Class<?> lastFaultClass;
    private String lastFaultMessage;
    private int faultsInRow;

    /** Failing cycles in a row the motors hold their last setpoint before going neutral. */
    static final int MAX_FAULTS_HOLDING = 25;

    private final SwerveSample sample;
    private final double[] chassis = new double[3];
    private final double[] lastTargetAngle;
    private final double[] wheelForce;
    private boolean haveTargets;
    private double lastTimestamp;

    /**
     * @param mx module x positions (m), FL FR BL BR
     * @param my module y positions (m)
     * @param maxWheelVelocity top wheel speed (m/s); requests are scaled to stay under it
     * @param periodSeconds nominal loop period (0.004 for 250 Hz)
     * @param priority real-time priority for this thread on the robot, 0 for none
     */
    public SwerveThread(
            SwerveIO io,
            SampleQueue queue,
            double[] mx,
            double[] my,
            double maxWheelVelocity,
            double periodSeconds,
            int priority) {
        this.io = io;
        this.queue = queue;
        this.mx = mx.clone();
        this.my = my.clone();
        this.maxWheelVelocity = maxWheelVelocity;
        this.periodSeconds = periodSeconds;
        this.priority = priority;
        int n = mx.length;
        sample = new SwerveSample(n);
        lastTargetAngle = new double[n];
        wheelForce = new double[n];
        steerVelocity = new double[n];
        current = new double[n];
        pushing = new boolean[n];
        robotFx = new double[n];
        robotFy = new double[n];
        commands = ModuleCommands.allNormal(n);
        generator = new SwerveSetpointGenerator(mx, my);
        guard =
                new SampleGuard(
                        n,
                        2 * maxWheelVelocity,
                        MAX_REAL_STEER_RATE,
                        MAX_REAL_YAW_RATE,
                        periodSeconds);
    }

    /** Fastest a module really turns (rad/s, MK4i free speed ~29) with margin, for the guard. */
    private static final double MAX_REAL_STEER_RATE = 40;

    /** Fastest the robot really spins (rad/s) with margin, for the guard. */
    private static final double MAX_REAL_YAW_RATE = 25;

    private final SampleGuard guard;

    /**
     * After a yaw jump (Pigeon reboot), new requests to wait for before trusting the yaw for
     * field-relative driving again: the main loop's heading offset must first follow the new yaw.
     * Two, since the first may have been built before the main loop saw the jump.
     */
    private int yawHoldRequests;

    private DriveRequest lastSeenRequest;

    /** Starts the loop on its own daemon thread. Call once, during robot init. */
    public void start() {
        running = true;
        Thread t = new Thread(this::run, "Swerve250Hz");
        t.setDaemon(true);
        t.start();
    }

    public void stop() {
        running = false;
    }

    /** Main thread: how each motor is commanded, from the next cycle on. */
    public void setModuleCommands(ModuleCommands c) {
        commands = c;
    }

    /** Main thread: what to do from the next cycle on. */
    public void setRequest(DriveRequest r) {
        request = r;
    }

    public void setMaxWheelVelocity(double metersPerSecond) {
        maxWheelVelocity = metersPerSecond;
    }

    /**
     * Seconds ahead to rotate field-relative requests by the measured yaw rate, compensating
     * steer/drive lag while spinning. Main thread, at boot.
     */
    public void setRotationLookahead(double seconds) {
        rotationLookahead = seconds;
    }

    /**
     * Turns the setpoint generator on or off and sets its limits. Main thread, at boot.
     *
     * @param maxDriveAcceleration largest wheel speed change (m/s^2)
     * @param maxSteerVelocity fastest module turn (rad/s)
     * @param maxSteerAcceleration how fast a module's turn rate may change (rad/s^2)
     */
    public void setSetpointGenerator(
            boolean enabled,
            double maxDriveAcceleration,
            double maxSteerVelocity,
            double maxSteerAcceleration) {
        this.maxDriveAcceleration = maxDriveAcceleration;
        this.maxSteerVelocity = maxSteerVelocity;
        this.maxSteerAcceleration = maxSteerAcceleration;
        this.useSetpointGenerator = enabled;
    }

    /**
     * Limits each wheel's planned speed-up to what its motor can give (null: off). Main thread, at
     * boot.
     */
    public void setMotorLimit(DriveMotorLimit limit) {
        generator.setMotorLimit(limit);
    }

    /**
     * Battery voltage for the motor limit (V), from the main loop's logged inputs. Ignored unless
     * between 6 and 16 V, so a bad reading never plans zero or wild acceleration.
     */
    public void setBusVoltage(double volts) {
        if (volts >= 6.0 && volts <= 16.0) {
            busVoltage = volts;
        }
    }

    public long faults() {
        return faults;
    }

    public String lastFault() {
        return lastFault;
    }

    /** Readings rejected as impossible jumps (device reboots), since boot. */
    public long rejectedJumps() {
        return guard.rejected();
    }

    /** Cycles in a row with stale or failed signals. */
    public long staleCycles() {
        return staleCycles;
    }

    public long cycles() {
        return cycles;
    }

    // Only this thread writes the counters, so the volatile increments are safe.
    @SuppressWarnings({"deprecation", "NonAtomicVolatileUpdate"})
    private void run() {
        if (priority > 0) {
            org.wpilib.system.Threads.setCurrentThreadPriority(priority);
        }
        while (running) {
            try {
                io.waitForSample(sample);
                guard.check(sample);
                if (guard.yawJumped()) {
                    yawHoldRequests = 2;
                    lastSeenRequest = request;
                }
                staleCycles = sample.allGood ? 0 : staleCycles + 1;
                // Late signals never stop the drivetrain (AGENTS.md "Finish the match"): a module
                // whose readings are stale is planned from its last target instead.
                computeTargets(request);
                System.arraycopy(sample.targetAngle, 0, lastTargetAngle, 0, lastTargetAngle.length);
                lastTimestamp = sample.timestamp;
                queue.offer(sample);
                cycles++;
                faultsInRow = 0;
            } catch (RuntimeException e) {
                faults++;
                faultsInRow++;
                // Reword only when the fault changes: a persistent bug would otherwise build a
                // string every cycle.
                if (e.getClass() != lastFaultClass
                        || !java.util.Objects.equals(e.getMessage(), lastFaultMessage)) {
                    lastFaultClass = e.getClass();
                    lastFaultMessage = e.getMessage();
                    lastFault = e.toString();
                }
                // A bug in our own code, not a device fault. The motors keep their last setpoint,
                // so a one-off keeps the robot driving. The one exception to "Finish the match":
                // if no command can be computed for 0.1 s, holding the last one would drive
                // blind, so go neutral until a cycle succeeds again.
                if (faultsInRow >= MAX_FAULTS_HOLDING) {
                    try {
                        io.setNeutral();
                    } catch (RuntimeException ignored) {
                        // Already counted; nothing more to do this cycle.
                    }
                }
                java.util.concurrent.locks.LockSupport.parkNanos((long) (periodSeconds * 1e9));
            }
        }
        io.setNeutral();
        io.close();
    }

    /** Fills the sample's targets from the request and sends them. Package-private for tests. */
    void computeTargets(DriveRequest r) {
        int n = mx.length;
        double dt = lastTimestamp > 0 ? sample.timestamp - lastTimestamp : periodSeconds;
        if (!(dt > 1e-4 && dt < 0.05)) {
            dt = periodSeconds;
        }
        if (!haveTargets) {
            System.arraycopy(sample.angle, 0, lastTargetAngle, 0, n);
            haveTargets = true;
        }
        double[] angle = sample.targetAngle;
        double[] velocity = sample.targetVelocity;
        System.arraycopy(lastTargetAngle, 0, angle, 0, n);
        java.util.Arrays.fill(wheelForce, 0.0);
        // Wheels that can push this cycle: driven at the full rate.
        ModuleCommands cmd = commands;
        allPushing = true;
        for (int i = 0; i < n; i++) {
            pushing[i] = cmd.drive()[i] && !cmd.driveSlow()[i];
            allPushing &= pushing[i];
        }
        java.util.Arrays.fill(steerVelocity, 0.0);
        sample.setpointFraction = 1.0;
        // Robot-frame rotation of the request: cos/sin of -heading (identity if robot-relative).
        double c = 1.0;
        double s = 0.0;

        // Keep the held heading current in every mode, so a gyro dropout right after the robot
        // was pushed or spun (neutral, X lock, robot-relative) holds today's heading.
        if (r != lastSeenRequest) {
            lastSeenRequest = r;
            if (yawHoldRequests > 0) {
                yawHoldRequests--;
            }
        }
        boolean yawUsable = sample.gyroOk && yawHoldRequests == 0;
        if (yawUsable && r.useGyro()) {
            heldHeading = sample.yaw + r.headingOffset();
        }
        switch (r.mode()) {
            case NEUTRAL -> {
                System.arraycopy(sample.angle, 0, angle, 0, n);
                java.util.Arrays.fill(velocity, 0.0);
                generator.reset(sample.velocity, sample.angle);
                lastOmega = sample.angularVelocityZWorld;
                io.setNeutral();
                return;
            }
            case X_LOCK -> {
                generator.reset(sample.velocity, sample.angle);
                lastOmega = sample.angularVelocityZWorld;
                for (int i = 0; i < n; i++) {
                    angle[i] = Math.atan2(my[i], mx[i]);
                    velocity[i] = 0.0;
                }
            }
            case VELOCITY -> {
                double vx = r.vx();
                double vy = r.vy();
                if (r.fieldRelative()) {
                    // With the gyro down, the main loop sends the odometry heading itself. Between
                    // the gyro dropping and the main loop noticing (up to ~0.1 s), this sample's
                    // yaw is stale and extrapolated: hold the last fresh heading and advance it by
                    // the turn rate we are commanding instead.
                    double heading;
                    if (!r.useGyro()) {
                        heading = r.headingOffset();
                    } else if (yawUsable) {
                        heading = sample.yaw + r.headingOffset();
                    } else if (Double.isNaN(heldHeading)) {
                        heading = sample.yaw + r.headingOffset(); // no fresh yaw seen yet
                    } else {
                        heading = heldHeading + lastOmega * dt;
                    }
                    heldHeading = heading;
                    // Aim with the heading the robot will have when the modules get there: the
                    // steer and drive loops lag, and a spinning robot turns meanwhile.
                    if (r.useGyro() && sample.gyroOk) {
                        heading += sample.angularVelocityZWorld * rotationLookahead;
                    }
                    c = Math.cos(heading);
                    s = Math.sin(heading);
                    // Field to robot frame: rotate by -heading.
                    double rx = vx * c + vy * s;
                    double ry = -vx * s + vy * c;
                    vx = rx;
                    vy = ry;
                }
                double omega = r.omega();
                double maxOmega = r.maxAngularVelocity();
                if (Double.isFinite(maxOmega)) {
                    maxOmega = Math.max(0.0, maxOmega);
                    omega = Math.max(-maxOmega, Math.min(maxOmega, omega));
                }
                double maxAlpha = r.maxAngularAcceleration();
                if (Double.isFinite(maxAlpha)) {
                    // Ramp the turn rate from last cycle's; a small floor so it can always stop.
                    double step = Math.max(0.5, maxAlpha) * dt;
                    omega = Math.max(lastOmega - step, Math.min(lastOmega + step, omega));
                }
                lastOmega = omega;
                SwerveMath.discretize(vx, vy, omega, dt, chassis);
                // A request may lower this cycle's limits (never raise them).
                double topSpeed = limit(maxWheelVelocity, r.maxWheelVelocity(), 0.0);
                double topAccel = limit(maxDriveAcceleration, r.maxDriveAcceleration(), 0.5);
                if (useSetpointGenerator) {
                    generator.setBusVoltage(busVoltage);
                    generator.generate(
                            chassis[0],
                            chassis[1],
                            chassis[2],
                            dt,
                            topSpeed,
                            topAccel,
                            maxSteerVelocity,
                            maxSteerAcceleration,
                            pushing);
                    sample.setpointFraction = generator.lastFraction();
                    System.arraycopy(generator.angle(), 0, angle, 0, n);
                    System.arraycopy(generator.velocity(), 0, velocity, 0, n);
                    System.arraycopy(generator.steerVelocity(), 0, steerVelocity, 0, n);
                } else {
                    SwerveMath.toSwerveModuleVelocities(
                            chassis[0], chassis[1], chassis[2], mx, my, velocity, angle);
                    desaturatePushing(velocity, topSpeed);
                }
            }
        }
        if (!(useSetpointGenerator && r.mode() == DriveRequest.Mode.VELOCITY)) {
            // The generator already turns the short way and cosine-scales against its own plan;
            // doing it again here would scale twice and could flip the target against the steer
            // feedforward.
            // Where each module points: measured, or last target if its reading is stale.
            for (int i = 0; i < n; i++) {
                current[i] = sample.moduleOk[i] ? sample.angle[i] : lastTargetAngle[i];
            }
            SwerveMath.optimize(velocity, angle, current);
            SwerveMath.cosineScale(velocity, angle, current);
        }
        double[] fx = r.wheelForceFeedforwardX();
        double[] fy = r.wheelForceFeedforwardY();
        if (r.mode() == DriveRequest.Mode.VELOCITY && fx != null && fy != null) {
            // Into the robot frame.
            double totalX = 0, totalY = 0, torque = 0;
            for (int i = 0; i < n; i++) {
                robotFx[i] = fx[i] * c + fy[i] * s;
                robotFy[i] = -fx[i] * s + fy[i] * c;
                totalX += robotFx[i];
                totalY += robotFy[i];
                torque += mx[i] * robotFy[i] - my[i] * robotFx[i];
            }
            if (!allPushing) {
                // Wheels that cannot push give their share to the ones that can.
                SwerveMath.distributeWrench(
                        totalX, totalY, torque, mx, my, pushing, robotFx, robotFy);
            }
            for (int i = 0; i < n; i++) {
                // Onto the wheel's (optimized) direction.
                wheelForce[i] = robotFx[i] * Math.cos(angle[i]) + robotFy[i] * Math.sin(angle[i]);
            }
        }
        io.setModules(angle, velocity, steerVelocity, wheelForce, commands);
    }

    /**
     * The lower of the configured limit and a request's. NaN or larger is ignored; a request is
     * clamped to at least {@code floor} (0 for speed; a small acceleration so the robot can still
     * slow down).
     */
    private static double limit(double configured, double requested, double floor) {
        if (!Double.isFinite(requested) || requested >= configured) {
            return configured;
        }
        return Math.max(floor, requested);
    }

    /** Desaturates using only the wheels that push; a free-rolling wheel may exceed top speed. */
    private void desaturatePushing(double[] velocity, double max) {
        double top = 0;
        for (int i = 0; i < velocity.length; i++) {
            if (pushing[i]) {
                top = Math.max(top, Math.abs(velocity[i]));
            }
        }
        if (top > max && top > 1e-9) {
            double k = max / top;
            for (int i = 0; i < velocity.length; i++) {
                velocity[i] *= k;
            }
        }
    }

    /** The sample being filled; for tests. */
    SwerveSample sampleForTest() {
        return sample;
    }
}
