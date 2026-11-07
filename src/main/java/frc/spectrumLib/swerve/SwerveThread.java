package frc.spectrumLib.swerve;

/**
 * The 250 Hz drivetrain loop: read every device, turn the latest {@link DriveRequest} into module
 * commands using that fresh reading, send them, and queue the sample for the main loop.
 *
 * <p>Runs only when there is hardware (real or Phoenix sim); in replay it never starts and the
 * samples come from the log. It never logs (AdvantageKit is main-thread only) and allocates nothing
 * per cycle. Faults are counted and the last message kept, for the main loop to log.
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
    private final SwerveSetpointGenerator generator;
    private final double[] steerVelocity;
    private volatile boolean running;
    private volatile long faults;
    private volatile String lastFault = "";
    private volatile long cycles;
    private volatile long staleCycles;

    /** Stale-signal cycles in a row before the thread stops driving (20 ms at 250 Hz). */
    static final int MAX_STALE_CYCLES = 5;

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
        generator = new SwerveSetpointGenerator(mx, my);
    }

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
     */
    public void setSetpointGenerator(
            boolean enabled, double maxDriveAcceleration, double maxSteerVelocity) {
        this.maxDriveAcceleration = maxDriveAcceleration;
        this.maxSteerVelocity = maxSteerVelocity;
        this.useSetpointGenerator = enabled;
    }

    public long faults() {
        return faults;
    }

    public String lastFault() {
        return lastFault;
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
                staleCycles = sample.allGood ? 0 : staleCycles + 1;
                // Never steer against frozen readings: after a few stale cycles, go neutral.
                computeTargets(staleCycles >= MAX_STALE_CYCLES ? DriveRequest.NEUTRAL : request);
                System.arraycopy(sample.targetAngle, 0, lastTargetAngle, 0, lastTargetAngle.length);
                lastTimestamp = sample.timestamp;
                queue.offer(sample);
                cycles++;
            } catch (RuntimeException e) {
                faults++;
                lastFault = e.toString();
                // Do not keep the last setpoint, and do not spin if every cycle throws.
                try {
                    io.setNeutral();
                } catch (RuntimeException ignored) {
                    // Already counted the fault; nothing more to do this cycle.
                }
                java.util.concurrent.locks.LockSupport.parkNanos((long) (periodSeconds * 1e9));
            }
        }
        io.setNeutral();
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
        java.util.Arrays.fill(steerVelocity, 0.0);
        sample.setpointFraction = 1.0;
        // Robot-frame rotation of the request: cos/sin of -heading (identity if robot-relative).
        double c = 1.0;
        double s = 0.0;

        switch (r.mode()) {
            case NEUTRAL -> {
                System.arraycopy(sample.angle, 0, angle, 0, n);
                java.util.Arrays.fill(velocity, 0.0);
                generator.reset(sample.velocity, sample.angle);
                io.setNeutral();
                return;
            }
            case X_LOCK -> {
                generator.reset(sample.velocity, sample.angle);
                for (int i = 0; i < n; i++) {
                    angle[i] = Math.atan2(my[i], mx[i]);
                    velocity[i] = 0.0;
                }
            }
            case VELOCITY -> {
                double vx = r.vx();
                double vy = r.vy();
                if (r.fieldRelative()) {
                    // With the gyro down, the main loop sends the odometry heading itself.
                    double heading =
                            r.useGyro() ? sample.yaw + r.headingOffset() : r.headingOffset();
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
                SwerveMath.discretize(vx, vy, r.omega(), dt, chassis);
                if (useSetpointGenerator) {
                    generator.generate(
                            chassis[0],
                            chassis[1],
                            chassis[2],
                            dt,
                            maxWheelVelocity,
                            maxDriveAcceleration,
                            maxSteerVelocity);
                    sample.setpointFraction = generator.lastFraction();
                    System.arraycopy(generator.angle(), 0, angle, 0, n);
                    System.arraycopy(generator.velocity(), 0, velocity, 0, n);
                    System.arraycopy(generator.steerVelocity(), 0, steerVelocity, 0, n);
                } else {
                    SwerveMath.toSwerveModuleVelocities(
                            chassis[0], chassis[1], chassis[2], mx, my, velocity, angle);
                    SwerveMath.desaturateWheelVelocities(velocity, maxWheelVelocity);
                }
            }
        }
        if (!(useSetpointGenerator && r.mode() == DriveRequest.Mode.VELOCITY)) {
            // The generator already turns the short way and cosine-scales against its own plan;
            // doing it again here would scale twice and could flip the target against the steer
            // feedforward.
            SwerveMath.optimize(velocity, angle, sample.angle);
            SwerveMath.cosineScale(velocity, angle, sample.angle);
        }
        double[] fx = r.wheelForceFeedforwardX();
        double[] fy = r.wheelForceFeedforwardY();
        if (r.mode() == DriveRequest.Mode.VELOCITY && fx != null && fy != null) {
            for (int i = 0; i < n; i++) {
                // Into the robot frame, then onto the wheel's (optimized) direction.
                double rfx = fx[i] * c + fy[i] * s;
                double rfy = -fx[i] * s + fy[i] * c;
                wheelForce[i] = rfx * Math.cos(angle[i]) + rfy * Math.sin(angle[i]);
            }
        }
        io.setModules(angle, velocity, steerVelocity, wheelForce);
    }

    /** The sample being filled; for tests. */
    SwerveSample sampleForTest() {
        return sample;
    }
}
