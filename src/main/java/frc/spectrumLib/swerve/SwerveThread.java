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

        switch (r.mode()) {
            case NEUTRAL -> {
                System.arraycopy(sample.angle, 0, angle, 0, n);
                java.util.Arrays.fill(velocity, 0.0);
                io.setNeutral();
                return;
            }
            case X_LOCK -> {
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
                    double c = Math.cos(heading);
                    double s = Math.sin(heading);
                    // Field to robot frame: rotate by -heading.
                    double rx = vx * c + vy * s;
                    double ry = -vx * s + vy * c;
                    vx = rx;
                    vy = ry;
                }
                SwerveMath.discretize(vx, vy, r.omega(), dt, chassis);
                SwerveMath.toSwerveModuleVelocities(
                        chassis[0], chassis[1], chassis[2], mx, my, velocity, angle);
                SwerveMath.desaturateWheelVelocities(velocity, maxWheelVelocity);
            }
        }
        SwerveMath.optimize(velocity, angle, sample.angle);
        SwerveMath.cosineScale(velocity, angle, sample.angle);
        io.setModules(angle, velocity);
    }

    /** The sample being filled; for tests. */
    SwerveSample sampleForTest() {
        return sample;
    }
}
