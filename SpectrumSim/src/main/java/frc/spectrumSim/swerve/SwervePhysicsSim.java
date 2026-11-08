package frc.spectrumSim.swerve;

import frc.spectrumLib.hardware.SimFaults;
import frc.spectrumLib.swerve.PhoenixSwerveConfig;
import frc.spectrumSim.MotorModel;
import frc.spectrumSim.field.Arena;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

/**
 * Desktop physics for the drivetrain. The simulated devices ({@link SwerveSimDevices}: our TalonFX
 * models on the sim clock, or Phoenix's own simulator) run the motors' closed loops; this reads the
 * current each motor asks for, turns it into motion with a DC motor model (current limited by
 * back-EMF and battery), and writes the motion back into the simulated motors, CANcoders and gyro.
 *
 * <p>The robot is one rigid body. Each wheel pushes along its direction with its motor's torque and
 * grips sideways with tire friction (both limited by friction on carpet); those forces move the
 * robot, and each wheel rolls with the robot's motion at that wheel. So a dead or coasting wheel
 * rolls along, and a wheel pointed the wrong way scrubs and fights the others, as on a real robot.
 *
 * <p>Each wheel spins on its own (motor and rotor through the gearing, plus the wheel), tied to the
 * ground only by the tire: tire force follows a grip curve of slip speed (peak at a few cm/s,
 * dropping to sliding friction past it), along and across the wheel, in one friction circle. So
 * wheels spin when the motor gives more than the tire holds, encoders see the spin, and odometry
 * drifts as on a real robot. A rebooted device also keeps its sim rotor position (a real TalonFX
 * restarts its rotor count; steering is safe because it fuses the CANcoder).
 *
 * <p>Stepped by the sim clock (or its own thread with Phoenix's simulator). The true pose is
 * published for the main loop to log as sim truth; robot logic must never read it.
 *
 * <p>Weight shifts between wheels as the robot accelerates ({@link WheelLoads}, set by {@link
 * #setCogHeight}); each tire ({@link Tire}) grips in proportion to its load.
 *
 * <p>Over 400 lines on purpose: one integration step touches every wheel, the body and the battery
 * together; the tire ({@link Tire}) and weight transfer ({@link WheelLoads}) are already apart.
 *
 * <p>Index: {@link #attach} | {@link #setArena} | {@link #step} | {@link #collide} | {@link
 * #stepSteer} | truth getters
 */
public final class SwervePhysicsSim {
    /** Battery open-circuit voltage (V), a charged battery. */
    private double batteryOpenVolts = 12.7;

    /**
     * Open-circuit voltage lost per amp-hour drawn: FM's TXMAN1 matches lost 0.6 to 1.6 V on about
     * 1.1 Ah each (open-circuit from voltage plus 0.012 ohm times current, first vs last 10 s).
     */
    private static final double VOLTS_PER_AMP_HOUR = 0.8;

    /** Charge drawn from the battery so far (Ah). */
    private volatile double chargeUsedAh;

    /**
     * Battery internal plus wiring resistance (ohm); voltage sags by this times total current.
     * Fitted from FM's 13 TXMAN1 matches (tools/battery_fit.py): 0.011 to 0.014, median 0.012.
     */
    private double batteryResistance = 0.012;

    /** Battery voltage forced from outside (a real log's), or NaN to use the battery model. */
    private double batteryOverride = Double.NaN;

    /** Turning inertia as a share of mass times module radius squared (a box robot: about 0.6). */
    private double inertiaFactor = 0.6;

    /** Lever arm of the tire contact patch against turning in place (m): sets steer scrub. */
    private static final double SCRUB_ARM = 0.012;

    /** Battery voltage now (V), sagged by last step's total current. */
    private volatile double batteryVolts = 12.7;

    private double supplyAmps;

    /** Share of motor torque the drive gearing passes to the wheel (spur plus bevel stages). */
    private double gearEfficiency = 0.9;

    /** Gearbox and bearing drag at the wheel (N m) while it turns. */
    private double gearDrag = 0.1;

    /**
     * Sets drive gearing losses, fitted from real logs (see tools/sim_vs_real.py).
     *
     * @param efficiency share of motor torque reaching the wheel, 0 to 1
     * @param dragNm constant drag at the wheel while turning (N m)
     */
    public synchronized void setGearLosses(double efficiency, double dragNm) {
        gearEfficiency = efficiency;
        gearDrag = dragNm;
    }

    /**
     * Sets the battery model.
     *
     * @param openVolts open-circuit voltage (V)
     * @param ohms internal plus wiring resistance
     */
    public synchronized void setBattery(double openVolts, double ohms) {
        batteryOpenVolts = openVolts;
        batteryResistance = ohms;
        chargeUsedAh = 0; // a fresh battery
    }

    /** Uses this battery voltage instead of the model (a real log's), or NaN for the model. */
    public synchronized void setBatteryOverride(double volts) {
        batteryOverride = volts;
    }

    /** Turning inertia as a share of mass times module radius squared (default 0.6). */
    public synchronized void setInertiaFactor(double factor) {
        inertiaFactor = factor;
    }

    /** Wheel's own spin inertia (kg m^2): a 4 in tread wheel and hub. */
    private static final double WHEEL_INERTIA = 0.0005;

    /** The field the robot drives on, or null for open floor with no walls. */
    private Arena arena;

    private final Arena.Body body = new Arena.Body();
    private volatile int contacts;

    /** Other robots' field poses (x, y, heading each) as of the last step. */
    private double[] otherSnapshot = new double[0];

    /** True pose (x, y, heading) as of the end of the last step, for the main thread. */
    private final double[] truthSnapshot = new double[3];

    /**
     * Puts the robot on a field: its bumpers ({@code length} by {@code width}, m) hit the arena's
     * walls and elements.
     */
    public synchronized void setArena(Arena arena, double length, double width) {
        this.arena = arena;
        body.length = length;
        body.width = width;
    }

    /** Center of gravity above the floor (m); sets weight transfer. */
    private double cogHeight = 0.152;

    /** Net tire force over mass last step (m/s^2, robot frame): what shifts the wheel loads. */
    private double lastAx;

    private double lastAy;

    private final WheelLoads loads;

    /** The floor under the wheels: tilt, height, gravity along a ramp. */
    private final TerrainContact terrainContact;

    /** Height, roll, pitch as of the last step, for the main thread. */
    private final double[] tiltSnapshot = new double[3];

    /**
     * Sets the center of gravity's height (m), for weight transfer: a high one unloads the front
     * wheels under hard acceleration, loses grip, and lifts wheels when it would tip.
     */
    public synchronized void setCogHeight(double meters) {
        cogHeight = meters;
    }

    /** Wheel rolling drag. */
    private static final double ROLLING_DRAG = 0.5; // N per m/s along the wheel

    private final MotorModel driveMotor;
    private final MotorModel steerMotor;
    private final double robotMassKg;
    private final double tireFriction;
    private final double steerInertia; // kg m^2, module about its steer axis
    private final double steerFriction; // N m per rad/s
    private final double[] mx;
    private final double[] my;

    private PhoenixSwerveConfig cfg;
    private SwerveSimDevices devices;

    private double[] steerAngle; // rad
    private double[] steerOmega; // rad/s
    private double[] wheelAngle; // rad
    private double[] wheelOmega; // rad/s
    private double[] driveAmps;
    private double[] steerAmpsNow;
    // Rigid-body robot velocity, in the robot frame.
    private double vx;
    private double vy;
    private double omega;

    private volatile double trueWheelRadius = Double.NaN;
    private volatile SimFaults faults;
    private String[] driveName;
    private String[] steerName;
    private String[] encoderName;
    private volatile double truthX;
    private volatile double truthY;
    private volatile double truthHeading;

    /**
     * @param driveMotor drive motor model
     * @param steerMotor steer motor model
     * @param robotMassKg robot weight ready to play (kg)
     * @param tireFriction tire-to-carpet friction coefficient
     * @param mx module x positions (m)
     * @param my module y positions (m)
     */
    public SwervePhysicsSim(
            MotorModel driveMotor,
            MotorModel steerMotor,
            double robotMassKg,
            double tireFriction,
            double steerInertia,
            double steerFriction,
            double[] mx,
            double[] my) {
        this.driveMotor = driveMotor;
        this.steerMotor = steerMotor;
        this.robotMassKg = robotMassKg;
        this.tireFriction = tireFriction;
        this.steerInertia = steerInertia;
        this.steerFriction = steerFriction;
        this.mx = mx.clone();
        this.my = my.clone();
        loads = new WheelLoads(mx, my);
        terrainContact = new TerrainContact(mx, my);
        loadLog = new double[mx.length];
        liftLog = new boolean[mx.length];
        loadSnapshot = new double[mx.length];
        liftSnapshot = new boolean[mx.length];
    }

    /** Called by the IO once its simulated devices exist. */
    void attach(PhoenixSwerveConfig cfg, SwerveSimDevices devices) {
        this.cfg = cfg;
        this.devices = devices;
        int n = cfg.modules();
        steerAngle = new double[n];
        steerOmega = new double[n];
        wheelAngle = new double[n];
        wheelOmega = new double[n];
        driveAmps = new double[n];
        steerAmpsNow = new double[n];
        driveName = new String[n];
        steerName = new String[n];
        encoderName = new String[n];
        for (int i = 0; i < n; i++) {
            driveName[i] = "Drive" + i;
            steerName[i] = "Steer" + i;
            encoderName[i] = "Encoder" + i;
        }
    }

    private long nextDeadlineNanos;

    /**
     * Runs the physics on its own daemon thread at {@code hz}, stepping by the measured wall time
     * (at most 20 ms per step). Call once, after {@link #attach}.
     */
    void start(double hz) {
        running = true;
        Thread t =
                new Thread(
                        () -> {
                            long period = (long) (1e9 / hz);
                            long last = System.nanoTime();
                            long next = last;
                            while (running) {
                                next += period;
                                long now;
                                while ((now = System.nanoTime()) < next) {
                                    java.util.concurrent.locks.LockSupport.parkNanos(next - now);
                                }
                                if (now - next > 20 * period) {
                                    next = now; // fell far behind: restart the schedule
                                }
                                double dt = Math.min(0.02, (now - last) * 1e-9);
                                last = now;
                                synchronized (this) {
                                    step(dt);
                                }
                                steps.incrementAndGet();
                            }
                        },
                        "SwervePhysicsSim");
        t.setDaemon(true);
        t.start();
    }

    /** Keeps a loop on a fixed 1/hz schedule; parkNanos can wake early on Windows. */
    void waitForNextPeriod(long startNanos, double hz) {
        long periodNanos = (long) (1e9 / hz);
        if (nextDeadlineNanos == 0 || startNanos - nextDeadlineNanos > periodNanos) {
            nextDeadlineNanos = startNanos; // first cycle, or fell far behind: restart the schedule
        }
        nextDeadlineNanos += periodNanos;
        long remaining;
        while ((remaining = nextDeadlineNanos - System.nanoTime()) > 0) {
            java.util.concurrent.locks.LockSupport.parkNanos(remaining);
        }
    }

    /** Longest single physics step (s): a late wake-up is split into steps no longer than this. */
    private static final double MAX_STEP = 0.001;

    /**
     * Advances the physics by {@code dt} seconds and writes the result to the sim devices. A long
     * {@code dt} (a GC pause, a late wake-up) runs as several short steps so the tire stays stable.
     */
    void step(double dt) {
        int parts = (int) Math.ceil(dt / MAX_STEP - 1e-9);
        for (int k = 0; k < parts; k++) {
            stepOnce(dt / parts);
        }
    }

    @SuppressWarnings("NonAtomicVolatileUpdate") // only the swerve thread writes truth
    private void stepOnce(double dt) {
        if (cfg == null) {
            return;
        }
        int n = steerAngle.length;
        double r = wheelRadius();
        double g2 = cfg.driveRatio() * cfg.driveRatio();
        // The wheel spins on its own: the drive rotor's inertia, through the gearing, and the
        // wheel's own. The tire, not a rolling constraint, ties it to the ground.
        double wheelInertia = driveMotor.rotorInertia() * g2 + WHEEL_INERTIA;
        double inertiaEff = robotInertia();
        // Each tire grips in proportion to its share of the weight, which shifts as the robot
        // accelerates (last step's acceleration: the loads lag it by one step).
        loads.update(robotMassKg, cogHeight, lastAx, lastAy);
        double gripMax = 0;
        for (int i = 0; i < n; i++) {
            gripMax = Math.max(gripMax, tireFriction * loads.load(i));
        }
        // Below peak slip the tire is a stiff damper on the light wheel (grip / PEAK_SLIP per m/s
        // against about J / r^2 of mass). Explicit steps are stable only while dt * k * r^2 / J
        // stays well under 2, so the wheel takes as many sub-steps as the most loaded wheel needs,
        // with the robot's speed held for the step.
        int wheelSteps =
                Math.max(
                        1,
                        (int)
                                Math.ceil(
                                        dt
                                                * gripMax
                                                / Tire.PEAK_SLIP
                                                * r
                                                * r
                                                / wheelInertia
                                                / 0.5));
        double wdt = dt / wheelSteps;
        double fx = 0, fy = 0, torque = 0;
        supplyAmps = 0;
        for (int i = 0; i < n; i++) {
            double cvx0 = vx - omega * my[i];
            double cvy0 = vy + omega * mx[i];
            double grip = tireFriction * loads.load(i); // most force this tire can give now
            stepSteer(i, dt, Math.hypot(cvx0, cvy0), grip);
            double c = Math.cos(steerAngle[i]);
            double sn = Math.sin(steerAngle[i]);
            // The robot's velocity at this wheel, along and across the wheel.
            double cvx = vx - omega * my[i];
            double cvy = vy + omega * mx[i];
            double along = cvx * c + cvy * sn;
            double across = -cvx * sn + cvy * c;
            double driveMotorOmega =
                    wheelOmega[i] * cfg.driveRatio() + steerOmega[i] * cfg.coupleRatio();
            double asked = devices.driveAmps(i, driveMotorOmega, batteryVolts, dt);
            double amps =
                    dead(driveName[i])
                            ? 0
                            : driveMotor.supplyLimited(
                                    driveMotor.limit(asked, driveMotorOmega, batteryVolts),
                                    driveMotorOmega,
                                    batteryVolts,
                                    cfg.driveSupplyCurrentLimit());
            supplyAmps +=
                    Math.max(0, driveMotor.supplyCurrent(amps, driveMotorOmega, batteryVolts));
            driveAmps[i] = amps;
            // Tire force from slip: the wheel surface's speed against the ground's, along the
            // wheel,
            // and the ground's sideways speed. Both follow the grip curve and share one friction
            // circle. A wheel spinning faster than the ground pushes the robot (up to grip) and
            // spins up when the motor gives more than the tire holds.
            // The wheel: motor torque through lossy gearing, less what the tire pushes back.
            double motorTorque = gearOutput(driveMotor.kT() * amps * cfg.driveRatio(), i);
            double push = 0, side = 0;
            for (int k = 0; k < wheelSteps; k++) {
                double slipAlong = wheelOmega[i] * r - along;
                // One slip vector, one force against it: its size from the grip curve of the
                // total slip, its direction the slip's.
                double slip = Math.hypot(slipAlong, across);
                double perSlip = slip > 1e-9 ? grip * Tire.gripCurve(slip) / slip : 0;
                double p = perSlip * slipAlong;
                double sd = -perSlip * across;
                double wheelTorque = motorTorque - p * r - ROLLING_DRAG * r * r * wheelOmega[i];
                wheelOmega[i] += wheelTorque / wheelInertia * wdt;
                push += p / wheelSteps;
                side += sd / wheelSteps;
            }
            double wfx = push * c - side * sn;
            double wfy = push * sn + side * c;
            fx += wfx;
            fy += wfy;
            torque += mx[i] * wfy - my[i] * wfx;
            wheelAngle[i] += wheelOmega[i] * dt;
            double steerRot = steerAngle[i] / (2 * Math.PI);
            devices.sense(
                    i,
                    steerRot,
                    steerOmega[i],
                    wheelAngle[i] / (2 * Math.PI) * cfg.driveRatio() + steerRot * cfg.coupleRatio(),
                    driveMotorOmega,
                    batteryVolts,
                    !dead(encoderName[i]));
        }
        chargeUsedAh += supplyAmps * dt / 3600;
        // The battery sags under everything drawn this step (drive and steer).
        batteryVolts =
                Double.isNaN(batteryOverride)
                        ? Math.max(
                                6.0,
                                batteryOpenVolts
                                        - VOLTS_PER_AMP_HOUR * chargeUsedAh
                                        - batteryResistance * supplyAmps)
                        : batteryOverride;
        lastAx = fx / robotMassKg;
        lastAy = fy / robotMassKg;
        synchronized (loadSnapshot) {
            for (int i = 0; i < n; i++) {
                loadSnapshot[i] = loads.load(i);
                liftSnapshot[i] = loads.lifted(i);
            }
        }
        // The floor under the wheels: on a ramp, gravity pulls the robot downhill. (Weight transfer
        // above follows the tire forces only: gravity acts at the center of gravity, no moment.)
        double gax = 0, gay = 0, roll = 0, pitch = 0, z = 0;
        if (arena != null && arena.terrain().bumps() > 0) {
            double c0 = Math.cos(arena.startHeading), s0 = Math.sin(arena.startHeading);
            terrainContact.update(
                    arena.terrain(),
                    arena.startX + truthX * c0 - truthY * s0,
                    arena.startY + truthX * s0 + truthY * c0,
                    arena.startHeading + truthHeading);
            gax = terrainContact.ax();
            gay = terrainContact.ay();
            roll = terrainContact.roll();
            pitch = terrainContact.pitch();
            z = terrainContact.height();
        }
        // Rigid body in the robot frame (the frame turns with the robot).
        vx += (fx / robotMassKg + gax + omega * vy) * dt;
        vy += (fy / robotMassKg + gay - omega * vx) * dt;
        omega += torque / inertiaEff * dt;
        double h = truthHeading;
        double ch = Math.cos(h);
        double sh = Math.sin(h);
        truthX += (vx * ch - vy * sh) * dt;
        truthY += (vx * sh + vy * ch) * dt;
        truthHeading = h + omega * dt;
        if (arena != null) {
            collide(inertiaEff, dt);
        }
        synchronized (loadSnapshot) {
            truthSnapshot[0] = truthX;
            truthSnapshot[1] = truthY;
            truthSnapshot[2] = truthHeading;
            tiltSnapshot[0] = z;
            tiltSnapshot[1] = roll;
            tiltSnapshot[2] = pitch;
        }
        if (!dead("Pigeon")) {
            devices.senseGyro(truthHeading, omega, roll, pitch, batteryVolts);
        }
    }

    /**
     * Bumpers against the field: the true pose (which starts at 0, 0, 0) placed on the field at the
     * arena's start pose, pushed out of walls and elements, and the contact impulses taken out of
     * the body's motion.
     */
    private void collide(double inertia, double dt) {
        double c0 = Math.cos(arena.startHeading), s0 = Math.sin(arena.startHeading);
        double heading = arena.startHeading + truthHeading;
        double ch = Math.cos(heading), sh = Math.sin(heading);
        body.x = arena.startX + truthX * c0 - truthY * s0;
        body.y = arena.startY + truthX * s0 + truthY * c0;
        body.heading = heading;
        body.vx = vx * ch - vy * sh;
        body.vy = vx * sh + vy * ch;
        body.omega = omega;
        body.mass = robotMassKg;
        body.inertia = inertia;
        int n = arena.step(body, dt);
        contacts = n;
        synchronized (loadSnapshot) {
            var others = arena.robots();
            if (otherSnapshot.length != 3 * others.size()) {
                otherSnapshot = new double[3 * others.size()];
            }
            for (int k = 0; k < others.size(); k++) {
                var o = others.get(k).body;
                otherSnapshot[3 * k] = o.x;
                otherSnapshot[3 * k + 1] = o.y;
                otherSnapshot[3 * k + 2] = o.heading;
            }
        }
        if (n == 0) {
            return;
        }
        double dx = body.x - arena.startX, dy = body.y - arena.startY;
        truthX = dx * c0 + dy * s0;
        truthY = -dx * s0 + dy * c0;
        vx = body.vx * ch + body.vy * sh;
        vy = -body.vx * sh + body.vy * ch;
        omega = body.omega;
    }

    /**
     * Torque reaching the wheel from {@code motorTorque} at the motor side of the gearing: gears
     * pass {@code gearEfficiency} of it while driving (and the wheel back-drives through the same
     * loss), and bearings and seals take a constant drag against the wheel's turning.
     */
    private double gearOutput(double motorTorque, int i) {
        double w = wheelOmega[i];
        boolean driving = motorTorque * w >= 0;
        double t = driving ? motorTorque * gearEfficiency : motorTorque / gearEfficiency;
        // Drag fades in over the first 0.5 rad/s so a wheel at rest does not chatter.
        return t - Math.max(-1, Math.min(1, w / 0.5)) * gearDrag;
    }

    /** Robot turning inertia (kg m^2). */
    private double robotInertia() {
        double r2 = 0;
        for (int i = 0; i < mx.length; i++) {
            r2 += mx[i] * mx[i] + my[i] * my[i];
        }
        // A box robot about as wide as its wheelbase: about 0.6 of mass times module radius^2.
        return inertiaFactor * robotMassKg * r2 / mx.length;
    }

    /** One module's steering: motor current to motion. */
    private void stepSteer(int i, double dt, double groundSpeed, double tireLoadFriction) {
        double steerRatio = cfg.steerRatio();
        double steerMotorOmega = steerOmega[i] * steerRatio;
        double steerAsked = devices.steerAmps(i, steerMotorOmega, batteryVolts, dt);
        double steerAmps =
                dead(steerName[i])
                        ? 0
                        : steerMotor.supplyLimited(
                                steerMotor.limit(steerAsked, steerMotorOmega, batteryVolts),
                                steerMotorOmega,
                                batteryVolts,
                                cfg.steerSupplyCurrentLimit());
        steerAmpsNow[i] = steerAmps;
        supplyAmps +=
                Math.max(0, steerMotor.supplyCurrent(steerAmps, steerMotorOmega, batteryVolts));
        double steerTorque =
                steerMotor.kT() * steerAmps * steerRatio - steerFriction * steerOmega[i];
        double steerJ = steerInertia + steerMotor.rotorInertia() * steerRatio * steerRatio;
        // Turning a loaded tire in place scrubs the carpet: friction torque against the turn, much
        // less once the wheel is rolling. It holds the module still if the motor cannot beat it.
        double scrub = tireLoadFriction * SCRUB_ARM / (1 + groundSpeed / 0.3);
        if (Math.abs(steerOmega[i]) < 1e-3 && Math.abs(steerTorque) <= scrub) {
            steerOmega[i] = 0;
        } else {
            double dir =
                    Math.abs(steerOmega[i]) >= 1e-3
                            ? Math.signum(steerOmega[i])
                            : Math.signum(steerTorque);
            double next = steerOmega[i] + (steerTorque - dir * scrub) / steerJ * dt;
            // Friction stops the turn; it never reverses it.
            steerOmega[i] = Math.signum(next) == -dir && Math.abs(steerOmega[i]) >= 1e-3 ? 0 : next;
        }
        steerAngle[i] += steerOmega[i] * dt;
    }

    /**
     * The real wheel radius in the sim (m). Normally the configured one; set it different to test
     * calibration, e.g. that the wheel-radius routine finds it.
     */
    public void setTrueWheelRadius(double meters) {
        trueWheelRadius = meters;
    }

    private double wheelRadius() {
        return Double.isNaN(trueWheelRadius) ? cfg.wheelRadius() : trueWheelRadius;
    }

    /** Devices without power make no torque and their sensors stop updating. */
    public void setSimFaults(SimFaults faults) {
        this.faults = faults;
    }

    private boolean dead(String device) {
        SimFaults f = faults;
        return f != null && f.unpowered(device);
    }

    // Loads as of the end of the last step, copied there so the main thread never sees one half
    // written (Phoenix mode steps on another thread).
    private final double[] loadLog;
    private final boolean[] liftLog;
    private final double[] loadSnapshot;
    private final boolean[] liftSnapshot;

    /**
     * Main thread: logs the true pose and odometry's error against it. Sim truth for judging
     * odometry; robot logic must never read it.
     */
    public void logTruth(double odometryX, double odometryY) {
        double x, y, truthH;
        synchronized (loadSnapshot) {
            x = truthSnapshot[0];
            y = truthSnapshot[1];
            truthH = truthSnapshot[2];
        }
        // The robot's true pose in the sim, from where it started (as odometry counts).
        org.littletonrobotics.junction.Logger.recordOutput(
                "Sim/TruthPose", new Pose2d(x, y, Rotation2d.fromRadians(truthH)));
        // Odometry position error against truth (m).
        org.littletonrobotics.junction.Logger.recordOutput(
                "Sim/OdometryError", Math.hypot(odometryX - x, odometryY - y));
        synchronized (loadSnapshot) {
            System.arraycopy(loadSnapshot, 0, loadLog, 0, loadLog.length);
            System.arraycopy(liftSnapshot, 0, liftLog, 0, liftLog.length);
        }
        if (arena != null) {
            double c0 = Math.cos(arena.startHeading), s0 = Math.sin(arena.startHeading);
            // The robot's true pose on the field (field frame), for AdvantageScope's field view.
            org.littletonrobotics.junction.Logger.recordOutput(
                    "Sim/FieldPose",
                    new Pose2d(
                            arena.startX + x * c0 - y * s0,
                            arena.startY + x * s0 + y * c0,
                            Rotation2d.fromRadians(arena.startHeading + truthH)));
            double z, roll, pitch;
            synchronized (loadSnapshot) {
                z = tiltSnapshot[0];
                roll = tiltSnapshot[1];
                pitch = tiltSnapshot[2];
            }
            // The robot's true pose on the field in 3D: height and tilt from the floor's bumps.
            org.littletonrobotics.junction.Logger.recordOutput(
                    "Sim/FieldPose3d",
                    new org.wpilib.math.geometry.Pose3d(
                            arena.startX + x * c0 - y * s0,
                            arena.startY + x * s0 + y * c0,
                            z,
                            new org.wpilib.math.geometry.Rotation3d(
                                    roll, pitch, arena.startHeading + truthH)));
            // Walls, field elements and robots the bumpers touched in the last physics step.
            org.littletonrobotics.junction.Logger.recordOutput("Sim/Contacts", contacts);
            Pose2d[] others;
            synchronized (loadSnapshot) {
                others = new Pose2d[otherSnapshot.length / 3];
                for (int k = 0; k < others.length; k++) {
                    others[k] =
                            new Pose2d(
                                    otherSnapshot[3 * k],
                                    otherSnapshot[3 * k + 1],
                                    Rotation2d.fromRadians(otherSnapshot[3 * k + 2]));
                }
            }
            // The other robots on the field (field frame), for AdvantageScope's field view.
            org.littletonrobotics.junction.Logger.recordOutput("Sim/OtherRobots", others);
        }
        // Charge drawn from the sim battery so far by the drivetrain (Ah).
        org.littletonrobotics.junction.Logger.recordOutput("Sim/BatteryChargeUsedAh", chargeUsedAh);
        // Each wheel's load on the floor in the sim, FL FR BL BR (N): weight transfer.
        org.littletonrobotics.junction.Logger.recordOutput("Sim/WheelLoads", loadLog);
        // Wheels the sim has lifted off the floor (the robot is starting to tip).
        org.littletonrobotics.junction.Logger.recordOutput("Sim/WheelLifted", liftLog);
    }

    /** Steer motor {@code i}'s stator current this step (A), after supply and battery limits. */
    double steerCurrent(int i) {
        return steerAmpsNow[i];
    }

    /** Drive motor {@code i}'s stator current this step (A). */
    double driveCurrent(int i) {
        return driveAmps[i];
    }

    MotorModel driveMotor() {
        return driveMotor;
    }

    MotorModel steerMotor() {
        return steerMotor;
    }

    /** Robot-frame velocity (m/s, m/s, rad/s), for comparing with real logs. */
    public double[] bodyVelocity() {
        return new double[] {vx, vy, omega};
    }

    private final java.util.concurrent.atomic.AtomicLong steps =
            new java.util.concurrent.atomic.AtomicLong();
    private volatile boolean running;

    /** Stops the physics thread (tests and replays that build more than one drivetrain). */
    public void stop() {
        running = false;
    }

    /** Physics steps taken since start, to check the step rate. */
    public long steps() {
        return steps.get();
    }

    /** The sim battery's voltage now (V), sagged under load. */
    public double batteryVolts() {
        return batteryVolts;
    }

    /** Each drive motor's current this step (A), for comparing with real logs. */
    public double[] driveCurrents() {
        return driveAmps.clone();
    }

    /** Puts the true robot at a pose (sim setup). */
    public void resetTruth(double x, double y, double heading) {
        lastAx = 0; // no acceleration carried across a reset
        lastAy = 0;
        truthX = x;
        truthY = y;
        truthHeading = heading;
    }

    /**
     * The robot's true pose on the field (field frame): the arena's start pose plus the true motion
     * since. Sim thread (the sim clock's), for simulated sensors such as cameras.
     */
    public Pose2d fieldPose() {
        double sx = arena != null ? arena.startX : 0;
        double sy = arena != null ? arena.startY : 0;
        double sh = arena != null ? arena.startHeading : 0;
        double c0 = Math.cos(sh), s0 = Math.sin(sh);
        return new Pose2d(
                sx + truthX * c0 - truthY * s0,
                sy + truthX * s0 + truthY * c0,
                Rotation2d.fromRadians(sh + truthHeading));
    }

    public double truthX() {
        return truthX;
    }

    public double truthY() {
        return truthY;
    }

    public double truthHeading() {
        return truthHeading;
    }
}
