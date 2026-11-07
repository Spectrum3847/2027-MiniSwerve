package frc.spectrumLib.swerve;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import frc.spectrumLib.hardware.SimFaults;

/**
 * Desktop physics for the drivetrain. Phoenix's simulator runs the motors' own closed loops; this
 * reads the torque current each motor asks for, turns it into motion with a DC motor model (current
 * limited by back-EMF and battery), and writes the motion back into the simulated motors, CANcoders
 * and Pigeon.
 *
 * <p>The robot is one rigid body. Each wheel pushes along its direction with its motor's torque and
 * grips sideways with tire friction (both limited by friction on carpet); those forces move the
 * robot, and each wheel rolls with the robot's motion at that wheel. So a dead or coasting wheel
 * rolls along, and a wheel pointed the wrong way scrubs and fights the others, as on a real robot.
 *
 * <p>Not modeled: wheel spin (slip along the wheel). A wheel always reports the robot's speed at
 * it, so sim odometry never sees spin; sim odometry accuracy is not evidence about the real robot.
 * A rebooted device also keeps its sim rotor position (a real TalonFX restarts its rotor count;
 * steering is safe because it fuses the CANcoder).
 *
 * <p>Runs on the swerve thread. The true pose is published for the main loop to log as sim truth;
 * robot logic must never read it.
 *
 * <p>Index: {@link #attach} | {@link #step} | {@link #stepSteer} | {@link #writeModule} | truth
 * getters
 */
public final class SwervePhysicsSim {
    /** Battery open-circuit voltage (V), a charged battery. */
    private static final double BATTERY_OPEN_VOLTS = 12.7;

    /** Battery internal plus wiring resistance (ohm); voltage sags by this times total current. */
    private static final double BATTERY_RESISTANCE = 0.025;

    /** Lever arm of the tire contact patch against turning in place (m): sets steer scrub. */
    private static final double SCRUB_ARM = 0.012;

    /** Battery voltage now (V), sagged by last step's total current. */
    private volatile double batteryVolts = BATTERY_OPEN_VOLTS;

    private double supplyAmps;

    /** Wheel rolling drag. */
    private static final double ROLLING_DRAG = 0.5; // N per m/s along the wheel

    private static final double GRAVITY = 9.81;

    private final MotorModel driveMotor;
    private final MotorModel steerMotor;
    private final double robotMassKg;
    private final double tireFriction;
    private final double steerInertia; // kg m^2, module about its steer axis
    private final double steerFriction; // N m per rad/s
    private final double[] mx;
    private final double[] my;

    private PhoenixSwerveConfig cfg;
    private TalonFXSimState[] driveSim;
    private TalonFXSimState[] steerSim;
    private CANcoderSimState[] encoderSim;
    private Pigeon2SimState pigeonSim;

    private double[] steerAngle; // rad
    private double[] steerOmega; // rad/s
    private double[] wheelAngle; // rad
    private double[] wheelOmega; // rad/s
    private double[] driveAmps;
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
    }

    /** Called by {@link SwerveIOPhoenix} once its devices exist. */
    void attach(
            PhoenixSwerveConfig cfg,
            TalonFX[] drive,
            TalonFX[] steer,
            CANcoder[] encoder,
            Pigeon2 pigeon) {
        this.cfg = cfg;
        int n = drive.length;
        driveSim = new TalonFXSimState[n];
        steerSim = new TalonFXSimState[n];
        encoderSim = new CANcoderSimState[n];
        steerAngle = new double[n];
        steerOmega = new double[n];
        wheelAngle = new double[n];
        wheelOmega = new double[n];
        driveAmps = new double[n];
        for (int i = 0; i < n; i++) {
            driveSim[i] = drive[i].getSimState();
            steerSim[i] = steer[i].getSimState();
            encoderSim[i] = encoder[i].getSimState();
        }
        pigeonSim = pigeon.getSimState();
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
                                steps++;
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

    /** Advances the physics by {@code dt} seconds and writes the result to the sim devices. */
    @SuppressWarnings("NonAtomicVolatileUpdate") // only the swerve thread writes truth
    void step(double dt) {
        if (cfg == null) {
            return;
        }
        int n = steerAngle.length;
        double r = wheelRadius();
        double g2 = cfg.driveRatio() * cfg.driveRatio();
        // The drive rotors' inertia, seen at the wheels, adds to the robot's.
        double rotorMass = driveMotor.rotorInertia() * g2 / (r * r);
        double massEff = robotMassKg + n * rotorMass;
        double inertiaEff = robotInertia();
        double grip = tireFriction * robotMassKg * GRAVITY / n; // most force a tire can give
        double fx = 0, fy = 0, torque = 0;
        supplyAmps = 0;
        for (int i = 0; i < n; i++) {
            double cvx0 = vx - omega * my[i];
            double cvy0 = vy + omega * mx[i];
            stepSteer(i, dt, Math.hypot(cvx0, cvy0), grip);
            double c = Math.cos(steerAngle[i]);
            double sn = Math.sin(steerAngle[i]);
            // The robot's velocity at this wheel, along and across the wheel.
            double cvx = vx - omega * my[i];
            double cvy = vy + omega * mx[i];
            double along = cvx * c + cvy * sn;
            double across = -cvx * sn + cvy * c;
            // The wheel rolls with the robot (no slip along it): a dead or coasting wheel just
            // follows; a powered one pushes.
            wheelOmega[i] = along / r;
            double driveMotorOmega =
                    wheelOmega[i] * cfg.driveRatio() + steerOmega[i] * cfg.coupleRatio();
            driveSim[i].setSupplyVoltage(batteryVolts);
            double amps =
                    dead(driveName[i])
                            ? 0
                            : driveMotor.supplyLimited(
                                    driveMotor.limit(
                                            driveSim[i].getTorqueCurrent(),
                                            driveMotorOmega,
                                            batteryVolts),
                                    driveMotorOmega,
                                    batteryVolts,
                                    cfg.driveSupplyCurrentLimit());
            supplyAmps +=
                    Math.max(0, driveMotor.supplyCurrent(amps, driveMotorOmega, batteryVolts));
            driveAmps[i] = amps;
            double push = driveMotor.kT() * amps * cfg.driveRatio() / r - ROLLING_DRAG * along;
            // Sideways the tire grips: it stops the slide within a step (the rotors do not resist
            // sideways motion, so the robot's own mass), up to its friction.
            double side = -across * (robotMassKg / n) / dt;
            // One friction circle per tire: push and grip share it.
            double total = Math.hypot(push, side);
            if (total > grip) {
                push *= grip / total;
                side *= grip / total;
            }
            double wfx = push * c - side * sn;
            double wfy = push * sn + side * c;
            fx += wfx;
            fy += wfy;
            torque += mx[i] * wfy - my[i] * wfx;
            wheelAngle[i] += wheelOmega[i] * dt;
            writeModule(i, driveMotorOmega);
        }
        // The battery sags under everything drawn this step (drive and steer).
        batteryVolts = Math.max(6.0, BATTERY_OPEN_VOLTS - BATTERY_RESISTANCE * supplyAmps);
        // Rigid body in the robot frame (the frame turns with the robot).
        vx += (fx / massEff + omega * vy) * dt;
        vy += (fy / massEff - omega * vx) * dt;
        omega += torque / inertiaEff * dt;
        double h = truthHeading;
        double ch = Math.cos(h);
        double sh = Math.sin(h);
        truthX += (vx * ch - vy * sh) * dt;
        truthY += (vx * sh + vy * ch) * dt;
        truthHeading = h + omega * dt;
        pigeonSim.setSupplyVoltage(batteryVolts);
        if (!dead("Pigeon")) {
            pigeonSim.setRawYaw(Math.toDegrees(truthHeading));
            pigeonSim.setAngularVelocityZ(Math.toDegrees(omega));
        }
    }

    /** Robot turning inertia (kg m^2), plus the drive rotors' share. */
    private double robotInertia() {
        double r2 = 0;
        for (int i = 0; i < mx.length; i++) {
            r2 += mx[i] * mx[i] + my[i] * my[i];
        }
        double r = wheelRadius();
        double rotorMass =
                driveMotor.rotorInertia() * cfg.driveRatio() * cfg.driveRatio() / (r * r);
        // A box robot about as wide as its wheelbase: about 0.6 of mass times module radius^2.
        return 0.6 * robotMassKg * r2 / mx.length + rotorMass * r2;
    }

    /**
     * One module's steering: motor current to motion.
     *
     * <p>Phoenix sim reports torque current in the motor's own (inverted) frame, which is already
     * the mechanism's direction, but takes rotor position and velocity in the chassis frame, before
     * the invert, and reports motor voltage in the chassis frame too. An inverted motor is mounted
     * reversed, so its rotor turns opposite to the mechanism. The MK4i steer-to-drive coupling is
     * the same in every drive motor's own (inverted) frame, which is what {@link SwerveIOPhoenix}
     * removes.
     */
    private void stepSteer(int i, double dt, double groundSpeed, double tireLoadFriction) {
        double steerRatio = cfg.steerRatio();
        double steerSign = cfg.steerInverted() ? -1.0 : 1.0;
        steerSim[i].setSupplyVoltage(batteryVolts);
        double steerMotorOmega = steerOmega[i] * steerRatio;
        double steerAsked =
                cfg.steerVoltage()
                        ? (steerSign * steerSim[i].getMotorVoltage()
                                        - steerMotor.kE() * steerMotorOmega)
                                / steerMotor.resistance()
                        : steerSim[i].getTorqueCurrent();
        double steerAmps =
                dead(steerName[i])
                        ? 0
                        : steerMotor.limit(steerAsked, steerMotorOmega, batteryVolts);
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

    /** Writes one module's motion back to its simulated motors and CANcoder. */
    private void writeModule(int i, double driveMotorOmega) {
        double steerRatio = cfg.steerRatio();
        double steerSign = cfg.steerInverted() ? -1.0 : 1.0;
        double driveSign = cfg.driveInverted()[i] ? -1.0 : 1.0;
        double steerRot = steerAngle[i] / (2 * Math.PI);
        double wheelRot = wheelAngle[i] / (2 * Math.PI);
        steerSim[i].setRawRotorPosition(steerSign * steerRot * steerRatio);
        steerSim[i].setRotorVelocity(steerSign * steerOmega[i] * steerRatio / (2 * Math.PI));
        driveSim[i].setRawRotorPosition(
                driveSign * (wheelRot * cfg.driveRatio() + steerRot * cfg.coupleRatio()));
        driveSim[i].setRotorVelocity(driveSign * driveMotorOmega / (2 * Math.PI));
        if (!dead(encoderName[i])) {
            encoderSim[i].setSupplyVoltage(batteryVolts);
            encoderSim[i].setRawPosition(steerRot - cfg.magnetOffsets()[i]);
            encoderSim[i].setVelocity(steerOmega[i] / (2 * Math.PI));
        }
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

    /**
     * Main thread: logs the true pose and odometry's error against it. Sim truth for judging
     * odometry; robot logic must never read it.
     */
    public void logTruth(double odometryX, double odometryY) {
        double x = truthX;
        double y = truthY;
        // The robot's true pose in the sim (field frame).
        org.littletonrobotics.junction.Logger.recordOutput(
                "Sim/TruthPose",
                new org.wpilib.math.geometry.Pose2d(
                        x, y, org.wpilib.math.geometry.Rotation2d.fromRadians(truthHeading)));
        // Odometry position error against truth (m).
        org.littletonrobotics.junction.Logger.recordOutput(
                "Sim/OdometryError", Math.hypot(odometryX - x, odometryY - y));
    }

    /** Robot-frame velocity (m/s, m/s, rad/s), for comparing with real logs. */
    public double[] bodyVelocity() {
        return new double[] {vx, vy, omega};
    }

    private volatile long steps;
    private volatile boolean running;

    /** Stops the physics thread (tests and replays that build more than one drivetrain). */
    public void stop() {
        running = false;
    }

    /** Physics steps taken since start, to check the step rate. */
    public long steps() {
        return steps;
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
        truthX = x;
        truthY = y;
        truthHeading = heading;
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
