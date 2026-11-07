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
 * <p>Runs on the swerve thread. The true pose is published for the main loop to log as sim truth;
 * robot logic must never read it.
 *
 * <p>Index: {@link #attach} | {@link #step} | {@link #stepSteer} | {@link #writeModule} | truth
 * getters
 */
public final class SwervePhysicsSim {
    /** Falcon 500 with FOC: stall torque / stall current (N m per A). */
    private static final double KT = 5.84 / 304.0;

    /** Winding resistance (ohm): 12 V / stall current. */
    private static final double R = 12.0 / 304.0;

    /** Back-EMF (V per rad/s of rotor): 12 V at free speed, 6080 rpm. */
    private static final double KE = 12.0 / (6080.0 * 2 * Math.PI / 60.0);

    private static final double BATTERY_VOLTS = 12.0;

    /** Falcon 500 rotor inertia (kg m^2); the gearing multiplies it by the ratio squared. */
    private static final double ROTOR_INERTIA = 0.0000565;

    /** Wheel rolling drag. */
    private static final double ROLLING_DRAG = 0.5; // N per m/s along the wheel

    /** Tire-to-carpet friction coefficient (typical FRC tread on carpet). */
    private static final double TIRE_FRICTION = 1.1;

    private static final double GRAVITY = 9.81;

    private final double robotMassKg;
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
    // Rigid-body robot velocity, in the robot frame.
    private double vx;
    private double vy;
    private double omega;

    private volatile double trueWheelRadius = Double.NaN;
    private volatile SimFaults faults;
    private volatile double truthX;
    private volatile double truthY;
    private volatile double truthHeading;

    /**
     * @param mx module x positions (m)
     * @param my module y positions (m)
     */
    public SwervePhysicsSim(
            double robotMassKg,
            double steerInertia,
            double steerFriction,
            double[] mx,
            double[] my) {
        this.robotMassKg = robotMassKg;
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
        for (int i = 0; i < n; i++) {
            driveSim[i] = drive[i].getSimState();
            steerSim[i] = steer[i].getSimState();
            encoderSim[i] = encoder[i].getSimState();
        }
        pigeonSim = pigeon.getSimState();
    }

    private long lastStepNanos;
    private long nextDeadlineNanos;

    /**
     * Steps the physics by the wall time since the last call (at most 20 ms).
     *
     * @return when this step started (ns), for {@link #waitForNextPeriod}
     */
    long stepRealTime(double hz) {
        long now = System.nanoTime();
        double dt = lastStepNanos == 0 ? 1.0 / hz : (now - lastStepNanos) * 1e-9;
        lastStepNanos = now;
        step(Math.min(dt, 0.02));
        return now;
    }

    /** Keeps the sim loop on a fixed 1/hz schedule; parkNanos can wake early on Windows. */
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
        double rotorMass = ROTOR_INERTIA * g2 / (r * r);
        double massEff = robotMassKg + n * rotorMass;
        double inertiaEff = robotInertia();
        double grip = TIRE_FRICTION * robotMassKg * GRAVITY / n; // most force a tire can give
        double fx = 0, fy = 0, torque = 0;
        for (int i = 0; i < n; i++) {
            stepSteer(i, dt);
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
            driveSim[i].setSupplyVoltage(BATTERY_VOLTS);
            double amps =
                    dead("Drive" + i) ? 0 : limit(driveSim[i].getTorqueCurrent(), driveMotorOmega);
            double push = KT * amps * cfg.driveRatio() / r - ROLLING_DRAG * along;
            push = Math.max(-grip, Math.min(grip, push));
            // Sideways the tire grips: it stops the slide within a step, up to its friction.
            double side = -across * (massEff / n) / dt;
            side = Math.max(-grip, Math.min(grip, side));
            double wfx = push * c - side * sn;
            double wfy = push * sn + side * c;
            fx += wfx;
            fy += wfy;
            torque += mx[i] * wfy - my[i] * wfx;
            wheelAngle[i] += wheelOmega[i] * dt;
            writeModule(i, driveMotorOmega);
        }
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
        pigeonSim.setSupplyVoltage(BATTERY_VOLTS);
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
        double rotorMass = ROTOR_INERTIA * cfg.driveRatio() * cfg.driveRatio() / (r * r);
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
    private void stepSteer(int i, double dt) {
        double steerRatio = cfg.steerRatio();
        double steerSign = cfg.steerInverted() ? -1.0 : 1.0;
        steerSim[i].setSupplyVoltage(BATTERY_VOLTS);
        double steerMotorOmega = steerOmega[i] * steerRatio;
        double steerAsked =
                cfg.steerVoltage()
                        ? (steerSign * steerSim[i].getMotorVoltage() - KE * steerMotorOmega) / R
                        : steerSim[i].getTorqueCurrent();
        double steerAmps = dead("Steer" + i) ? 0 : limit(steerAsked, steerMotorOmega);
        double steerTorque = KT * steerAmps * steerRatio - steerFriction * steerOmega[i];
        double steerJ = steerInertia + ROTOR_INERTIA * steerRatio * steerRatio;
        steerOmega[i] += steerTorque / steerJ * dt;
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
        if (!dead("Encoder" + i)) {
            encoderSim[i].setSupplyVoltage(BATTERY_VOLTS);
            encoderSim[i].setRawPosition(steerRot - cfg.magnetOffsets()[i]);
            encoderSim[i].setVelocity(steerOmega[i] / (2 * Math.PI));
        }
    }

    /** Current the battery can push through a motor turning at {@code rotorOmega} (rad/s). */
    private static double limit(double amps, double rotorOmega) {
        double emf = KE * rotorOmega;
        double max = (BATTERY_VOLTS - emf) / R;
        double min = (-BATTERY_VOLTS - emf) / R;
        return Math.max(min, Math.min(max, amps));
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
        return faults != null && faults.unpowered(device);
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
