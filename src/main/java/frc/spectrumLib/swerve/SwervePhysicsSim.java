package frc.spectrumLib.swerve;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

/**
 * Desktop physics for the drivetrain. Phoenix's simulator runs the motors' own closed loops; this
 * reads the torque current each motor asks for, turns it into motion with a DC motor model (current
 * limited by back-EMF and battery), and writes the motion back into the simulated motors, CANcoders
 * and Pigeon. Each module carries a quarter of the robot's mass; the robot's true motion is the
 * least-squares fit of the four wheels.
 *
 * <p>Runs on the swerve thread. The true pose is published for the main loop to log as sim truth;
 * robot logic must never read it.
 *
 * <p>Index: {@link #attach} | {@link #step} | {@link #stepModule} | truth getters
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

    /** Wheel rolling drag (N m per rad/s of wheel). */
    private static final double ROLLING_DRAG = 0.01;

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
    private double[] dx;
    private double[] dy;
    private final double[] twist = new double[3];

    private volatile double trueWheelRadius = Double.NaN;
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
        dx = new double[n];
        dy = new double[n];
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
        for (int i = 0; i < n; i++) {
            double oldWheel = wheelAngle[i];
            stepModule(i, dt);
            double d = (wheelAngle[i] - oldWheel) * r;
            dx[i] = d * Math.cos(steerAngle[i]);
            dy[i] = d * Math.sin(steerAngle[i]);
        }
        SwerveMath.toChassisVelocities(dx, dy, mx, my, twist);
        double h = truthHeading;
        double c = Math.cos(h + twist[2] / 2);
        double s = Math.sin(h + twist[2] / 2);
        truthX += twist[0] * c - twist[1] * s;
        truthY += twist[0] * s + twist[1] * c;
        truthHeading = h + twist[2];
        pigeonSim.setSupplyVoltage(BATTERY_VOLTS);
        pigeonSim.setRawYaw(Math.toDegrees(truthHeading));
        pigeonSim.setAngularVelocityZ(Math.toDegrees(twist[2] / dt));
    }

    /**
     * One module: motor currents to motion, motion back to the sim devices.
     *
     * <p>Phoenix sim reports torque current in the motor's own (inverted) frame, which is already
     * the mechanism's direction, but takes rotor position and velocity in the chassis frame, before
     * the invert. An inverted motor is mounted reversed, so its rotor turns opposite to the
     * mechanism: {@code sign} is -1 for it when writing rotor values back. The MK4i steer-to-drive
     * coupling is the same in every drive motor's own (inverted) frame, which is what {@link
     * SwerveIOPhoenix} removes.
     */
    private void stepModule(int i, double dt) {
        double steerRatio = cfg.steerRatio();
        double driveRatio = cfg.driveRatio();
        double steerSign = cfg.steerInverted() ? -1.0 : 1.0;
        double driveSign = cfg.driveInverted()[i] ? -1.0 : 1.0;
        double couple = cfg.coupleRatio();

        steerSim[i].setSupplyVoltage(BATTERY_VOLTS);
        // Torque current and the back-EMF limit are in the motor's own frame (measured: Phoenix
        // reports torque current after the invert, but takes rotor values before it).
        double steerMotorOmega = steerOmega[i] * steerRatio;
        // Voltage control: the winding current follows from volts minus back-EMF. Phoenix reports
        // motor voltage in the chassis frame (before the invert), unlike torque current.
        double steerAsked =
                cfg.steerVoltage()
                        ? (steerSign * steerSim[i].getMotorVoltage() - KE * steerMotorOmega) / R
                        : steerSim[i].getTorqueCurrent();
        double steerAmps = limit(steerAsked, steerMotorOmega);
        double steerTorque = KT * steerAmps * steerRatio - steerFriction * steerOmega[i];
        double steerJ = steerInertia + ROTOR_INERTIA * steerRatio * steerRatio;
        steerOmega[i] += steerTorque / steerJ * dt;
        steerAngle[i] += steerOmega[i] * dt;

        driveSim[i].setSupplyVoltage(BATTERY_VOLTS);
        double driveMotorOmega = wheelOmega[i] * driveRatio + steerOmega[i] * couple;
        double driveAmps = limit(driveSim[i].getTorqueCurrent(), driveMotorOmega);
        double r = wheelRadius();
        double wheelInertia =
                robotMassKg / steerAngle.length * r * r + ROTOR_INERTIA * driveRatio * driveRatio;
        double rolling = ROLLING_DRAG * wheelOmega[i];
        wheelOmega[i] += (KT * driveAmps * driveRatio - rolling) / wheelInertia * dt;
        wheelAngle[i] += wheelOmega[i] * dt;

        double steerRot = steerAngle[i] / (2 * Math.PI);
        double wheelRot = wheelAngle[i] / (2 * Math.PI);
        steerSim[i].setRawRotorPosition(steerSign * steerRot * steerRatio);
        steerSim[i].setRotorVelocity(steerSign * steerOmega[i] * steerRatio / (2 * Math.PI));
        driveSim[i].setRawRotorPosition(driveSign * (wheelRot * driveRatio + steerRot * couple));
        driveSim[i].setRotorVelocity(driveSign * driveMotorOmega / (2 * Math.PI));
        encoderSim[i].setSupplyVoltage(BATTERY_VOLTS);
        encoderSim[i].setRawPosition(steerRot - cfg.magnetOffsets()[i]);
        encoderSim[i].setVelocity(steerOmega[i] / (2 * Math.PI));
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
