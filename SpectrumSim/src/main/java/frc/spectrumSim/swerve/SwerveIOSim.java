package frc.spectrumSim.swerve;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.spectrumLib.hardware.SimFaults;
import frc.spectrumLib.swerve.ModuleCommands;
import frc.spectrumLib.swerve.PhoenixSwerveConfig;
import frc.spectrumLib.swerve.SwerveHealth;
import frc.spectrumLib.swerve.SwerveIO;
import frc.spectrumLib.swerve.SwerveMath;
import frc.spectrumLib.swerve.SwervePhoenixConfigs;
import frc.spectrumLib.swerve.SwerveRequests;
import frc.spectrumLib.swerve.SwerveSample;
import frc.spectrumSim.SimClock;
import frc.spectrumSim.TalonFXModel;
import org.wpilib.driverstation.RobotState;
import org.wpilib.system.RobotController;

/**
 * The drivetrain in the desktop sim, repeatable: each motor is a {@link TalonFXModel} given the
 * same configs ({@link SwervePhoenixConfigs}) and requests ({@link SwerveRequests}) the robot
 * sends, and {@link SwervePhysicsSim} moves them, all on the {@link SimClock}. Two runs of the same
 * script give the same log, and a run is as fast as the computer allows.
 *
 * <p>Simulated faults ({@link SimFaults}) act here as on the robot: an unheard device's readings
 * freeze, an unpowered one ignores commands and makes no torque, failed configs leave a motor
 * unconfigured (no gains: it makes no torque) until a retry works, and a reboot forgets the motor's
 * request.
 *
 * <p>With no clock it steps itself one period per {@link #waitForSample} (tests and tools).
 *
 * <p>Index: constructor | {@link #waitForSample} | {@link #setModules} | {@link
 * #retryFailedConfigs} | {@link #updateHealth} | the physics' devices
 */
public final class SwerveIOSim implements SwerveIO {
    private final PhoenixSwerveConfig cfg;
    private final SwervePhysicsSim sim;
    private final SimFaults faults;
    private volatile SimClock.Participant clock;

    /** The sim clock itself, to join again if the swerve thread is restarted. */
    private final SimClock simClock;

    private volatile boolean left;
    private final int n;
    private final TalonFXModel[] drive;
    private final TalonFXModel[] steer;
    private final SwerveRequests requests;
    private final long ticksPerPeriod;

    /**
     * Spacing between samples on a real SystemCore CAN bus at 250 Hz (ms, and share): FM's 13
     * TXMAN1 matches, 457,911 samples, rounded to whole ms (mean 4.1 ms, about 243 Hz).
     */
    private static final int[] SPACING_MS = {2, 3, 4, 5, 6, 7, 8};

    private static final double[] SPACING_SHARE = {
        0.0068, 0.2138, 0.5037, 0.2307, 0.0298, 0.0103, 0.0047
    };

    private static final double SPACING_TOTAL = 0.9998;

    /** Share of reads that fail (stale frames), from the same matches. */
    private static final double FAILED_READ = 350.0 / 420060;

    /** Seeded: sample timing repeats run to run. */
    private final java.util.Random canRandom = new java.util.Random(250);

    /** Ticks until the next sample: the measured spacing, scaled to this loop's rate. */
    private long nextSpacing() {
        double u = canRandom.nextDouble();
        double total = SPACING_TOTAL;
        double sum = 0;
        int ms = 4;
        for (int i = 0; i < SPACING_MS.length; i++) {
            sum += SPACING_SHARE[i] / total;
            if (u < sum) {
                ms = SPACING_MS[i];
                break;
            }
        }
        return Math.max(1, Math.round(ms * ticksPerPeriod / 4.0));
    }

    private final double metersPerWheelRotation;

    /** Device names as PhoenixDeviceSet names them: Drive0, Steer0, Encoder0, ..., Pigeon. */
    private final String[] names;

    private final boolean[] configured;
    private final double[] measuredSteerRps;

    // What the physics last wrote to the sensors (rotor rotations and rps, mechanism direction).
    private final double[] driveRotor;
    private final double[] driveRotorRps;
    private final double[] steerRotor;
    private final double[] steerRotorRps;
    private double heading;
    private double omega;

    /** Roll and pitch (rad) from one physics step, written and read together under its lock. */
    private final double[] tilt = new double[2];

    /** What a TalonFX runs on before its config arrives. */
    private static final com.ctre.phoenix6.configs.TalonFXConfiguration UNCONFIGURED =
            new com.ctre.phoenix6.configs.TalonFXConfiguration();

    private long tick;
    private double selfTime;
    private long resets;
    private boolean driveBrake = true;

    /**
     * @param clock the sim clock, or null to step one period per sample
     * @param faults simulated device faults, or null
     */
    public SwerveIOSim(
            PhoenixSwerveConfig cfg, SwervePhysicsSim sim, SimFaults faults, SimClock clock) {
        this.cfg = cfg;
        this.sim = sim;
        this.faults = faults;
        n = cfg.modules();
        drive = new TalonFXModel[n];
        steer = new TalonFXModel[n];
        names = new String[3 * n + 1];
        configured = new boolean[names.length];
        for (int i = 0; i < n; i++) {
            names[i] = "Drive" + i;
            names[n + i] = "Steer" + i;
            names[2 * n + i] = "Encoder" + i;
            drive[i] = new TalonFXModel(names[i], sim.driveMotor());
            steer[i] = new TalonFXModel(names[n + i], sim.steerMotor());
        }
        names[3 * n] = "Pigeon";
        for (int d = 0; d < names.length; d++) {
            tryConfigure(d);
        }
        measuredSteerRps = new double[n];
        driveRotor = new double[n];
        driveRotorRps = new double[n];
        steerRotor = new double[n];
        steerRotorRps = new double[n];
        requests = new SwerveRequests(cfg, faults);
        metersPerWheelRotation = 2 * Math.PI * cfg.wheelRadius();
        ticksPerPeriod = Math.max(1, Math.round(1.0 / cfg.signalHz() / SimClock.TICK_SECONDS));
        sim.attach(cfg, new Devices());
        sim.setSimFaults(faults);
        simClock = clock;
        if (clock != null) {
            this.clock = clock.join();
            clock.addStepper(this::physicsStep);
        } else {
            this.clock = null;
        }
    }

    /** One physics tick: the motors see the robot's enable, then everything moves. */
    private void physicsStep(double dt) {
        boolean enabled = RobotState.isEnabled();
        for (int i = 0; i < n; i++) {
            drive[i].setEnabled(enabled);
            steer[i].setEnabled(enabled);
        }
        sim.step(dt);
    }

    @Override
    public void waitForSample(SwerveSample out) {
        // A read now and then misses its frames, as Phoenix's waitForAll times out (FM's 13 TXMAN1
        // matches failed 350 of 420,060): it waits out the timeout (two periods) and returns the
        // readings it had, stale.
        boolean failed = canRandom.nextDouble() < FAILED_READ;
        long wait = nextSpacing() + (failed ? 2 * ticksPerPeriod : 0);
        if (clock != null) {
            tick = clock.awaitTick(tick + wait);
            // When it was read, on the robot clock the cameras use. (Timer.getTimestamp is
            // AdvantageKit's loop start: every sample in a loop would share it.)
            out.timestamp = RobotController.getMonotonicTime() * 1e-9;
        } else {
            for (long k = 0; k < wait; k++) {
                physicsStep(SimClock.TICK_SECONDS);
            }
            selfTime += wait * SimClock.TICK_SECONDS;
            out.timestamp = selfTime;
        }
        out.allGood = !failed;
        if (failed) {
            return; // stale: the readings stay as they were
        }
        double couple = cfg.coupleRatio() / cfg.driveRatio();
        for (int i = 0; i < n; i++) {
            if (unheard(names[i]) || unheard(names[n + i])) {
                out.moduleOk[i] = false; // no frames from this module: its readings freeze
                continue;
            }
            double steerRot = steer[i].position();
            double wheelRot = drive[i].position();
            out.distance[i] = (wheelRot - steerRot * couple) * metersPerWheelRotation;
            out.velocity[i] =
                    (drive[i].velocity() - steer[i].velocity() * couple) * metersPerWheelRotation;
            out.angle[i] = SwerveMath.angleModulus(steerRot * 2 * Math.PI);
            out.moduleOk[i] = true;
        }
        out.gyroOk = !unheard("Pigeon");
        if (out.gyroOk) {
            out.yaw = heading;
            out.angularVelocityZWorld = omega;
        }
    }

    @Override
    public void setModules(
            double[] angle,
            double[] velocity,
            double[] steerVelocityPlan,
            double[] wheelForce,
            ModuleCommands commands) {
        for (int i = 0; i < n; i++) {
            // An unheard steer motor's velocity signal is frozen at its last value.
            if (!unheard(names[n + i])) {
                measuredSteerRps[i] = steer[i].velocity();
            }
        }
        requests.setModules(
                angle, velocity, steerVelocityPlan, wheelForce, commands, measuredSteerRps, motors);
    }

    private final SwerveRequests.Motors motors =
            new SwerveRequests.Motors() {
                @Override
                public void setDrive(int i, ControlRequest r) {
                    drive[i].setControl(r);
                }

                @Override
                public void setSteer(int i, ControlRequest r) {
                    steer[i].setControl(r);
                }
            };

    @Override
    public void close() {
        if (clock != null) {
            clock.leave();
            left = true;
        }
    }

    @Override
    public void open() {
        // A restarted swerve thread: the clock must wait for it again.
        if (left && simClock != null) {
            left = false;
            clock = simClock.join();
            tick = clock.now();
        }
    }

    @Override
    public void setNeutral() {
        requests.setNeutral(motors);
    }

    @Override
    public void setDriveBrake(boolean brake) {
        driveBrake = brake;
        for (int i = 0; i < n; i++) {
            if (configured[i]) {
                drive[i].setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
            }
        }
    }

    /** As PhoenixDeviceSet: a rebooted device is reconfigured; a failed config is retried. */
    @Override
    public void retryFailedConfigs() {
        for (int d = 0; d < names.length; d++) {
            if (faults != null && faults.consumeReset(names[d])) {
                resets++;
                configured[d] = false;
                TalonFXModel m = motor(d);
                if (m != null) {
                    m.reboot();
                }
            }
            if (!configured[d]) {
                tryConfigure(d);
            }
        }
    }

    private void tryConfigure(int d) {
        boolean ok = faults == null || !faults.configFails(names[d]);
        configured[d] = ok;
        TalonFXModel m = motor(d);
        if (m == null) {
            return;
        }
        if (!ok) {
            // A TalonFX that never got its config runs on defaults: no gains, so no torque.
            m.configure(UNCONFIGURED);
        } else if (d < n) {
            m.configure(SwervePhoenixConfigs.drive(cfg, d, driveBrake));
        } else {
            m.configure(SwervePhoenixConfigs.steer(cfg, d - n));
        }
    }

    /** The motor model for a device index, or null for an encoder or the gyro. */
    private TalonFXModel motor(int d) {
        return d < n ? drive[d] : d < 2 * n ? steer[d - n] : null;
    }

    @Override
    public void updateHealth(SwerveHealth h) {
        double volts = sim.batteryVolts();
        for (int i = 0; i < n; i++) {
            h.driveConnected[i] = !unheard(names[i]);
            h.steerConnected[i] = !unheard(names[n + i]);
            h.encoderConnected[i] = !unheard(names[2 * n + i]);
            h.driveConfigured[i] = configured[i];
            h.steerConfigured[i] = configured[n + i];
            h.encoderConfigured[i] = configured[2 * n + i];
            double driveAmps = sim.driveCurrent(i);
            double steerAmps = sim.steerCurrent(i);
            h.driveStatorCurrent[i] = driveAmps;
            h.steerStatorCurrent[i] = steerAmps;
            h.driveSupplyCurrent[i] =
                    sim.driveMotor()
                            .supplyCurrent(driveAmps, driveRotorRps[i] * 2 * Math.PI, volts);
            h.steerSupplyCurrent[i] =
                    sim.steerMotor()
                            .supplyCurrent(steerAmps, steerRotorRps[i] * 2 * Math.PI, volts);
            h.driveDeviceTemp[i] = 25;
            h.steerDeviceTemp[i] = 25;
            // An unplugged CANcoder leaves the fused steer sensor out of sync.
            h.fusedOk[i] = !unheard(names[2 * n + i]);
            h.magnetOk[i] = true;
        }
        h.supplyVoltage = volts;
        // No wiring resistance modeled: every motor sees the battery.
        java.util.Arrays.fill(h.driveSupplyVoltage, volts);
        java.util.Arrays.fill(h.steerSupplyVoltage, volts);
        h.pigeonConnected = !unheard("Pigeon") && configured[3 * n];
        h.tiltOk = h.pigeonConnected;
        synchronized (tilt) {
            h.roll = h.tiltOk ? tilt[0] : 0;
            h.pitch = h.tiltOk ? tilt[1] : 0;
        }
        h.timesync = false;
        h.deviceResets = resets;
        h.signalsSet = true;
        h.driveBrake = driveBrake;
    }

    private boolean unheard(String device) {
        return faults != null && faults.unheard(device);
    }

    /** The physics' view of the devices: our TalonFX models and ideal sensors. */
    private final class Devices implements SwerveSimDevices {
        @Override
        public double driveAmps(int i, double driveMotorOmega, double batteryVolts, double dt) {
            double rps = driveMotorOmega / (2 * Math.PI);
            return drive[i].update(driveRotor[i], rps, batteryVolts, dt);
        }

        @Override
        public double steerAmps(int i, double steerMotorOmega, double batteryVolts, double dt) {
            double rps = steerMotorOmega / (2 * Math.PI);
            return steer[i].update(steerRotor[i], rps, batteryVolts, dt);
        }

        @Override
        public void sense(
                int i,
                double steerRot,
                double steerOmega,
                double driveRotorRot,
                double driveMotorOmega,
                double batteryVolts,
                boolean encoderAlive) {
            // The fused steer sensor is the CANcoder; rotor and CANcoder agree in the sim.
            steerRotor[i] = steerRot * cfg.steerRatio();
            steerRotorRps[i] = steerOmega * cfg.steerRatio() / (2 * Math.PI);
            driveRotor[i] = driveRotorRot;
            driveRotorRps[i] = driveMotorOmega / (2 * Math.PI);
        }

        @Override
        public void senseGyro(
                double heading, double omega, double roll, double pitch, double batteryVolts) {
            synchronized (tilt) {
                tilt[0] = roll;
                tilt[1] = pitch;
            }
            SwerveIOSim.this.heading = heading;
            SwerveIOSim.this.omega = omega;
        }
    }
}
