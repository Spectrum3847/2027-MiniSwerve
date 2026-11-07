package frc.spectrumLib.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import frc.spectrumLib.hardware.PhoenixBackground;
import java.util.ArrayList;
import java.util.List;

/**
 * Our swerve on plain Phoenix 6 Pro devices (no CTRE swerve API). Each module's control runs on the
 * motors at 1 kHz: steer with PositionTorqueCurrentFOC on the fused CANcoder, drive with
 * VelocityTorqueCurrentFOC. The 250 Hz thread only sends setpoints and reads latency-compensated
 * signals.
 *
 * <p>In simulation the same devices run through Phoenix's simulator and {@link SwervePhysicsSim}
 * moves them, so sim exercises the exact control path the robot uses.
 *
 * <p>Index: constructor (configure) | {@link #waitForSample} | {@link #setModules} | {@link
 * #updateHealth}
 */
public final class SwerveIOPhoenix implements SwerveIO {
    private final PhoenixSwerveConfig cfg;
    private final int n;
    private final TalonFX[] drive;
    private final TalonFX[] steer;
    private final CANcoder[] encoder;
    private final Pigeon2 pigeon;

    private final StatusSignal<?>[] drivePosition;
    private final StatusSignal<?>[] driveVelocity;
    private final StatusSignal<?>[] steerPosition;
    private final StatusSignal<?>[] steerVelocity;
    private final StatusSignal<?> yaw;
    private final StatusSignal<?> yawRate;
    private final BaseStatusSignal[] fast;
    private final BaseStatusSignal[] slow;

    // UpdateFreqHz 0: send each setpoint once when we call setControl, as CTRE's swerve does,
    // instead of Phoenix also repeating it on its own 100 Hz timer. Halves control traffic.
    private final VelocityTorqueCurrentFOC driveRequest =
            new VelocityTorqueCurrentFOC(0).withUpdateFreqHz(0);
    private final PositionTorqueCurrentFOC steerRequest =
            new PositionTorqueCurrentFOC(0).withUpdateFreqHz(0);
    private final MotionMagicExpoTorqueCurrentFOC steerExpoRequest =
            new MotionMagicExpoTorqueCurrentFOC(0).withUpdateFreqHz(0);
    private final PositionVoltage steerVoltageRequest =
            new PositionVoltage(0).withEnableFOC(true).withUpdateFreqHz(0);
    private final MotionMagicExpoVoltage steerExpoVoltageRequest =
            new MotionMagicExpoVoltage(0).withEnableFOC(true).withUpdateFreqHz(0);
    private final NeutralOut neutral = new NeutralOut();
    private final double metersPerWheelRotation;
    private final SwervePhysicsSim sim;
    private long lastSimNanos;

    /** Each drive motor's boot MotorOutput group, re-applied with only NeutralMode changed. */
    private final MotorOutputConfigs[] driveMotorOutput;

    /** Whether each module's three devices took their config at boot. */
    private final boolean[] configured;

    private final boolean pigeonConfigured;
    private final boolean timesync;
    private long nextDeadlineNanos;

    /**
     * Builds and configures every device. Blocking (config applies): call during robot init only.
     *
     * @param sim physics to step each cycle, or null on the real robot
     */
    public SwerveIOPhoenix(PhoenixSwerveConfig cfg, SwervePhysicsSim sim) {
        this.cfg = cfg;
        this.sim = sim;
        n = cfg.modules();
        CANBus bus = new CANBus(cfg.canBus());
        // CANivore (CAN FD) with Pro: Phoenix timesync lines up every device's frames and applies
        // setpoints at the same instant. SystemCore's own bus does not support it.
        timesync = bus.isNetworkFD() && cfg.allowTimesync();
        driveRequest.UseTimesync = timesync;
        steerRequest.UseTimesync = timesync;
        steerExpoRequest.UseTimesync = timesync;
        steerVoltageRequest.UseTimesync = timesync;
        steerExpoVoltageRequest.UseTimesync = timesync;
        drive = new TalonFX[n];
        steer = new TalonFX[n];
        encoder = new CANcoder[n];
        drivePosition = new StatusSignal<?>[n];
        driveVelocity = new StatusSignal<?>[n];
        steerPosition = new StatusSignal<?>[n];
        steerVelocity = new StatusSignal<?>[n];
        metersPerWheelRotation = 2 * Math.PI * cfg.wheelRadius();
        pigeon = new Pigeon2(cfg.pigeonId(), bus);
        configured = new boolean[n];
        driveMotorOutput = new MotorOutputConfigs[n];
        driveBrakeApplied = new boolean[n];
        java.util.Arrays.fill(driveBrakeApplied, true);
        var pigeonConfig = new Pigeon2Configuration();
        pigeonConfig.MountPose.MountPoseYaw = cfg.pigeonMountPose()[0];
        pigeonConfig.MountPose.MountPosePitch = cfg.pigeonMountPose()[1];
        pigeonConfig.MountPose.MountPoseRoll = cfg.pigeonMountPose()[2];
        pigeonConfigured = apply(() -> pigeon.getConfigurator().apply(pigeonConfig));

        List<BaseStatusSignal> fastList = new ArrayList<>();
        List<BaseStatusSignal> slowList = new ArrayList<>();
        for (int i = 0; i < n; i++) {
            encoder[i] = new CANcoder(cfg.encoderIds()[i], bus);
            drive[i] = new TalonFX(cfg.driveIds()[i], bus);
            steer[i] = new TalonFX(cfg.steerIds()[i], bus);
            final int m = i;
            // Configure all three even if one fails, so each device's state is known.
            boolean encoderOk = apply(() -> encoder[m].getConfigurator().apply(encoderConfig(m)));
            boolean steerOk = apply(() -> steer[m].getConfigurator().apply(steerConfig(m)));
            boolean driveOk = apply(() -> drive[m].getConfigurator().apply(driveConfig(m)));
            configured[i] = encoderOk && steerOk && driveOk;

            drivePosition[i] = drive[i].getPosition();
            driveVelocity[i] = drive[i].getVelocity();
            steerPosition[i] = steer[i].getPosition();
            steerVelocity[i] = steer[i].getVelocity();
            fastList.addAll(
                    List.of(
                            drivePosition[i],
                            driveVelocity[i],
                            steerPosition[i],
                            steerVelocity[i]));
            slowList.addAll(
                    List.of(
                            drive[i].getStatorCurrent(),
                            steer[i].getStatorCurrent(),
                            drive[i].getSupplyCurrent(),
                            steer[i].getSupplyCurrent(),
                            drive[i].getDeviceTemp(),
                            steer[i].getDeviceTemp(),
                            drive[i].getSupplyVoltage(),
                            steer[i].getFault_FusedSensorOutOfSync(),
                            steer[i].getFault_RemoteSensorDataInvalid(),
                            encoder[i].getFault_BadMagnet()));
        }
        yaw = pigeon.getYaw();
        yawRate = pigeon.getAngularVelocityZWorld();
        fastList.add(yaw);
        fastList.add(yawRate);
        fast = fastList.toArray(new BaseStatusSignal[0]);
        slow = slowList.toArray(new BaseStatusSignal[0]);
        for (int t = 0; t < CONFIG_TRIES; t++) {
            if (BaseStatusSignal.setUpdateFrequencyForAll(cfg.signalHz(), fast).isOK()) {
                break;
            }
        }
        // The steer motors' FusedCANcoder feedback reads these frames: keep them on, or
        // optimizeBusUtilization below turns them off and the steer loop runs blind.
        for (int i = 0; i < n; i++) {
            BaseStatusSignal.setUpdateFrequencyForAll(
                    cfg.signalHz(), encoder[i].getPosition(), encoder[i].getVelocity());
        }
        BaseStatusSignal.setUpdateFrequencyForAll(10, slow);
        // Everything else off the bus: only what we read is sent.
        for (int i = 0; i < n; i++) {
            drive[i].optimizeBusUtilization();
            steer[i].optimizeBusUtilization();
            encoder[i].optimizeBusUtilization();
        }
        pigeon.optimizeBusUtilization();
        if (sim != null) {
            sim.attach(cfg, drive, steer, encoder, pigeon);
        }
    }

    private static final int CONFIG_TRIES = 5;

    /** Slow signals per module: 3 currents x2 motors, 2 temps, voltage, 3 faults. */
    private static final int SLOW_PER_MODULE = 10;

    /** Runs a blocking Phoenix call until it returns OK, up to {@link #CONFIG_TRIES} times. */
    private static boolean apply(java.util.function.Supplier<com.ctre.phoenix6.StatusCode> call) {
        for (int t = 0; t < CONFIG_TRIES; t++) {
            if (call.get().isOK()) {
                return true;
            }
        }
        return false;
    }

    private CANcoderConfiguration encoderConfig(int i) {
        var c = new CANcoderConfiguration();
        c.MagnetSensor.MagnetOffset = cfg.magnetOffsets()[i];
        c.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        c.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        return c;
    }

    private TalonFXConfiguration steerConfig(int i) {
        var c = new TalonFXConfiguration();
        c.MotorOutput.Inverted =
                cfg.steerInverted()
                        ? InvertedValue.Clockwise_Positive
                        : InvertedValue.CounterClockwise_Positive;
        c.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        c.Feedback.FeedbackRemoteSensorID = cfg.encoderIds()[i];
        c.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        c.Feedback.RotorToSensorRatio = cfg.steerRatio();
        c.Feedback.SensorToMechanismRatio = 1.0;
        c.ClosedLoopGeneral.ContinuousWrap = true;
        // Used only when steering with MotionMagicExpo; cruise velocity 0 means "as fast as kV
        // allows".
        c.MotionMagic.MotionMagicCruiseVelocity = 0;
        c.MotionMagic.MotionMagicExpo_kV = cfg.steerExpoKv();
        c.MotionMagic.MotionMagicExpo_kA = cfg.steerExpoKa();
        c.CurrentLimits.StatorCurrentLimit = cfg.steerStatorCurrentLimit();
        c.CurrentLimits.StatorCurrentLimitEnable = true;
        c.TorqueCurrent.PeakForwardTorqueCurrent = cfg.steerStatorCurrentLimit();
        c.TorqueCurrent.PeakReverseTorqueCurrent = -cfg.steerStatorCurrentLimit();
        c.Slot0 = slot0(cfg.steer());
        return c;
    }

    private TalonFXConfiguration driveConfig(int i) {
        var c = new TalonFXConfiguration();
        driveMotorOutput[i] = c.MotorOutput;
        c.MotorOutput.Inverted =
                cfg.driveInverted()[i]
                        ? InvertedValue.Clockwise_Positive
                        : InvertedValue.CounterClockwise_Positive;
        c.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        // Position and velocity in wheel rotations.
        c.Feedback.SensorToMechanismRatio = cfg.driveRatio();
        c.CurrentLimits.StatorCurrentLimit = cfg.driveStatorCurrentLimit();
        c.CurrentLimits.StatorCurrentLimitEnable = true;
        c.TorqueCurrent.PeakForwardTorqueCurrent = cfg.driveStatorCurrentLimit();
        c.TorqueCurrent.PeakReverseTorqueCurrent = -cfg.driveStatorCurrentLimit();
        c.Slot0 = slot0(cfg.drive());
        return c;
    }

    private static Slot0Configs slot0(PhoenixSwerveConfig.Gains g) {
        var s = new Slot0Configs();
        s.kP = g.kP();
        s.kI = g.kI();
        s.kD = g.kD();
        s.kS = g.kS();
        s.kV = g.kV();
        s.kA = g.kA();
        return s;
    }

    @Override
    public void waitForSample(SwerveSample out) {
        boolean good;
        if (sim != null) {
            // Desktop: step the physics one period, then read what Phoenix's simulator reports.
            long now = System.nanoTime();
            double dt = lastSimNanos == 0 ? 1.0 / cfg.signalHz() : (now - lastSimNanos) * 1e-9;
            lastSimNanos = now;
            sim.step(Math.min(dt, 0.02));
            good = BaseStatusSignal.refreshAll(fast).isOK();
            sleepUntilNextPeriod(now);
        } else {
            good = BaseStatusSignal.waitForAll(2.0 / cfg.signalHz(), fast).isOK();
        }
        out.allGood = good;
        // Capture time: the mean over fresh signals of their own timestamps (system receive time on
        // a
        // plain bus, the synchronized CANivore time with timesync), not when we got around to
        // reading them. Both are on Phoenix's clock (Utils.getCurrentTimeSeconds).
        double sum = 0;
        int count = 0;
        for (BaseStatusSignal sig : fast) {
            if (sig.getStatus().isOK()) {
                var all = sig.getAllTimestamps();
                sum += (timesync ? all.getCANivoreTimestamp() : all.getSystemTimestamp()).getTime();
                count++;
            }
        }
        out.timestamp = count > 0 ? sum / count : Utils.getCurrentTimeSeconds();
        out.gyroOk = yaw.getStatus().isOK() && yawRate.getStatus().isOK();
        double couple = cfg.coupleRatio() / cfg.driveRatio();
        for (int i = 0; i < n; i++) {
            double steerRot =
                    BaseStatusSignal.getLatencyCompensatedValueAsDouble(
                            steerPosition[i], steerVelocity[i]);
            double wheelRot =
                    BaseStatusSignal.getLatencyCompensatedValueAsDouble(
                            drivePosition[i], driveVelocity[i]);
            // MK4i: turning the module turns the drive gearing; remove that from wheel travel.
            out.distance[i] = (wheelRot - steerRot * couple) * metersPerWheelRotation;
            out.velocity[i] =
                    (driveVelocity[i].getValueAsDouble()
                                    - steerVelocity[i].getValueAsDouble() * couple)
                            * metersPerWheelRotation;
            out.angle[i] = SwerveMath.angleModulus(steerRot * 2 * Math.PI);
            out.moduleOk[i] =
                    drivePosition[i].getStatus().isOK()
                            && driveVelocity[i].getStatus().isOK()
                            && steerPosition[i].getStatus().isOK()
                            && steerVelocity[i].getStatus().isOK();
        }
        out.yaw = Math.toRadians(BaseStatusSignal.getLatencyCompensatedValueAsDouble(yaw, yawRate));
        out.angularVelocityZWorld = Math.toRadians(yawRate.getValueAsDouble());
    }

    /** Keeps the sim loop on a fixed 1/signalHz schedule; parkNanos can wake early on Windows. */
    private void sleepUntilNextPeriod(long startNanos) {
        long periodNanos = (long) (1e9 / cfg.signalHz());
        if (nextDeadlineNanos == 0 || startNanos - nextDeadlineNanos > periodNanos) {
            nextDeadlineNanos = startNanos; // first cycle, or fell far behind: restart the schedule
        }
        nextDeadlineNanos += periodNanos;
        long remaining;
        while ((remaining = nextDeadlineNanos - System.nanoTime()) > 0) {
            java.util.concurrent.locks.LockSupport.parkNanos(remaining);
        }
    }

    @Override
    public void setModules(
            double[] angle, double[] velocity, double[] steerVelocityPlan, double[] wheelForce) {
        double couple = cfg.coupleRatio() / cfg.driveRatio();
        // Wheel force (N) to drive torque current (A): F * r / (ratio * Kt).
        double ampsPerNewton = cfg.wheelRadius() / (cfg.driveRatio() * cfg.driveKt());
        for (int i = 0; i < n; i++) {
            double target = angle[i] / (2 * Math.PI);
            double steerRps = steerVelocityPlan[i] / (2 * Math.PI);
            // The planned turn rate keeps the steer D term from fighting the motion.
            if (cfg.steerVoltage()) {
                steer[i].setControl(
                        cfg.steerMotionMagicExpo()
                                ? steerExpoVoltageRequest.withPosition(target)
                                : steerVoltageRequest.withPosition(target).withVelocity(steerRps));
            } else {
                steer[i].setControl(
                        cfg.steerMotionMagicExpo()
                                ? steerExpoRequest.withPosition(target)
                                : steerRequest.withPosition(target).withVelocity(steerRps));
            }
            double wheelRps = velocity[i] / metersPerWheelRotation;
            // Coupling feedforward from the module's measured turn rate: a commanded rate would
            // spike whenever the target angle jumps.
            double measuredSteerRps = steerVelocity[i].getValueAsDouble();
            drive[i].setControl(
                    driveRequest
                            .withVelocity(wheelRps + measuredSteerRps * couple)
                            .withFeedForward(wheelForce[i] * ampsPerNewton));
        }
    }

    /** Each drive motor's neutral mode as last applied (true brake). */
    private final boolean[] driveBrakeApplied;

    /** Bumped on every request; a queued apply from an older request does nothing. */
    private final java.util.concurrent.atomic.AtomicInteger brakeGeneration =
            new java.util.concurrent.atomic.AtomicInteger();

    @Override
    public void setDriveBrake(boolean brake) {
        int generation = brakeGeneration.incrementAndGet();
        // A config apply per motor: off the robot loop, on the shared Phoenix background thread.
        for (int i = 0; i < n; i++) {
            final int m = i;
            PhoenixBackground.run(
                    "Swerve/DriveBrake/" + i,
                    () -> {
                        // A newer request (e.g. brake at enable) supersedes this one: skip it
                        // rather than make the newer one wait behind a retrying apply.
                        if (brakeGeneration.get() != generation) {
                            return com.ctre.phoenix6.StatusCode.OK;
                        }
                        // The boot MotorOutput group with only NeutralMode changed, so the
                        // inversion (a hard stop) is never rebuilt here.
                        var out = driveMotorOutput[m].clone();
                        out.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
                        var code = drive[m].getConfigurator().apply(out);
                        if (code.isOK()) {
                            driveBrakeApplied[m] = brake;
                        }
                        return code;
                    },
                    null);
        }
    }

    @Override
    public void setNeutral() {
        for (int i = 0; i < n; i++) {
            drive[i].setControl(neutral);
            steer[i].setControl(neutral);
        }
    }

    @Override
    public void updateHealth(SwerveHealth h) {
        BaseStatusSignal.refreshAll(slow);
        for (int i = 0; i < n; i++) {
            int b = i * SLOW_PER_MODULE;
            h.driveConnected[i] = drive[i].isConnected();
            h.steerConnected[i] = steer[i].isConnected();
            h.encoderConnected[i] = encoder[i].isConnected();
            h.driveStatorCurrent[i] = slow[b].getValueAsDouble();
            h.steerStatorCurrent[i] = slow[b + 1].getValueAsDouble();
            h.driveSupplyCurrent[i] = slow[b + 2].getValueAsDouble();
            h.steerSupplyCurrent[i] = slow[b + 3].getValueAsDouble();
            h.driveDeviceTemp[i] = slow[b + 4].getValueAsDouble();
            h.steerDeviceTemp[i] = slow[b + 5].getValueAsDouble();
            // Fault signals are Boolean; getValueAsDouble is 1 when the fault is active.
            h.fusedOk[i] =
                    slow[b + 7].getValueAsDouble() == 0 && slow[b + 8].getValueAsDouble() == 0;
            h.magnetOk[i] = slow[b + 9].getValueAsDouble() == 0;
        }
        h.supplyVoltage = slow[6].getValueAsDouble();
        h.pigeonConnected = pigeon.isConnected() && pigeonConfigured;
        System.arraycopy(configured, 0, h.configured, 0, n);
        h.timesync = timesync;
        // Brake only if every drive motor is in brake.
        boolean allBrake = true;
        for (boolean b : driveBrakeApplied) {
            allBrake &= b;
        }
        h.driveBrake = allBrake;
    }
}
