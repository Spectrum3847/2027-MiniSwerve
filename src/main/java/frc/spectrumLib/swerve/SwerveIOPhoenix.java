package frc.spectrumLib.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
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

    private final VelocityTorqueCurrentFOC driveRequest = new VelocityTorqueCurrentFOC(0);
    private final PositionTorqueCurrentFOC steerRequest = new PositionTorqueCurrentFOC(0);
    private final NeutralOut neutral = new NeutralOut();
    private final double metersPerWheelRotation;
    private final SwervePhysicsSim sim;
    private long lastSimNanos;
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
        drive = new TalonFX[n];
        steer = new TalonFX[n];
        encoder = new CANcoder[n];
        drivePosition = new StatusSignal<?>[n];
        driveVelocity = new StatusSignal<?>[n];
        steerPosition = new StatusSignal<?>[n];
        steerVelocity = new StatusSignal<?>[n];
        metersPerWheelRotation = 2 * Math.PI * cfg.wheelRadius();
        pigeon = new Pigeon2(cfg.pigeonId(), bus);
        pigeon.getConfigurator().apply(new Pigeon2Configuration());

        List<BaseStatusSignal> fastList = new ArrayList<>();
        List<BaseStatusSignal> slowList = new ArrayList<>();
        for (int i = 0; i < n; i++) {
            encoder[i] = new CANcoder(cfg.encoderIds()[i], bus);
            drive[i] = new TalonFX(cfg.driveIds()[i], bus);
            steer[i] = new TalonFX(cfg.steerIds()[i], bus);
            encoder[i].getConfigurator().apply(encoderConfig(i));
            steer[i].getConfigurator().apply(steerConfig(i));
            drive[i].getConfigurator().apply(driveConfig(i));

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
                            drive[i].getSupplyVoltage()));
        }
        yaw = pigeon.getYaw();
        yawRate = pigeon.getAngularVelocityZWorld();
        fastList.add(yaw);
        fastList.add(yawRate);
        fast = fastList.toArray(new BaseStatusSignal[0]);
        slow = slowList.toArray(new BaseStatusSignal[0]);
        BaseStatusSignal.setUpdateFrequencyForAll(cfg.signalHz(), fast);
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
        c.CurrentLimits.StatorCurrentLimit = cfg.steerStatorCurrentLimit();
        c.CurrentLimits.StatorCurrentLimitEnable = true;
        c.TorqueCurrent.PeakForwardTorqueCurrent = cfg.steerStatorCurrentLimit();
        c.TorqueCurrent.PeakReverseTorqueCurrent = -cfg.steerStatorCurrentLimit();
        c.Slot0 = slot0(cfg.steer());
        return c;
    }

    private TalonFXConfiguration driveConfig(int i) {
        var c = new TalonFXConfiguration();
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
        out.timestamp = Utils.getCurrentTimeSeconds();
        out.allGood = good;
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
    public void setModules(double[] angle, double[] velocity, double[] steerVelocity) {
        double couple = cfg.coupleRatio() / cfg.driveRatio();
        for (int i = 0; i < n; i++) {
            steer[i].setControl(steerRequest.withPosition(angle[i] / (2 * Math.PI)));
            double wheelRps = velocity[i] / metersPerWheelRotation;
            double steerRps = steerVelocity[i] / (2 * Math.PI);
            drive[i].setControl(driveRequest.withVelocity(wheelRps + steerRps * couple));
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
            int b = i * 7;
            h.driveConnected[i] = drive[i].isConnected();
            h.steerConnected[i] = steer[i].isConnected();
            h.encoderConnected[i] = encoder[i].isConnected();
            h.driveStatorCurrent[i] = slow[b].getValueAsDouble();
            h.steerStatorCurrent[i] = slow[b + 1].getValueAsDouble();
            h.driveSupplyCurrent[i] = slow[b + 2].getValueAsDouble();
            h.steerSupplyCurrent[i] = slow[b + 3].getValueAsDouble();
            h.driveDeviceTemp[i] = slow[b + 4].getValueAsDouble();
            h.steerDeviceTemp[i] = slow[b + 5].getValueAsDouble();
        }
        h.supplyVoltage = slow[6].getValueAsDouble();
        h.pigeonConnected = pigeon.isConnected();
    }
}
