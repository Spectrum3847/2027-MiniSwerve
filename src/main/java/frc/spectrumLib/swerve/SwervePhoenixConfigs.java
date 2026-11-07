package frc.spectrumLib.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

/**
 * The Phoenix device configs {@link SwerveIOPhoenix} applies, built from {@link
 * PhoenixSwerveConfig}.
 */
final class SwervePhoenixConfigs {
    private SwervePhoenixConfigs() {}

    static CANcoderConfiguration encoder(PhoenixSwerveConfig cfg, int i) {
        var c = new CANcoderConfiguration();
        c.MagnetSensor.MagnetOffset = cfg.magnetOffsets()[i];
        c.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        c.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        return c;
    }

    static TalonFXConfiguration steer(PhoenixSwerveConfig cfg, int i) {
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
        if (cfg.steerSupplyCurrentLimit() > 0) {
            c.CurrentLimits.SupplyCurrentLimit = cfg.steerSupplyCurrentLimit();
            c.CurrentLimits.SupplyCurrentLimitEnable = true;
        }
        c.TorqueCurrent.PeakForwardTorqueCurrent = cfg.steerStatorCurrentLimit();
        c.TorqueCurrent.PeakReverseTorqueCurrent = -cfg.steerStatorCurrentLimit();
        c.Slot0 = slot0(cfg.steer());
        return c;
    }

    static TalonFXConfiguration drive(PhoenixSwerveConfig cfg, int i, boolean brake) {
        var c = new TalonFXConfiguration();
        c.MotorOutput.Inverted =
                cfg.driveInverted()[i]
                        ? InvertedValue.Clockwise_Positive
                        : InvertedValue.CounterClockwise_Positive;
        c.MotorOutput.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        // Position and velocity in wheel rotations.
        c.Feedback.SensorToMechanismRatio = cfg.driveRatio();
        c.CurrentLimits.StatorCurrentLimit = cfg.driveStatorCurrentLimit();
        c.CurrentLimits.StatorCurrentLimitEnable = true;
        if (cfg.driveSupplyCurrentLimit() > 0) {
            c.CurrentLimits.SupplyCurrentLimit = cfg.driveSupplyCurrentLimit();
            c.CurrentLimits.SupplyCurrentLimitEnable = true;
        }
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

    /**
     * Sets every signal's update rate and turns the rest off. Not saved on the devices: a device
     * that reboots (a connector wiggle, a brown-out) comes back with default rates, so this runs
     * again whenever one resets. Blocking: boot, or the Phoenix background thread.
     */
    static com.ctre.phoenix6.StatusCode signals(
            double hz,
            com.ctre.phoenix6.BaseStatusSignal[] fast,
            com.ctre.phoenix6.BaseStatusSignal[] slow,
            com.ctre.phoenix6.hardware.TalonFX[] drive,
            com.ctre.phoenix6.hardware.TalonFX[] steer,
            com.ctre.phoenix6.hardware.CANcoder[] encoder,
            com.ctre.phoenix6.hardware.Pigeon2 pigeon) {
        int n = drive.length;
        var code = com.ctre.phoenix6.BaseStatusSignal.setUpdateFrequencyForAll(hz, fast);
        // The steer motors' FusedCANcoder feedback reads these frames: keep them on, or
        // optimizeBusUtilization below turns them off and the steer loop runs blind.
        for (int i = 0; i < n; i++) {
            var c =
                    com.ctre.phoenix6.BaseStatusSignal.setUpdateFrequencyForAll(
                            hz, encoder[i].getPosition(), encoder[i].getVelocity());
            code = code.isOK() ? c : code;
        }
        var c = com.ctre.phoenix6.BaseStatusSignal.setUpdateFrequencyForAll(10, slow);
        code = code.isOK() ? c : code;
        // Everything else off the bus: only what we read is sent.
        for (int i = 0; i < n; i++) {
            drive[i].optimizeBusUtilization();
            steer[i].optimizeBusUtilization();
            encoder[i].optimizeBusUtilization();
        }
        pigeon.optimizeBusUtilization();
        return code;
    }
}
