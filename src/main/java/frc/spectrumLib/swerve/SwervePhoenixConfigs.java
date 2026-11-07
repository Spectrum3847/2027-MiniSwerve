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
}
