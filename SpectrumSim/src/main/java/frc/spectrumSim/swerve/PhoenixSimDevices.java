package frc.spectrumSim.swerve;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import frc.spectrumLib.swerve.PhoenixSwerveConfig;
import frc.spectrumSim.MotorModel;

/**
 * {@link SwerveSimDevices} on Phoenix's own simulator: the motors' closed loops run in CTRE's
 * simulated firmware, on the wall clock. Kept for checking our TalonFX model against CTRE's and for
 * tests of the exact Phoenix path; runs do not repeat exactly.
 *
 * <p>Phoenix sim reports torque current in the motor's own (inverted) frame, which is the
 * mechanism's direction, but takes rotor position and velocity in the chassis frame, before the
 * invert, and reports motor voltage in the chassis frame too. An inverted motor is mounted
 * reversed, so its rotor turns opposite to the mechanism.
 */
final class PhoenixSimDevices implements SwerveSimDevices {
    private final PhoenixSwerveConfig cfg;
    private final MotorModel steerMotor;
    private final TalonFXSimState[] driveSim;
    private final TalonFXSimState[] steerSim;
    private final CANcoderSimState[] encoderSim;
    private final Pigeon2SimState pigeonSim;

    PhoenixSimDevices(
            PhoenixSwerveConfig cfg,
            MotorModel steerMotor,
            TalonFX[] drive,
            TalonFX[] steer,
            CANcoder[] encoder,
            Pigeon2 pigeon) {
        this.cfg = cfg;
        this.steerMotor = steerMotor;
        int n = drive.length;
        driveSim = new TalonFXSimState[n];
        steerSim = new TalonFXSimState[n];
        encoderSim = new CANcoderSimState[n];
        for (int i = 0; i < n; i++) {
            driveSim[i] = drive[i].getSimState();
            steerSim[i] = steer[i].getSimState();
            encoderSim[i] = encoder[i].getSimState();
        }
        pigeonSim = pigeon.getSimState();
    }

    @Override
    public double driveAmps(int i, double driveMotorOmega, double batteryVolts, double dt) {
        driveSim[i].setSupplyVoltage(batteryVolts);
        return driveSim[i].getTorqueCurrent();
    }

    @Override
    public double steerAmps(int i, double steerMotorOmega, double batteryVolts, double dt) {
        steerSim[i].setSupplyVoltage(batteryVolts);
        double steerSign = cfg.steerInverted() ? -1.0 : 1.0;
        return cfg.steerVoltage()
                ? (steerSign * steerSim[i].getMotorVoltage() - steerMotor.kE() * steerMotorOmega)
                        / steerMotor.resistance()
                : steerSim[i].getTorqueCurrent();
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
        double steerRatio = cfg.steerRatio();
        double steerSign = cfg.steerInverted() ? -1.0 : 1.0;
        double driveSign = cfg.driveInverted()[i] ? -1.0 : 1.0;
        steerSim[i].setRawRotorPosition(steerSign * steerRot * steerRatio);
        steerSim[i].setRotorVelocity(steerSign * steerOmega * steerRatio / (2 * Math.PI));
        driveSim[i].setRawRotorPosition(driveSign * driveRotorRot);
        driveSim[i].setRotorVelocity(driveSign * driveMotorOmega / (2 * Math.PI));
        if (encoderAlive) {
            encoderSim[i].setSupplyVoltage(batteryVolts);
            encoderSim[i].setRawPosition(steerRot - cfg.magnetOffsets()[i]);
            encoderSim[i].setVelocity(steerOmega / (2 * Math.PI));
        }
    }

    @Override
    public void senseGyro(double heading, double omega, double batteryVolts) {
        pigeonSim.setSupplyVoltage(batteryVolts);
        pigeonSim.setRawYaw(Math.toDegrees(heading));
        pigeonSim.setAngularVelocityZ(Math.toDegrees(omega));
    }
}
