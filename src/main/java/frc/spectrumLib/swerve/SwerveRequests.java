package frc.spectrumLib.swerve;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import frc.spectrumLib.hardware.SimFaults;

/**
 * Module targets to Phoenix control requests, shared by {@link SwerveIOPhoenix} (real TalonFXs) and
 * SpectrumSim's {@code SwerveIOSim} (TalonFX models), so the sim runs on exactly the requests the
 * robot sends. Reuses one request object per kind: no allocation per cycle.
 */
public final class SwerveRequests {
    /** Where the requests go. */
    public interface Motors {
        void setDrive(int i, ControlRequest r);

        void setSteer(int i, ControlRequest r);
    }

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
    private final NeutralOut neutral = new NeutralOut().withUpdateFreqHz(0);
    // Once per call too: Phoenix would otherwise resend it at 100 Hz to a dead motor.
    private final CoastOut coast = new CoastOut().withUpdateFreqHz(0);

    private final PhoenixSwerveConfig cfg;
    private final SimFaults faults;
    private final String[] driveName;
    private final String[] steerName;
    private final double metersPerWheelRotation;

    /** Cycles between commands to a disconnected motor: 10 Hz instead of the loop rate. */
    private final int slowDivisor;

    private long cycle;

    /**
     * @param faults simulated faults (an unpowered motor ignores commands); null on the robot
     */
    public SwerveRequests(PhoenixSwerveConfig cfg, SimFaults faults) {
        this.cfg = cfg;
        this.faults = faults;
        int n = cfg.modules();
        driveName = new String[n];
        steerName = new String[n];
        for (int i = 0; i < n; i++) {
            driveName[i] = "Drive" + i;
            steerName[i] = "Steer" + i;
        }
        metersPerWheelRotation = 2 * Math.PI * cfg.wheelRadius();
        slowDivisor = Math.max(1, (int) Math.round(cfg.signalHz() / 10.0));
    }

    /** Phoenix timesync on every setpoint (CANivore only). */
    public void setTimesync(boolean timesync) {
        driveRequest.UseTimesync = timesync;
        steerRequest.UseTimesync = timesync;
        steerExpoRequest.UseTimesync = timesync;
        steerVoltageRequest.UseTimesync = timesync;
        steerExpoVoltageRequest.UseTimesync = timesync;
    }

    /**
     * Sends one cycle's module targets.
     *
     * @param measuredSteerRps each module's measured turn rate (rps), for the coupling feedforward
     */
    public void setModules(
            double[] angle,
            double[] velocity,
            double[] steerVelocityPlan,
            double[] wheelForce,
            ModuleCommands commands,
            double[] measuredSteerRps,
            Motors m) {
        double couple = cfg.coupleRatio() / cfg.driveRatio();
        // Wheel force (N) to drive torque current (A): F * r / (ratio * Kt).
        double ampsPerNewton = cfg.wheelRadius() / (cfg.driveRatio() * cfg.driveKt());
        // A disconnected motor still gets its command, but only every slowDivisor cycles: it may
        // be alive and listening, and the rest of the bus should not pay for it every cycle.
        boolean slowTurn = cycle++ % slowDivisor == 0;
        for (int i = 0; i < angle.length; i++) {
            if ((!commands.steerSlow()[i] || slowTurn) && !unpowered(steerName[i])) {
                if (commands.steer()[i]) {
                    double target = angle[i] / (2 * Math.PI);
                    double steerRps = steerVelocityPlan[i] / (2 * Math.PI);
                    // The planned turn rate keeps the steer D term from fighting the motion.
                    if (cfg.steerVoltage()) {
                        m.setSteer(
                                i,
                                cfg.steerMotionMagicExpo()
                                        ? steerExpoVoltageRequest.withPosition(target)
                                        : steerVoltageRequest
                                                .withPosition(target)
                                                .withVelocity(steerRps));
                    } else {
                        m.setSteer(
                                i,
                                cfg.steerMotionMagicExpo()
                                        ? steerExpoRequest.withPosition(target)
                                        : steerRequest.withPosition(target).withVelocity(steerRps));
                    }
                } else {
                    m.setSteer(i, coast);
                }
            }
            if ((!commands.driveSlow()[i] || slowTurn) && !unpowered(driveName[i])) {
                if (commands.drive()[i]) {
                    double wheelRps = velocity[i] / metersPerWheelRotation;
                    // Coupling feedforward from the module's measured turn rate: a commanded rate
                    // would spike whenever the target angle jumps.
                    m.setDrive(
                            i,
                            driveRequest
                                    .withVelocity(wheelRps + measuredSteerRps[i] * couple)
                                    .withFeedForward(wheelForce[i] * ampsPerNewton));
                } else {
                    m.setDrive(i, coast);
                }
            }
        }
    }

    /** Every motor to its neutral mode. */
    public void setNeutral(Motors m) {
        for (int i = 0; i < cfg.modules(); i++) {
            m.setDrive(i, neutral);
            m.setSteer(i, neutral);
        }
    }

    /** Sim: a device without power ignores commands. */
    private boolean unpowered(String device) {
        return faults != null && faults.unpowered(device);
    }
}
