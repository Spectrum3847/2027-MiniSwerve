package frc.spectrumLib.sim;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

/**
 * A TalonFX's own control loop, for deterministic sims: it takes the same {@link
 * TalonFXConfiguration} and {@link ControlRequest} objects the robot sends a real TalonFX, runs the
 * closed loop each physics step (1 kHz, the firmware's rate) on the sim clock, and returns the
 * stator current its motor makes. Phoenix's own simulator runs that loop on the wall clock, so two
 * runs never match; this one does.
 *
 * <p>Everything is in the mechanism's frame (after the motor's invert): the physics passes rotor
 * motion in the mechanism's direction and gets current in that direction. Positions are mechanism
 * rotations: rotor / SensorToMechanismRatio, and also / RotorToSensorRatio for a remote or fused
 * sensor.
 *
 * <p>Supported requests: NeutralOut, CoastOut, StaticBrake, VoltageOut, DutyCycleOut,
 * TorqueCurrentFOC, VelocityVoltage, VelocityTorqueCurrentFOC, PositionVoltage,
 * PositionTorqueCurrentFOC, MotionMagicExpoVoltage, MotionMagicExpoTorqueCurrentFOC. Any other
 * coasts the motor and warns once: an unsupported request must be added, not silently ignored. Slot
 * 0 only.
 *
 * <p>Not one thread-safe object: one thread sets requests and steps it (the sim clock runs them in
 * turn).
 *
 * <p>Index: {@link #configure} | {@link #setControl} | {@link #update} | closed loop | getters
 */
public final class TalonFXModel {
    private enum Kind {
        NEUTRAL,
        COAST,
        BRAKE,
        VOLTAGE,
        DUTY_CYCLE,
        TORQUE,
        VELOCITY,
        POSITION,
        EXPO
    }

    private final String name;
    private final MotorModel motor;
    private final MotionMagicExpoProfile profile = new MotionMagicExpoProfile();

    // Config, copied from TalonFXConfiguration (Phoenix's names).
    private boolean brakeNeutral;
    private double rotorPerMechanism = 1;
    private boolean continuousWrap;
    private double kP, kI, kD, kS, kV, kA;
    private double expoKv, expoKa, cruiseVelocity;
    private double peakForwardTorque = 800, peakReverseTorque = -800;
    private double peakForwardVoltage = 16, peakReverseVoltage = -16;
    private double statorLimit = Double.POSITIVE_INFINITY;

    // The request, copied out of the ControlRequest when it is set, as the firmware does.
    private Kind kind = Kind.NEUTRAL;
    private boolean torqueOutput;
    private double output; // V, duty cycle or A for open loop
    private double refPosition, refVelocity, refAcceleration, feedForward;
    private boolean expoStarted;
    private String unsupported;

    // State.
    private boolean enabled;
    private boolean wasActive;
    private double position, velocity; // mechanism rotations, rps
    private double integral, lastVelocityError;
    private double statorAmps, motorVolts, closedLoopError;

    /**
     * @param name for warnings, e.g. "Drive0"
     * @param motor the motor this TalonFX drives (Kraken X60, Falcon 500...)
     */
    public TalonFXModel(String name, MotorModel motor) {
        this.name = name;
        this.motor = motor;
    }

    /** Takes the parts of a config the control loop uses. Same object the robot applies. */
    public void configure(TalonFXConfiguration c) {
        brakeNeutral = c.MotorOutput.NeutralMode == NeutralModeValue.Brake;
        boolean rotorSensor =
                c.Feedback.FeedbackSensorSource == FeedbackSensorSourceValue.RotorSensor;
        rotorPerMechanism =
                c.Feedback.SensorToMechanismRatio
                        * (rotorSensor ? 1 : c.Feedback.RotorToSensorRatio);
        continuousWrap = c.ClosedLoopGeneral.ContinuousWrap;
        kP = c.Slot0.kP;
        kI = c.Slot0.kI;
        kD = c.Slot0.kD;
        kS = c.Slot0.kS;
        kV = c.Slot0.kV;
        kA = c.Slot0.kA;
        expoKv = c.MotionMagic.MotionMagicExpo_kV;
        expoKa = c.MotionMagic.MotionMagicExpo_kA;
        cruiseVelocity = c.MotionMagic.MotionMagicCruiseVelocity;
        peakForwardTorque = c.TorqueCurrent.PeakForwardTorqueCurrent;
        peakReverseTorque = c.TorqueCurrent.PeakReverseTorqueCurrent;
        peakForwardVoltage = c.Voltage.PeakForwardVoltage;
        peakReverseVoltage = c.Voltage.PeakReverseVoltage;
        statorLimit =
                c.CurrentLimits.StatorCurrentLimitEnable
                        ? c.CurrentLimits.StatorCurrentLimit
                        : Double.POSITIVE_INFINITY;
    }

    /** Neutral mode only (a brake/coast change applied on its own). */
    public void setNeutralMode(NeutralModeValue mode) {
        brakeNeutral = mode == NeutralModeValue.Brake;
    }

    /** As {@code TalonFX.setControl}: the request's values are copied now. */
    public void setControl(ControlRequest r) {
        Kind before = kind;
        if (r instanceof NeutralOut) {
            kind = Kind.NEUTRAL;
        } else if (r instanceof CoastOut) {
            kind = Kind.COAST;
        } else if (r instanceof StaticBrake) {
            kind = Kind.BRAKE;
        } else if (r instanceof VoltageOut q) {
            open(Kind.VOLTAGE, q.Output);
        } else if (r instanceof DutyCycleOut q) {
            open(Kind.DUTY_CYCLE, q.Output);
        } else if (r instanceof TorqueCurrentFOC q) {
            open(Kind.TORQUE, q.Output);
        } else if (r instanceof VelocityVoltage q) {
            closed(Kind.VELOCITY, false, 0, q.Velocity, q.Acceleration, q.FeedForward);
        } else if (r instanceof VelocityTorqueCurrentFOC q) {
            closed(Kind.VELOCITY, true, 0, q.Velocity, q.Acceleration, q.FeedForward);
        } else if (r instanceof PositionVoltage q) {
            closed(Kind.POSITION, false, q.Position, q.Velocity, 0, q.FeedForward);
        } else if (r instanceof PositionTorqueCurrentFOC q) {
            closed(Kind.POSITION, true, q.Position, q.Velocity, 0, q.FeedForward);
        } else if (r instanceof MotionMagicExpoVoltage q) {
            closed(Kind.EXPO, false, q.Position, 0, 0, q.FeedForward);
        } else if (r instanceof MotionMagicExpoTorqueCurrentFOC q) {
            closed(Kind.EXPO, true, q.Position, 0, 0, q.FeedForward);
        } else {
            kind = Kind.COAST;
            if (unsupported == null) {
                unsupported = r.getName();
                org.wpilib.driverstation.DriverStationErrors.reportWarning(
                        "TalonFXModel " + name + ": " + unsupported + " not modeled; coasting",
                        false);
            }
        }
        if (kind != before) {
            integral = 0;
            lastVelocityError = 0;
            expoStarted = expoStarted && kind == Kind.EXPO;
        }
    }

    private void open(Kind k, double out) {
        kind = k;
        output = out;
    }

    private void closed(Kind k, boolean torque, double pos, double vel, double acc, double ff) {
        kind = k;
        torqueOutput = torque;
        refPosition = pos;
        refVelocity = vel;
        refAcceleration = acc;
        feedForward = ff;
    }

    /** Robot enabled (the TalonFX's enable signal); disabled means neutral output. */
    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    /** A reboot: the request is forgotten until the robot sends the next one. */
    public void reboot() {
        kind = Kind.NEUTRAL;
        integral = 0;
        expoStarted = false;
    }

    /**
     * One control step.
     *
     * @param rotorRotations rotor position (rotations, mechanism direction)
     * @param rotorRps rotor velocity (rps, mechanism direction)
     * @param batteryVolts supply voltage now (V)
     * @param dt step (s)
     * @return stator current (A, mechanism direction), before the physics' supply and battery
     *     limits
     */
    public double update(double rotorRotations, double rotorRps, double batteryVolts, double dt) {
        position = rotorRotations / rotorPerMechanism;
        velocity = rotorRps / rotorPerMechanism;
        double rotorOmega = rotorRps * 2 * Math.PI;
        Kind k = enabled ? kind : Kind.NEUTRAL;
        if (k == Kind.NEUTRAL) {
            k = brakeNeutral ? Kind.BRAKE : Kind.COAST;
        }
        boolean active = k != Kind.BRAKE && k != Kind.COAST;
        if (active && !wasActive) {
            integral = 0;
            lastVelocityError = 0;
            expoStarted = false;
        }
        wasActive = active;
        closedLoopError = 0;
        double amps;
        switch (k) {
            case COAST -> {
                amps = 0;
                motorVolts = 0;
            }
            case BRAKE -> amps = fromVolts(0, rotorOmega);
            case VOLTAGE -> amps = fromVolts(output, rotorOmega);
            case DUTY_CYCLE -> amps = fromVolts(output * batteryVolts, rotorOmega);
            case TORQUE -> amps = fromTorque(output);
            default -> {
                double out = closedLoop(k, dt);
                amps = torqueOutput ? fromTorque(out) : fromVolts(out, rotorOmega);
            }
        }
        statorAmps = amps;
        return amps;
    }

    /** Slot 0 closed loop for the request; output in volts or amps. */
    private double closedLoop(Kind k, double dt) {
        double refPos = refPosition, refVel = refVelocity, refAcc = refAcceleration;
        if (k == Kind.EXPO) {
            if (!expoStarted) {
                profile.reset(position, velocity);
                expoStarted = true;
            }
            profile.step(refPosition, expoKv, expoKa, cruiseVelocity, continuousWrap, dt);
            refPos = profile.position();
            refVel = profile.velocity();
            refAcc = profile.acceleration();
        }
        double p, d, errorForI;
        if (k == Kind.VELOCITY) {
            double e = refVel - velocity;
            d = kD * (e - lastVelocityError) / dt;
            lastVelocityError = e;
            p = kP * e;
            errorForI = e;
            closedLoopError = e;
        } else {
            double e = refPos - position;
            if (continuousWrap) {
                e = Math.IEEEremainder(e, 1.0);
            }
            p = kP * e;
            d = kD * (refVel - velocity);
            errorForI = e;
            closedLoopError = e;
        }
        double ff = kV * refVel + kA * refAcc + kS * Math.signum(refVel) + feedForward;
        double next = integral + errorForI * dt;
        double out = p + kI * next + d + ff;
        double lo = torqueOutput ? peakReverseTorque : peakReverseVoltage;
        double hi = torqueOutput ? peakForwardTorque : peakForwardVoltage;
        if (out > hi || out < lo) {
            out = Math.max(lo, Math.min(hi, out)); // saturated: hold the integral
        } else {
            integral = next;
        }
        return out;
    }

    private double fromTorque(double amps) {
        double a = Math.max(peakReverseTorque, Math.min(peakForwardTorque, amps));
        a = Math.max(-statorLimit, Math.min(statorLimit, a));
        motorVolts = a * motor.resistance(); // plus back-EMF, filled by the caller's physics
        return a;
    }

    private double fromVolts(double volts, double rotorOmega) {
        double v = Math.max(peakReverseVoltage, Math.min(peakForwardVoltage, volts));
        motorVolts = v;
        double a = (v - motor.kE() * rotorOmega) / motor.resistance();
        return Math.max(-statorLimit, Math.min(statorLimit, a));
    }

    /** Mechanism position from the last step (rotations). */
    public double position() {
        return position;
    }

    /** Mechanism velocity from the last step (rps). */
    public double velocity() {
        return velocity;
    }

    /** Stator current from the last step (A, mechanism direction). */
    public double statorCurrent() {
        return statorAmps;
    }

    /** Commanded motor voltage from the last step (V); torque modes give only the I R part. */
    public double motorVoltage() {
        return motorVolts;
    }

    /** Closed-loop error from the last step (rotations or rps). */
    public double closedLoopError() {
        return closedLoopError;
    }

    /** Name of a request this model does not run, or null. */
    public String unsupportedRequest() {
        return unsupported;
    }
}
