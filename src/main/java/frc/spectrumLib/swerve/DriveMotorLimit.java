package frc.spectrumLib.swerve;

/**
 * How hard one drive motor can speed its wheel up at a given wheel speed and battery voltage, so
 * the setpoint generator never plans more than the motor can deliver (the idea from team 254's
 * generator). At low speed the stator (slip) current limit caps torque; as the wheel speeds up,
 * back-EMF leaves less voltage to drive current, and the supply current limit caps torque further.
 * Without this the generator plans a constant acceleration the motor cannot follow near top speed,
 * and the wheels fall behind their setpoints.
 *
 * <p>Only speeding up is limited here: braking is helped by back-EMF, so it stays at the traction
 * limit. Rotor inertia is left out (a few percent at these gear ratios). Allocation-free; safe on
 * the swerve thread.
 */
public final class DriveMotorLimit {
    private final double kT;
    private final double resistance;
    private final double kE;
    private final double rotorPerWheelMeter;
    private final double forcePerAmp;
    private final double statorLimit;
    private final double supplyLimit;
    private final double massPerWheel;

    /**
     * @param stallTorque motor stall torque at 12 V (N m)
     * @param stallCurrent motor stall current at 12 V (A)
     * @param freeSpeedRpm motor free speed at 12 V (rpm)
     * @param gearRatio motor rotations per wheel rotation
     * @param wheelRadius m
     * @param statorLimit drive stator current limit (A)
     * @param supplyLimit drive supply current limit (A), 0 or less for none
     * @param robotMass kg, shared evenly by the wheels
     * @param wheels number of driven wheels
     */
    public DriveMotorLimit(
            double stallTorque,
            double stallCurrent,
            double freeSpeedRpm,
            double gearRatio,
            double wheelRadius,
            double statorLimit,
            double supplyLimit,
            double robotMass,
            int wheels) {
        kT = stallTorque / stallCurrent;
        resistance = 12.0 / stallCurrent;
        kE = 12.0 / (freeSpeedRpm * 2 * Math.PI / 60.0);
        rotorPerWheelMeter = gearRatio / wheelRadius;
        forcePerAmp = kT * gearRatio / wheelRadius;
        this.statorLimit = statorLimit;
        this.supplyLimit = supplyLimit;
        massPerWheel = robotMass / wheels;
    }

    /**
     * Largest acceleration (m/s^2) one motor can give its share of the robot while speeding up,
     * never negative.
     *
     * @param wheelSpeed current wheel speed (m/s), either sign
     * @param volts battery voltage at the motor (V)
     */
    public double maxSpeedUp(double wheelSpeed, double volts) {
        double backEmf = kE * Math.abs(wheelSpeed) * rotorPerWheelMeter;
        double amps = Math.min(statorLimit, (volts - backEmf) / resistance);
        if (supplyLimit > 0 && amps > 0) {
            // Battery current is stator current times motor volts over battery volts.
            double supply = amps * (amps * resistance + backEmf) / volts;
            if (supply > supplyLimit) {
                // Solve R I^2 + backEmf I = limit * volts for I.
                amps =
                        (-backEmf
                                        + Math.sqrt(
                                                backEmf * backEmf
                                                        + 4 * resistance * supplyLimit * volts))
                                / (2 * resistance);
            }
        }
        return Math.max(0.0, amps * forcePerAmp / massPerWheel);
    }
}
