package frc.spectrumSim;

/**
 * A DC motor for the physics sim: torque per amp, winding resistance, back-EMF and rotor inertia,
 * from the maker's published stall and free-speed numbers at 12 V.
 *
 * @param name for logs
 * @param kT torque constant (N m per A)
 * @param resistance winding resistance (ohm)
 * @param kE back-EMF (V per rad/s of rotor)
 * @param rotorInertia rotor moment of inertia (kg m^2)
 */
public record MotorModel(
        String name, double kT, double resistance, double kE, double rotorInertia) {

    /**
     * From the published curve at 12 V.
     *
     * @param stallTorque N m
     * @param stallCurrent A
     * @param freeSpeedRpm rotor speed with no load
     */
    public static MotorModel fromCurve(
            String name,
            double stallTorque,
            double stallCurrent,
            double freeSpeedRpm,
            double rotorInertia) {
        double freeRadPerSec = freeSpeedRpm * 2 * Math.PI / 60.0;
        return new MotorModel(
                name,
                stallTorque / stallCurrent,
                12.0 / stallCurrent,
                12.0 / freeRadPerSec,
                rotorInertia);
    }

    /** CTRE Falcon 500 with FOC: 5.84 N m and 304 A stall, 6080 rpm free. */
    public static final MotorModel FALCON_500_FOC =
            fromCurve("Falcon500FOC", 5.84, 304, 6080, 0.0000565);

    /**
     * WCP Kraken X60 with FOC: 9.37 N m and 483 A stall, 5800 rpm free. Rotor inertia is an
     * estimate (not published); check it against a free-spin log.
     */
    public static final MotorModel KRAKEN_X60_FOC =
            fromCurve("KrakenX60FOC", 9.37, 483, 5800, 0.00006);

    /** A motor by {@link #name}: "KrakenX60FOC" or "Falcon500FOC". */
    public static MotorModel byName(String name) {
        for (MotorModel m : new MotorModel[] {KRAKEN_X60_FOC, FALCON_500_FOC}) {
            if (m.name().equals(name)) {
                return m;
            }
        }
        throw new IllegalArgumentException("unknown motor " + name);
    }

    /**
     * Stator current allowed by a supply (battery-side) current limit at this rotor speed: battery
     * current is stator current times the motor's voltage over the battery's, so at speed a supply
     * limit caps torque well below the stator limit. Only limits motoring (not braking).
     *
     * @param supplyLimit amps; 0 or less for none
     */
    public double supplyLimited(
            double amps, double rotorOmega, double batteryVolts, double supplyLimit) {
        if (supplyLimit <= 0 || amps == 0) {
            return amps;
        }
        double motorVolts = amps * resistance + kE * rotorOmega;
        double supply = amps * motorVolts / batteryVolts;
        if (supply <= supplyLimit) {
            return amps;
        }
        // Solve R I^2 + kE w I = limit * Vbat for I, on the side of the requested current.
        double b = kE * rotorOmega * Math.signum(amps);
        double i =
                (-b + Math.sqrt(b * b + 4 * resistance * supplyLimit * batteryVolts))
                        / (2 * resistance);
        return Math.copySign(Math.min(Math.abs(amps), i), amps);
    }

    /** Battery-side current for this stator current (A). */
    public double supplyCurrent(double amps, double rotorOmega, double batteryVolts) {
        return amps * (amps * resistance + kE * rotorOmega) / batteryVolts;
    }

    /** Most current the battery can push through the motor turning at {@code rotorOmega}. */
    public double limit(double amps, double rotorOmega, double batteryVolts) {
        double emf = kE * rotorOmega;
        double max = (batteryVolts - emf) / resistance;
        double min = (-batteryVolts - emf) / resistance;
        return Math.max(min, Math.min(max, amps));
    }
}
