package frc.spectrumLib.swerve;

/**
 * Slow-changing drivetrain readings, read on the main thread at a low rate. Names follow Phoenix's
 * status signals (StatorCurrent, SupplyCurrent, DeviceTemp). Arrays are per module, FL FR BL BR.
 */
public final class SwerveHealth {
    public final boolean[] driveConnected;
    public final boolean[] steerConnected;
    public final boolean[] encoderConnected;
    public boolean pigeonConnected;

    /**
     * The robot's tilt from the Pigeon (rad), in WPILib's {@code Rotation3d} convention: roll about
     * forward (positive: left side up), pitch about left (positive: nose down). 0 while the Pigeon
     * is not connected.
     */
    public double roll;

    public double pitch;

    /** Each device took its config (at boot, or on a later background retry). */
    public final boolean[] driveConfigured;

    public final boolean[] steerConfigured;
    public final boolean[] encoderConfigured;

    /**
     * Each steer motor is fusing its CANcoder: no FusedSensorOutOfSync or RemoteSensorDataInvalid
     * fault. False means the absolute angle is no longer trusted.
     */
    public final boolean[] fusedOk;

    /** Each CANcoder's magnet is readable (no BadMagnet fault). */
    public final boolean[] magnetOk;

    /** The bus is a CANivore (CAN FD) and controls and signals use Phoenix timesync. */
    public boolean timesync;

    /** Device reboots detected since boot (any drivetrain device). */
    public long deviceResets;

    /** Signal update rates are set on every device (false right after a reboot, until redone). */
    public boolean signalsSet = true;

    /** The drive motors are in brake (false: coast), as last applied. */
    public boolean driveBrake = true;

    /** Amps. */
    public final double[] driveStatorCurrent;

    public final double[] steerStatorCurrent;
    public final double[] driveSupplyCurrent;
    public final double[] steerSupplyCurrent;

    /** Celsius. */
    public final double[] driveDeviceTemp;

    public final double[] steerDeviceTemp;

    /** Volts at the drive motors: the highest among connected drive motors, 0 if none. */
    public double supplyVoltage;

    /** Volts at each motor, per module: differences show voltage drop in the wiring to a motor. */
    public final double[] driveSupplyVoltage;

    public final double[] steerSupplyVoltage;

    public SwerveHealth(int modules) {
        driveConnected = new boolean[modules];
        steerConnected = new boolean[modules];
        encoderConnected = new boolean[modules];
        driveConfigured = new boolean[modules];
        steerConfigured = new boolean[modules];
        encoderConfigured = new boolean[modules];
        fusedOk = new boolean[modules];
        magnetOk = new boolean[modules];
        driveStatorCurrent = new double[modules];
        steerStatorCurrent = new double[modules];
        driveSupplyCurrent = new double[modules];
        steerSupplyCurrent = new double[modules];
        driveDeviceTemp = new double[modules];
        steerDeviceTemp = new double[modules];
        driveSupplyVoltage = new double[modules];
        steerSupplyVoltage = new double[modules];
    }
}
