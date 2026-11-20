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
     * forward (positive: left side up), pitch about left (positive: nose down). 0 while {@link
     * #tiltOk} is false.
     */
    public double roll;

    public double pitch;

    /** The Pigeon is connected and all four quaternion signals read OK, so roll and pitch hold. */
    public boolean tiltOk;

    /** Each device took its config (at boot, or on a later background retry). */
    public final boolean[] driveConfigured;

    public final boolean[] steerConfigured;
    public final boolean[] encoderConfigured;

    /**
     * Each steer motor is fusing its CANcoder: no FusedSensorOutOfSync or RemoteSensorDataInvalid
     * fault. False means the absolute angle is no longer trusted.
     */
    public final boolean[] fusedOk;

    /**
     * Each steer motor's FusedSensorOutOfSync fault: its rotor and its CANcoder disagree by more
     * than the fused sensor allows (gear or belt skip, slop, a moved magnet). Part of {@link
     * #fusedOk}.
     */
    public final boolean[] fusedOutOfSync;

    /**
     * Each steer motor's RemoteSensorDataInvalid fault: its CANcoder's frames are not arriving or
     * are bad (CAN wiring, bus load, CANcoder reboot). Part of {@link #fusedOk}.
     */
    public final boolean[] remoteSensorInvalid;

    /** Each CANcoder's magnet is readable (no BadMagnet fault). */
    public final boolean[] magnetOk;

    /**
     * Each CANcoder's magnet health, Phoenix's MagnetHealthValue names: "Magnet_Green" (good),
     * "Magnet_Orange" (weak or wobbling: still used), "Magnet_Red" (bad), "Magnet_Invalid"
     * (unknown).
     */
    public final String[] magnetHealth;

    /** The bus is a CANivore (CAN FD) and controls and signals use Phoenix timesync. */
    public boolean timesync;

    /** Device reboots detected since boot (any drivetrain device). */
    public long deviceResets;

    /**
     * Reboots of each steer motor detected since boot. A count, not a flag: a reboot that is
     * noticed and reconfigured between two health readings still shows.
     */
    public final long[] steerResets;

    /** Signal update rates are set on every device (false right after a reboot, until redone). */
    public boolean signalsSet = true;

    /** The drive motors are in brake (false: coast), as last applied. */
    public boolean driveBrake = true;

    /** The steer motors are in brake (false: coast), as last applied. */
    public boolean steerBrake = true;

    /** Each steer motor's sticky faults since boot, bits as SwerveStickyFaults.STEER_BITS. */
    public final long[] steerSticky;

    /** Each CANcoder's sticky faults since boot, bits as SwerveStickyFaults.ENCODER_BITS. */
    public final long[] encoderSticky;

    /** Result of clearing the sticky faults at boot (Phoenix StatusCode name; "not run" in sim). */
    public String stickyClear = "not run";

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
        fusedOutOfSync = new boolean[modules];
        remoteSensorInvalid = new boolean[modules];
        magnetOk = new boolean[modules];
        magnetHealth = new String[modules];
        steerSticky = new long[modules];
        encoderSticky = new long[modules];
        java.util.Arrays.fill(magnetHealth, "Magnet_Invalid");
        driveStatorCurrent = new double[modules];
        steerStatorCurrent = new double[modules];
        driveSupplyCurrent = new double[modules];
        steerSupplyCurrent = new double[modules];
        driveDeviceTemp = new double[modules];
        steerDeviceTemp = new double[modules];
        driveSupplyVoltage = new double[modules];
        steerSupplyVoltage = new double[modules];
        steerResets = new long[modules];
    }
}
