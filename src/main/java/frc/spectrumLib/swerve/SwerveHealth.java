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

    /** Each module's drive, steer and CANcoder took their config at boot. */
    public final boolean[] configured;

    /** Amps. */
    public final double[] driveStatorCurrent;

    public final double[] steerStatorCurrent;
    public final double[] driveSupplyCurrent;
    public final double[] steerSupplyCurrent;

    /** Celsius. */
    public final double[] driveDeviceTemp;

    public final double[] steerDeviceTemp;

    /** Volts at the drive motors. */
    public double supplyVoltage;

    public SwerveHealth(int modules) {
        driveConnected = new boolean[modules];
        steerConnected = new boolean[modules];
        encoderConnected = new boolean[modules];
        configured = new boolean[modules];
        driveStatorCurrent = new double[modules];
        steerStatorCurrent = new double[modules];
        driveSupplyCurrent = new double[modules];
        steerSupplyCurrent = new double[modules];
        driveDeviceTemp = new double[modules];
        steerDeviceTemp = new double[modules];
    }
}
