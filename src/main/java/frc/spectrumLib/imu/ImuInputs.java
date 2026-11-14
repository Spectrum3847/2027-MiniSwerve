package frc.spectrumLib.imu;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * Logged inputs from an IMU read alongside the drivetrain's gyro (for now the Flywoo Goku F7 over
 * USB). Angles in degrees, WPILib frame: yaw CCW positive, continuous (does not wrap).
 */
public final class ImuInputs implements LoggableInputs {
    /** Its frames are arriving. */
    public boolean connected;

    /** It finished its at-rest gyro calibration (false: still calibrating, keep still). */
    public boolean calibrated;

    /** It reports the robot moving. */
    public boolean moving;

    /** Yaw (deg), CCW positive, continuous. */
    public double yawDeg;

    /** Pitch (deg). */
    public double pitchDeg;

    /** Roll (deg). */
    public double rollDeg;

    /** Yaw rate about world Z (deg/s), CCW positive. */
    public double yawRateDps;

    /** Robot-clock time of the frame these values came from (s), Timer.getTimestamp base. */
    public double timestamp;

    /** Board temperature (C). */
    public double temperatureC;

    /** Active fault bits (0: none). */
    public int faultFlags;

    /** Sticky fault bits (0: none). */
    public int stickyFaultFlags;

    /** Firmware, board and serial number, read once connected; "" until then. */
    public String deviceInfo = "";

    /** Why it is not giving readings (open failed, a read threw), or "". */
    public String error = "";

    @Override
    public void toLog(LogTable table) {
        // IMU frames are arriving.
        table.put("Connected", connected);
        // IMU finished its at-rest gyro calibration.
        table.put("Calibrated", calibrated);
        // IMU reports the robot moving.
        table.put("Moving", moving);
        // IMU yaw (deg), CCW positive, continuous.
        table.put("YawDeg", yawDeg);
        // IMU pitch (deg).
        table.put("PitchDeg", pitchDeg);
        // IMU roll (deg).
        table.put("RollDeg", rollDeg);
        // IMU yaw rate about world Z (deg/s), CCW positive.
        table.put("YawRateDps", yawRateDps);
        // Robot-clock time of the IMU frame (s).
        table.put("Timestamp", timestamp);
        // IMU board temperature (C).
        table.put("TemperatureC", temperatureC);
        // IMU active fault bits, 0 for none.
        table.put("FaultFlags", faultFlags);
        // IMU sticky fault bits, 0 for none.
        table.put("StickyFaultFlags", stickyFaultFlags);
        // IMU firmware, board and serial, "" until read.
        table.put("DeviceInfo", deviceInfo);
        // Why the IMU gives no readings, "" when it does.
        table.put("Error", error);
    }

    @Override
    public void fromLog(LogTable table) {
        connected = table.get("Connected", connected);
        calibrated = table.get("Calibrated", calibrated);
        moving = table.get("Moving", moving);
        yawDeg = table.get("YawDeg", yawDeg);
        pitchDeg = table.get("PitchDeg", pitchDeg);
        rollDeg = table.get("RollDeg", rollDeg);
        yawRateDps = table.get("YawRateDps", yawRateDps);
        timestamp = table.get("Timestamp", timestamp);
        temperatureC = table.get("TemperatureC", temperatureC);
        faultFlags = table.get("FaultFlags", faultFlags);
        stickyFaultFlags = table.get("StickyFaultFlags", stickyFaultFlags);
        deviceInfo = table.get("DeviceInfo", deviceInfo);
        error = table.get("Error", error);
    }
}
