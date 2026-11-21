package frc.spectrumLib.leds;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * Logged inputs from one SpectrumShine LED controller: what its status and power frames say. The
 * board measures the battery and its own LED current, so these are worth having in every log.
 */
public final class ShineInputs implements LoggableInputs {
    /** A status frame arrived in the last half second. Act on the rest only while this is true. */
    public boolean connected;

    /** Live fault bits ({@link ShineProtocol#faultNames}). */
    public int faults;

    /** Fault bits seen since the board booted. */
    public int stickyFaults;

    /**
     * What is holding the LED current down: 0 nothing, 1 the board's configured limit, 2 battery
     * voltage, 3 temperature, 4 heat budget, 5 rail sag, 6 off.
     */
    public int limitReason;

    /** The board's processor temperature (C). */
    public int temperatureC;

    /** Share of the asked brightness going out after the board's current limit, 0..1. */
    public double outputScale;

    /** The board has the robot's clock, so its animations and event times run on it. */
    public boolean timeSynced;

    /** The board is running at least one animation slot. */
    public boolean anySlot;

    /** Battery voltage at the board (V). */
    public double batteryVolts;

    /** The board's 5 V LED rail (V). */
    public double railVolts;

    /** LED current the board measures (A). */
    public double measuredAmps;

    /** LED current the board expects from the pixels it is sending (A). */
    public double estimatedAmps;

    /** Firmware "major.minor", "" until the board has said. */
    public String firmware = "";

    /** The board's unique id (hex), "" until it has said. */
    public String uid = "";

    /** Times the board announced itself unasked: it booted (power or reset) while we ran. */
    public long boots;

    /** Frames handed to the CAN bus since boot. */
    public long framesSent;

    /** Frames the CAN bus refused since boot. */
    public long sendErrors;

    @Override
    public void toLog(LogTable table) {
        // A status frame arrived from the LED controller in the last 0.5 s.
        table.put("Connected", connected);
        // LED controller live fault bits (ShineProtocol.faultNames).
        table.put("Faults", faults);
        // LED controller fault bits seen since it booted.
        table.put("StickyFaults", stickyFaults);
        // What limits LED current: 0 none, 1 config, 2 battery, 3 temperature, 4 heat, 5 rail, 6
        // off.
        table.put("LimitReason", limitReason);
        // LED controller processor temperature (C).
        table.put("TemperatureC", temperatureC);
        // Share of the asked brightness going out after the current limit (0..1).
        table.put("OutputScale", outputScale);
        // The LED controller has the robot's clock.
        table.put("TimeSynced", timeSynced);
        // The LED controller is running at least one animation slot.
        table.put("AnySlot", anySlot);
        // Battery voltage at the LED controller (V).
        table.put("BatteryVolts", batteryVolts);
        // The LED controller's 5 V rail (V).
        table.put("RailVolts", railVolts);
        // LED current measured by the controller (A).
        table.put("MeasuredAmps", measuredAmps);
        // LED current the controller expects from its pixel data (A).
        table.put("EstimatedAmps", estimatedAmps);
        // LED controller firmware "major.minor", "" until read.
        table.put("Firmware", firmware);
        // LED controller unique id (hex), "" until read.
        table.put("UID", uid);
        // Times the LED controller booted while the robot code ran (count).
        table.put("Boots", boots);
        // Frames sent to the LED controller since boot (count).
        table.put("FramesSent", framesSent);
        // Frames the CAN bus refused since boot (count).
        table.put("SendErrors", sendErrors);
    }

    @Override
    public void fromLog(LogTable table) {
        connected = table.get("Connected", false);
        faults = table.get("Faults", 0);
        stickyFaults = table.get("StickyFaults", 0);
        limitReason = table.get("LimitReason", 0);
        temperatureC = table.get("TemperatureC", 0);
        outputScale = table.get("OutputScale", 0.0);
        timeSynced = table.get("TimeSynced", false);
        anySlot = table.get("AnySlot", false);
        batteryVolts = table.get("BatteryVolts", 0.0);
        railVolts = table.get("RailVolts", 0.0);
        measuredAmps = table.get("MeasuredAmps", 0.0);
        estimatedAmps = table.get("EstimatedAmps", 0.0);
        firmware = table.get("Firmware", "");
        uid = table.get("UID", "");
        boots = table.get("Boots", 0L);
        framesSent = table.get("FramesSent", 0L);
        sendErrors = table.get("SendErrors", 0L);
    }
}
