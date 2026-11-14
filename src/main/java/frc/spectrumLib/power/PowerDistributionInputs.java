package frc.spectrumLib.power;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** Logged inputs from the power distribution board (CTRE PDP or REV PDH) through WPILib. */
public final class PowerDistributionInputs implements LoggableInputs {
    /**
     * The readings look valid: read this loop and the battery at least 4 V. False when the board is
     * missing, a read failed, or the battery is nearly dead (then the rest read 0 or are low).
     */
    public boolean connected;

    /** Battery voltage at the board (V). */
    public double voltage;

    /** Sum of all channel currents (A). */
    public double totalCurrent;

    /** Board temperature (C). */
    public double temperature;

    /** Current per channel (A). Its length is the board's channel count (16 PDP, 24 PDH). */
    public double[] channelCurrents = new double[0];

    /** Firmware read at boot, "major.minor.fix", or "" if it could not be read. */
    public String firmware = "";

    /** Why the last read failed, or "". */
    public String error = "";

    @Override
    public void toLog(LogTable table) {
        // The board's readings look valid (read this loop, battery at least 4 V).
        table.put("Connected", connected);
        // Battery voltage at the board (V).
        table.put("Voltage", voltage);
        // Sum of all channel currents (A).
        table.put("TotalCurrent", totalCurrent);
        // Board temperature (C).
        table.put("Temperature", temperature);
        // Current per channel (A), channel 0 first.
        table.put("ChannelCurrents", channelCurrents);
        // Firmware read at boot, "" if unread.
        table.put("Firmware", firmware);
        // Why the last read failed, "" when it worked.
        table.put("Error", error);
    }

    @Override
    public void fromLog(LogTable table) {
        connected = table.get("Connected", connected);
        voltage = table.get("Voltage", voltage);
        totalCurrent = table.get("TotalCurrent", totalCurrent);
        temperature = table.get("Temperature", temperature);
        channelCurrents = table.get("ChannelCurrents", channelCurrents);
        firmware = table.get("Firmware", firmware);
        error = table.get("Error", error);
    }
}
