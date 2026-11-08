package frc.spectrumLib.battery;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * Logged inputs from the battery's NFC tag reader: which battery is in the robot, as the reader
 * last saw it. Inputs, not outputs, so replay and anything that reacts to the battery (the
 * pre-match check, per-battery limits) see the same value the robot did.
 */
public final class BatteryTagInputs implements LoggableInputs {
    /** The reader answered its last command. */
    public boolean readerConnected;

    /** A tag was on the reader at the last read. */
    public boolean tagPresent;

    /** Tag UID, upper-case hex with no separators (the battery's key), or "" with no tag. */
    public String uid = "";

    /** Name written on the tag (NDEF text record), or "" if none was read. */
    public String tagName = "";

    /** Completed reads since boot (with or without a tag). */
    public long reads;

    /** Why the reader is not answering (port missing, open failed, no reply), or "". */
    public String error = "";

    /**
     * Seconds since the last completed read, -1 if none yet. The reader polls only while disabled,
     * so during a match this is the age of the pre-match read.
     */
    public double secondsSinceRead = -1;

    /**
     * The team list's name for this UID, or "". An input so replay names the battery as the robot
     * did even if the list has changed since.
     */
    public String listName = "";

    @Override
    public void toLog(LogTable table) {
        // The NFC reader answered its last command.
        table.put("ReaderConnected", readerConnected);
        // A battery tag was on the reader at the last read.
        table.put("TagPresent", tagPresent);
        // Battery tag UID (hex), "" with no tag.
        table.put("TagUID", uid);
        // Battery name written on the tag, "" if none.
        table.put("TagName", tagName);
        // Completed tag reads since boot (count).
        table.put("Reads", reads);
        // Why the reader is not answering, "" when it is.
        table.put("Error", error);
        // Time since the last completed tag read (s), -1 if none yet.
        table.put("SecondsSinceRead", secondsSinceRead);
        // The team list's name for this UID (deploy batteries.csv), "" if not listed.
        table.put("ListName", listName);
    }

    @Override
    public void fromLog(LogTable table) {
        readerConnected = table.get("ReaderConnected", false);
        tagPresent = table.get("TagPresent", false);
        uid = table.get("TagUID", "");
        tagName = table.get("TagName", "");
        reads = table.get("Reads", 0L);
        error = table.get("Error", "");
        secondsSinceRead = table.get("SecondsSinceRead", -1.0);
        listName = table.get("ListName", "");
    }
}
