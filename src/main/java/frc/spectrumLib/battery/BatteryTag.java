package frc.spectrumLib.battery;

import frc.spectrumLib.telemetry.Alert;
import org.littletonrobotics.junction.Logger;

/**
 * Which battery is in the robot, from the NFC tag on it, so every log says which battery it ran on
 * and per-battery history (sag, internal resistance, energy per match) comes from the logs.
 *
 * <p>Each loop it logs the reader's inputs ({@link BatteryTagInputs}, under {@code Battery/}) and
 * resolves the battery's name: the name written on the tag, else the team list's name for the UID
 * ({@link BatteryNames}), else the UID itself. A missing reader or tag raises an alert and the
 * robot runs on; nothing waits for it.
 *
 * <p>Index: constructor | {@link #periodic} | accessors | alerts
 */
public final class BatteryTag {
    private final BatteryTagIO io;
    private final BatteryNames names;
    private final BatteryTagInputs inputs = new BatteryTagInputs();
    private final Alert noReader = new Alert("", Alert.Level.LOW);
    private final Alert noTag = new Alert("No battery tag on the reader", Alert.Level.LOW);
    private final Alert unnamed = new Alert("", Alert.Level.LOW);

    private String lastUid = "";

    /** The last non-empty UID seen, so a swap through "no tag" still counts. */
    private String lastBatteryUid = "";

    private String lastTagName = "";
    private String lastError = "";
    private String name = "";
    private int swaps;

    /**
     * @param io the reader (or a stand-in, or {@link BatteryTagIO#NONE} in replay)
     * @param names the team's battery list, for tags with no name written on them
     */
    public BatteryTag(BatteryTagIO io, BatteryNames names) {
        this.io = io;
        this.names = names;
    }

    /**
     * Main loop, each loop.
     *
     * @param disabled whether the robot is disabled: the reader polls only then (batteries are
     *     swapped while disabled), after its one read at boot
     */
    public void periodic(boolean disabled) {
        io.updateInputs(inputs, disabled);
        inputs.listName = names.name(inputs.uid);
        Logger.processInputs("Battery", inputs);

        if (!inputs.uid.equals(lastUid) || !inputs.tagName.equals(lastTagName)) {
            if (!inputs.uid.isEmpty()) {
                if (!lastBatteryUid.isEmpty() && !inputs.uid.equals(lastBatteryUid)) {
                    swaps++;
                }
                lastBatteryUid = inputs.uid;
            }
            lastUid = inputs.uid;
            lastTagName = inputs.tagName;
            name = resolveName();
            unnamed.setText("Battery " + inputs.uid + " has no name (tag or list)");
        }
        if (!inputs.error.equals(lastError)) {
            lastError = inputs.error;
            noReader.setText("Battery tag reader not answering: " + inputs.error);
        }

        // Quiet while the reader thread makes its first attempt at boot.
        noReader.set(!inputs.readerConnected && !inputs.error.equals(BatteryTagIOSerial.STARTING));
        noTag.set(inputs.readerConnected && inputs.reads > 0 && !inputs.tagPresent);
        unnamed.set(inputs.tagPresent && inputs.tagName.isEmpty() && inputs.listName.isEmpty());

        // Battery in the robot: tag name, else the list's name, else the UID; "" if unknown.
        Logger.recordOutput("Battery/Name", name);
        // Times a different tag replaced the one before, since boot (count).
        Logger.recordOutput("Battery/Swaps", swaps);
    }

    private String resolveName() {
        if (inputs.uid.isEmpty()) {
            return "";
        }
        if (!inputs.tagName.isEmpty()) {
            return inputs.tagName;
        }
        return inputs.listName.isEmpty() ? inputs.uid : inputs.listName;
    }

    /** Times a different battery replaced the one before, since boot. */
    public int swaps() {
        return swaps;
    }

    /** The battery's tag UID, or "" if no tag is known. */
    public String uid() {
        return inputs.uid;
    }

    /** The battery's name (tag, list, or UID), or "" if no tag is known. */
    public String name() {
        return name;
    }

    /**
     * Whether the last read found a tag. The reader polls only while disabled, so during a match
     * this is the pre-match read (see {@link BatteryTagInputs#secondsSinceRead}).
     */
    public boolean identified() {
        return inputs.readerConnected && inputs.tagPresent;
    }
}
