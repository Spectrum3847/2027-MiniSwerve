package frc.spectrumLib.battery;

/**
 * Where the battery tag reading comes from: the USB reader, a sim stand-in, or nothing (replay).
 */
public interface BatteryTagIO {
    /**
     * Main thread, each loop: copy the latest reading into the inputs. Never blocks.
     *
     * @param mayRead whether the reader may poll now (the robot is disabled, so batteries can be
     *     swapped; a match spends no USB traffic on it)
     */
    void updateInputs(BatteryTagInputs inputs, boolean mayRead);

    /** Replay: inputs come from the log. */
    BatteryTagIO NONE = (inputs, mayRead) -> {};

    /** A reader that always sees this tag (sim). */
    static BatteryTagIO fixed(String uid, String tagName) {
        return (inputs, mayRead) -> {
            inputs.readerConnected = true;
            inputs.tagPresent = true;
            inputs.uid = uid;
            inputs.tagName = tagName;
            inputs.error = "";
            inputs.reads = Math.max(inputs.reads, 1);
        };
    }
}
