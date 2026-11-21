package frc.spectrumLib.leds;

/**
 * One SpectrumShine LED controller: the board on CAN, a sim stand-in, or nothing (replay).
 *
 * <p>The board draws up to eight slots, each an animation over a range of LEDs, higher slots over
 * lower. The robot says what each slot should show ({@link #setSlot}) as often as it likes; the IO
 * sends only what changed, and repeats it now and then so a board that lost power comes back right.
 */
public interface ShineIO {
    /** Main thread, each loop: copy the latest readings into the inputs. Never blocks. */
    void updateInputs(ShineInputs inputs);

    /**
     * What a slot should show. Cheap to call every loop with the same values.
     *
     * @param slot 0..7; higher slots draw over lower
     * @param pattern what to draw ({@link LedPattern#OFF} to let lower slots show through)
     * @param start first LED of the range (output A's LEDs come first, then output B's)
     * @param length LEDs in the range
     * @param brightness 0..255
     */
    default void setSlot(int slot, LedPattern pattern, int start, int length, int brightness) {}

    /** Main thread, each loop, after the slots are set: sends what is due. Never blocks. */
    default void send() {}

    /** Replay: inputs come from the log, nothing is sent. */
    ShineIO NONE = inputs -> {};

    /** Sim: a board that is always there and healthy on a full battery. */
    static ShineIO simulated() {
        return inputs -> {
            inputs.connected = true;
            inputs.timeSynced = true;
            inputs.anySlot = true;
            inputs.outputScale = 1.0;
            inputs.batteryVolts = 12.5;
            inputs.railVolts = 5.0;
            inputs.firmware = "sim";
        };
    }
}
