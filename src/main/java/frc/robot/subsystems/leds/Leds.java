package frc.robot.subsystems.leds;

import frc.spectrumLib.leds.LedColor;
import frc.spectrumLib.leds.LedPattern;
import frc.spectrumLib.leds.ShineAnimation;
import frc.spectrumLib.leds.ShineIO;
import frc.spectrumLib.leds.ShineInputs;
import frc.spectrumLib.leds.ShineProtocol;
import frc.spectrumLib.telemetry.Alert;
import org.littletonrobotics.junction.Logger;

/**
 * The robot's LED strip: what the robot is doing and whether it needs attention, readable from
 * across the pit. One short strip on a SpectrumShine board, in Spectrum's purple and white.
 *
 * <p>The middle of the strip shows the mode ({@link Strip}); a few LEDs at each end show trouble
 * ({@link Ends}), so a warning never hides whether the robot is enabled. Two things take the whole
 * strip: an e-stop, and a low battery while disabled (when it can still be swapped).
 *
 * <p>Purple and white mean the robot is fine. Yellow is a warning, orange the battery, and red is
 * kept for faults and e-stop.
 *
 * <p>LEDs are feedback only (AGENTS.md "Finish the match"): a missing or faulted board is an alert
 * and nothing else. Everything here is decided from logged inputs, so replay shows the same strip.
 *
 * <p>Index: {@link Strip} | {@link Ends} | {@link #strip} | {@link #ends} | {@link #periodic}
 */
public final class Leds {
    /** What the middle of the strip shows, in priority order (first that applies wins). */
    public enum Strip {
        /** Solid red: the robot is e-stopped. */
        ESTOPPED(LedPattern.solid(LedColor.RED)),
        /** Orange flashing, whole strip: resting battery is low and it is disabled. Swap it. */
        LOW_BATTERY(LedPattern.of(ShineAnimation.STROBE, LedColor.ORANGE, 135)),
        /** A purple eye sweeping side to side: no Driver Station connected. */
        NO_DRIVER_STATION(
                new LedPattern(
                        ShineAnimation.LARSON, LedColor.PURPLE, LedColor.BLACK, 23, 2, false)),
        /** Purple breathing, about 3 s a breath: disabled and ready. */
        DISABLED(LedPattern.of(ShineAnimation.SINGLE_FADE, LedColor.PURPLE, 41)),
        /** White dots marching over purple: autonomous. */
        AUTO(LedPattern.of(ShineAnimation.THEATER, LedColor.WHITE, LedColor.PURPLE, 11, 0)),
        /** Purple and white halves swapping twice a second: a utility OpMode is running. */
        UTILITY(LedPattern.of(ShineAnimation.ALTERNATE, LedColor.PURPLE, LedColor.WHITE, 217, 0)),
        /** A white band flowing through purple: teleop, driving. */
        TELEOP(LedPattern.of(ShineAnimation.WAVE, LedColor.PURPLE, LedColor.WHITE, 127, 0));

        final LedPattern pattern;

        Strip(LedPattern pattern) {
            this.pattern = pattern;
        }
    }

    /** What the end LEDs show, in priority order (first that applies wins). */
    public enum Ends {
        /** Red, fast flash: a high-level alert is up. Something is broken. */
        FAULT(LedPattern.of(ShineAnimation.STROBE, LedColor.RED, 200)),
        /** Orange flash: resting battery was low before this enable. */
        LOW_BATTERY(LedPattern.of(ShineAnimation.STROBE, LedColor.ORANGE, 135)),
        /** Yellow, slow blink: a medium-level alert is up. Look at the alerts. */
        WARNING(LedPattern.of(ShineAnimation.STROBE, LedColor.YELLOW, 10)),
        /** Nothing: the mode pattern runs the whole strip. */
        NONE(LedPattern.OFF);

        final LedPattern pattern;

        Ends(LedPattern pattern) {
            this.pattern = pattern;
        }
    }

    private static final int SLOT_STRIP = 0;
    private static final int SLOT_LEFT_END = 1;
    private static final int SLOT_RIGHT_END = 2;

    private final ShineIO io;
    private final ShineInputs inputs = new ShineInputs();
    private final int count;
    private final int endLeds;
    private final int brightness;

    private final Alert notConnected =
            new Alert("LED controller (SpectrumShine) is not answering on CAN", Alert.Level.LOW);
    private final Alert boardFault = new Alert("", Alert.Level.MEDIUM);
    private int shownFaults;

    /**
     * @param io the board
     * @param count LEDs on the strip
     * @param endLeds LEDs at each end for warnings and faults (kept under half the strip)
     * @param brightness 0..255
     */
    public Leds(ShineIO io, int count, int endLeds, int brightness) {
        this.io = io;
        this.count = Math.max(1, count);
        this.endLeds = Math.max(0, Math.min(endLeds, (this.count - 1) / 2));
        this.brightness = Math.max(0, Math.min(255, brightness));
    }

    /**
     * The mode pattern for this loop.
     *
     * @param batteryLow the resting battery voltage is under the low-battery setting
     */
    public static Strip strip(
            boolean eStopped,
            boolean driverStationAttached,
            boolean enabled,
            boolean autonomous,
            boolean utility,
            boolean batteryLow) {
        if (eStopped) {
            return Strip.ESTOPPED;
        }
        if (!enabled) {
            if (batteryLow) {
                return Strip.LOW_BATTERY;
            }
            return driverStationAttached ? Strip.DISABLED : Strip.NO_DRIVER_STATION;
        }
        if (autonomous) {
            return Strip.AUTO;
        }
        return utility ? Strip.UTILITY : Strip.TELEOP;
    }

    /**
     * The end markers for this loop.
     *
     * @param faults active high-level alerts
     * @param warnings active medium-level alerts, the low-battery alert among them
     * @param batteryLow the resting battery voltage is under the low-battery setting (itself a
     *     medium-level alert, shown as the battery rather than as a general warning)
     * @param strip what the middle shows: a low battery already on the whole strip is not repeated
     */
    public static Ends ends(int faults, int warnings, boolean batteryLow, Strip strip) {
        if (faults > 0) {
            return Ends.FAULT;
        }
        if (batteryLow && strip != Strip.LOW_BATTERY) {
            return Ends.LOW_BATTERY;
        }
        // The battery is already on the whole strip: its own alert is not a second warning.
        int others = batteryLow ? warnings - 1 : warnings;
        return others > 0 ? Ends.WARNING : Ends.NONE;
    }

    /**
     * Main loop, each loop, after the alerts have been counted.
     *
     * @param faults active high-level alerts
     * @param warnings active medium-level alerts
     */
    public void periodic(
            boolean eStopped,
            boolean driverStationAttached,
            boolean enabled,
            boolean autonomous,
            boolean utility,
            boolean batteryLow,
            int faults,
            int warnings) {
        io.updateInputs(inputs);
        Logger.processInputs("Leds", inputs);

        notConnected.set(!inputs.connected);
        // The battery bits are left to the robot's own battery check; reset notes are not faults.
        int board = inputs.connected ? inputs.faults & ShineProtocol.FAULTS_BOARD : 0;
        if (board != shownFaults) {
            shownFaults = board;
            if (board != 0) {
                boardFault.setText("LED controller fault: " + ShineProtocol.faultNames(board));
            }
        }
        boardFault.set(board != 0);

        Strip strip =
                strip(eStopped, driverStationAttached, enabled, autonomous, utility, batteryLow);
        Ends ends = ends(faults, warnings, batteryLow, strip);
        io.setSlot(SLOT_STRIP, strip.pattern, 0, count, brightness);
        io.setSlot(SLOT_LEFT_END, ends.pattern, 0, endLeds, brightness);
        io.setSlot(SLOT_RIGHT_END, ends.pattern, count - endLeds, endLeds, brightness);
        io.send();

        // What the middle of the LED strip shows.
        Logger.recordOutput("Leds/Strip", strip);
        // What the LEDs at each end of the strip show.
        Logger.recordOutput("Leds/Ends", ends);
    }
}
