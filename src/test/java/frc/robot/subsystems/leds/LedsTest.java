package frc.robot.subsystems.leds;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.subsystems.leds.Leds.Ends;
import frc.robot.subsystems.leds.Leds.Strip;
import frc.spectrumLib.leds.LedPattern;
import frc.spectrumLib.leds.ShineIO;
import frc.spectrumLib.leds.ShineInputs;
import org.junit.jupiter.api.Test;

/** Which state wins. */
class LedsTest {
    private static Strip strip(
            boolean eStop,
            boolean ds,
            boolean enabled,
            boolean auto,
            boolean utility,
            boolean low) {
        return Leds.strip(eStop, ds, enabled, auto, utility, low);
    }

    @Test
    void disabledShowsDriverStationThenReady() {
        assertEquals(Strip.NO_DRIVER_STATION, strip(false, false, false, false, false, false));
        assertEquals(Strip.DISABLED, strip(false, true, false, false, false, false));
        // The selected mode does not matter until the robot is enabled.
        assertEquals(Strip.DISABLED, strip(false, true, false, true, false, false));
    }

    @Test
    void enabledShowsTheMode() {
        assertEquals(Strip.AUTO, strip(false, true, true, true, false, false));
        assertEquals(Strip.UTILITY, strip(false, true, true, false, true, false));
        assertEquals(Strip.TELEOP, strip(false, true, true, false, false, false));
    }

    @Test
    void lowBatteryTakesTheStripOnlyWhileDisabled() {
        assertEquals(Strip.LOW_BATTERY, strip(false, true, false, false, false, true));
        assertEquals(Strip.LOW_BATTERY, strip(false, false, false, false, false, true));
        // Enabled: the driver still sees the mode; the ends carry the battery.
        assertEquals(Strip.TELEOP, strip(false, true, true, false, false, true));
        assertEquals(Ends.LOW_BATTERY, Leds.ends(0, 1, true, Strip.TELEOP));
        assertEquals(Ends.NONE, Leds.ends(0, 1, true, Strip.LOW_BATTERY));
        // Another warning beside the battery's own still shows.
        assertEquals(Ends.WARNING, Leds.ends(0, 2, true, Strip.LOW_BATTERY));
    }

    @Test
    void eStopBeatsEverything() {
        assertEquals(Strip.ESTOPPED, strip(true, true, false, false, false, true));
        assertEquals(Strip.ESTOPPED, strip(true, false, true, true, false, false));
    }

    @Test
    void endsShowTheWorstThing() {
        assertEquals(Ends.NONE, Leds.ends(0, 0, false, Strip.DISABLED));
        assertEquals(Ends.WARNING, Leds.ends(0, 2, false, Strip.DISABLED));
        assertEquals(Ends.FAULT, Leds.ends(1, 2, false, Strip.TELEOP));
        assertEquals(Ends.FAULT, Leds.ends(1, 1, true, Strip.LOW_BATTERY));
    }

    /** Records what each slot was last set to: "pattern start+length". */
    private static final class FakeIo implements ShineIO {
        final String[] slots = new String[3];

        @Override
        public void updateInputs(ShineInputs inputs) {
            inputs.connected = true;
        }

        @Override
        public void setSlot(int slot, LedPattern pattern, int start, int length, int brightness) {
            slots[slot] = pattern.animation() + " " + start + "+" + length;
        }
    }

    private static String[] shown(int count, int endLeds, int warnings) {
        FakeIo io = new FakeIo();
        new Leds(io, count, endLeds, 110)
                .periodic(false, true, false, false, false, false, 0, warnings);
        return io.slots;
    }

    @Test
    void modeFillsTheStripUntilTheEndsLightThenKeepsToTheMiddle() {
        assertArrayEquals(
                new String[] {"SINGLE_FADE 0+16", "OFF 0+3", "OFF 13+3"}, shown(16, 3, 0));
        assertArrayEquals(
                new String[] {"SINGLE_FADE 3+10", "STROBE 0+3", "STROBE 13+3"}, shown(16, 3, 1));
    }

    @Test
    void oddSizesStayInsideTheStrip() {
        // No end LEDs: the mode keeps the whole strip, warning or not.
        assertArrayEquals(
                new String[] {"SINGLE_FADE 0+16", "STROBE 0+0", "STROBE 16+0"}, shown(16, 0, 1));
        // End LEDs never take the whole strip: at least one LED is left for the mode.
        assertArrayEquals(
                new String[] {"SINGLE_FADE 2+1", "STROBE 0+2", "STROBE 3+2"}, shown(5, 64, 1));
        assertArrayEquals(
                new String[] {"SINGLE_FADE 0+1", "STROBE 0+0", "STROBE 1+0"}, shown(1, 3, 1));
    }
}
