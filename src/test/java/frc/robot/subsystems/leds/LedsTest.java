package frc.robot.subsystems.leds;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.subsystems.leds.Leds.Ends;
import frc.robot.subsystems.leds.Leds.Strip;
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
}
