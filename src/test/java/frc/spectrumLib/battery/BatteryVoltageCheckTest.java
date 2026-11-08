package frc.spectrumLib.battery;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class BatteryVoltageCheckTest {
    @Test
    void judgesOnlyAfterSettlingWhileDisabled() {
        BatteryVoltageCheck c = new BatteryVoltageCheck(12.3, 3.0);
        c.periodic(11.0, false, 0); // sag while enabled: ignored
        assertTrue(Double.isNaN(c.restingVolts()));
        c.periodic(12.0, true, 1.0);
        c.periodic(12.0, true, 3.5); // only 2.5 s disabled
        assertTrue(Double.isNaN(c.restingVolts()));
        c.periodic(12.1, true, 4.0);
        assertEquals(12.1, c.restingVolts(), 1e-9);
        c.periodic(0.0, true, 5.0); // no motor reporting: keep the last good reading
        assertEquals(12.1, c.restingVolts(), 1e-9);
        c.periodic(10.5, false, 6.0); // enabled again: the resting value stays
        assertEquals(12.1, c.restingVolts(), 1e-9);
    }
}
