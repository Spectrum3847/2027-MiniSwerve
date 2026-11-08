package frc.spectrumLib.battery;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class BatteryTagTest {
    /** A reader whose next reading the test sets. */
    private static final class Scripted implements BatteryTagIO {
        String uid = "";
        String tagName = "";
        boolean connected = true;

        @Override
        public void updateInputs(BatteryTagInputs inputs, boolean mayRead) {
            inputs.readerConnected = connected;
            inputs.tagPresent = connected && !uid.isEmpty();
            inputs.uid = connected ? uid : "";
            inputs.tagName = connected ? tagName : "";
            inputs.reads++;
            inputs.error = connected ? "" : "no reader on USB";
        }
    }

    @Test
    void namesFromTagThenListThenUid() {
        Scripted io = new Scripted();
        BatteryTag tag = new BatteryTag(io, BatteryNames.parse("AAAA0001,Listed\n"));
        io.uid = "AAAA0001";
        io.tagName = "On tag";
        tag.periodic(true);
        assertEquals("On tag", tag.name());
        io.tagName = "";
        tag.periodic(true);
        assertEquals("Listed", tag.name());
        io.uid = "BBBB0002";
        tag.periodic(true);
        assertEquals("BBBB0002", tag.name());
        assertTrue(tag.identified());
    }

    @Test
    void swapThroughNoTagCounts() {
        Scripted io = new Scripted();
        BatteryTag tag = new BatteryTag(io, new BatteryNames());
        io.uid = "AAAA0001";
        tag.periodic(true);
        io.uid = ""; // battery out
        tag.periodic(true);
        assertFalse(tag.identified());
        assertEquals("", tag.name());
        io.uid = "BBBB0002"; // a different one in
        tag.periodic(true);
        io.uid = "BBBB0002";
        tag.periodic(true);
        assertEquals("BBBB0002", tag.uid());
        // Same battery back after a lost read is not a swap; a different one is.
        io.connected = false;
        tag.periodic(true);
        io.connected = true;
        tag.periodic(true);
        assertTrue(tag.identified());
        assertEquals(1, tag.swaps());
    }
}
