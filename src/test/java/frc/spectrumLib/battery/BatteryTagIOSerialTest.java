package frc.spectrumLib.battery;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Optional;
import org.junit.jupiter.api.Test;

class BatteryTagIOSerialTest {
    private static final String FIRMWARE = "0000FF00FF00 0000FF06FA D503 32 01 06 07 E8 00";
    private static final String OK_SAM = "0000FF00FF00 0000FF02FE D515 16 00";
    private static final String OK_RF = "0000FF00FF00 0000FF02FE D533 F8 00";

    /** The captured MIFARE reply from Pn532Test, then InRelease. */
    private static final String TAG =
            "0000FF00FF00 0000FF0CF4 D54B 01 01 0004 08 04 D7A89B9E 16 00";

    private static final String RELEASE = "0000FF00FF00 0000FF03FD D55300 D8 00";

    private static BatteryTagInputs inputs(BatteryTagIOSerial io) {
        BatteryTagInputs in = new BatteryTagInputs();
        io.updateInputs(in, true);
        return in;
    }

    @Test
    void noPortReportsMissingReader() {
        BatteryTagIOSerial io = new BatteryTagIOSerial(Optional::empty, p -> null, false);
        assertEquals(BatteryTagIOSerial.POLL_MILLIS, io.cycle());
        BatteryTagInputs in = inputs(io);
        assertFalse(in.readerConnected);
        assertEquals("no reader on USB", in.error);
    }

    @Test
    void missingNativeLibraryRetriesSlowly() {
        BatteryTagIOSerial io =
                new BatteryTagIOSerial(
                        () -> {
                            throw new UnsatisfiedLinkError("no jSerialComm");
                        },
                        p -> null,
                        false);
        assertEquals(BatteryTagIOSerial.NATIVE_RETRY_MILLIS, io.cycle());
        assertTrue(inputs(io).error.startsWith("serial library unavailable"));
    }

    @Test
    void readsTagThenReportsReaderLoss() throws Exception {
        Pn532Test.FakeTransport t = new Pn532Test.FakeTransport();
        t.reply(FIRMWARE);
        t.reply(OK_SAM);
        t.reply(OK_RF);
        BatteryTagIOSerial io = new BatteryTagIOSerial(() -> Optional.of("ttyUSB0"), p -> t, false);
        Pn532 reader = new Pn532(t);
        reader.begin(2);
        t.reply(TAG);
        t.reply(RELEASE);
        io.read(reader);
        BatteryTagInputs in = inputs(io);
        assertTrue(in.readerConnected);
        assertTrue(in.tagPresent);
        assertEquals("D7A89B9E", in.uid);
        assertEquals("", in.tagName);
        assertEquals(1, in.reads);

        // The reader stops answering: the next cycle opens it, gets no reply, and reports that.
        assertEquals(BatteryTagIOSerial.POLL_MILLIS, io.cycle());
        in = inputs(io);
        assertFalse(in.readerConnected);
        assertFalse(in.tagPresent);
        assertTrue(in.error.startsWith("ttyUSB0: "), in.error);
        assertEquals(1, in.reads);
    }
}
