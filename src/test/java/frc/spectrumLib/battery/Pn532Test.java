package frc.spectrumLib.battery;

import static org.junit.jupiter.api.Assertions.*;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.HexFormat;
import org.junit.jupiter.api.Test;

class Pn532Test {

    /** Replays canned reader output, one queued chunk per command written. */
    static class FakeTransport implements Pn532Transport {
        final Deque<byte[]> replies = new ArrayDeque<>();
        final Deque<byte[]> written = new ArrayDeque<>();
        byte[] pending = new byte[0];
        int pos;

        void reply(String hex) {
            replies.add(HexFormat.of().parseHex(hex.replace(" ", "")));
        }

        @Override
        public void write(byte[] data) {
            written.add(data);
            if (!replies.isEmpty()) {
                pending = replies.poll();
                pos = 0;
            }
        }

        @Override
        public int read(byte[] buf, int timeoutMs) {
            if (pos >= pending.length) return 0;
            buf[0] = pending[pos++];
            return 1;
        }

        @Override
        public void discardInput() {}

        @Override
        public void close() {}
    }

    @Test
    void buildsInListPassiveTargetFrame() {
        assertEquals(
                "00 00 FF 04 FC D4 4A 01 00 E1 00",
                Pn532.hex(Pn532.buildFrame(0x4A, new byte[] {1, 0})));
    }

    /**
     * Bytes captured from the bench reader (CH340 + PN532 v1.6) with a MIFARE Classic tag on it.
     */
    @Test
    void parsesCapturedTagReply() throws Exception {
        FakeTransport t = new FakeTransport();
        t.reply("0000FF00FF00 0000FF0CF4 D54B 01 01 0004 08 04 D7A89B9E 16 00");
        t.reply("0000FF00FF00 0000FF03FD D55300 D8 00"); // InRelease ok
        TagInfo tag = new Pn532(t).readTag(100).orElseThrow();
        assertEquals("D7A89B9E", tag.uidHex());
        assertEquals(0x0004, tag.atqa());
        assertEquals(0x08, tag.sak());
        assertEquals("MIFARE Classic 1K", tag.typeName());
    }

    @Test
    void noTagIsEmpty() throws Exception {
        FakeTransport t = new FakeTransport();
        t.reply("0000FF00FF00 0000FF03FD D54B00 E0 00");
        assertTrue(new Pn532(t).readTag(100).isEmpty());
    }

    @Test
    void parsesCapturedFirmware() throws Exception {
        FakeTransport t = new FakeTransport();
        t.reply("0000FF00FF00 0000FF06FA D503 32 01 06 07 E8 00");
        Pn532.Firmware fw = new Pn532(t).getFirmwareVersion();
        assertEquals(0x32, fw.ic());
        assertEquals(1, fw.version());
        assertEquals(6, fw.revision());
    }

    @Test
    void silentReaderThrows() {
        assertThrows(
                Pn532.Pn532Exception.class,
                () -> new Pn532(new FakeTransport()).getFirmwareVersion());
    }
}
