package frc.spectrumLib.battery;

import static org.junit.jupiter.api.Assertions.*;

import java.io.ByteArrayOutputStream;
import java.nio.charset.StandardCharsets;
import java.util.Arrays;
import org.junit.jupiter.api.Test;

class NdefTextTest {

    /** NTAG215 memory image: 135 pages; READ returns 16 bytes like the real tag. */
    static final class FakeNtag implements NdefText.PageReader {
        final byte[] mem = new byte[135 * 4];
        int reads;

        FakeNtag(byte[] ndefMessage) {
            // CC: NDEF magic, version 1.0, 496-byte data area (0x3E * 8), read/write.
            mem[12] = (byte) 0xE1;
            mem[13] = 0x10;
            mem[14] = 0x3E;
            mem[15] = 0x00;
            byte[] tlv = tlv(ndefMessage);
            System.arraycopy(tlv, 0, mem, 16, tlv.length);
        }

        @Override
        public byte[] read16(int page) {
            reads++;
            return Arrays.copyOfRange(mem, page * 4, page * 4 + 16);
        }
    }

    static byte[] tlv(byte[] msg) {
        ByteArrayOutputStream o = new ByteArrayOutputStream();
        o.write(0x03);
        if (msg.length < 0xFF) {
            o.write(msg.length);
        } else {
            o.write(0xFF);
            o.write(msg.length >> 8);
            o.write(msg.length & 0xFF);
        }
        o.writeBytes(msg);
        o.write(0xFE);
        return o.toByteArray();
    }

    /**
     * Well-known record; short form (1-byte length) when the payload fits, else 4-byte length.
     * {@code first}/{@code last} set the MB/ME flags.
     */
    static byte[] record(char type, byte[] payload, boolean first, boolean last) {
        ByteArrayOutputStream o = new ByteArrayOutputStream();
        boolean shortRecord = payload.length < 256;
        o.write((first ? 0x80 : 0) | (last ? 0x40 : 0) | (shortRecord ? 0x10 : 0) | 0x01);
        o.write(1);
        if (shortRecord) {
            o.write(payload.length);
        } else {
            o.write(payload.length >>> 24);
            o.write(payload.length >>> 16);
            o.write(payload.length >>> 8);
            o.write(payload.length);
        }
        o.write(type);
        o.writeBytes(payload);
        return o.toByteArray();
    }

    static byte[] text(String s) {
        byte[] t = s.getBytes(StandardCharsets.UTF_8);
        byte[] p = new byte[3 + t.length];
        p[0] = 0x02; // UTF-8, language "en"
        p[1] = 'e';
        p[2] = 'n';
        System.arraycopy(t, 0, p, 3, t.length);
        return p;
    }

    static byte[] uri(String afterHttpsWww) {
        byte[] u = afterHttpsWww.getBytes(StandardCharsets.UTF_8);
        byte[] p = new byte[1 + u.length];
        p[0] = 0x02; // "https://www."
        System.arraycopy(u, 0, p, 1, u.length);
        return p;
    }

    static byte[] concat(byte[]... parts) {
        ByteArrayOutputStream o = new ByteArrayOutputStream();
        for (byte[] p : parts) o.writeBytes(p);
        return o.toByteArray();
    }

    @Test
    void readsNameWhenItIsTheOnlyRecord() throws Exception {
        FakeNtag tag = new FakeNtag(record('T', text("2026-1 Aqua"), true, true));
        assertEquals("2026-1 Aqua", NdefText.read(tag));
    }

    /** The planned layout: team URL first, so finders' phones open it; battery name second. */
    @Test
    void skipsUrlRecordBeforeName() throws Exception {
        FakeNtag tag =
                new FakeNtag(
                        concat(
                                record('U', uri("spectrum3847.org"), true, false),
                                record('T', text("2026-1 Aqua"), false, true)));
        assertEquals("2026-1 Aqua", NdefText.read(tag));
    }

    @Test
    void nameBeforeUrlAlsoWorks() throws Exception {
        FakeNtag tag =
                new FakeNtag(
                        concat(
                                record('T', text("2026-12 Magenta"), true, false),
                                record('U', uri("spectrum3847.org"), false, true)));
        assertEquals("2026-12 Magenta", NdefText.read(tag));
    }

    @Test
    void urlOnlyHasNoName() throws Exception {
        assertNull(NdefText.read(new FakeNtag(record('U', uri("spectrum3847.org"), true, true))));
    }

    @Test
    void blankTagHasNoName() throws Exception {
        FakeNtag tag = new FakeNtag(new byte[0]);
        Arrays.fill(tag.mem, 12, tag.mem.length, (byte) 0); // factory-fresh: no CC
        assertNull(NdefText.read(tag));
    }

    @Test
    void readsOnlyAsManyPagesAsNeeded() throws Exception {
        FakeNtag tag =
                new FakeNtag(
                        concat(
                                record('U', uri("spectrum3847.org"), true, false),
                                record('T', text("2026-1 Aqua"), false, true)));
        NdefText.read(tag);
        // ~40 bytes of TLV: the CC read (pages 3-6) plus two more.
        assertTrue(tag.reads <= 3, "reads: " + tag.reads);
    }

    @Test
    void handlesLongTlvLength() throws Exception {
        String longName = "x".repeat(300);
        assertEquals(
                longName, NdefText.read(new FakeNtag(record('T', text(longName), true, true))));
    }

    @Test
    void truncatedMessageIsNull() {
        byte[] msg = record('T', text("2026-1 Aqua"), true, true);
        assertNull(NdefText.firstText(msg, 0, msg.length - 4));
    }
}
