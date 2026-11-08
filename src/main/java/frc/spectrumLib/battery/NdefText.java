package frc.spectrumLib.battery;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.nio.charset.StandardCharsets;

/**
 * Finds the battery name on an NTAG21x: the first NDEF Text record ("T"), as written by phone apps
 * such as NFC Tools. Other records, like the team URL, are skipped, so the name and the URL can be
 * in either order.
 *
 * <p>NTAG layout (NFC Forum Type 2): page 3 is the capability container ({@code E1 ..} when NDEF
 * formatted, byte 2 = data area size / 8), user data starts at page 4 as TLVs. The NDEF message is
 * TLV type {@code 03}; {@code 00} is padding and {@code FE} ends the data.
 */
final class NdefText {

    /** Reads 16 bytes (four pages) starting at a page. */
    interface PageReader {
        byte[] read16(int page) throws IOException;
    }

    private static final int CC_PAGE = 3;
    private static final int TLV_NULL = 0x00;
    private static final int TLV_NDEF = 0x03;
    private static final int TLV_TERMINATOR = 0xFE;
    private static final int TNF_WELL_KNOWN = 0x01;

    private NdefText() {}

    /**
     * @return the first Text record's text, or null if the tag isn't NDEF formatted or has none
     */
    static String read(PageReader reader) throws IOException {
        byte[] first = reader.read16(CC_PAGE);
        if ((first[0] & 0xFF) != 0xE1) return null;
        int dataSize = (first[2] & 0xFF) * 8;
        Data data = new Data(reader, dataSize);
        data.append(first, 4, 12); // pages 4-6 arrived with the CC

        int pos = 0;
        while (pos < dataSize) {
            if (!data.ensure(pos + 1)) return null;
            int type = data.at(pos);
            if (type == TLV_NULL) {
                pos++;
                continue;
            }
            if (type == TLV_TERMINATOR) return null;
            if (!data.ensure(pos + 2)) return null;
            int len = data.at(pos + 1);
            int header = 2;
            if (len == 0xFF) {
                if (!data.ensure(pos + 4)) return null;
                len = (data.at(pos + 2) << 8) | data.at(pos + 3);
                header = 4;
            }
            if (type == TLV_NDEF) {
                if (!data.ensure(pos + header + len)) return null;
                return firstText(data.bytes(), pos + header, len);
            }
            pos += header + len;
        }
        return null;
    }

    /** Walks the records of one NDEF message; null if none is a Text record or it's malformed. */
    static String firstText(byte[] b, int off, int len) {
        int p = off;
        int end = off + len;
        try {
            while (p < end) {
                int h = b[p++] & 0xFF;
                boolean shortRecord = (h & 0x10) != 0;
                boolean hasId = (h & 0x08) != 0;
                int tnf = h & 0x07;
                int typeLen = b[p++] & 0xFF;
                int payloadLen;
                if (shortRecord) {
                    payloadLen = b[p++] & 0xFF;
                } else {
                    payloadLen =
                            ((b[p] & 0xFF) << 24)
                                    | ((b[p + 1] & 0xFF) << 16)
                                    | ((b[p + 2] & 0xFF) << 8)
                                    | (b[p + 3] & 0xFF);
                    p += 4;
                }
                int idLen = hasId ? b[p++] & 0xFF : 0;
                int typeOff = p;
                p += typeLen + idLen;
                int payloadOff = p;
                p += payloadLen;
                if (p > end) return null;
                if (tnf == TNF_WELL_KNOWN && typeLen == 1 && b[typeOff] == 'T' && payloadLen > 0) {
                    // Status byte: bit 7 = UTF-16, bits 0-5 = language code length ("en").
                    int status = b[payloadOff] & 0xFF;
                    int langLen = status & 0x3F;
                    int textOff = payloadOff + 1 + langLen;
                    int textLen = payloadLen - 1 - langLen;
                    if (textLen < 0) return null;
                    return new String(
                                    b,
                                    textOff,
                                    textLen,
                                    (status & 0x80) != 0
                                            ? StandardCharsets.UTF_16
                                            : StandardCharsets.UTF_8)
                            .strip();
                }
                if ((h & 0x40) != 0) break; // ME: last record
            }
        } catch (ArrayIndexOutOfBoundsException e) {
            return null;
        }
        return null;
    }

    /** User-data bytes from page 4 on, read from the tag only as far as the parser needs. */
    private static final class Data {
        private final PageReader reader;
        private final int limit;
        private final ByteArrayOutputStream buf = new ByteArrayOutputStream();
        private byte[] cache = new byte[0];

        Data(PageReader reader, int limit) {
            this.reader = reader;
            this.limit = limit;
        }

        void append(byte[] b, int off, int len) {
            buf.write(b, off, Math.min(len, limit - buf.size()));
            cache = buf.toByteArray();
        }

        /**
         * Reads more pages until {@code n} bytes are available; false if the data area is shorter.
         */
        boolean ensure(int n) throws IOException {
            if (n > limit) return false;
            while (cache.length < n) {
                append(reader.read16(4 + cache.length / 4), 0, 16);
            }
            return true;
        }

        int at(int i) {
            return cache[i] & 0xFF;
        }

        byte[] bytes() {
            return cache;
        }
    }
}
