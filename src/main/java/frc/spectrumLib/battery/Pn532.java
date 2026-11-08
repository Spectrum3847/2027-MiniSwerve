package frc.spectrumLib.battery;

import java.io.IOException;
import java.util.Arrays;
import java.util.Optional;

/**
 * Minimal PN532 driver for HSU (UART, 115200 8N1), the mode the common red/blue PN532 boards ship
 * in when wired through a CH340 USB-serial adapter.
 *
 * <p>The PN532 is command/response: it never reports a tag on its own. A tag that was already on
 * the antenna when the reader powered up is found by the first {@link #readTag} after {@link
 * #begin}, the same as one placed later. That is the property the battery logger relies on.
 *
 * <p>Every command is sent behind the HSU wake-up preamble, so the driver recovers without a fresh
 * {@link #begin} if the reader browns out and comes back in its power-on low-VBAT state.
 *
 * <p>Frame format (PN532 User Manual section 6.2): {@code 00 00 FF LEN LCS TFI DATA... DCS 00}, TFI
 * {@code D4} host to PN532, {@code D5} back. Each command is answered first by an ACK frame {@code
 * 00 00 FF 00 FF 00}, then the response.
 */
public class Pn532 implements AutoCloseable {

    /** Firmware version reply. {@code ic} is 0x32 for a PN532. */
    public record Firmware(int ic, int version, int revision, int support) {
        @Override
        public String toString() {
            return String.format(
                    "IC 0x%02X, firmware %d.%d, support 0x%02X", ic, version, revision, support);
        }
    }

    public static class Pn532Exception extends IOException {
        public Pn532Exception(String message) {
            super(message);
        }
    }

    private static final int CMD_GET_FIRMWARE_VERSION = 0x02;
    private static final int CMD_SAM_CONFIGURATION = 0x14;
    private static final int CMD_RF_CONFIGURATION = 0x32;
    private static final int CMD_IN_DATA_EXCHANGE = 0x40;
    private static final int CMD_IN_LIST_PASSIVE_TARGET = 0x4A;
    private static final int CMD_IN_RELEASE = 0x52;

    /** NTAG21x READ: returns 16 bytes, four 4-byte pages starting at the given page. */
    private static final int NTAG_READ = 0x30;

    private static final int HOST_TO_PN532 = 0xD4;
    private static final int PN532_TO_HOST = 0xD5;

    /** HSU wake-up: 0x55 0x55 then enough zeros for the PN532 to leave power-down (UM 7.2.11). */
    private static final byte[] WAKE_PREAMBLE = {
        0x55, 0x55, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    };

    private static final byte[] ACK_FRAME = {0x00, 0x00, (byte) 0xFF, 0x00, (byte) 0xFF, 0x00};

    private static final int ACK_TIMEOUT_MS = 100;

    /** Sentinel returned by {@link #readFrame} for an ACK. */
    private static final byte[] ACK = new byte[0];

    private final Pn532Transport transport;
    private final byte[] one = new byte[1];

    public Pn532(Pn532Transport transport) {
        this.transport = transport;
    }

    /**
     * Wakes the reader and puts it in normal mode with a short tag search, so {@link #readTag}
     * returns promptly when no tag is present.
     *
     * @param activationRetries passive activation retries per {@link #readTag}; 0xFF retries
     *     forever. 1 or 2 keeps an empty read to a few tens of milliseconds.
     */
    public Firmware begin(int activationRetries) throws IOException {
        Firmware fw = getFirmwareVersion();
        // Normal mode, virtual-card timeout 1 s (0x14 * 50 ms), IRQ pin used.
        command(CMD_SAM_CONFIGURATION, new byte[] {0x01, 0x14, 0x01}, 200);
        // CfgItem 5: MxRtyATR, MxRtyPSL, MxRtyPassiveActivation.
        command(
                CMD_RF_CONFIGURATION,
                new byte[] {0x05, (byte) 0xFF, 0x01, (byte) activationRetries},
                200);
        return fw;
    }

    public Firmware getFirmwareVersion() throws IOException {
        byte[] r = command(CMD_GET_FIRMWARE_VERSION, new byte[0], 200);
        if (r.length < 4) throw new Pn532Exception("short firmware reply: " + hex(r));
        return new Firmware(r[0] & 0xFF, r[1] & 0xFF, r[2] & 0xFF, r[3] & 0xFF);
    }

    /**
     * Looks for one ISO14443A (MIFARE / NTAG) tag.
     *
     * @return the tag, or empty when none is on the antenna
     */
    public Optional<TagInfo> readTag(int timeoutMs) throws IOException {
        return readTag(timeoutMs, false);
    }

    /**
     * Looks for one ISO14443A tag and, if {@code readName}, the battery name written on an NTAG as
     * an NDEF text record (what phone apps like NFC Tools write). A tag with no readable name is
     * still returned, with a null name: the UID is the battery's key, the name is a label.
     *
     * @return the tag, or empty when none is on the antenna
     */
    public Optional<TagInfo> readTag(int timeoutMs, boolean readName) throws IOException {
        // MaxTg 1, BrTy 0 = 106 kbps type A.
        byte[] r = command(CMD_IN_LIST_PASSIVE_TARGET, new byte[] {0x01, 0x00}, timeoutMs);
        if (r.length < 1 || r[0] == 0) return Optional.empty();
        // NbTg, Tg, SENS_RES(2), SEL_RES, NFCIDLength, NFCID...
        if (r.length < 6) throw new Pn532Exception("short target reply: " + hex(r));
        int atqa = ((r[2] & 0xFF) << 8) | (r[3] & 0xFF);
        int sak = r[4] & 0xFF;
        int uidLen = r[5] & 0xFF;
        if (r.length < 6 + uidLen) throw new Pn532Exception("truncated UID: " + hex(r));
        TagInfo tag = new TagInfo(Arrays.copyOfRange(r, 6, 6 + uidLen), atqa, sak, null);
        if (readName && tag.isNtag()) {
            // Read while the tag is still selected, before InRelease.
            try {
                String name = NdefText.read(this::ntagRead);
                if (name != null && !name.isEmpty()) tag = tag.withName(name);
            } catch (Pn532Exception e) {
                // Tag moved mid-read or isn't NDEF formatted: report it without a name.
            }
        }
        // Release it so the next read re-activates from scratch instead of finding it held.
        command(CMD_IN_RELEASE, new byte[] {0x00}, 200);
        return Optional.of(tag);
    }

    /** NTAG21x READ through InDataExchange on target 1: 16 bytes from {@code page} onward. */
    byte[] ntagRead(int page) throws IOException {
        byte[] r = command(CMD_IN_DATA_EXCHANGE, new byte[] {0x01, NTAG_READ, (byte) page}, 200);
        // Status byte, then the tag's 16 data bytes.
        if (r.length < 1 || r[0] != 0) {
            throw new Pn532Exception(
                    String.format("NTAG read of page %d failed: %s", page, hex(r)));
        }
        if (r.length < 17) throw new Pn532Exception("short NTAG read: " + hex(r));
        return Arrays.copyOfRange(r, 1, 17);
    }

    /**
     * Sends one command and returns the response data after the response code.
     *
     * @throws Pn532Exception on no ACK, a timeout, a bad checksum, or an error frame
     */
    byte[] command(int cmd, byte[] params, int timeoutMs) throws IOException {
        transport.discardInput();
        transport.write(concat(WAKE_PREAMBLE, buildFrame(cmd, params)));

        byte[] ack = readFrame(ACK_TIMEOUT_MS);
        if (ack != ACK) {
            throw new Pn532Exception(
                    ack == null
                            ? String.format(
                                    "no ACK for command 0x%02X (reader unplugged or wrong port?)",
                                    cmd)
                            : String.format("expected ACK for 0x%02X, got %s", cmd, hex(ack)));
        }

        byte[] resp = readFrame(timeoutMs);
        if (resp == null) {
            // Abort the command so the PN532 is idle for the next one.
            transport.write(ACK_FRAME);
            throw new Pn532Exception(String.format("timeout waiting for reply to 0x%02X", cmd));
        }
        if (resp == ACK || resp.length < 2) {
            throw new Pn532Exception("unexpected frame: " + hex(resp));
        }
        if ((resp[0] & 0xFF) == 0x7F) throw new Pn532Exception("PN532 error frame");
        if ((resp[0] & 0xFF) != PN532_TO_HOST || (resp[1] & 0xFF) != cmd + 1) {
            throw new Pn532Exception(
                    String.format("reply to 0x%02X has wrong header: %s", cmd, hex(resp)));
        }
        return Arrays.copyOfRange(resp, 2, resp.length);
    }

    static byte[] buildFrame(int cmd, byte[] params) {
        int len = params.length + 2; // TFI + CMD + params
        byte[] f = new byte[len + 7];
        f[0] = 0x00;
        f[1] = 0x00;
        f[2] = (byte) 0xFF;
        f[3] = (byte) len;
        f[4] = (byte) (0x100 - len);
        f[5] = (byte) HOST_TO_PN532;
        f[6] = (byte) cmd;
        System.arraycopy(params, 0, f, 7, params.length);
        int sum = HOST_TO_PN532 + cmd;
        for (byte b : params) sum += b & 0xFF;
        f[7 + params.length] = (byte) (0x100 - (sum & 0xFF));
        f[8 + params.length] = 0x00;
        return f;
    }

    /**
     * Reads the next frame, skipping any bytes before a start code.
     *
     * @return the frame body (TFI onward, without DCS), {@link #ACK} for an ACK frame, or null on
     *     timeout
     */
    private byte[] readFrame(int timeoutMs) throws IOException {
        long deadline = System.nanoTime() + timeoutMs * 1_000_000L;
        int prev = -1;
        while (true) {
            int b = readByte(deadline);
            if (b < 0) return null;
            if (!(prev == 0x00 && b == 0xFF)) {
                prev = b;
                continue;
            }
            prev = -1;
            int len = readByte(deadline);
            int lcs = readByte(deadline);
            if (len < 0 || lcs < 0) return null;
            if (len == 0x00 && lcs == 0xFF) return ACK;
            if (((len + lcs) & 0xFF) != 0) continue; // not a real header, keep scanning
            byte[] body = new byte[len];
            int sum = 0;
            for (int i = 0; i < len; i++) {
                int d = readByte(deadline);
                if (d < 0) return null;
                body[i] = (byte) d;
                sum += d;
            }
            int dcs = readByte(deadline);
            if (dcs < 0) return null;
            if (((sum + dcs) & 0xFF) != 0)
                throw new Pn532Exception("bad data checksum: " + hex(body));
            return body;
        }
    }

    private int readByte(long deadlineNanos) throws IOException {
        long remainingMs = (deadlineNanos - System.nanoTime()) / 1_000_000L;
        if (remainingMs <= 0) return -1;
        int n = transport.read(one, (int) Math.min(remainingMs, Integer.MAX_VALUE));
        return n == 1 ? one[0] & 0xFF : -1;
    }

    private static byte[] concat(byte[] a, byte[] b) {
        byte[] r = Arrays.copyOf(a, a.length + b.length);
        System.arraycopy(b, 0, r, a.length, b.length);
        return r;
    }

    static String hex(byte[] b) {
        if (b == null) return "null";
        StringBuilder sb = new StringBuilder();
        for (byte x : b) sb.append(String.format("%02X ", x));
        return sb.toString().trim();
    }

    @Override
    public void close() {
        transport.close();
    }
}
