package frc.spectrumLib.battery;

import java.util.HexFormat;

/**
 * One ISO14443A tag seen by the PN532.
 *
 * @param uid tag UID (4, 7 or 10 bytes)
 * @param atqa SENS_RES / ATQA, e.g. 0x0004 for MIFARE Classic 1K, 0x0044 for NTAG21x
 * @param sak SEL_RES / SAK, e.g. 0x08 for MIFARE Classic 1K, 0x00 for NTAG21x
 * @param name battery name from the tag's NDEF text record, or null if not read or not written
 */
public record TagInfo(byte[] uid, int atqa, int sak, String name) {

    /** UID as upper-case hex with no separators, e.g. {@code D7A89B9E}. This is the battery key. */
    public String uidHex() {
        return HexFormat.of().withUpperCase().formatHex(uid);
    }

    /** NTAG21x / Ultralight: the tags whose memory {@link Pn532} can read without a key. */
    public boolean isNtag() {
        return sak == 0x00 && uid.length == 7;
    }

    public TagInfo withName(String newName) {
        return new TagInfo(uid, atqa, sak, newName);
    }

    /** Best-guess tag family from ATQA/SAK, for the test printout only. */
    public String typeName() {
        if (sak == 0x08 && uid.length == 4) return "MIFARE Classic 1K";
        if (sak == 0x18) return "MIFARE Classic 4K";
        if (isNtag()) return "NTAG21x / Ultralight";
        if ((sak & 0x20) != 0) return "ISO14443-4 (DESFire etc.)";
        return "unknown";
    }

    @Override
    public String toString() {
        String s = String.format("%s (%s, ATQA %04X, SAK %02X)", uidHex(), typeName(), atqa, sak);
        return name == null ? s : s + " name \"" + name + "\"";
    }
}
