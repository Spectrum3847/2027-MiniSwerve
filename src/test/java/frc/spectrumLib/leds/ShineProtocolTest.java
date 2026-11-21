package frc.spectrumLib.leds;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

/** Frame layouts against the firmware's proto.h, byte for byte. */
class ShineProtocolTest {
    @Test
    void animationFrameIsAnimFlagsStartLengthSpeedBrightness() {
        byte[] out = new byte[8];
        LedPattern p =
                new LedPattern(
                        ShineAnimation.LARSON, LedColor.PURPLE, LedColor.BLACK, 200, 3, true);
        ShineProtocol.packAnimation(out, p, 300, 513, 255);
        // anim 4, reverse flag, start 300 = 0x012C, length 513 = 0x0201, speed 200, brightness 255
        assertArrayEquals(new byte[] {4, 1, 0x2C, 0x01, 0x01, 0x02, (byte) 200, (byte) 255}, out);
    }

    @Test
    void colorFrameIsColor1WithWhiteThenColor2ThenParam() {
        byte[] out = new byte[8];
        LedPattern p =
                new LedPattern(
                        ShineAnimation.WAVE,
                        new LedColor(130, 0, 255, 7),
                        new LedColor(255, 254, 253),
                        0,
                        16,
                        false);
        ShineProtocol.packColors(out, p);
        assertArrayEquals(
                new byte[] {(byte) 130, 0, (byte) 255, 7, (byte) 255, (byte) 254, (byte) 253, 16},
                out);
    }

    @Test
    void configFrameIsKeyThenValueInTheUpperFourBytes() {
        byte[] out = {9, 9, 9, 9, 9, 9, 9, 9};
        ShineProtocol.packConfig(out, ShineProtocol.KEY_COUNT_A, 16);
        assertArrayEquals(new byte[] {3, 0, 0, 0, 16, 0, 0, 0}, out);
    }

    @Test
    void timeFrameIsLittleEndianMicroseconds() {
        byte[] out = new byte[8];
        ShineProtocol.packTime(out, 0x0102030405060708L);
        assertArrayEquals(new byte[] {8, 7, 6, 5, 4, 3, 2, 1}, out);
    }

    @Test
    void readsUnsignedLittleEndian() {
        byte[] d = {
            (byte) 0xFF, (byte) 0xFF, 0x34, 0x12, (byte) 0xEF, (byte) 0xBE, (byte) 0xAD, (byte) 0xDE
        };
        assertEquals(0xFFFF, ShineProtocol.u16(d, 0));
        assertEquals(0x1234, ShineProtocol.u16(d, 2));
        assertEquals(0xDEADBEEFL, ShineProtocol.u32(d, 4));
    }

    @Test
    void animationIdsMatchTheFirmwareOrder() {
        // anim.h: the enum order is the wire format.
        assertEquals(0, ShineAnimation.OFF.id);
        assertEquals(8, ShineAnimation.STROBE.id);
        assertEquals(14, ShineAnimation.WAVE.id);
        assertEquals(23, ShineAnimation.CONVERGE.id);
        for (ShineAnimation a : ShineAnimation.values()) {
            assertEquals(a.ordinal(), a.id, a.name());
        }
    }

    @Test
    void namesFaultBits() {
        assertEquals("", ShineProtocol.faultNames(0));
        assertEquals("BatteryLow OverTemp", ShineProtocol.faultNames(1 | 1 << 5));
        assertEquals(
                "RailSag OverCurrent OverTemp HeatBudget NoCrystal",
                ShineProtocol.faultNames(ShineProtocol.FAULTS_BOARD));
    }
}
