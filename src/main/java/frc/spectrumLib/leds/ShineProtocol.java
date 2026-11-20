package frc.spectrumLib.leds;

/**
 * The SpectrumShine board's CAN protocol: API numbers, frame layouts, fault bits. A copy of the
 * SpectrumShine repo's {@code firmware/src/proto.h} (and the config keys in {@code app.c}); change
 * both together. Frames are classic CAN, 8 bytes, little-endian, on FRC-style 29-bit ids: device
 * type 10 (miscellaneous), manufacturer 8 (team use), which are WPILib {@code CAN}'s defaults.
 *
 * <p>Index: API numbers | config keys | fault bits | {@link #packAnimation} | {@link #packColors} |
 * {@link #packConfig} | {@link #packTime} | {@link #faultNames}
 */
public final class ShineProtocol {
    private ShineProtocol() {}

    /** Every board acts on frames sent to this device id. */
    public static final int BROADCAST_ID = 63;

    /** Slots per board; higher slots draw over lower ones. */
    public static final int SLOTS = 8;

    /** LEDs a board can drive, both outputs together. */
    public static final int MAX_LEDS = 512;

    // ---- robot -> board ----
    /** + slot: animation, flags, start, length, speed, brightness. */
    public static final int API_SLOT_ANIM = 0x010;

    /** + slot: color 1 (r g b w), color 2 (r g b), param. */
    public static final int API_SLOT_COLOR = 0x018;

    /** Sets a config value in RAM (not saved to the board's flash). */
    public static final int API_SET_CONFIG = 0x030;

    /** The robot's log clock (microseconds), about once a second, to {@link #BROADCAST_ID}. */
    public static final int API_TIME_SYNC = 0x040;

    /** Asks for {@link #API_VERSION}. */
    public static final int API_PING = 0x052;

    // ---- board -> robot ----
    /** Faults, sticky faults, what limits current, temperature, output scale, flags. */
    public static final int API_STATUS = 0x100;

    /** Battery mV, 5 V rail mV, measured mA, estimated mA. */
    public static final int API_POWER = 0x101;

    /** Firmware major, minor, 0, CAN id, UID hash. Sent at boot and after a ping. */
    public static final int API_VERSION = 0x105;

    // ---- config keys (app.c) ----
    /** LEDs on output A, the first in the board's index space. */
    public static final int KEY_COUNT_A = 3;

    /** LEDs on output B, which follow A's in the index space. */
    public static final int KEY_COUNT_B = 4;

    /** Animation frame flag: run the pattern from the far end. */
    public static final int FLAG_REVERSE = 0x01;

    // ---- status flags (byte 7 of API_STATUS) ----
    /** The board has the robot's clock. */
    public static final int FLAG_TIME_SYNCED = 0x01;

    /** At least one slot is running an animation. */
    public static final int FLAG_ANY_SLOT = 0x08;

    /** Fault bit names, bit 0 first (proto.h {@code F_*}). */
    private static final String[] FAULTS = {
        "BatteryLow",
        "BatteryDim",
        "BatteryCritical",
        "RailSag",
        "OverCurrent",
        "OverTemp",
        "HeatBudget",
        "NoCrystal",
        "CanError",
        "ConfigReset",
        "WatchdogReset",
        "NoBattery"
    };

    /**
     * Faults that mean the board, its strip or its wiring needs attention: 5 V rail sag, over
     * current, over temperature, heat budget, no crystal. Not the battery ones (the robot has its
     * own battery check) and not the notes about the last reset.
     */
    public static final int FAULTS_BOARD = (1 << 3) | (1 << 4) | (1 << 5) | (1 << 6) | (1 << 7);

    /** Fills an 8-byte animation frame for {@link #API_SLOT_ANIM} + slot. */
    public static void packAnimation(
            byte[] out, LedPattern pattern, int start, int length, int brightness) {
        out[0] = (byte) pattern.animation().id;
        out[1] = (byte) (pattern.reverse() ? FLAG_REVERSE : 0);
        out[2] = (byte) start;
        out[3] = (byte) (start >> 8);
        out[4] = (byte) length;
        out[5] = (byte) (length >> 8);
        out[6] = (byte) pattern.speed();
        out[7] = (byte) Math.max(0, Math.min(255, brightness));
    }

    /** Fills an 8-byte color frame for {@link #API_SLOT_COLOR} + slot. */
    public static void packColors(byte[] out, LedPattern pattern) {
        LedColor a = pattern.color1();
        LedColor b = pattern.color2();
        out[0] = (byte) a.r();
        out[1] = (byte) a.g();
        out[2] = (byte) a.b();
        out[3] = (byte) a.w();
        out[4] = (byte) b.r();
        out[5] = (byte) b.g();
        out[6] = (byte) b.b();
        out[7] = (byte) pattern.param();
    }

    /** Fills an 8-byte frame for {@link #API_SET_CONFIG}. */
    public static void packConfig(byte[] out, int key, int value) {
        out[0] = (byte) key;
        out[1] = 0;
        out[2] = 0;
        out[3] = 0;
        for (int i = 0; i < 4; i++) {
            out[4 + i] = (byte) (value >>> (8 * i));
        }
    }

    /** Fills an 8-byte frame for {@link #API_TIME_SYNC}. */
    public static void packTime(byte[] out, long micros) {
        for (int i = 0; i < 8; i++) {
            out[i] = (byte) (micros >>> (8 * i));
        }
    }

    /** An unsigned 16-bit little-endian value at {@code at}. */
    public static int u16(byte[] data, int at) {
        return (data[at] & 0xFF) | (data[at + 1] & 0xFF) << 8;
    }

    /** An unsigned 32-bit little-endian value at {@code at}. */
    public static long u32(byte[] data, int at) {
        return u16(data, at) | (long) u16(data, at + 2) << 16;
    }

    /** The names of the set fault bits, space-separated, "" for none. Allocates: not per loop. */
    public static String faultNames(int faults) {
        StringBuilder s = new StringBuilder();
        for (int bit = 0; bit < 16; bit++) {
            if ((faults & (1 << bit)) != 0) {
                if (s.length() > 0) {
                    s.append(' ');
                }
                s.append(bit < FAULTS.length ? FAULTS[bit] : "Bit" + bit);
            }
        }
        return s.toString();
    }
}
