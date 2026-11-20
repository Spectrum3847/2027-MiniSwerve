package frc.spectrumLib.leds;

/**
 * One LED color as the SpectrumShine board takes it: red, green, blue and (RGBW strips only) white,
 * each 0..255 in sRGB terms. The board applies gamma at its output, so mid values look right.
 */
public record LedColor(int r, int g, int b, int w) {
    public LedColor {
        r = clamp(r);
        g = clamp(g);
        b = clamp(b);
        w = clamp(w);
    }

    /** A color with no white channel. */
    public LedColor(int r, int g, int b) {
        this(r, g, b, 0);
    }

    public static final LedColor BLACK = new LedColor(0, 0, 0);
    public static final LedColor WHITE = new LedColor(255, 255, 255);

    /** Spectrum purple as it reads on WS2812 / SK6812 strips. */
    public static final LedColor PURPLE = new LedColor(130, 0, 255);

    /** Kept for real faults and e-stop: on a robot, red reads as broken. */
    public static final LedColor RED = new LedColor(255, 0, 0);

    /** Orange-red: low battery (teams 6328 and 3061 use it too, apart from the red of a fault). */
    public static final LedColor ORANGE = new LedColor(255, 60, 0);

    public static final LedColor YELLOW = new LedColor(255, 180, 0);

    private static int clamp(int v) {
        return Math.max(0, Math.min(255, v));
    }
}
