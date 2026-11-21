package frc.spectrumLib.leds;

/**
 * The SpectrumShine board's animations, with the ids its firmware uses (SpectrumShine repo, {@code
 * firmware/src/anim.h}; the order there is the wire format, so add new ones at the end in both
 * places). The first ten match the CTRE CANdle's set.
 */
public enum ShineAnimation {
    /** The slot draws nothing. */
    OFF(0),
    /** Color 1. */
    SOLID(1),
    /** Color 1 wipes on, then off, along the range. */
    COLOR_FLOW(2),
    /** Fire: speed is sparking, param is cooling. */
    FIRE(3),
    /** A color 1 eye bouncing end to end; param is the eye size (0 = 2 LEDs each side). */
    LARSON(4),
    /** Moving rainbow. */
    RAINBOW(5),
    /** The whole range fades through the hue wheel. */
    RGB_FADE(6),
    /** Color 1 breathes in and out; one breath takes 127.5 / (speed + 1) seconds. */
    SINGLE_FADE(7),
    /** Color 1 on and off; a full cycle is 20 + (255 - speed) x 4 ms. */
    STROBE(8),
    /** Random LEDs light to color 1 and fade; param is the percent lit each second. */
    TWINKLE(9),
    /** All lit with color 1, random LEDs go dark. */
    TWINKLE_OFF(10),
    /** Color 1 on every third LED, marching, color 2 between. */
    THEATER(11),
    /** A color 1 head with a fading tail of param LEDs, wrapping. */
    COMET(12),
    /** A still blend from color 1 to color 2. */
    GRADIENT(13),
    /**
     * A sine between color 1 and color 2 moving along the range; param is the wavelength in LEDs (0
     * = the range), and it passes (speed + 1) / 256 times a second.
     */
    WAVE(14),
    /** Color 2 background with brief color 1 sparkles. */
    SPARKLE(15),
    /** The two halves swap color 1 and color 2. */
    ALTERNATE(16),
    /** A double pulse of color 1; one beat takes 400 + (255 - speed) x 6 ms. */
    HEARTBEAT(17),
    /** The first param / 255 of the range in color 1, the rest in color 2. */
    PROGRESS(18),
    /** Two eyes (color 1, color 2) from the ends, crossing. */
    CHASE_BOUNCE(19),
    /** A slow two-sine hue field. */
    PLASMA(20),
    /** Bursts that spread and fade. */
    FIREWORKS(21),
    /** Slow drifting bands blending color 1 and color 2. */
    AURORA(22),
    /** Color 1 sweeps in from both ends to the middle over color 2. */
    CONVERGE(23);

    /** The id on the wire. */
    public final int id;

    ShineAnimation(int id) {
        this.id = id;
    }
}
