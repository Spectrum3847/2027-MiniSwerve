package frc.spectrumLib.leds;

/**
 * What one slot of a SpectrumShine board shows: an animation, its two colors and its knobs. Make
 * them once as constants; the per-loop path only compares them.
 *
 * @param animation which animation
 * @param color1 the main color
 * @param color2 the second color, for animations that use one (see {@link ShineAnimation})
 * @param speed 0..255, about 1..256 LEDs per second for moving patterns
 * @param param the animation's own knob (eye size, tail length, wavelength...), 0 for its default
 * @param reverse run the pattern from the far end of the range
 */
public record LedPattern(
        ShineAnimation animation,
        LedColor color1,
        LedColor color2,
        int speed,
        int param,
        boolean reverse) {
    public LedPattern {
        speed = Math.max(0, Math.min(255, speed));
        param = Math.max(0, Math.min(255, param));
    }

    /** The slot draws nothing, so lower slots show through. */
    public static final LedPattern OFF =
            new LedPattern(ShineAnimation.OFF, LedColor.BLACK, LedColor.BLACK, 0, 0, false);

    /** One color, not moving. */
    public static LedPattern solid(LedColor color) {
        return new LedPattern(ShineAnimation.SOLID, color, LedColor.BLACK, 0, 0, false);
    }

    /** An animation of one color at a speed, with the animation's default knob. */
    public static LedPattern of(ShineAnimation animation, LedColor color, int speed) {
        return new LedPattern(animation, color, LedColor.BLACK, speed, 0, false);
    }

    /** An animation of two colors at a speed, with its knob. */
    public static LedPattern of(
            ShineAnimation animation, LedColor color1, LedColor color2, int speed, int param) {
        return new LedPattern(animation, color1, color2, speed, param, false);
    }
}
