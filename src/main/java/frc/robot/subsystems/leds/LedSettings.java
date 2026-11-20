package frc.robot.subsystems.leds;

import frc.spectrumLib.robotsettings.BooleanSetting;
import frc.spectrumLib.robotsettings.IntSetting;
import frc.spectrumLib.robotsettings.RobotSettings;
import frc.spectrumLib.robotsettings.SettingGroup;

/** The LED strip, in {@code deploy/settings/leds.json}. Read at boot. */
public final class LedSettings {
    private LedSettings() {}

    private static final SettingGroup G = RobotSettings.group("leds");

    /** LEDs on the strip. MiniSwerve: one strip of 16 across the middle of the robot, facing up. */
    public static final IntSetting COUNT = G.integer("count", 16, "LEDs", 1, 512);

    /**
     * LEDs at each end of the strip that show warnings and faults, leaving the middle for the mode.
     * 0 turns the end markers off.
     */
    public static final IntSetting END_LEDS = G.integer("endLeds", 3, "LEDs", 0, 64);

    /**
     * Brightness, 0..255. The strip faces up at people standing over the robot, so not full: other
     * teams run 10 to 60 %.
     */
    public static final IntSetting BRIGHTNESS = G.integer("brightness", 110, "0-255", 0, 255);

    /** The strip is on the board's output B instead of output A. */
    public static final BooleanSetting ON_OUTPUT_B = G.bool("onOutputB", false);
}
