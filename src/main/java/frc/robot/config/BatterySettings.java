package frc.robot.config;

import frc.spectrumLib.robotsettings.DoubleSetting;
import frc.spectrumLib.robotsettings.RobotSettings;
import frc.spectrumLib.robotsettings.SettingGroup;

/** The low-battery warning, in {@code deploy/settings/battery.json}. Read at boot. */
public final class BatterySettings {
    private BatterySettings() {}

    private static final SettingGroup G = RobotSettings.group("battery");

    /** Warn when the resting battery voltage is below this (V). */
    public static final DoubleSetting LOW_VOLTS = G.number("lowVolts", 12.3, "V", 10, 13.5);

    /** Seconds disabled before a reading counts as resting (no load sag). */
    public static final DoubleSetting SETTLE_SECONDS = G.number("settleSeconds", 3.0, "s", 0.5, 30);
}
