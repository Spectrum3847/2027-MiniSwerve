package frc.spectrumLib.robotsettings;

import java.util.function.Consumer;

/**
 * Declares the settings of one group. The group name is the file name: group {@code cameras} reads
 * {@code settings/cameras.json}. Get one from {@link RobotSettings#group(String)}.
 *
 * <pre>{@code
 * private static final SettingGroup CAMERAS = RobotSettings.group("cameras");
 * public static final DoubleSetting RESEND_SECONDS =
 *         CAMERAS.number("settingsResendPeriodSeconds", 2.0, "s", 0.5, 30);
 * }</pre>
 */
public final class SettingGroup {
    private final String name;
    private final Consumer<Setting<?>> registrar;

    SettingGroup(String name, Consumer<Setting<?>> registrar) {
        this.name = name;
        this.registrar = registrar;
    }

    public String name() {
        return name;
    }

    /** A number with no bounds beyond being finite. */
    public DoubleSetting number(String key, double defaultValue, String units) {
        return number(key, defaultValue, units, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
    }

    /** A number within {@code [min, max]}; a file or dashboard value outside it is refused. */
    public DoubleSetting number(
            String key, double defaultValue, String units, double min, double max) {
        return add(new DoubleSetting(name, key, defaultValue, units, min, max));
    }

    public IntSetting integer(String key, int defaultValue, String units) {
        return integer(key, defaultValue, units, Integer.MIN_VALUE, Integer.MAX_VALUE);
    }

    /** A whole number within {@code [min, max]}. */
    public IntSetting integer(String key, int defaultValue, String units, int min, int max) {
        return add(new IntSetting(name, key, defaultValue, units, min, max));
    }

    public BooleanSetting bool(String key, boolean defaultValue) {
        return add(new BooleanSetting(name, key, defaultValue));
    }

    public StringSetting string(String key, String defaultValue) {
        return add(new StringSetting(name, key, defaultValue));
    }

    private <P extends Setting<?>> P add(P setting) {
        registrar.accept(setting);
        return setting;
    }
}
