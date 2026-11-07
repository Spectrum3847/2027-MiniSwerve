package frc.spectrumLib.robotsettings;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.HashMap;
import java.util.List;
import org.junit.jupiter.api.Test;

class RobotSettingsChangesTest {
    @Test
    void eachChangedValueIsOneLineWithItsSource() {
        var store = new SettingStore();
        var g = new SettingGroup("Shooter", store::register);
        DoubleSetting rpm = g.number("rpm", 3000, "");
        g.bool("enabled", true);
        var last = new HashMap<String, Object>();

        assertEquals(List.of(), RobotSettings.changes(store, last), "first values are not changes");
        store.restore(rpm, 3200.0, SettingSource.LIVE);
        assertEquals(
                List.of("setting " + rpm.path() + ": 3000.0 -> 3200.0 (LIVE)"),
                RobotSettings.changes(store, last));
        assertEquals(List.of(), RobotSettings.changes(store, last), "unchanged since: nothing");
    }
}
