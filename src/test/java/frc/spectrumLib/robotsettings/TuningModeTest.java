package frc.spectrumLib.robotsettings;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

class TuningModeTest {
    @AfterEach
    void off() {
        TuningMode.set(false);
    }

    /** Records what was published; the dashboard always asks for 99. */
    private static final class RecordingTransport implements SettingTransport {
        final List<String> published = new ArrayList<>();

        @Override
        public LiveValue publish(Setting<?> setting) {
            published.add(setting.path());
            return LiveValue.of(() -> 99.0, v -> {});
        }

        @Override
        public LiveValue button(String name) {
            published.add(name);
            return LiveValue.of(() -> false, v -> {});
        }
    }

    @Test
    void offByDefault() {
        assertFalse(TuningMode.enabled());
    }

    @Test
    void settingsIgnoreTheTransportWhenOff() {
        DoubleSetting p = RobotSettings.group("tuningModeTest").number("off", 1.0, "");
        var t = new RecordingTransport();
        RobotSettings.start(null, null, null, t);
        assertTrue(t.published.isEmpty(), "nothing published for tuning");
        assertFalse(RobotSettings.liveTuning());
        assertEquals(1.0, p.get());
    }
}
