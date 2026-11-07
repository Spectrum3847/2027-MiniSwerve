package frc.spectrumLib.robotsettings;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.littletonrobotics.junction.networktables.LoggedNetworkString;

/**
 * Publishes settings at {@code <prefix>/<group>/<key>} on NetworkTables through AdvantageKit's
 * LoggedNetwork inputs, so the dashboard's requests are in the log too.
 */
public final class NetworkTablesTransport implements SettingTransport {
    private final String prefix;

    /**
     * @param prefix NT path the settings go under, e.g. {@code /RobotSettings}
     */
    public NetworkTablesTransport(String prefix) {
        this.prefix = prefix;
    }

    @Override
    public LiveValue publish(Setting<?> setting) {
        String key = prefix + "/" + setting.path();
        switch (setting.type()) {
            case NUMBER:
            case INTEGER:
                {
                    var n = new LoggedNetworkNumber(key, ((Number) setting.get()).doubleValue());
                    return LiveValue.of(n::get, v -> n.set(((Number) v).doubleValue()));
                }
            case BOOLEAN:
                {
                    var b = new LoggedNetworkBoolean(key, (Boolean) setting.get());
                    return LiveValue.of(b::get, v -> b.set((Boolean) v));
                }
            case STRING:
            default:
                {
                    var s = new LoggedNetworkString(key, setting.get().toString());
                    return LiveValue.of(s::get, v -> s.set(v.toString()));
                }
        }
    }

    @Override
    public LiveValue button(String name) {
        var b = new LoggedNetworkBoolean(prefix + "/" + name, false);
        return LiveValue.of(b::get, v -> b.set((Boolean) v));
    }
}
