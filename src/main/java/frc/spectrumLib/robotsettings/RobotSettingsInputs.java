package frc.spectrumLib.robotsettings;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * Every setting's applied value and source, as an AdvantageKit input: {@code
 * RobotSettings/Values/<group>/ <key>} and {@code RobotSettings/Sources/<group>/<key>}. Replay
 * restores both, so a replayed match runs on the values the robot ran on, whatever the files on the
 * replaying PC say. A key missing from the log (a log from before the setting existed) keeps the
 * value resolved here.
 */
final class RobotSettingsInputs implements LoggableInputs {
    private final SettingStore store;

    RobotSettingsInputs(SettingStore store) {
        this.store = store;
    }

    @Override
    public void toLog(LogTable table) {
        for (Setting<?> p : store.settings()) {
            String key = "Values/" + p.path();
            switch (p.type()) {
                case NUMBER -> table.put(key, (Double) p.get());
                case INTEGER -> table.put(key, (long) (Integer) p.get());
                case BOOLEAN -> table.put(key, (Boolean) p.get());
                case STRING -> table.put(key, (String) p.get());
            }
            // RobotSettings/Sources/<group>/<key>: which layer each value came from: DEFAULT,
            // DEPLOY,
            // ROBOT, LOCAL or LIVE.
            table.put("Sources/" + p.path(), p.source().name());
        }
    }

    @Override
    public void fromLog(LogTable table) {
        for (Setting<?> p : store.settings()) {
            String key = "Values/" + p.path();
            Object value =
                    switch (p.type()) {
                        case NUMBER -> table.get(key, (Double) p.get());
                        case INTEGER -> (int) table.get(key, (long) (Integer) p.get());
                        case BOOLEAN -> table.get(key, (Boolean) p.get());
                        case STRING -> table.get(key, (String) p.get());
                    };
            SettingSource source;
            try {
                source = SettingSource.valueOf(table.get("Sources/" + p.path(), p.source().name()));
            } catch (IllegalArgumentException e) {
                source = p.source();
            }
            store.restore(p, value, source);
        }
    }
}
