package frc.spectrumLib.robotsettings;

import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * How settings reach a dashboard for live tuning. {@link NetworkTablesTransport} is today's (on
 * AdvantageKit's LoggedNetwork inputs); WPILib's Tunables can replace it without touching any code
 * that declares or reads a setting.
 *
 * <p>Whatever the transport, what it reads is only a request: {@link RobotSettings} applies it on
 * the main thread, only while disabled, and logs the applied value as an input.
 */
public interface SettingTransport {
    /** A published value: what the dashboard last set, and a way to show a value there. */
    interface LiveValue {
        /** The dashboard's value: a Double, Boolean or String, or null if there is none. */
        Object get();

        /** Shows a value on the dashboard, e.g. to undo a refused change. */
        void set(Object value);

        static LiveValue of(Supplier<Object> get, Consumer<Object> set) {
            return new LiveValue() {
                @Override
                public Object get() {
                    return get.get();
                }

                @Override
                public void set(Object value) {
                    set.accept(value);
                }
            };
        }
    }

    /** Publishes a setting with its current value. Called on the main thread. */
    LiveValue publish(Setting<?> setting);

    /** Publishes a button (a Boolean the dashboard sets true; the code sets it back). */
    LiveValue button(String name);
}
