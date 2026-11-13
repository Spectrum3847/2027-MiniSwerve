package frc.spectrumLib.framework;

import frc.spectrumLib.telemetry.Alert;
import java.util.function.Predicate;
import org.littletonrobotics.junction.Logger;

/**
 * One stage of the robot loop (or one optional part of boot), run so that a throw inside it cannot
 * end the robot program (AGENTS.md "Finish the match"): an exception in the battery reader, a
 * camera or a command would otherwise restart the whole robot mid-match. The throw is counted,
 * logged, raised as an alert and printed (once per distinct fault), and the loop carries on with
 * the next stage.
 *
 * <p>Catches {@link Throwable}: a {@code StackOverflowError} or {@code NoClassDefFoundError} in one
 * stage is no more reason to stop driving than an exception.
 *
 * <p>Main thread only. Logged under {@code Stages/<name>/}: {@code Faults}, {@code LastFault}.
 */
public final class LoopStage {
    /** Sim only: makes the named stage throw, to check the robot keeps going. */
    private static volatile Predicate<String> injector;

    private final String name;
    private final String faultsKey;
    private final String lastKey;
    private final Alert alert;
    private long faults;
    private long loggedFaults = -1;
    private String lastFault = "";
    private String printed = "";

    /**
     * @param name the stage, for logs and the alert (e.g. "Vision")
     */
    public LoopStage(String name) {
        this.name = name;
        faultsKey = "Stages/" + name + "/Faults";
        lastKey = "Stages/" + name + "/LastFault";
        alert = new Alert("", Alert.Level.HIGH);
    }

    /** Sim only: stages for which {@code shouldThrow} is true throw instead of running. */
    public static void setFaultInjector(Predicate<String> shouldThrow) {
        injector = shouldThrow;
    }

    /**
     * Runs the stage.
     *
     * @return false if it threw (the rest of the loop still runs)
     */
    public boolean run(Runnable stage) {
        boolean ok = true;
        try {
            Predicate<String> inject = injector;
            if (inject != null && inject.test(name)) {
                throw new IllegalStateException("injected fault in " + name);
            }
            stage.run();
        } catch (Throwable t) {
            ok = false;
            record(t);
        }
        if (faults != loggedFaults) {
            try {
                // Times this stage threw since boot (count).
                Logger.recordOutput(faultsKey, faults);
                // The last thing it threw.
                Logger.recordOutput(lastKey, lastFault);
                loggedFaults = faults;
            } catch (Throwable t) {
                // Logging the fault must not become the fault.
            }
        }
        return ok;
    }

    /** Runs a boot step that makes something; null (with the fault recorded) if it threw. */
    public <T> T make(java.util.function.Supplier<T> maker) {
        try {
            Predicate<String> inject = injector;
            if (inject != null && inject.test(name)) {
                throw new IllegalStateException("injected fault in " + name);
            }
            return maker.get();
        } catch (Throwable t) {
            record(t);
            return null;
        }
    }

    private void record(Throwable t) {
        faults++;
        String what;
        try {
            what = t.toString();
        } catch (Throwable inner) {
            what = t.getClass().getName();
        }
        // Print and reword only when the fault changes: a stage that throws every loop must not
        // flood the console or build strings at 100 Hz.
        if (!what.equals(lastFault)) {
            lastFault = what;
            try {
                alert.setText(name + " failed, robot still running: " + what);
            } catch (Throwable inner) {
                // The alert is a report, not the work.
            }
        }
        if (!what.equals(printed)) {
            printed = what;
            try {
                System.err.println("Stage " + name + " threw (robot keeps running):");
                t.printStackTrace();
            } catch (Throwable inner) {
                // Nothing more to do.
            }
        }
        try {
            alert.set(true);
        } catch (Throwable inner) {
            // As above.
        }
    }

    /** Times this stage threw since boot. */
    public long faults() {
        return faults;
    }

    /** The last thing it threw, or "" if nothing. */
    public String lastFault() {
        return lastFault;
    }
}
