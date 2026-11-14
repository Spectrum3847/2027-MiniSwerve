package frc.spectrumLib.hardware;

import com.ctre.phoenix6.StatusCode;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * The one place blocking Phoenix calls run while the robot is up: config applies, {@code
 * setPosition}, {@code setUpdateFrequency} and the like can each wait ~100 ms on the CAN bus, so
 * the main loop and the 250 Hz swerve thread never make them. Calls queue on a single daemon
 * thread, in order, with retries.
 *
 * <p>Each call has a name; its latest result is kept so the main loop can log it (for example as a
 * health input) and act on it. Boot-time configuration may still block directly, before the robot
 * loop starts.
 *
 * <p>Index: {@link #run} | {@link #result} | {@link #pending} | {@link #logResults}
 */
public final class PhoenixBackground {
    private PhoenixBackground() {}

    /** Tries a call gets unless told otherwise. */
    public static final int TRIES = 5;

    private static final ExecutorService THREAD =
            Executors.newSingleThreadExecutor(
                    r -> {
                        Thread t =
                                new Thread(
                                        () -> {
                                            // Never real-time: these calls wait on the bus and
                                            // must not compete with the robot loop or Phoenix.
                                            normalPriority();
                                            r.run();
                                        },
                                        "PhoenixBackground");
                        t.setDaemon(true);
                        return t;
                    });

    private static final Map<String, StatusCode> RESULTS = new ConcurrentHashMap<>();
    private static final Map<String, Integer> PENDING = new ConcurrentHashMap<>();

    /** The last exception a call (or its onDone) threw, by name, for logs. */
    private static final Map<String, String> THROWN = new ConcurrentHashMap<>();

    /** The last exception the call {@code name} threw, or "" if none has. */
    public static String thrown(String name) {
        return THROWN.getOrDefault(name, "");
    }

    /**
     * Queues a blocking Phoenix call. It is tried up to five times until it returns OK.
     *
     * @param name what the call is, for {@link #result} and logs (e.g. "Swerve/DriveBrake")
     * @param call the blocking call
     * @param onDone run on the background thread with whether it finally succeeded, or null
     */
    public static void run(
            String name, Supplier<StatusCode> call, java.util.function.Consumer<Boolean> onDone) {
        run(name, call, onDone, TRIES);
    }

    /**
     * {@link #run(String, Supplier, java.util.function.Consumer)} with a number of tries: one for a
     * device that is not answering, so its timeouts do not hold up the calls queued behind it (it
     * is queued again later).
     *
     * @param tries how many times to try until it returns OK (at least 1)
     */
    public static void run(
            String name,
            Supplier<StatusCode> call,
            java.util.function.Consumer<Boolean> onDone,
            int tries) {
        final int limit = Math.max(1, tries);
        PENDING.merge(name, 1, Integer::sum);
        THREAD.execute(
                () -> {
                    StatusCode code = StatusCode.StatusCodeNotInitialized;
                    try {
                        for (int t = 0; t < limit; t++) {
                            try {
                                code = call.get();
                            } catch (RuntimeException | LinkageError e) {
                                // A throwing call is a failed try, not the end of this device:
                                // record it and try again (here, then from the device's poll).
                                code = StatusCode.GeneralError;
                                THROWN.put(name, e.toString());
                            }
                            if (code != null && code.isOK()) {
                                break;
                            }
                        }
                    } finally {
                        // Always finish: a call left pending would never be queued again.
                        if (code == null) {
                            code = StatusCode.GeneralError;
                        }
                        RESULTS.put(name, code);
                        PENDING.computeIfPresent(name, (k, v) -> v > 1 ? v - 1 : null);
                        if (onDone != null) {
                            try {
                                onDone.accept(code.isOK());
                            } catch (RuntimeException e) {
                                THROWN.put(name, "onDone: " + e);
                            }
                        }
                    }
                });
    }

    @SuppressWarnings("deprecation") // WPILib marks it "know what you are doing"
    private static void normalPriority() {
        try {
            org.wpilib.system.Threads.setCurrentThreadPriority(0);
        } catch (RuntimeException e) {
            // Desktop or a platform without priorities: nothing to undo.
        }
    }

    /**
     * Starts the background thread now. Call at boot, before the main thread is made real-time, so
     * the thread is not created from a real-time thread.
     */
    public static void start() {
        THREAD.execute(() -> {});
    }

    /** The last result of the named call, or null if it has not finished yet. */
    public static StatusCode result(String name) {
        return RESULTS.get(name);
    }

    /** Log keys by call name, made once so logging each loop does not build strings. */
    private static final Map<String, String[]> KEYS = new ConcurrentHashMap<>();

    /**
     * Logs every call's latest result and last exception. Main thread only (the Logger is). Output
     * only: logic reads {@link #result}, through its own logged inputs.
     */
    public static void logResults() {
        RESULTS.forEach(
                (name, code) -> {
                    String[] k =
                            KEYS.computeIfAbsent(
                                    name,
                                    n ->
                                            new String[] {
                                                "PhoenixBackground/" + n,
                                                "PhoenixBackground/" + n + "/Thrown"
                                            });
                    // The call's latest StatusCode name, e.g. OK or ConfigFailed.
                    Logger.recordOutput(k[0], code.getName());
                    // The last exception the call threw, or "" if none.
                    Logger.recordOutput(k[1], THROWN.getOrDefault(name, ""));
                });
    }

    /** Whether the named call is queued or running. */
    public static boolean pending(String name) {
        return PENDING.containsKey(name);
    }
}
