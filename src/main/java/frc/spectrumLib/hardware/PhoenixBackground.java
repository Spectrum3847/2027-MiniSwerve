package frc.spectrumLib.hardware;

import com.ctre.phoenix6.StatusCode;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.function.Supplier;

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
 * <p>Index: {@link #run} | {@link #result} | {@link #pending}
 */
public final class PhoenixBackground {
    private PhoenixBackground() {}

    private static final int TRIES = 5;

    private static final ExecutorService THREAD =
            Executors.newSingleThreadExecutor(
                    r -> {
                        Thread t = new Thread(r, "PhoenixBackground");
                        t.setDaemon(true);
                        return t;
                    });

    private static final Map<String, StatusCode> RESULTS = new ConcurrentHashMap<>();
    private static final Map<String, Boolean> PENDING = new ConcurrentHashMap<>();

    /**
     * Queues a blocking Phoenix call. It is tried up to five times until it returns OK.
     *
     * @param name what the call is, for {@link #result} and logs (e.g. "Swerve/DriveBrake")
     * @param call the blocking call
     * @param onDone run on the background thread with whether it finally succeeded, or null
     */
    public static void run(
            String name, Supplier<StatusCode> call, java.util.function.Consumer<Boolean> onDone) {
        PENDING.put(name, true);
        THREAD.execute(
                () -> {
                    StatusCode code = StatusCode.StatusCodeNotInitialized;
                    for (int t = 0; t < TRIES; t++) {
                        code = call.get();
                        if (code.isOK()) {
                            break;
                        }
                    }
                    RESULTS.put(name, code);
                    PENDING.remove(name);
                    if (onDone != null) {
                        onDone.accept(code.isOK());
                    }
                });
    }

    /** The last result of the named call, or null if it has not finished yet. */
    public static StatusCode result(String name) {
        return RESULTS.get(name);
    }

    /** Whether the named call is queued or running. */
    public static boolean pending(String name) {
        return PENDING.containsKey(name);
    }
}
