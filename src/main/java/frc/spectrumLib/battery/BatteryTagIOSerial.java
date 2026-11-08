package frc.spectrumLib.battery;

import java.util.Optional;
import java.util.function.Supplier;

/**
 * The battery's NFC tag read through a PN532 reader on USB serial (CH340, {@code /dev/ttyUSB0} on
 * SystemCore), on its own low-priority daemon thread: serial I/O never runs on the main loop or the
 * swerve thread.
 *
 * <p>It reads once at boot (a tag already on the reader at power-up is found by the first read),
 * then every {@link #POLL_MILLIS} while the main loop allows it ({@code mayRead}, i.e. disabled),
 * so a swap in the pits is picked up and a match spends no USB traffic. A missing or failing reader
 * is reported as an input and retried; it never stops anything else. If the serial library's native
 * code cannot load on this machine, it says so once and retries slowly.
 *
 * <p>The thread only publishes an immutable snapshot; {@link #updateInputs} copies it on the main
 * thread, where AdvantageKit logs it.
 */
public final class BatteryTagIOSerial implements BatteryTagIO {
    /** Time between reads while reading is allowed, and between tries to find the reader (ms). */
    static final int POLL_MILLIS = 2000;

    /** Time between tries when the serial library cannot load at all (ms). */
    static final int NATIVE_RETRY_MILLIS = 30_000;

    /** How long one tag search waits for the reader's reply (ms). */
    private static final int READ_TIMEOUT_MILLIS = 500;

    /** Passive activation retries per search: keeps a read with no tag to tens of ms. */
    private static final int ACTIVATION_RETRIES = 2;

    /** What the thread last saw. Immutable so the main loop never sees half an update. */
    private record Snapshot(
            boolean readerConnected,
            boolean tagPresent,
            String uid,
            String tagName,
            long reads,
            String error,
            long readNanos) {}

    private volatile Snapshot latest = new Snapshot(false, false, "", "", 0, STARTING, 0);

    /** The error reported until the reader thread's first attempt finishes. */
    static final String STARTING = "starting";

    private volatile boolean mayRead;
    private long lastReadNanos;
    private final Supplier<Optional<String>> findPort;
    private final PortOpener open;
    private long reads;

    /** Opens a transport on a port (a seam for tests). */
    interface PortOpener {
        Pn532Transport open(String port) throws Exception;
    }

    /** Starts the reader thread on the first CH340 (or only USB serial) port. */
    public BatteryTagIOSerial() {
        this(SerialTransport::findReaderPort, SerialTransport::new, true);
    }

    BatteryTagIOSerial(Supplier<Optional<String>> findPort, PortOpener open, boolean start) {
        this.findPort = findPort;
        this.open = open;
        if (start) {
            Thread t = new Thread(this::run, "BatteryTag");
            t.setDaemon(true);
            t.setPriority(Thread.MIN_PRIORITY);
            t.start();
        }
    }

    @Override
    public void updateInputs(BatteryTagInputs inputs, boolean mayRead) {
        this.mayRead = mayRead;
        Snapshot s = latest;
        inputs.readerConnected = s.readerConnected();
        inputs.tagPresent = s.tagPresent();
        inputs.uid = s.uid();
        inputs.tagName = s.tagName();
        inputs.reads = s.reads();
        inputs.error = s.error();
        // Elapsed wall time, as a logged input: replay reads it back rather than recomputing it.
        inputs.secondsSinceRead =
                s.readNanos() == 0 ? -1 : (System.nanoTime() - s.readNanos()) * 1e-9;
    }

    private void run() {
        while (!Thread.currentThread().isInterrupted()) {
            int waitMillis;
            try {
                waitMillis = cycle();
            } catch (Throwable t) {
                // Never let the thread die silently: the robot would keep showing the last read.
                disconnected("reader thread: " + t);
                waitMillis = POLL_MILLIS;
            }
            if (!sleep(waitMillis)) {
                return;
            }
        }
    }

    /**
     * One attempt: find and open the reader, then read until it fails. Returns how long to wait
     * before the next attempt (ms).
     */
    int cycle() {
        Optional<String> port;
        try {
            port = findPort.get();
        } catch (LinkageError e) {
            // jSerialComm's native library did not load on this machine.
            disconnected("serial library unavailable: " + e);
            return NATIVE_RETRY_MILLIS;
        } catch (RuntimeException e) {
            disconnected("listing serial ports: " + e);
            return POLL_MILLIS;
        }
        if (port.isEmpty()) {
            disconnected("no reader on USB");
            return POLL_MILLIS;
        }
        try (Pn532 reader = new Pn532(open.open(port.get()))) {
            reader.begin(ACTIVATION_RETRIES);
            boolean first = true;
            while (true) {
                if (first || mayRead) {
                    read(reader);
                    first = false;
                }
                if (!sleep(POLL_MILLIS)) {
                    return 0;
                }
            }
        } catch (LinkageError e) {
            disconnected("serial library unavailable: " + e);
            return NATIVE_RETRY_MILLIS;
        } catch (Exception e) {
            // Unplugged, browned out or busy: report it and reopen.
            disconnected(port.get() + ": " + e.getMessage());
            return POLL_MILLIS;
        }
    }

    /** One tag search; throws when the reader stops answering. */
    void read(Pn532 reader) throws Exception {
        Optional<TagInfo> tag = reader.readTag(READ_TIMEOUT_MILLIS, true);
        reads++;
        lastReadNanos = System.nanoTime();
        latest =
                new Snapshot(
                        true,
                        tag.isPresent(),
                        tag.map(TagInfo::uidHex).orElse(""),
                        tag.map(TagInfo::name).map(String::strip).orElse(""),
                        reads,
                        "",
                        lastReadNanos);
    }

    private void disconnected(String error) {
        latest =
                new Snapshot(
                        false,
                        false,
                        "",
                        "",
                        reads,
                        error == null ? "unknown" : error,
                        lastReadNanos);
    }

    /** Returns false if interrupted (the thread should stop). */
    private static boolean sleep(int millis) {
        if (millis <= 0) {
            return !Thread.currentThread().isInterrupted();
        }
        try {
            Thread.sleep(millis);
            return true;
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            return false;
        }
    }
}
