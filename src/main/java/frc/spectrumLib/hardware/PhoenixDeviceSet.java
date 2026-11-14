package frc.spectrumLib.hardware;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.ParentDevice;
import frc.spectrumLib.telemetry.Alert;
import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;
import java.util.Set;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicIntegerArray;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicLongArray;
import java.util.concurrent.atomic.AtomicReferenceArray;
import java.util.function.Supplier;
import org.wpilib.util.Alert.Level;

/**
 * A mechanism's Phoenix devices, kept configured for the whole match: devices that reboot (a
 * connector wiggle, a brown-out) or miss a config apply are noticed and set up again in the
 * background, over and over until it works (AGENTS.md "Finish the match").
 *
 * <ul>
 *   <li>Boot: each device's config applies (blocking, retried); then the signal rates.
 *   <li>Every {@link #poll()} (main thread): a device whose {@code hasResetOccurred()} is set loses
 *       its "configured" flag and the signal rates are marked for redoing (rates are not saved on
 *       the device); then every device not configured, and the signal rates if needed, are queued
 *       on {@link PhoenixBackground} unless already queued.
 *   <li>A per-device generation counter makes sure a retry that finishes after a newer reboot does
 *       not mark the device configured again.
 *   <li>A config the device refuses for good (no Pro licence, a value it does not take, wrong
 *       firmware) is not retried: each retry rewrites the whole config, and the device looks like
 *       it keeps rebooting. It raises one alert instead, and a reboot (a reflash, a licence added)
 *       gives it fresh tries.
 *   <li>Other failures (a timeout, a device not answering) wait longer between rounds, 0.25 s
 *       doubling up to 4 s, so a device that stays down does not keep the background thread busy.
 *   <li>Devices that are answering go first and get the full retries; a device that is not
 *       answering is queued after them with one try. All of this shares one background thread, and
 *       a dead device's five timed-out tries (about half a second) used to stand in front of the
 *       devices a brown-out had just brought back.
 * </ul>
 *
 * <p>Flags are atomics: written on the background thread, read on the main thread.
 *
 * <p>Index: {@link #add} | {@link #setSignals} | {@link #configureAtBoot} | {@link #poll} |
 * configDone | updateRefusedAlert | getters
 */
public final class PhoenixDeviceSet {
    private static final int BOOT_TRIES = 5;
    private static final long FIRST_WAIT_NANOS = 250_000_000L;
    private static final long MAX_WAIT_NANOS = 4_000_000_000L;

    /** Answers no retry can change: the device itself has to (reboot, reflash, licence). */
    static final Set<StatusCode> REFUSED_FOR_GOOD =
            EnumSet.of(
                    StatusCode.UsingProFeatureOnUnlicensedDevice,
                    StatusCode.InvalidParamValue,
                    StatusCode.FeatureNotSupported,
                    StatusCode.NotSupported,
                    StatusCode.FirmwareTooOld,
                    StatusCode.FirmwareTooNew,
                    StatusCode.FirmwareVersNotCompatible);

    private record Device(String name, ParentDevice device, Supplier<StatusCode> config) {}

    private final String prefix;
    private final List<Device> devices = new ArrayList<>();
    private AtomicIntegerArray configured = new AtomicIntegerArray(0);
    private AtomicIntegerArray generation = new AtomicIntegerArray(0);
    // Why a device's config is not retried (null while it is), and when its next round may start.
    private AtomicReferenceArray<StatusCode> refused = new AtomicReferenceArray<>(0);
    private AtomicLongArray nextTryNanos = new AtomicLongArray(0);
    private AtomicLongArray waitNanos = new AtomicLongArray(0);
    private final Alert refusedAlert = new Alert("", Level.HIGH);
    private final StringBuilder refusedText = new StringBuilder();
    private String refusedShown = "";
    private boolean[] wasConnected = new boolean[0];
    private Supplier<StatusCode> signals = () -> StatusCode.OK;
    private final AtomicInteger signalsGeneration = new AtomicInteger();
    private volatile boolean signalsSet;
    private final AtomicLong resets = new AtomicLong();
    private volatile SimFaults faults;

    /**
     * @param prefix names the background calls, e.g. "Swerve"
     */
    public PhoenixDeviceSet(String prefix) {
        this.prefix = prefix;
    }

    /**
     * Adds a device. Call before {@link #configureAtBoot}.
     *
     * @param config applies the device's whole config (blocking); used at boot and on every retry
     * @return the device's index, for {@link #configured(int)}
     */
    public int add(String name, ParentDevice device, Supplier<StatusCode> config) {
        // In sim, a faulted device's config applies fail like a real unplugged one's.
        Supplier<StatusCode> guarded =
                () ->
                        faults != null && faults.configFails(name)
                                ? StatusCode.TxTimeout
                                : config.get();
        devices.add(new Device(name, device, guarded));
        return devices.size() - 1;
    }

    /** Sim only: devices answer as {@code faults} says (unplugged, unheard, rebooted). */
    public void setSimFaults(SimFaults faults) {
        this.faults = faults;
    }

    /** The name a device was added with (how {@link SimFaults} refers to it). */
    public String name(int index) {
        return devices.get(index).name();
    }

    /** What sets every signal's update rate (blocking); redone after any device reboots. */
    public void setSignals(Supplier<StatusCode> signals) {
        this.signals = signals;
    }

    /** Applies every config and the signal rates now (blocking, retried). Boot only. */
    public void configureAtBoot() {
        int n = devices.size();
        configured = new AtomicIntegerArray(n);
        generation = new AtomicIntegerArray(n);
        refused = new AtomicReferenceArray<>(n);
        nextTryNanos = new AtomicLongArray(n);
        waitNanos = new AtomicLongArray(n);
        wasConnected = new boolean[n];
        for (int i = 0; i < n; i++) {
            StatusCode code = tryHard(devices.get(i).config());
            configured.set(i, code.isOK() ? 1 : 0);
            if (REFUSED_FOR_GOOD.contains(code)) {
                refused.set(i, code);
            }
        }
        signalsSet = tryHard(signals).isOK();
        // A freshly constructed device reports a reset; clear it so boot is not counted as one.
        for (Device d : devices) {
            d.device().hasResetOccurred();
        }
    }

    /** Up to five tries; the last answer (OK, or why it failed). Stops early on a refusal. */
    private static StatusCode tryHard(Supplier<StatusCode> call) {
        StatusCode code = StatusCode.GeneralError;
        for (int t = 0; t < BOOT_TRIES; t++) {
            try {
                code = call.get();
                if (code == null) {
                    code = StatusCode.GeneralError;
                }
                if (code.isOK() || REFUSED_FOR_GOOD.contains(code)) {
                    return code;
                }
            } catch (RuntimeException | LinkageError e) {
                // Left unconfigured; poll() retries it in the background through the match.
                System.err.println("Phoenix config threw at boot: " + e);
                code = StatusCode.GeneralError;
            }
        }
        return code;
    }

    /**
     * Notices reboots and queues another try for anything not set up. Main thread, a few times a
     * second; non-blocking.
     */
    public void poll() {
        for (int i = 0; i < devices.size(); i++) {
            Device d = devices.get(i);
            boolean reset = d.device().hasResetOccurred();
            if (faults != null && faults.consumeReset(d.name())) {
                reset = true;
            }
            if (reset) {
                configured.set(i, 0);
                // Generation first, then the clears: a round finishing now either sees the new
                // generation and writes nothing, or writes before the clears, or undoes its write
                // when it checks again (configDone).
                generation.incrementAndGet(i);
                // A reboot may be the fix (a reflash, a licence): fresh tries, no waiting.
                clearRetryState(i);
                signalsGeneration.incrementAndGet();
                signalsSet = false;
                resets.incrementAndGet();
            }
        }
        // A device answering again without a reboot (a connector wiggle) tries now, not after
        // the wait it built up while it was silent.
        for (int i = 0; i < devices.size(); i++) {
            boolean up = connected(i);
            if (up && !wasConnected[i]) {
                waitNanos.set(i, 0);
            }
            wasConnected[i] = up;
        }
        // Answering devices first, with every retry; silent ones after, one try each.
        long now = System.nanoTime();
        for (int pass = 0; pass < 2; pass++) {
            boolean wantConnected = pass == 0;
            for (int i = 0; i < devices.size(); i++) {
                Device d = devices.get(i);
                String name = prefix + "/Config/" + d.name();
                if (configured.get(i) == 0
                        && refused.get(i) == null
                        && (waitNanos.get(i) == 0 || now - nextTryNanos.get(i) >= 0)
                        && connected(i) == wantConnected
                        && !PhoenixBackground.pending(name)) {
                    final int index = i;
                    final int gen = generation.get(i);
                    PhoenixBackground.run(
                            name,
                            d.config(),
                            ok -> configDone(index, gen, ok, PhoenixBackground.result(name)),
                            wantConnected ? PhoenixBackground.TRIES : 1,
                            REFUSED_FOR_GOOD::contains);
                }
            }
        }
        updateRefusedAlert();
        String signalsName = prefix + "/Signals";
        if (!signalsSet && !PhoenixBackground.pending(signalsName)) {
            final int gen = signalsGeneration.get();
            PhoenixBackground.run(
                    signalsName,
                    signals,
                    ok -> {
                        if (ok && signalsGeneration.get() == gen) {
                            signalsSet = true;
                        }
                    });
        }
    }

    /** Background thread: records how a config round ended. */
    private void configDone(int index, int gen, boolean ok, StatusCode code) {
        if (generation.get(index) != gen) {
            return; // it rebooted meanwhile, and that reboot already reset its state
        }
        if (ok) {
            configured.set(index, 1);
            waitNanos.set(index, 0);
            return;
        }
        if (code != null && REFUSED_FOR_GOOD.contains(code)) {
            refused.set(index, code);
        } else {
            long wait =
                    Math.min(MAX_WAIT_NANOS, Math.max(FIRST_WAIT_NANOS, 2 * waitNanos.get(index)));
            nextTryNanos.set(index, System.nanoTime() + wait);
            waitNanos.set(index, wait);
        }
        // A reboot between the check above and these writes already cleared this state: undo.
        if (generation.get(index) != gen) {
            clearRetryState(index);
        }
    }

    /** Fresh tries for a device: not refused, no waiting. */
    private void clearRetryState(int index) {
        refused.set(index, null);
        waitNanos.set(index, 0);
    }

    /** Main thread: one alert naming each refused device and why; rebuilt only on a change. */
    private void updateRefusedAlert() {
        refusedText.setLength(0);
        for (int i = 0; i < devices.size(); i++) {
            StatusCode code = refused.get(i);
            if (code != null) {
                refusedText.append(refusedText.length() == 0 ? "" : ", ");
                refusedText.append(devices.get(i).name()).append(' ').append(code.getName());
            }
        }
        if (!refusedShown.contentEquals(refusedText)) {
            refusedShown = refusedText.toString();
            refusedAlert.setText(
                    prefix + ": config refused, retried after a reboot: " + refusedShown);
            refusedAlert.set(!refusedShown.isEmpty());
        }
    }

    /** Why the device's config is not being retried, or null if it is (or it took). */
    public StatusCode refused(int index) {
        return refused.get(index);
    }

    /** The device has taken its config since its last reboot. */
    public boolean configured(int index) {
        return configured.get(index) == 1;
    }

    /** The device's status frames are arriving (Phoenix {@code isConnected}). */
    public boolean connected(int index) {
        Device d = devices.get(index);
        return d.device().isConnected() && (faults == null || !faults.unheard(d.name()));
    }

    /** Every device's signal rates are set (false after a reboot until redone). */
    public boolean signalsSet() {
        return signalsSet;
    }

    /** Reboots of one device seen since boot. */
    public long resets(int index) {
        return generation.get(index);
    }

    /** Device reboots seen since boot. */
    public long resets() {
        return resets.get();
    }
}
