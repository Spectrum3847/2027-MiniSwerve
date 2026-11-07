package frc.spectrumLib.hardware;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.ParentDevice;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicIntegerArray;
import java.util.concurrent.atomic.AtomicLong;
import java.util.function.Supplier;

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
 * </ul>
 *
 * <p>Flags are atomics: written on the background thread, read on the main thread.
 *
 * <p>Index: {@link #add} | {@link #setSignals} | {@link #configureAtBoot} | {@link #poll} | getters
 */
public final class PhoenixDeviceSet {
    private static final int BOOT_TRIES = 5;

    private record Device(String name, ParentDevice device, Supplier<StatusCode> config) {}

    private final String prefix;
    private final List<Device> devices = new ArrayList<>();
    private AtomicIntegerArray configured = new AtomicIntegerArray(0);
    private AtomicIntegerArray generation = new AtomicIntegerArray(0);
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
        for (int i = 0; i < n; i++) {
            configured.set(i, tryHard(devices.get(i).config()) ? 1 : 0);
        }
        signalsSet = tryHard(signals);
        // A freshly constructed device reports a reset; clear it so boot is not counted as one.
        for (Device d : devices) {
            d.device().hasResetOccurred();
        }
    }

    private static boolean tryHard(Supplier<StatusCode> call) {
        for (int t = 0; t < BOOT_TRIES; t++) {
            if (call.get().isOK()) {
                return true;
            }
        }
        return false;
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
                generation.incrementAndGet(i);
                signalsGeneration.incrementAndGet();
                signalsSet = false;
                resets.incrementAndGet();
            }
        }
        for (int i = 0; i < devices.size(); i++) {
            Device d = devices.get(i);
            String name = prefix + "/Config/" + d.name();
            if (configured.get(i) == 0 && !PhoenixBackground.pending(name)) {
                final int index = i;
                final int gen = generation.get(i);
                PhoenixBackground.run(
                        name,
                        d.config(),
                        ok -> {
                            if (ok && generation.get(index) == gen) {
                                configured.set(index, 1);
                            }
                        });
            }
        }
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

    /** Device reboots seen since boot. */
    public long resets() {
        return resets.get();
    }
}
