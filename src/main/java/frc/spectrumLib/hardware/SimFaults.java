package frc.spectrumLib.hardware;

import java.util.Map;
import java.util.TreeMap;
import java.util.concurrent.ConcurrentHashMap;

/**
 * Simulated hardware faults, so degraded behavior can be driven and checked in sim. Phoenix's
 * simulator cannot disconnect one device, so our code asks this instead, at the few places it
 * touches devices: {@link PhoenixDeviceSet} (connected, rebooted, config applies), the drivetrain
 * IO (signal freshness, commands) and the physics (does the motor make torque).
 *
 * <p>Devices are named as {@link PhoenixDeviceSet} names them (e.g. {@code Drive1}, {@code Steer2},
 * {@code Encoder3}, {@code Pigeon}). Sim only: on the robot there is no {@code SimFaults} and every
 * check is false. Thread-safe: the script sets faults on the main thread; the swerve thread and the
 * background thread read them.
 */
public final class SimFaults {
    /** What is wrong with a device. */
    public enum Fault {
        /** Power and data lost: no status frames, commands ignored, no torque, configs fail. */
        UNPLUGGED,
        /** Status frames lost, but it still hears and obeys commands. */
        UNHEARD
    }

    private final Map<String, Fault> faults = new ConcurrentHashMap<>();
    private final Map<String, Integer> configFailures = new ConcurrentHashMap<>();
    private final Map<String, Boolean> pendingReset = new ConcurrentHashMap<>();

    /** Starts a fault on a device. */
    public void set(String device, Fault fault) {
        faults.put(device, fault);
    }

    /** Ends a device's fault. Plugging an unplugged device back in reboots it. */
    public void clear(String device) {
        Fault was = faults.remove(device);
        if (was == Fault.UNPLUGGED) {
            pendingReset.put(device, true);
        }
    }

    /** The device reboots (a brown-out) without being unplugged. */
    public void reboot(String device) {
        pendingReset.put(device, true);
    }

    /** The device's next {@code count} config applies fail. */
    public void failConfigs(String device, int count) {
        configFailures.put(device, count);
    }

    /** Its status frames are not arriving. */
    public boolean unheard(String device) {
        return faults.containsKey(device);
    }

    /** It has no power: commands do nothing and the motor makes no torque. */
    public boolean unpowered(String device) {
        return faults.get(device) == Fault.UNPLUGGED;
    }

    /** A config apply to it should fail now (unplugged, or a failure was asked for). */
    public boolean configFails(String device) {
        if (unpowered(device)) {
            return true;
        }
        Integer left = configFailures.get(device);
        if (left != null && left > 0) {
            configFailures.put(device, left - 1);
            return true;
        }
        return false;
    }

    /** Whether it rebooted since the last call (consumes the reboot). */
    public boolean consumeReset(String device) {
        return pendingReset.remove(device) != null;
    }

    /** Active faults as "device: fault" lines, sorted, for the log. */
    public String[] describe() {
        Map<String, Fault> sorted = new TreeMap<>(faults);
        return sorted.entrySet().stream()
                .map(e -> e.getKey() + ": " + e.getValue())
                .toArray(String[]::new);
    }
}
