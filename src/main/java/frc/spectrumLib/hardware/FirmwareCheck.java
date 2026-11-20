package frc.spectrumLib.hardware;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Finds firmware problems in a group of devices of one type (all the TalonFX, all the CANcoders): a
 * device on a different version from the rest (a missed update, a swapped-in spare) and a device
 * whose version could not be read. The most common version in the group is taken as the expected
 * one.
 *
 * <p>Pure: no devices, so it can run on logged inputs (and in replay) and be tested.
 */
public final class FirmwareCheck {
    private FirmwareCheck() {}

    /**
     * Problems in one group, one line each, empty when every device reads the same version.
     *
     * @param names each device's name, for the message (e.g. "FL steer")
     * @param versions each device's version; "" or starting with "unknown" means unread
     */
    public static List<String> problems(String[] names, String[] versions) {
        List<String> out = new ArrayList<>();
        Map<String, Integer> counts = new HashMap<>();
        for (int i = 0; i < versions.length; i++) {
            if (unread(versions[i])) {
                out.add(names[i] + ": firmware not read (" + versions[i] + ")");
            } else {
                counts.merge(versions[i], 1, Integer::sum);
            }
        }
        String expected = null;
        int best = 0;
        boolean tie = false;
        for (var e : counts.entrySet()) {
            if (e.getValue() > best) {
                expected = e.getKey();
                best = e.getValue();
                tie = false;
            } else if (e.getValue() == best) {
                tie = true;
            }
        }
        if (counts.size() > 1) {
            for (int i = 0; i < versions.length; i++) {
                if (!unread(versions[i]) && (tie || !versions[i].equals(expected))) {
                    out.add(
                            names[i]
                                    + ": firmware "
                                    + versions[i]
                                    + (tie ? " (versions split evenly)" : ", others " + expected));
                }
            }
        }
        return out;
    }

    private static boolean unread(String v) {
        return v == null || v.isEmpty() || v.startsWith("unknown");
    }
}
