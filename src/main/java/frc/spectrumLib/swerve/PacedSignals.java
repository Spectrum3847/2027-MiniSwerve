package frc.spectrumLib.swerve;

import com.ctre.phoenix6.BaseStatusSignal;

/**
 * Paces the swerve thread on the fast signals of the devices that are answering, so one unplugged
 * or browned-out device does not make every cycle wait out the timeout (which would slow the whole
 * drivetrain to about half rate). Signals come in groups, one per device. A group that misses
 * {@link #DROP_AFTER} cycles in a row stops being waited on; it is still refreshed every cycle (no
 * wait) and rejoins as soon as its frames come back.
 *
 * <p>Swerve thread only. Allocates only when a device drops out or comes back.
 */
final class PacedSignals {
    /** Cycles a device may miss before the thread stops waiting for it. */
    static final int DROP_AFTER = 3;

    private final BaseStatusSignal[] all;
    private final int[][] groups;
    private final int[] missed;
    private final boolean[] waiting;
    private BaseStatusSignal[] waitSet;
    private boolean anyDropped;

    /**
     * @param all every fast signal (refreshed each cycle)
     * @param groups indices into {@code all}, one group per device
     */
    PacedSignals(BaseStatusSignal[] all, int[][] groups) {
        this.all = all;
        this.groups = groups;
        missed = new int[groups.length];
        waiting = new boolean[groups.length];
        java.util.Arrays.fill(waiting, true);
        waitSet = all;
    }

    /**
     * Waits for the answering devices' next frames (or {@code timeoutSeconds}), then refreshes the
     * rest without waiting. Returns true when every device answered.
     */
    boolean waitForSample(double timeoutSeconds) {
        boolean good;
        if (waitSet.length == 0) {
            // Nothing answering: keep the cycle's period so the thread neither spins nor stops.
            sleep(timeoutSeconds / 2);
            good = BaseStatusSignal.refreshAll(all).isOK();
        } else {
            good = BaseStatusSignal.waitForAll(timeoutSeconds, waitSet).isOK();
            if (anyDropped) {
                BaseStatusSignal.refreshAll(all);
            }
        }
        boolean changed = false;
        for (int g = 0; g < groups.length; g++) {
            changed |= update(g, groupOk(g));
        }
        if (changed) {
            rebuild();
        }
        return good && !anyDropped;
    }

    /**
     * Records one cycle's result for group {@code g}; returns true if it started or stopped being
     * waited on.
     */
    boolean update(int g, boolean ok) {
        if (ok) {
            missed[g] = 0;
            if (!waiting[g]) {
                waiting[g] = true;
                return true;
            }
            return false;
        }
        if (waiting[g] && ++missed[g] >= DROP_AFTER) {
            waiting[g] = false;
            return true;
        }
        return false;
    }

    /** Whether group {@code g} is waited on. */
    boolean waiting(int g) {
        return waiting[g];
    }

    private boolean groupOk(int g) {
        for (int i : groups[g]) {
            if (!all[i].getStatus().isOK()) {
                return false;
            }
        }
        return true;
    }

    private void rebuild() {
        int count = 0;
        anyDropped = false;
        for (int g = 0; g < groups.length; g++) {
            if (waiting[g]) {
                count += groups[g].length;
            } else {
                anyDropped = true;
            }
        }
        BaseStatusSignal[] set = new BaseStatusSignal[count];
        int k = 0;
        for (int g = 0; g < groups.length; g++) {
            if (waiting[g]) {
                for (int i : groups[g]) {
                    set[k++] = all[i];
                }
            }
        }
        waitSet = set;
    }

    private static void sleep(double seconds) {
        try {
            Thread.sleep(Math.max(1, (long) (seconds * 1000)));
        } catch (InterruptedException e) {
            // The swerve thread stops by its running flag, not by interrupt; re-setting the flag
            // here would make every later sleep return at once and spin.
        }
    }
}
