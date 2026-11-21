package frc.spectrumLib.statemachine;

import frc.spectrumLib.telemetry.Telemetry;
import org.littletonrobotics.junction.Logger;
import org.wpilib.system.Timer;

/**
 * A subsystem's wanted state, chosen each loop from everyone asking for one, and logged.
 *
 * <p>The Spectrum wanted-state pattern: commands, autos and bindings only <em>ask</em> for a state
 * ({@link #request}); the subsystem's periodic decides what it can actually do this loop and
 * reports that as its current state ({@link #setCurrent}). Keeping the two apart is what lets an
 * auto say "be at X" and lets the log show why the robot did something else.
 *
 * <p>A request lasts one loop: whoever wants a state asks again every loop it still wants it, the
 * way a held button or a running command does. Several can ask at once; the newest asker wins, and
 * when it stops asking the state goes back to the one before it that is still asking, then to the
 * default. So X-lock held during a drive, then let go, returns to that drive rather than to the
 * default, and a command that dies without cleaning up (a throw, a skipped loop stage) lets go on
 * its own instead of holding its state for the rest of the match. {@link #release} lets go at once,
 * for a command's end.
 *
 * <p>The subsystem calls {@link #resolve} once a loop, after everyone has had a chance to ask
 * (after the OpMode and the scheduler). A request made after that loop's resolve counts for the
 * next one, so order within the loop does not matter.
 *
 * <p>Logged under {@code <prefix>/State/}: {@code Wanted}, {@code Current}, {@code Requester},
 * {@code SecondsInWanted}, {@code Requests} (everyone asking, newest last), and an event each time
 * the wanted state or its requester changes or an asker stops asking without releasing.
 *
 * <p>Index: {@link #request} | {@link #release} | {@link #resolve} | {@link #setCurrent} | Asker
 *
 * @param <S> the subsystem's state enum
 */
public final class WantedState<S extends Enum<S>> {
    /**
     * More askers than a subsystem has bindings, commands and OpModes asking at once. One past this
     * replaces the oldest, so the newest ask is never lost.
     */
    static final int MAX_ASKERS = 8;

    private static final String DEFAULT_REQUESTER = "default";

    private final S defaultState;
    private S wanted;
    private S current;
    private String requester = DEFAULT_REQUESTER;
    private double wantedSince;

    /** Everyone asking, in order of when they started asking (oldest first). Preallocated. */
    private final Asker<S>[] askers;

    private int askerCount;

    /** Counts resolves; an asker is alive if it asked since the last one. */
    private long resolveCount;

    /** Orders askers by when they started asking. */
    private long nextStartOrder;

    /** Everyone asking, rebuilt only when it changes, logged every loop. */
    private String[] requestsLog = new String[0];

    private boolean requestsChanged;

    // Log keys, built once: this logs every loop.
    private final String wantedKey;
    private final String currentKey;
    private final String requesterKey;
    private final String secondsKey;
    private final String requestsKey;
    private final String eventsKey;

    @SuppressWarnings("unchecked")
    public WantedState(String prefix, S defaultState) {
        wantedKey = prefix + "/State/Wanted";
        currentKey = prefix + "/State/Current";
        requesterKey = prefix + "/State/Requester";
        secondsKey = prefix + "/State/SecondsInWanted";
        requestsKey = prefix + "/State/Requests";
        eventsKey = prefix + "/State/Events";
        this.defaultState = defaultState;
        this.wanted = defaultState;
        this.current = defaultState;
        askers = new Asker[MAX_ASKERS];
        for (int i = 0; i < MAX_ASKERS; i++) {
            askers[i] = new Asker<>();
        }
    }

    /**
     * Asks for a state for this loop. Call every loop while it is wanted; the newest asker wins.
     * Asking for a different state than this requester asked for last counts as a new ask, so it
     * becomes the newest.
     *
     * @param requester who asks, for the log and for {@link #release} (a command, binding or auto
     *     step name; a constant, so asking every loop does not allocate)
     */
    public void request(S state, String requester) {
        int i = find(requester);
        if (i >= 0) {
            Asker<S> asker = askers[i];
            asker.askedAt = resolveCount;
            if (asker.state != state) {
                asker.state = state;
                asker.startOrder = nextStartOrder++;
                requestsChanged = true;
            }
            return;
        }
        if (askerCount == MAX_ASKERS) {
            Telemetry.logEvent(
                    eventsKey,
                    "too many askers; dropped " + askers[oldest()].requester + " for " + requester);
            remove(oldest());
        }
        Asker<S> asker = askers[askerCount++];
        asker.requester = requester;
        asker.state = state;
        asker.askedAt = resolveCount;
        asker.startOrder = nextStartOrder++;
        requestsChanged = true;
    }

    /** Stops asking at once, e.g. when the command that asked ends. */
    public void release(String requester) {
        int i = find(requester);
        if (i >= 0) {
            remove(i);
        }
    }

    /**
     * Picks this loop's wanted state from everyone who asked since the last resolve. Call once a
     * loop, after the OpMode and the scheduler, before acting on it.
     *
     * @return the wanted state, also {@link #wanted()} until the next resolve
     */
    public S resolve() {
        // Drop askers that did not ask since the last resolve: they stopped without releasing.
        for (int i = askerCount - 1; i >= 0; i--) {
            if (askers[i].askedAt != resolveCount) {
                Telemetry.logEvent(
                        eventsKey, askers[i].requester + " stopped asking for " + askers[i].state);
                remove(i);
            }
        }
        resolveCount++;

        int newest = newest();
        S nextState = newest >= 0 ? askers[newest].state : defaultState;
        String nextRequester = newest >= 0 ? askers[newest].requester : DEFAULT_REQUESTER;
        if (nextState != wanted || !nextRequester.equals(requester)) {
            Telemetry.logEvent(eventsKey, wanted + " -> " + nextState + " (" + nextRequester + ")");
            wantedSince = Timer.getTimestamp();
            wanted = nextState;
            requester = nextRequester;
        }
        if (requestsChanged) {
            requestsChanged = false;
            rebuildRequestsLog();
        }
        return wanted;
    }

    /** This loop's wanted state, from the last {@link #resolve}. */
    public S wanted() {
        return wanted;
    }

    public S current() {
        return current;
    }

    /** Who asked for {@link #wanted()}; "default" when nobody is asking. */
    public String requester() {
        return requester;
    }

    /** How many are asking, as of the last request or release. */
    public int askerCount() {
        return askerCount;
    }

    /** The subsystem reports what it is actually doing this loop, and the state is logged. */
    public void setCurrent(S state) {
        current = state;
        // What was asked for (the newest asker's state, or the default), and by whom.
        Logger.recordOutput(wantedKey, wanted);
        Logger.recordOutput(requesterKey, requester);
        // What the subsystem is actually doing this loop.
        Logger.recordOutput(currentKey, current);
        // Seconds since the wanted state or its requester last changed.
        Logger.recordOutput(secondsKey, Timer.getTimestamp() - wantedSince);
        // Everyone asking as of the last resolve, "STATE (requester)", oldest first: the last one
        // wins, and the one before it takes over when it stops asking.
        Logger.recordOutput(requestsKey, requestsLog);
    }

    private int find(String requester) {
        for (int i = 0; i < askerCount; i++) {
            if (askers[i].requester.equals(requester)) {
                return i;
            }
        }
        return -1;
    }

    private int newest() {
        int best = -1;
        for (int i = 0; i < askerCount; i++) {
            if (best < 0 || askers[i].startOrder > askers[best].startOrder) {
                best = i;
            }
        }
        return best;
    }

    private int oldest() {
        int best = 0;
        for (int i = 1; i < askerCount; i++) {
            if (askers[i].startOrder < askers[best].startOrder) {
                best = i;
            }
        }
        return best;
    }

    /** Removes slot {@code i}, keeping the Asker objects for reuse. */
    private void remove(int i) {
        Asker<S> gone = askers[i];
        askers[i] = askers[askerCount - 1];
        askers[askerCount - 1] = gone;
        gone.requester = null;
        gone.state = null;
        askerCount--;
        requestsChanged = true;
    }

    /** Allocates, but only when the askers change, not every loop. */
    private void rebuildRequestsLog() {
        String[] log = new String[askerCount];
        boolean[] used = new boolean[askerCount];
        for (int n = 0; n < askerCount; n++) {
            int pick = -1;
            for (int i = 0; i < askerCount; i++) {
                if (!used[i] && (pick < 0 || askers[i].startOrder < askers[pick].startOrder)) {
                    pick = i;
                }
            }
            used[pick] = true;
            log[n] = askers[pick].state + " (" + askers[pick].requester + ")";
        }
        requestsLog = log;
    }

    /** One requester's ask. Mutable and reused so asking every loop does not allocate. */
    private static final class Asker<S> {
        String requester;
        S state;

        /** The resolve count when it last asked. */
        long askedAt;

        /** When it started asking for this state; higher is newer. */
        long startOrder;
    }
}
