package frc.spectrumLib.statemachine;

import frc.spectrumLib.telemetry.Telemetry;
import org.littletonrobotics.junction.Logger;
import org.wpilib.system.Timer;

/**
 * A subsystem's wanted state and who asked for it, logged every loop.
 *
 * <p>The Spectrum wanted-state pattern: commands, autos and bindings only <em>ask</em> for a state
 * ({@link #request}); the subsystem's periodic decides what it can actually do this loop and
 * reports that as its current state ({@link #setCurrent}). Keeping the two apart is what lets an
 * auto say "be at X" and lets the log show why the robot did something else.
 *
 * <p>Logged under {@code <prefix>/State/}: {@code Wanted}, {@code Current}, {@code Requester},
 * {@code SecondsInWanted}, and an event each time the wanted state changes.
 *
 * @param <S> the subsystem's state enum
 */
public final class WantedState<S extends Enum<S>> {
    private final String prefix;
    private final S defaultState;
    private S wanted;
    private S current;
    private String requester = "default";
    private double wantedSince;

    // Log keys, built once: this logs every loop.
    private final String wantedKey;
    private final String currentKey;
    private final String requesterKey;
    private final String secondsKey;

    public WantedState(String prefix, S defaultState) {
        this.prefix = prefix;
        wantedKey = prefix + "/State/Wanted";
        currentKey = prefix + "/State/Current";
        requesterKey = prefix + "/State/Requester";
        secondsKey = prefix + "/State/SecondsInWanted";
        this.defaultState = defaultState;
        this.wanted = defaultState;
        this.current = defaultState;
    }

    /**
     * Asks for a state. The subsystem acts on it from its next periodic.
     *
     * @param requester who asked, for the log (a command, binding or auto step name)
     */
    public void request(S state, String requester) {
        if (state != wanted || !requester.equals(this.requester)) {
            Telemetry.logEvent(
                    prefix + "/State/Events", wanted + " -> " + state + " (" + requester + ")");
            wantedSince = Timer.getTimestamp();
        }
        wanted = state;
        this.requester = requester;
    }

    /** Back to the default state, e.g. when the command that asked ends. */
    public void release(String requester) {
        if (requester.equals(this.requester)) {
            request(defaultState, "default");
        }
    }

    public S wanted() {
        return wanted;
    }

    public S current() {
        return current;
    }

    public String requester() {
        return requester;
    }

    /** The subsystem reports what it is actually doing this loop, and the state is logged. */
    public void setCurrent(S state) {
        current = state;
        // What was asked for, by whom, and what the subsystem is actually doing.
        Logger.recordOutput(wantedKey, wanted);
        Logger.recordOutput(currentKey, current);
        Logger.recordOutput(requesterKey, requester);
        // Seconds since the wanted state last changed.
        Logger.recordOutput(secondsKey, Timer.getTimestamp() - wantedSince);
    }
}
