package frc.spectrumLib.statemachine;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

/** Overlapping asks: the newest wins, and the state falls back when it stops asking. */
class WantedStateTest {
    enum S {
        DRIVE,
        LOCK,
        AUTO,
        SPIN
    }

    private final WantedState<S> state = new WantedState<>("Test", S.DRIVE);

    @Test
    void nobodyAskingIsTheDefault() {
        assertEquals(S.DRIVE, state.resolve());
        assertEquals("default", state.requester());
    }

    @Test
    void theNewestAskerWinsAndTheOlderTakesOverWhenItLetsGo() {
        state.request(S.AUTO, "auto");
        assertEquals(S.AUTO, state.resolve());

        // X-lock held during the auto.
        state.request(S.AUTO, "auto");
        state.request(S.LOCK, "xlock");
        assertEquals(S.LOCK, state.resolve());
        assertEquals("xlock", state.requester());

        // Renewed every loop, order within the loop does not matter: the newest still wins.
        state.request(S.LOCK, "xlock");
        state.request(S.AUTO, "auto");
        assertEquals(S.LOCK, state.resolve());

        // X-lock let go: back to the auto, not to the default.
        state.request(S.AUTO, "auto");
        state.release("xlock");
        assertEquals(S.AUTO, state.resolve());
        assertEquals("auto", state.requester());
    }

    @Test
    void anAskerThatStopsAskingLetsGoOnItsOwn() {
        state.request(S.AUTO, "auto");
        state.request(S.LOCK, "xlock");
        assertEquals(S.LOCK, state.resolve());

        // The X-lock command died without releasing; the auto keeps asking.
        state.request(S.AUTO, "auto");
        assertEquals(S.AUTO, state.resolve());

        // Nobody asks: the default.
        assertEquals(S.DRIVE, state.resolve());
        assertEquals(0, state.askerCount());
    }

    @Test
    void anAskAfterTheResolveCountsForTheNextLoop() {
        assertEquals(S.DRIVE, state.resolve());
        // Asked late in the loop, after the subsystem resolved.
        state.request(S.SPIN, "late");
        assertEquals(S.SPIN, state.resolve());
    }

    @Test
    void changingItsAskMakesARequesterTheNewest() {
        state.request(S.AUTO, "auto");
        state.request(S.LOCK, "xlock");
        assertEquals(S.LOCK, state.resolve());

        state.request(S.LOCK, "xlock");
        state.request(S.SPIN, "auto");
        assertEquals(S.SPIN, state.resolve());
    }

    @Test
    void releasingWhatSomeoneElseAskedDoesNothing() {
        state.request(S.LOCK, "xlock");
        state.release("nobody");
        assertEquals(S.LOCK, state.resolve());
    }

    @Test
    void tooManyAskersDropsTheOldestNeverTheNewest() {
        for (int i = 0; i < WantedState.MAX_ASKERS; i++) {
            state.request(S.AUTO, "asker" + i);
        }
        state.request(S.SPIN, "newest");
        assertEquals(WantedState.MAX_ASKERS, state.askerCount());
        assertEquals(S.SPIN, state.resolve());
        assertEquals("newest", state.requester());
    }
}
