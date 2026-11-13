package frc.spectrumLib.framework;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

/** A stage that throws is counted and reported, and never throws out of {@code run}. */
class LoopStageTest {
    @Test
    void aThrowIsCaughtAndCounted() {
        LoopStage stage = new LoopStage("TestThrow");
        assertTrue(stage.run(() -> {}));
        assertEquals(0, stage.faults());
        assertFalse(
                stage.run(
                        () -> {
                            throw new IllegalStateException("boom");
                        }));
        assertEquals(1, stage.faults());
        assertTrue(stage.lastFault().contains("boom"));
        // Keeps working after a fault.
        assertTrue(stage.run(() -> {}));
    }

    @Test
    void errorsAreCaughtToo() {
        LoopStage stage = new LoopStage("TestError");
        assertFalse(
                stage.run(
                        () -> {
                            throw new StackOverflowError();
                        }));
        assertEquals(1, stage.faults());
    }

    @Test
    void makeReturnsNullWhenTheMakerThrows() {
        LoopStage stage = new LoopStage("TestMake");
        assertEquals("ok", stage.make(() -> "ok"));
        assertNull(
                stage.<String>make(
                        () -> {
                            throw new IllegalArgumentException("no file");
                        }));
        assertEquals(1, stage.faults());
    }
}
