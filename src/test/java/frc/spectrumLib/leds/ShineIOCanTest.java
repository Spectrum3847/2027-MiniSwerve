package frc.spectrumLib.leds;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;

/** What goes on the bus, and when, against a stand-in bus and clock. */
class ShineIOCanTest {
    private static final long MS = 1_000_000L;
    private static final LedPattern PURPLE = LedPattern.solid(LedColor.PURPLE);

    /** A bus that records writes and hands out the frames a test queues. */
    private static final class FakeBus implements ShineIOCan.Bus {
        /** Each write as "api" or "*api" (to every board). */
        final List<String> sent = new ArrayList<>();

        final List<byte[]> sentData = new ArrayList<>();
        final Map<Integer, byte[]> incoming = new HashMap<>();
        boolean refuse;

        @Override
        public boolean read(int api, byte[] data) {
            byte[] frame = incoming.remove(api);
            if (frame == null) {
                return false;
            }
            System.arraycopy(frame, 0, data, 0, 8);
            return true;
        }

        @Override
        public boolean write(boolean toEveryBoard, int api, byte[] data) {
            if (refuse) {
                return false;
            }
            sent.add((toEveryBoard ? "*" : "") + Integer.toHexString(api));
            sentData.add(data.clone());
            return true;
        }

        void status() {
            incoming.put(ShineProtocol.API_STATUS, new byte[] {0, 0, 0, 0, 0, 25, (byte) 255, 9});
        }

        void version() {
            incoming.put(ShineProtocol.API_VERSION, new byte[] {0, 1, 0, 0, 1, 2, 3, 4});
        }
    }

    private final FakeBus bus = new FakeBus();
    private final ShineInputs inputs = new ShineInputs();
    private long now = 10_000 * MS;
    private final ShineIOCan io = new ShineIOCan(bus, () -> now, () -> now / 1000, 16, 0);

    /** One robot loop, 10 ms later, with the board sending status. */
    private void loop() {
        now += 10 * MS;
        bus.status();
        io.updateInputs(inputs);
        io.send();
    }

    private void loops(int n) {
        for (int i = 0; i < n; i++) {
            loop();
        }
    }

    @Test
    void firstLoopsSendClockThenSetupThenEverySlotFourFramesAtATime() {
        io.setSlot(0, PURPLE, 0, 16, 110);
        loop();
        // The clock to every board, then LED counts A and B and a ping.
        assertEquals(List.of("*40", "30", "30", "52"), bus.sent);
        assertEquals(3, bus.sentData.get(1)[0]); // key: count A
        assertEquals(16, bus.sentData.get(1)[4]);
        assertEquals(4, bus.sentData.get(2)[0]); // key: count B
        assertEquals(0, bus.sentData.get(2)[4]);
        bus.sent.clear();
        loop();
        // Slot 0: colors before the animation. Then the unused slots, one OFF frame each.
        assertEquals(List.of("18", "10", "11", "12"), bus.sent);
        bus.sent.clear();
        loops(2);
        assertEquals(List.of("13", "14", "15", "16", "17"), bus.sent);
        io.updateInputs(inputs); // the counts are as of the start of a loop
        assertTrue(inputs.connected);
        assertEquals(13, inputs.framesSent);
    }

    @Test
    void sameSlotAgainSendsNothingAndAChangeSendsOnlyThatSlot() {
        io.setSlot(0, PURPLE, 0, 16, 110);
        loops(5);
        bus.sent.clear();
        io.setSlot(0, PURPLE, 0, 16, 110);
        loops(3);
        assertEquals(List.of(), bus.sent);
        io.setSlot(1, LedPattern.solid(LedColor.YELLOW), 0, 3, 110);
        loop();
        assertEquals(List.of("19", "11"), bus.sent);
    }

    @Test
    void everySlotRepeatsEachSecondWithTheClock() {
        io.setSlot(0, PURPLE, 0, 16, 110);
        loops(5);
        bus.sent.clear();
        loops(100); // one second
        assertEquals(1, bus.sent.stream().filter("*40"::equals).count());
        assertEquals(1, bus.sent.stream().filter("18"::equals).count());
        assertEquals(1, bus.sent.stream().filter("10"::equals).count());
        assertEquals(1, bus.sent.stream().filter("17"::equals).count());
        // No setup again: the board never went away.
        assertFalse(bus.sent.contains("30"));
        assertEquals(10, bus.sent.size());
    }

    @Test
    void aBootAnnouncedByTheBoardGetsSetupAndSlotsAgain() {
        io.setSlot(0, PURPLE, 0, 16, 110);
        loops(2);
        bus.version(); // the answer to our ping
        loops(3);
        assertEquals(0, inputs.boots);
        assertEquals("0.1", inputs.firmware);
        assertEquals("04030201", inputs.uid);
        bus.sent.clear();
        bus.version(); // unasked: it rebooted
        loop();
        assertEquals(1, inputs.boots);
        assertEquals(List.of("30", "30", "52"), bus.sent.subList(0, 3));
        loops(3);
        assertTrue(bus.sent.containsAll(List.of("18", "10", "17")));
    }

    @Test
    void aLostPingAnswerDoesNotHideALaterBoot() {
        loops(5); // ping sent, never answered
        now += 2_000 * MS;
        bus.version();
        loop();
        assertEquals(1, inputs.boots);
    }

    @Test
    void aSilentBoardIsSentFarLessAndCaughtUpWhenItAnswers() {
        io.setSlot(0, PURPLE, 0, 16, 110);
        // No status frames at all.
        for (int i = 0; i < 1000; i++) { // ten seconds
            now += 10 * MS;
            io.updateInputs(inputs);
            io.send();
        }
        assertFalse(inputs.connected);
        long toBoard = bus.sent.stream().filter(s -> !s.startsWith("*")).count();
        // Setup (3) and slots (9) at the start and at each 5 s repeat, not every second.
        assertTrue(toBoard <= 36, "frames to a silent board in 10 s: " + toBoard);
        bus.sent.clear();
        loop(); // it answers
        assertTrue(inputs.connected);
        loops(3);
        assertTrue(bus.sent.containsAll(List.of("30", "52", "18", "10")));
    }

    @Test
    void refusedFramesAreCountedNotRetriedEveryLoop() {
        bus.refuse = true;
        io.setSlot(0, PURPLE, 0, 16, 110);
        loops(50);
        io.updateInputs(inputs);
        // One attempt at each frame: clock, setup (3), slot 0 (2), seven OFF slots.
        assertEquals(13, inputs.sendErrors);
        assertEquals(0, inputs.framesSent);
    }

    @Test
    void readsStatusAndGoesDisconnectedAfterHalfASecondOfSilence() {
        loop();
        assertTrue(inputs.connected);
        assertEquals(25, inputs.temperatureC);
        assertEquals(1.0, inputs.outputScale, 1e-9);
        assertTrue(inputs.timeSynced);
        assertTrue(inputs.anySlot);
        now += 600 * MS;
        io.updateInputs(inputs);
        assertFalse(inputs.connected);
    }
}
