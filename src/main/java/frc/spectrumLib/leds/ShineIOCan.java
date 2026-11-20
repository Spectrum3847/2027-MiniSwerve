package frc.spectrumLib.leds;

import static org.wpilib.units.Units.Microseconds;

import org.wpilib.hardware.bus.CAN;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.hardware.hal.can.CANReceiveMessage;
import org.wpilib.system.RobotController;

/**
 * A SpectrumShine board on CAN through WPILib's {@link CAN} (the board uses the team-use
 * manufacturer and miscellaneous device type, WPILib's defaults). Reads take the latest frame the
 * HAL already has and writes hand a frame to the HAL; neither waits on the bus.
 *
 * <p>Sending: a slot goes out when it changes, and every slot is repeated once a second, so a board
 * that lost power, or missed a frame, shows the right thing again within a second without the robot
 * having to notice. Repeats do not restart an animation: the board ignores a frame equal to what it
 * has. At most {@link #FRAMES_PER_LOOP} frames leave per loop.
 *
 * <p>When the board appears (first status frame, or again after silence) it is told how many LEDs
 * are on each output and asked for its firmware version. The LED counts are set in the board's RAM
 * only: nothing is saved to its flash from here.
 *
 * <p>Once a second the robot's clock goes to every board (device id 63), so animations on several
 * boards stay in step and the board's event times match the log.
 *
 * <p>Index: constructor | {@link #updateInputs} | {@link #setSlot} | {@link #send}
 */
public final class ShineIOCan implements ShineIO {
    /** Frames per loop at most: a full picture change is spread over a few loops. */
    private static final int FRAMES_PER_LOOP = 4;

    /** The board sends status every 100 ms by default; silent this long is disconnected. */
    private static final long CONNECTED_NANOS = 500_000_000L;

    private static final long TIME_SYNC_NANOS = 1_000_000_000L;
    private static final long RESEND_NANOS = 1_000_000_000L;

    private final CAN board;
    private final CAN everyBoard;
    private final int countA;
    private final int countB;

    private final CANReceiveMessage rx = new CANReceiveMessage();
    private final byte[] tx = new byte[8];

    // What each slot should show, and whether the board has been told.
    private final LedPattern[] pattern = new LedPattern[ShineProtocol.SLOTS];
    private final int[] start = new int[ShineProtocol.SLOTS];
    private final int[] length = new int[ShineProtocol.SLOTS];
    private final int[] brightness = new int[ShineProtocol.SLOTS];
    private final boolean[] due = new boolean[ShineProtocol.SLOTS];

    // Elapsed wall time (System.nanoTime), not the robot clock: these are bus timeouts.
    private long lastStatusNanos;
    private boolean statusSeen;
    private boolean wasConnected;
    private long lastTimeSyncNanos;
    private long lastResendNanos;

    /** Setup frames still to send after the board appears: 0 count A, 1 count B, 2 ping, 3 none. */
    private int setupStep = SETUP_DONE;

    private static final int SETUP_DONE = 3;

    /** A version frame we asked for is on its way, so the next one is not a boot. */
    private boolean pingOutstanding;

    private int faults;
    private int stickyFaults;
    private int limitReason;
    private int temperatureC;
    private int scale;
    private int flags;
    private int batteryMillivolts;
    private int railMillivolts;
    private int measuredMilliamps;
    private int estimatedMilliamps;
    private String firmware = "";
    private String uid = "";
    private long boots;
    private long framesSent;
    private long sendErrors;

    /**
     * Opens the board (boot only).
     *
     * @param bus the CAN bus the board is on
     * @param id its CAN id, 0..62 (a new board is 0)
     * @param ledsOnA LEDs on output A
     * @param ledsOnB LEDs on output B (they follow A's in the index space)
     */
    public ShineIOCan(CANPort bus, int id, int ledsOnA, int ledsOnB) {
        if (id < 0 || id >= ShineProtocol.BROADCAST_ID) {
            throw new IllegalArgumentException("SpectrumShine id " + id + " is not 0..62");
        }
        board = new CAN(bus, id);
        everyBoard = new CAN(bus, ShineProtocol.BROADCAST_ID);
        countA = Math.max(0, Math.min(ShineProtocol.MAX_LEDS, ledsOnA));
        countB = Math.max(0, Math.min(ShineProtocol.MAX_LEDS - countA, ledsOnB));
        java.util.Arrays.fill(pattern, LedPattern.OFF);
        // Tell a board that is already running what every slot is, unused ones included: it may
        // still hold slots from before a code restart.
        java.util.Arrays.fill(due, true);
    }

    @Override
    public void updateInputs(ShineInputs inputs) {
        long now = System.nanoTime();
        if (board.readPacketNew(ShineProtocol.API_STATUS, rx) && rx.length >= 8) {
            byte[] d = rx.data;
            faults = ShineProtocol.u16(d, 0);
            stickyFaults = ShineProtocol.u16(d, 2);
            limitReason = d[4] & 0xFF;
            temperatureC = d[5]; // signed
            scale = d[6] & 0xFF;
            flags = d[7] & 0xFF;
            lastStatusNanos = now;
            statusSeen = true;
        }
        if (board.readPacketNew(ShineProtocol.API_POWER, rx) && rx.length >= 8) {
            byte[] d = rx.data;
            batteryMillivolts = ShineProtocol.u16(d, 0);
            railMillivolts = ShineProtocol.u16(d, 2);
            measuredMilliamps = ShineProtocol.u16(d, 4);
            estimatedMilliamps = ShineProtocol.u16(d, 6);
        }
        if (board.readPacketNew(ShineProtocol.API_VERSION, rx) && rx.length >= 8) {
            byte[] d = rx.data;
            // Only when a version frame arrives (boot or our ping), so the strings are not per
            // loop.
            firmware = (d[0] & 0xFF) + "." + (d[1] & 0xFF);
            uid = String.format("%08X", ShineProtocol.u32(d, 4));
            if (pingOutstanding) {
                pingOutstanding = false;
            } else {
                // Unasked: the board just booted. Its slots may be empty and its LED counts are
                // back to what its flash holds.
                boots++;
                setupStep = 0;
                java.util.Arrays.fill(due, true);
            }
        }
        boolean connected = statusSeen && now - lastStatusNanos < CONNECTED_NANOS;
        if (connected && !wasConnected) {
            setupStep = 0;
            java.util.Arrays.fill(due, true);
        }
        wasConnected = connected;

        inputs.connected = connected;
        inputs.faults = faults;
        inputs.stickyFaults = stickyFaults;
        inputs.limitReason = limitReason;
        inputs.temperatureC = temperatureC;
        inputs.outputScale = scale / 255.0;
        inputs.timeSynced = (flags & ShineProtocol.FLAG_TIME_SYNCED) != 0;
        inputs.anySlot = (flags & ShineProtocol.FLAG_ANY_SLOT) != 0;
        inputs.batteryVolts = batteryMillivolts / 1000.0;
        inputs.railVolts = railMillivolts / 1000.0;
        inputs.measuredAmps = measuredMilliamps / 1000.0;
        inputs.estimatedAmps = estimatedMilliamps / 1000.0;
        inputs.firmware = firmware;
        inputs.uid = uid;
        inputs.boots = boots;
        inputs.framesSent = framesSent;
        inputs.sendErrors = sendErrors;
    }

    @Override
    public void setSlot(int slot, LedPattern wanted, int first, int count, int bright) {
        if (slot < 0 || slot >= ShineProtocol.SLOTS) {
            return;
        }
        if (pattern[slot].equals(wanted)
                && start[slot] == first
                && length[slot] == count
                && brightness[slot] == bright) {
            return;
        }
        pattern[slot] = wanted;
        start[slot] = first;
        length[slot] = count;
        brightness[slot] = bright;
        due[slot] = true;
    }

    @Override
    public void send() {
        long now = System.nanoTime();
        int budget = FRAMES_PER_LOOP;
        if (now - lastTimeSyncNanos >= TIME_SYNC_NANOS) {
            lastTimeSyncNanos = now;
            // The clock the log is on. Through the unit, since WPILib 2027's raw long is not
            // microseconds.
            ShineProtocol.packTime(tx, (long) RobotController.getMeasureTime().in(Microseconds));
            write(everyBoard, ShineProtocol.API_TIME_SYNC);
            budget--;
        }
        // Setup goes out whether or not the board is answering: frames to a missing board cost
        // nothing, and its status frames may be the ones getting lost.
        while (setupStep != SETUP_DONE && budget > 0) {
            switch (setupStep) {
                case 0 -> {
                    ShineProtocol.packConfig(tx, ShineProtocol.KEY_COUNT_A, countA);
                    write(board, ShineProtocol.API_SET_CONFIG);
                }
                case 1 -> {
                    ShineProtocol.packConfig(tx, ShineProtocol.KEY_COUNT_B, countB);
                    write(board, ShineProtocol.API_SET_CONFIG);
                }
                default -> {
                    java.util.Arrays.fill(tx, (byte) 0);
                    pingOutstanding = true;
                    write(board, ShineProtocol.API_PING);
                }
            }
            setupStep++;
            budget--;
        }
        if (now - lastResendNanos >= RESEND_NANOS) {
            lastResendNanos = now;
            java.util.Arrays.fill(due, true);
        }
        for (int slot = 0; slot < ShineProtocol.SLOTS && budget > 0; slot++) {
            if (!due[slot]) {
                continue;
            }
            boolean off = pattern[slot].animation() == ShineAnimation.OFF;
            if (!off) {
                if (budget < 2) {
                    break; // colors and animation leave in the same loop
                }
                // Colors first: an animation frame alone would show the animation's default
                // colors for a frame on a board that has none yet.
                ShineProtocol.packColors(tx, pattern[slot]);
                write(board, ShineProtocol.API_SLOT_COLOR + slot);
                budget--;
            }
            ShineProtocol.packAnimation(
                    tx, pattern[slot], start[slot], length[slot], brightness[slot]);
            write(board, ShineProtocol.API_SLOT_ANIM + slot);
            budget--;
            due[slot] = false;
        }
    }

    /** Sends {@link #tx}. A refused frame is counted, not retried: the next repeat covers it. */
    private void write(CAN to, int api) {
        if (to.writePacketNoThrow(api, tx, 8, 0) == 0) {
            framesSent++;
        } else {
            sendErrors++;
        }
    }
}
