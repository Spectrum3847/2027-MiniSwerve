package frc.spectrumLib.leds;

import static org.wpilib.units.Units.Microseconds;

import java.util.Arrays;
import java.util.function.LongSupplier;
import org.wpilib.hardware.bus.CAN;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.hardware.hal.can.CANReceiveMessage;
import org.wpilib.system.RobotController;

/**
 * A SpectrumShine board on CAN through WPILib's {@link CAN} (the board uses the team-use
 * manufacturer and miscellaneous device type, WPILib's defaults). Reads take the latest frame the
 * HAL already has. Writes hand a frame to the HAL's CAN writer, which normally takes microseconds;
 * to keep it that way on a bus where nothing answers, a silent board is sent far less (below).
 *
 * <p>The board is classic CAN only: it must be on a bus with no CAN FD traffic (SpectrumShine
 * SPEC_V3). SystemCore's own buses are classic today.
 *
 * <p>Sending: a slot goes out when it changes, and every slot is repeated once a second, so a board
 * that lost power, or missed a frame, shows the right thing again within a second without the robot
 * having to notice. Repeats do not restart an animation: the board's animations run on time, and a
 * frame equal to what it has changes nothing. At most {@link #FRAMES_PER_LOOP} frames leave per
 * loop.
 *
 * <p>While the board is silent the repeat slows to every {@link #SILENT_RESEND_NANOS five seconds}
 * (changes still go out as they happen): enough to bring up a board whose status frames are off or
 * lost, without filling the transmit queue of a bus with nobody on it.
 *
 * <p>When the board appears (first status frame, again after silence, or it announces a boot) it is
 * told how many LEDs are on each output and asked for its firmware version. The LED counts are set
 * in the board's RAM only: nothing is saved to its flash from here.
 *
 * <p>Once a second the robot's clock goes to every board (device id 63), so animations on several
 * boards stay in step and the board's event times match the log.
 *
 * <p>Index: {@link Bus} | constructors | {@link #updateInputs} | {@link #setSlot} | {@link #send}
 */
public final class ShineIOCan implements ShineIO {
    /** Frames per loop at most: a full picture change is spread over a few loops. */
    static final int FRAMES_PER_LOOP = 4;

    /** The board sends status every 100 ms by default; silent this long is disconnected. */
    static final long CONNECTED_NANOS = 500_000_000L;

    static final long TIME_SYNC_NANOS = 1_000_000_000L;
    static final long RESEND_NANOS = 1_000_000_000L;
    static final long SILENT_RESEND_NANOS = 5_000_000_000L;

    /** A version frame this long after our ping is not its answer: the board booted. */
    static final long PING_REPLY_NANOS = 1_000_000_000L;

    /** The CAN bus as this class uses it, so tests can stand in for it. */
    interface Bus {
        /**
         * The newest frame of an API from the board, once.
         *
         * @param data filled with its 8 bytes
         * @return whether there was a new 8-byte frame
         */
        boolean read(int api, byte[] data);

        /**
         * Queues one 8-byte frame.
         *
         * @param toEveryBoard to device id 63 instead of the board
         * @return whether the bus took it
         */
        boolean write(boolean toEveryBoard, int api, byte[] data);
    }

    /** The bus through WPILib. */
    private static final class WpilibBus implements Bus {
        private final CAN board;
        private final CAN everyBoard;
        private final CANReceiveMessage rx = new CANReceiveMessage();

        WpilibBus(CANPort port, int id) {
            board = new CAN(port, id);
            everyBoard = new CAN(port, ShineProtocol.BROADCAST_ID);
        }

        @Override
        public boolean read(int api, byte[] data) {
            if (!board.readPacketNew(api, rx) || rx.length < 8) {
                return false;
            }
            System.arraycopy(rx.data, 0, data, 0, 8);
            return true;
        }

        @Override
        public boolean write(boolean toEveryBoard, int api, byte[] data) {
            // Flags 0: a classic CAN data frame.
            return (toEveryBoard ? everyBoard : board).writePacketNoThrow(api, data, 8, 0) == 0;
        }
    }

    private final Bus bus;
    private final LongSupplier nanos;
    private final LongSupplier robotMicros;
    private final int countA;
    private final int countB;

    private final byte[] rx = new byte[8];
    private final byte[] tx = new byte[8];

    // What each slot should show, and whether the board has been told.
    private final LedPattern[] pattern = new LedPattern[ShineProtocol.SLOTS];
    private final int[] start = new int[ShineProtocol.SLOTS];
    private final int[] length = new int[ShineProtocol.SLOTS];
    private final int[] brightness = new int[ShineProtocol.SLOTS];
    private final boolean[] due = new boolean[ShineProtocol.SLOTS];

    // Elapsed wall time (System.nanoTime on the robot), not the robot clock: these are bus
    // timeouts.
    private long lastStatusNanos;
    private boolean statusSeen;
    private boolean connected;
    private long lastTimeSyncNanos;
    private long lastResendNanos;
    private boolean sentOnce;

    private static final int SETUP_COUNT_A = 0;
    private static final int SETUP_COUNT_B = 1;
    private static final int SETUP_PING = 2;
    private static final int SETUP_DONE = 3;

    /** The next setup frame to send after the board appears. */
    private int setupStep = SETUP_COUNT_A;

    /** A version frame we asked for is on its way, so the next one is not a boot. */
    private boolean pingOutstanding;

    private long pingNanos;

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
     * @param port the CAN bus the board is on (classic CAN, no CAN FD traffic)
     * @param id its CAN id, 0..62 (a new board is 0)
     * @param ledsOnA LEDs on output A
     * @param ledsOnB LEDs on output B (they follow A's in the index space)
     */
    public ShineIOCan(CANPort port, int id, int ledsOnA, int ledsOnB) {
        this(
                new WpilibBus(port, checkedId(id)),
                System::nanoTime,
                // The clock the log is on. Through the unit, since WPILib 2027's raw long is not
                // microseconds.
                () -> (long) RobotController.getMeasureTime().in(Microseconds),
                ledsOnA,
                ledsOnB);
    }

    /**
     * @param nanos elapsed wall time (ns)
     * @param robotMicros the robot's log clock (us)
     */
    ShineIOCan(Bus bus, LongSupplier nanos, LongSupplier robotMicros, int ledsOnA, int ledsOnB) {
        this.bus = bus;
        this.nanos = nanos;
        this.robotMicros = robotMicros;
        countA = Math.max(0, Math.min(ShineProtocol.MAX_LEDS, ledsOnA));
        countB = Math.max(0, Math.min(ShineProtocol.MAX_LEDS - countA, ledsOnB));
        Arrays.fill(pattern, LedPattern.OFF);
        // Tell a board that is already running what every slot is, unused ones included: it may
        // still hold slots from before a code restart.
        Arrays.fill(due, true);
    }

    private static int checkedId(int id) {
        if (id < 0 || id >= ShineProtocol.BROADCAST_ID) {
            throw new IllegalArgumentException("SpectrumShine id " + id + " is not 0..62");
        }
        return id;
    }

    @Override
    public void updateInputs(ShineInputs inputs) {
        long now = nanos.getAsLong();
        if (bus.read(ShineProtocol.API_STATUS, rx)) {
            faults = ShineProtocol.u16(rx, 0);
            stickyFaults = ShineProtocol.u16(rx, 2);
            limitReason = rx[4] & 0xFF;
            temperatureC = rx[5]; // signed
            scale = rx[6] & 0xFF;
            flags = rx[7] & 0xFF;
            lastStatusNanos = now;
            statusSeen = true;
        }
        if (bus.read(ShineProtocol.API_POWER, rx)) {
            batteryMillivolts = ShineProtocol.u16(rx, 0);
            railMillivolts = ShineProtocol.u16(rx, 2);
            measuredMilliamps = ShineProtocol.u16(rx, 4);
            estimatedMilliamps = ShineProtocol.u16(rx, 6);
        }
        if (bus.read(ShineProtocol.API_VERSION, rx)) {
            // Only when a version frame arrives (boot or our ping), so the strings are not per
            // loop.
            firmware = (rx[0] & 0xFF) + "." + (rx[1] & 0xFF);
            uid = String.format("%08X", ShineProtocol.u32(rx, 4));
            if (pingOutstanding && now - pingNanos < PING_REPLY_NANOS) {
                pingOutstanding = false;
            } else {
                // Unasked: the board just booted. Its slots may be empty and its LED counts are
                // back to what its flash holds.
                boots++;
                startSetup();
            }
        }
        boolean nowConnected = statusSeen && now - lastStatusNanos < CONNECTED_NANOS;
        if (nowConnected && !connected) {
            startSetup();
        }
        connected = nowConnected;

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

    /** The board is new to us: LED counts, version, and every slot again. */
    private void startSetup() {
        setupStep = SETUP_COUNT_A;
        pingOutstanding = false;
        Arrays.fill(due, true);
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
        long now = nanos.getAsLong();
        int budget = FRAMES_PER_LOOP;
        if (!sentOnce || now - lastTimeSyncNanos >= TIME_SYNC_NANOS) {
            lastTimeSyncNanos = now;
            ShineProtocol.packTime(tx, robotMicros.getAsLong());
            write(true, ShineProtocol.API_TIME_SYNC);
            budget--;
        }
        boolean repeat =
                !sentOnce
                        || now - lastResendNanos
                                >= (connected ? RESEND_NANOS : SILENT_RESEND_NANOS);
        sentOnce = true;
        if (repeat) {
            lastResendNanos = now;
            Arrays.fill(due, true);
            if (!connected) {
                setupStep = SETUP_COUNT_A; // a board we cannot hear may have just been plugged in
            }
        }
        while (setupStep != SETUP_DONE && budget > 0) {
            switch (setupStep) {
                case SETUP_COUNT_A -> {
                    ShineProtocol.packConfig(tx, ShineProtocol.KEY_COUNT_A, countA);
                    write(false, ShineProtocol.API_SET_CONFIG);
                }
                case SETUP_COUNT_B -> {
                    ShineProtocol.packConfig(tx, ShineProtocol.KEY_COUNT_B, countB);
                    write(false, ShineProtocol.API_SET_CONFIG);
                }
                case SETUP_PING -> {
                    Arrays.fill(tx, (byte) 0);
                    pingOutstanding = true;
                    pingNanos = now;
                    write(false, ShineProtocol.API_PING);
                }
                default -> {}
            }
            setupStep++;
            budget--;
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
                write(false, ShineProtocol.API_SLOT_COLOR + slot);
                budget--;
            }
            ShineProtocol.packAnimation(
                    tx, pattern[slot], start[slot], length[slot], brightness[slot]);
            write(false, ShineProtocol.API_SLOT_ANIM + slot);
            budget--;
            due[slot] = false;
        }
    }

    /** Sends {@link #tx}. A refused frame is counted, not retried: the next repeat covers it. */
    private void write(boolean toEveryBoard, int api) {
        if (bus.write(toEveryBoard, api, tx)) {
            framesSent++;
        } else {
            sendErrors++;
        }
    }
}
