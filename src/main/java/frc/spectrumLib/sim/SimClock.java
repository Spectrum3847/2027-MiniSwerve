package frc.spectrumLib.sim;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleConsumer;
import org.wpilib.hardware.hal.simulation.SimulatorJNI;
import org.wpilib.simulation.SimHooks;
import org.wpilib.system.RobotController;

/**
 * One clock for the whole desktop sim, so a run repeats exactly and can run faster than real time.
 * It pauses WPILib's sim timing and steps everything in a fixed order, one tick (1 ms) at a time:
 *
 * <ol>
 *   <li>physics steppers (the drivetrain's, later each mechanism's), with the TalonFX models;
 *   <li>WPILib's clock ({@link SimHooks#stepTiming}), which runs the robot's main loop when it is
 *       due and returns once that loop is waiting again;
 *   <li>participant threads due at this tick (the 250 Hz swerve thread), which each run one cycle
 *       and wait again before the next tick.
 * </ol>
 *
 * <p>Nothing advances while any of them is still working, so how busy the computer is no longer
 * changes the result. {@code speed} paces it: 1 is real time (driving with a gamepad), 0 is as fast
 * as it goes (scripts and tests).
 *
 * <p>The robot calls {@link #loopStarted} and {@link #loopDone} around each main loop, so the clock
 * also waits for a loop WPILib runs without an alarm (the first one, or one that fell behind).
 *
 * <p>Index: {@link #addStepper} | {@link #join} / {@link Participant#awaitTick} | {@link
 * #loopStarted} / {@link #loopDone} | {@link #start} | the tick loop
 */
public final class SimClock {
    /** One tick (s): the TalonFX's control rate. */
    public static final double TICK_SECONDS = 0.001;

    private static final long TICK_NANOS = 1_000_000L;

    /**
     * Sim time when the clock is built (ns). Fixed, so every run computes the same time differences
     * with the same rounding: a start taken from the wall clock made runs drift apart.
     */
    private static final long START_NANOS = 10_000_000_000L;

    private final double speed;
    private final List<DoubleConsumer> steppers = new ArrayList<>();
    private final List<Participant> participants = new ArrayList<>();
    private final Object lock = new Object();
    private long tick;
    private long loopsStarted;
    private long loopsDone;
    private volatile boolean running;

    /**
     * Pauses WPILib's sim timing and sets it to a fixed start, so no time passes until the clock
     * runs. Build it first in robot init, before logging starts or anything reads the time.
     *
     * @param speed multiple of real time; 0 for as fast as possible
     */
    public SimClock(double speed) {
        this.speed = speed;
        SimHooks.pauseTiming();
        SimulatorJNI.stepTimingAsync(START_NANOS - RobotController.getMonotonicTime());
    }

    /** Runs {@code stepper} with the tick length each tick, on the clock's thread. */
    public void addStepper(DoubleConsumer stepper) {
        synchronized (lock) {
            steppers.add(stepper);
        }
    }

    /** A thread the clock waits for: it runs between ticks and parks in {@link #awaitTick}. */
    public final class Participant {
        private long wakeTick;
        private boolean parked;

        private Participant() {}

        /**
         * Parks this thread until the clock reaches {@code target} ticks; returns the tick it woke
         * at. The clock does not move on while this thread is between wake-ups.
         */
        public long awaitTick(long target) {
            synchronized (lock) {
                wakeTick = target;
                parked = true;
                lock.notifyAll();
                while (parked) {
                    try {
                        lock.wait();
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                        parked = false;
                    }
                }
                return tick;
            }
        }

        /** Ticks so far (the participant's own view, between waits). */
        public long now() {
            synchronized (lock) {
                return tick;
            }
        }
    }

    /** Registers a participant. Call before {@link #start}, from the thread's owner. */
    public Participant join() {
        synchronized (lock) {
            Participant p = new Participant();
            participants.add(p);
            return p;
        }
    }

    /**
     * Main thread, first thing each robot loop. WPILib runs a loop without waiting for its alarm
     * when it is behind (always the first loop): the clock must not tick while one runs.
     */
    public void loopStarted() {
        synchronized (lock) {
            loopsStarted++;
        }
    }

    /** Main thread, last thing each robot loop. */
    public void loopDone() {
        synchronized (lock) {
            loopsDone++;
            lock.notifyAll();
        }
    }

    /** Seconds of sim time a number of ticks is. */
    public static double seconds(long ticks) {
        return ticks * TICK_SECONDS;
    }

    /**
     * Starts the clock thread. It waits for the robot program to start its loop, then ticks until
     * the program ends.
     */
    public void start() {
        running = true;
        Thread t = new Thread(this::run, "SimClock");
        t.setDaemon(true);
        t.start();
    }

    private void run() {
        // No time passes until the robot's first loop has run (it runs without an alarm).
        synchronized (lock) {
            while (loopsDone == 0 && running) {
                try {
                    lock.wait();
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    return;
                }
            }
        }
        long wallStart = System.nanoTime();
        long ticks = 0;
        while (running) {
            awaitAllParked();
            for (DoubleConsumer s : steppers) {
                s.accept(TICK_SECONDS);
            }
            SimulatorJNI.stepTiming(TICK_NANOS);
            ticks++;
            synchronized (lock) {
                tick = ticks;
                for (Participant p : participants) {
                    if (p.parked && p.wakeTick <= ticks) {
                        p.parked = false;
                    }
                }
                lock.notifyAll();
            }
            if (speed > 0) {
                long due = wallStart + (long) (seconds(ticks) / speed * 1e9);
                long wait;
                while ((wait = due - System.nanoTime()) > 0) {
                    java.util.concurrent.locks.LockSupport.parkNanos(wait);
                }
            }
        }
    }

    private void awaitAllParked() {
        synchronized (lock) {
            while (!allParked() || loopsStarted > loopsDone) {
                try {
                    lock.wait();
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    running = false;
                    return;
                }
            }
        }
    }

    private boolean allParked() {
        for (Participant p : participants) {
            if (!p.parked) {
                return false;
            }
        }
        return true;
    }

    /** Stops ticking (tests). */
    public void stop() {
        running = false;
    }
}
