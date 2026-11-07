package frc.robot;

import frc.spectrumLib.sim.SimClock;
import org.wpilib.framework.RobotBase;

/**
 * How this process is running, and the timing every part of the robot agrees on.
 *
 * <p>Index: {@link Mode} · {@link #CURRENT} · loop rates · {@link #TUNING_MODE}
 */
public final class RunMode {
    private RunMode() {}

    public enum Mode {
        /** On the robot. */
        REAL,
        /** Desktop physics simulation. */
        SIM,
        /** Desktop AdvantageKit replay of a log. Nothing touches hardware. */
        REPLAY
    }

    /** Desktop runs replay with {@code -Dminiswerve.replay=true} or {@code MINISWERVE_REPLAY=1}. */
    private static final boolean REPLAY_REQUESTED =
            Boolean.getBoolean("miniswerve.replay")
                    || "1".equals(System.getenv("MINISWERVE_REPLAY"));

    public static final Mode CURRENT =
            RobotBase.isReal() ? Mode.REAL : REPLAY_REQUESTED ? Mode.REPLAY : Mode.SIM;

    /** Main robot loop: 100 Hz. Code reads {@code RobotLoop.periodSeconds()}, not this. */
    public static final double MAIN_LOOP_PERIOD_SECONDS = 0.01;

    /**
     * Real-time priority of the main thread on the robot, 0 for none. 1 is the lowest real-time
     * level, which sits below Phoenix's CAN threads (2-3). FM measured it fixes late loops on
     * SystemCore; going above Phoenix starved CAN on the 2026 roboRIO.
     */
    public static final int MAIN_THREAD_RT_PRIORITY = 1;

    /**
     * Dashboards may change Robot Settings (applied while disabled, off the field). Turn on for a
     * tuning session only; CI fails a merge to main that leaves it on.
     */
    public static final boolean TUNING_MODE = false;

    /**
     * The sim clock for this run, or null (robot, replay, or {@code MINISWERVE_SIM_DEVICES=phoenix}
     * for Phoenix's own simulator on the wall clock). Pace from {@code MINISWERVE_SIM_SPEED}: a
     * multiple of real time, or {@code max}; default max when a sim script runs, else real time (a
     * person driving).
     */
    public static SimClock simClock() {
        if (CURRENT != Mode.SIM || "phoenix".equals(System.getenv("MINISWERVE_SIM_DEVICES"))) {
            return null;
        }
        String speed = System.getenv("MINISWERVE_SIM_SPEED");
        if (speed == null || speed.isBlank()) {
            speed = System.getenv("MINISWERVE_SIM_SCRIPT") != null ? "max" : "1";
        }
        return new SimClock("max".equals(speed) ? 0 : Double.parseDouble(speed));
    }

    /** Whether hardware, real or simulated, exists: false in replay. */
    public static boolean hasHardware() {
        return CURRENT != Mode.REPLAY;
    }
}
