package frc.spectrumLib.framework;

import org.littletonrobotics.junction.LoggedOpModeRobot;
import org.wpilib.command2.CommandScheduler;

/**
 * Base robot class for Spectrum robots on SystemCore: AdvantageKit's {@link LoggedOpModeRobot},
 * which runs the loop, records every input (including the Driver Station's selected OpMode), drives
 * replay, and runs the OpMode the Driver Station picked. Robots register their OpModes with {@code
 * addOpMode} in the constructor, passing dependencies through the supplier.
 */
public class SpectrumRobot extends LoggedOpModeRobot {

    /**
     * Loop length above which WPILib prints "Loop time of Xs overrun" plus a per-section epoch
     * breakdown to the Driver Station.
     *
     * <p>0.20 s keeps the console quiet in matches. It also hides everything below it: in the
     * 2026-09-05 roboRIO logs 60 to 90 percent of enabled loops ran over 25 ms and none of that
     * reached the console. AdvantageKit's {@code /RealOutputs/LoggedOpModeRobot/*} timings and the
     * {@code Scheduler/*} timers are the record of loop time.
     */
    public static final double LOOP_OVERRUN_WARNING_SECONDS = 0.20;

    /**
     * Starts the robot loop at {@code periodSeconds} and sets the loop overrun warnings to {@link
     * #LOOP_OVERRUN_WARNING_SECONDS}.
     *
     * @param periodSeconds the robot loop period; SystemCore runs 0.01 (100 Hz) comfortably
     */
    public SpectrumRobot(double periodSeconds) {
        super(periodSeconds);
        RobotLoop.setPeriodSeconds(periodSeconds);
        setLoopOverrunWarningTimeout(LOOP_OVERRUN_WARNING_SECONDS);
        CommandScheduler.getInstance().setPeriod(LOOP_OVERRUN_WARNING_SECONDS);
    }
}
