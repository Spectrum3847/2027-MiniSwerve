package frc.robot;

import frc.robot.pilot.Pilot;
import frc.robot.subsystems.swerve.Swerve;
import frc.spectrumLib.framework.RobotLoop;
import frc.spectrumLib.framework.SpectrumRobot;
import frc.spectrumLib.hardware.SimFaults;
import frc.spectrumLib.robotsettings.NetworkTablesTransport;
import frc.spectrumLib.robotsettings.RobotSettings;
import frc.spectrumLib.robotsettings.TuningMode;
import frc.spectrumLib.telemetry.Alert;
import frc.spectrumLib.telemetry.Telemetry;
import java.nio.file.Path;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.system.Filesystem;
import org.wpilib.util.Alert.Level;

/**
 * The robot: boots everything in order and runs one main loop with a fixed, readable order.
 *
 * <p>There is no RobotContainer. Robot owns each subsystem as a plain field and passes what a part
 * needs into its constructor; nothing is a static global.
 *
 * <p>Main loop order (100 Hz), see {@link #robotPeriodic()}:
 *
 * <ol>
 *   <li>{@link RobotLoop#next()} - loop counter
 *   <li>{@link RobotSettings#periodic()} - apply live tuning, log values (inputs)
 *   <li>{@link Swerve#updateInputs()} - drain 250 Hz samples, log, odometry
 *   <li>{@link CommandScheduler#run()} - wanted-state requests and commands
 *   <li>{@link Swerve#applyWantedState()} - wanted state to the swerve thread
 *   <li>alerts and telemetry
 * </ol>
 */
public class Robot extends SpectrumRobot {
    /** Folder on the robot that survives deploys, for settings saved in the pit. */
    private static final Path LOCAL_SETTINGS = Path.of("/home/systemcore/settings");

    private final Alert mainThreadPriorityAlert = new Alert("", Level.MEDIUM);
    private final Swerve swerve;
    private final SimScript simScript;

    public Robot() {
        super(RunMode.MAIN_LOOP_PERIOD_SECONDS);
        // Before anything tunable exists, so a tunable made early fails safe (off).
        TuningMode.set(RunMode.TUNING_MODE);
        LoggingSetup.start(this);
        RobotSettings.start(
                Filesystem.getDeployDirectory().toPath().resolve("settings"),
                "MiniSwerve",
                RunMode.CURRENT == RunMode.Mode.REAL ? LOCAL_SETTINGS : null,
                new NetworkTablesTransport(RobotSettings.NT_PREFIX));
        // Subsystems are built here, after settings and before the RT priority below.
        // Sim only: devices the scripted sim can unplug, reboot or make fail.
        SimFaults simFaults = RunMode.CURRENT == RunMode.Mode.SIM ? new SimFaults() : null;
        swerve = new Swerve(simFaults);
        simScript = SimScript.fromEnvironment(simFaults);
        // The pilot only binds controls; the bindings keep it alive.
        new Pilot(swerve);
        // Before the main thread goes real-time, so its thread does not inherit that.
        frc.spectrumLib.hardware.PhoenixBackground.start();
        if (RunMode.CURRENT == RunMode.Mode.REAL) {
            setMainThreadPriority(RunMode.MAIN_THREAD_RT_PRIORITY);
        }
    }

    @Override
    public void robotPeriodic() {
        RobotLoop.next();
        if (simScript != null) {
            simScript.periodic();
        }
        RobotSettings.periodic();
        swerve.updateInputs();
        CommandScheduler.getInstance().run();
        swerve.applyWantedState();
        Alert.periodic();
        Telemetry.periodic();
    }

    @SuppressWarnings("deprecation") // WPILib deprecates it as "know what you are doing"
    private void setMainThreadPriority(int priority) {
        if (priority <= 0) {
            return;
        }
        // Read it back: SystemCore has reported false even when the priority took.
        org.wpilib.system.Threads.setCurrentThreadPriority(priority);
        int now = org.wpilib.system.Threads.getCurrentThreadPriority();
        // The main thread's real-time priority after setting it.
        Telemetry.log("System/MainThreadPriority", now);
        mainThreadPriorityAlert.setText("Main thread real-time priority " + priority + " failed");
        mainThreadPriorityAlert.set(now != priority);
    }
}
