package frc.robot;

import frc.robot.opmodes.DemoTeleop;
import frc.robot.opmodes.DoNothingAuto;
import frc.robot.opmodes.DriveForwardAuto;
import frc.robot.opmodes.DriveTeleop;
import frc.robot.opmodes.WheelRadiusUtility;
import frc.robot.pilot.Pilot;
import frc.robot.subsystems.swerve.Swerve;
import frc.spectrumLib.framework.DesktopSim;
import frc.spectrumLib.framework.OpModeLoop;
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
import org.wpilib.hardware.hal.RobotMode;
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
 *   <li>{@link OpModeLoop#run()} - the running OpMode's requests
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
    private final DesktopSim desktopSim;
    private final OpModeLoop opModeLoop = new OpModeLoop();

    public Robot() {
        super(RunMode.MAIN_LOOP_PERIOD_SECONDS);
        // Before anything tunable exists, so a tunable made early fails safe (off).
        TuningMode.set(RunMode.TUNING_MODE);
        // Desktop sim (SpectrumSim, when on the classpath): fixes sim time before logging reads it.
        desktopSim =
                RunMode.CURRENT == RunMode.Mode.SIM
                        ? DesktopSim.load(System.getenv("MINISWERVE_SIM_SCRIPT") != null)
                        : null;
        if (RunMode.CURRENT == RunMode.Mode.SIM && desktopSim == null) {
            // Without SpectrumSim the robot would run with no physics and look broken.
            throw new IllegalStateException(
                    "SpectrumSim is not on the classpath: run the sim with ./gradlew run or WPILib's"
                            + " Simulate Robot Code");
        }
        LoggingSetup.start(this);
        RobotSettings.start(
                Filesystem.getDeployDirectory().toPath().resolve("settings"),
                "MiniSwerve",
                RunMode.CURRENT == RunMode.Mode.REAL ? LOCAL_SETTINGS : null,
                new NetworkTablesTransport(RobotSettings.NT_PREFIX));
        // Subsystems are built here, after settings and before the RT priority below.
        // Sim only: devices the scripted sim can unplug, reboot or make fail.
        SimFaults simFaults = RunMode.CURRENT == RunMode.Mode.SIM ? new SimFaults() : null;
        swerve = new Swerve(simFaults, desktopSim);
        simScript = SimScript.fromEnvironment(simFaults);
        // The pilot only binds controls; the bindings keep it alive.
        new Pilot(swerve);
        registerOpModes();
        // Before the main thread goes real-time, so its thread does not inherit that.
        frc.spectrumLib.hardware.PhoenixBackground.start();
        if (desktopSim != null) {
            desktopSim.start(); // sim time moves once the robot loop is waiting for its first alarm
        }
        if (RunMode.CURRENT == RunMode.Mode.REAL) {
            setMainThreadPriority(RunMode.MAIN_THREAD_RT_PRIORITY);
        }
    }

    @Override
    public void robotPeriodic() {
        if (desktopSim != null) {
            desktopSim.loopStarted();
        }
        RobotLoop.next();
        if (simScript != null) {
            simScript.periodic();
        }
        RobotSettings.periodic();
        swerve.updateInputs();
        opModeLoop.run();
        CommandScheduler.getInstance().run();
        swerve.applyWantedState();
        Alert.periodic();
        Telemetry.periodic();
        if (desktopSim != null) {
            desktopSim.loopDone();
        }
    }

    /**
     * Everything the Driver Station can pick. Each supplier makes a fresh OpMode when it is chosen,
     * so an OpMode keeps no state between runs.
     */
    private void registerOpModes() {
        DemoTeleop.Limits demo = DemoTeleop.Limits.fromSettings();
        addOpMode(RobotMode.TELEOPERATED, "Drive", "Match", "Full-speed driving", DriveTeleop::new);
        addOpMode(
                RobotMode.TELEOPERATED,
                "Demo",
                "Demo",
                "Driving with speed and turn capped, for visitors",
                () -> new DemoTeleop(swerve, demo, opModeLoop));
        addOpMode(RobotMode.AUTONOMOUS, "Do nothing", "Match", "Sits still", DoNothingAuto::new);
        addOpMode(
                RobotMode.AUTONOMOUS,
                "Drive forward",
                "Test",
                "Drives ahead at 1 m/s for 1.5 s, then holds still",
                () -> new DriveForwardAuto(swerve, opModeLoop));
        addOpMode(
                RobotMode.UTILITY,
                "Wheel radius",
                "Calibration",
                "Spins in place to measure the wheel radius",
                () -> new WheelRadiusUtility(swerve));
        publishOpModes();
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
