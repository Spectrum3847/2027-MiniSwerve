package frc.robot;

import frc.robot.config.BatterySettings;
import frc.robot.opmodes.DemoTeleop;
import frc.robot.opmodes.DoNothingAuto;
import frc.robot.opmodes.DriveForwardAuto;
import frc.robot.opmodes.DriveTeleop;
import frc.robot.opmodes.WheelRadiusUtility;
import frc.robot.pilot.Pilot;
import frc.robot.subsystems.pose.Localization;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionSettings;
import frc.spectrumLib.battery.BatteryNames;
import frc.spectrumLib.battery.BatteryTag;
import frc.spectrumLib.battery.BatteryTagIO;
import frc.spectrumLib.battery.BatteryTagIOSerial;
import frc.spectrumLib.battery.BatteryVoltageCheck;
import frc.spectrumLib.framework.DesktopSim;
import frc.spectrumLib.framework.LoopStage;
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
import org.wpilib.fields.Field;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.math.geometry.Transform3d;
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
 *   <li>{@link BatteryTag#periodic} - which battery is in (NFC tag, logged inputs)
 *   <li>{@link BatteryVoltageCheck#periodic} - low-battery warning at rest
 *   <li>vision, then localization
 *   <li>{@link OpModeLoop#run()} - the running OpMode's requests
 *   <li>{@link CommandScheduler#run()} - wanted-state requests and commands
 *   <li>{@link Swerve#applyWantedState()} - wanted state to the swerve thread
 *   <li>alerts and telemetry
 * </ol>
 *
 * <p>Every step runs as a {@link LoopStage}: if one throws, it is counted and alerted and the rest
 * of the loop still runs, so a bug in the battery reader or a camera never restarts the robot
 * mid-match (AGENTS.md "Finish the match"). If turning the wanted state into a drive request
 * throws, the sticks still drive, robot-relative ({@link Swerve#applyLimp}). Boot is the same: the
 * parts the robot can drive without (cameras, pose, the battery tag) are left out with an alert if
 * making them throws.
 *
 * <p>Index: constructor | {@link #robotPeriodic} | {@link #registerOpModes}
 */
public class Robot extends SpectrumRobot {
    /** Folder on the robot that survives deploys, for settings saved in the pit. */
    private static final Path LOCAL_SETTINGS = Path.of("/home/systemcore/settings");

    private final Alert mainThreadPriorityAlert = new Alert("", Level.MEDIUM);
    private final Swerve swerve;
    private final Vision vision;
    private final BatteryTag battery;
    private final BatteryVoltageCheck batteryVoltage;
    private final Localization localization;
    private final SimScript simScript;
    private final DesktopSim desktopSim;
    private final OpModeLoop opModeLoop = new OpModeLoop();

    // The loop's stages and their work, made once so the loop allocates nothing for them.
    private final LoopStage settingsStage = new LoopStage("Settings");
    private final LoopStage swerveInputsStage = new LoopStage("SwerveInputs");
    private final LoopStage batteryStage = new LoopStage("Battery");
    private final LoopStage visionStage = new LoopStage("Vision");
    private final LoopStage localizationStage = new LoopStage("Localization");
    private final LoopStage opModeStage = new LoopStage("OpMode");
    private final LoopStage commandsStage = new LoopStage("Commands");
    private final LoopStage swerveApplyStage = new LoopStage("SwerveApply");
    private final LoopStage limpStage = new LoopStage("SwerveLimp");
    private final LoopStage alertsStage = new LoopStage("Alerts");
    private final LoopStage telemetryStage = new LoopStage("Telemetry");
    private final Runnable settingsWork = RobotSettings::periodic;
    private final Runnable swerveInputsWork;
    private final Runnable batteryWork = this::batteryPeriodic;
    private final Runnable visionWork;
    private final Runnable localizationWork = this::localizationPeriodic;
    private final Runnable opModeWork = opModeLoop::run;
    private final Runnable commandsWork = CommandScheduler.getInstance()::run;
    private final Runnable swerveApplyWork;
    private final Runnable limpWork;
    private final Runnable alertsWork = Alert::periodic;
    private final Runnable telemetryWork =
            () -> {
                Telemetry.periodic();
                frc.spectrumLib.hardware.PhoenixBackground.logResults();
            };

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
        // Settings that fail to start leave every setting at its code default: still a robot.
        new LoopStage("Boot/Settings")
                .run(
                        () ->
                                RobotSettings.start(
                                        Filesystem.getDeployDirectory()
                                                .toPath()
                                                .resolve("settings"),
                                        "MiniSwerve",
                                        RunMode.CURRENT == RunMode.Mode.REAL
                                                ? LOCAL_SETTINGS
                                                : null,
                                        new NetworkTablesTransport(RobotSettings.NT_PREFIX)));
        // After settings, so files and pit-saved values apply (a field initializer would read
        // the code defaults).
        batteryVoltage =
                new BatteryVoltageCheck(
                        BatterySettings.LOW_VOLTS.get(), BatterySettings.SETTLE_SECONDS.get());
        // Subsystems are built here, after settings and before the RT priority below.
        // Sim only: devices the scripted sim can unplug, reboot or make fail.
        SimFaults simFaults = RunMode.CURRENT == RunMode.Mode.SIM ? new SimFaults() : null;
        if (simFaults != null) {
            // Sim scripts make a stage throw by "unplugging" Code/<stage>.
            LoopStage.setFaultInjector(name -> simFaults.unheard("Code/" + name));
        }
        // The drivetrain is the one part with no fallback: without it there is no robot.
        swerve = new Swerve(simFaults, desktopSim);
        swerveInputsWork = swerve::updateInputs;
        swerveApplyWork = swerve::applyWantedState;
        limpWork = swerve::applyLimp;
        simScript = SimScript.fromEnvironment(simFaults);
        // Everything below is optional: the robot drives without it.
        VisionSettings.Camera[] cameras = VisionSettings.cameras();
        Vision madeVision = new LoopStage("Boot/Vision").make(() -> new Vision(cameras));
        vision = madeVision;
        visionWork = madeVision != null ? madeVision::periodic : () -> {};
        Field field = new LoopStage("Boot/Field").make(() -> Field.loadField(VisionSettings.FIELD));
        localization =
                madeVision != null && field != null
                        ? new LoopStage("Boot/Localization")
                                .make(() -> new Localization(swerve, madeVision, field))
                        : null;
        if (desktopSim != null && madeVision != null && field != null) {
            new LoopStage("Boot/SimCameras")
                    .run(
                            () -> {
                                Transform3d[] mounts = new Transform3d[cameras.length];
                                for (int i = 0; i < cameras.length; i++) {
                                    mounts[i] = cameras[i].robotToCamera();
                                }
                                madeVision.expectFrames(
                                        desktopSim.cameras(field, madeVision.names(), mounts));
                            });
        }
        battery =
                new LoopStage("Boot/Battery")
                        .make(
                                () ->
                                        new BatteryTag(
                                                batteryReader(),
                                                BatteryNames.load(
                                                        Filesystem.getDeployDirectory()
                                                                .toPath()
                                                                .resolve("batteries.csv"))));
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

    /**
     * The battery tag reader: the USB NFC reader on the robot, a fixed sim battery on the desktop,
     * the log in replay.
     */
    private static BatteryTagIO batteryReader() {
        return switch (RunMode.CURRENT) {
            case REAL -> new BatteryTagIOSerial();
            case SIM -> BatteryTagIO.fixed("SIM00001", "sim-battery");
            case REPLAY -> BatteryTagIO.NONE;
        };
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
        settingsStage.run(settingsWork);
        swerveInputsStage.run(swerveInputsWork);
        batteryStage.run(batteryWork);
        visionStage.run(visionWork);
        localizationStage.run(localizationWork);
        opModeStage.run(opModeWork);
        commandsStage.run(commandsWork);
        if (!swerveApplyStage.run(swerveApplyWork)) {
            // The wanted-state path threw: the sticks still drive, by the shortest path there is.
            limpStage.run(limpWork);
        }
        alertsStage.run(alertsWork);
        telemetryStage.run(telemetryWork);
        if (desktopSim != null) {
            desktopSim.loopDone();
        }
    }

    private void batteryPeriodic() {
        boolean disabled = org.wpilib.driverstation.RobotState.isDisabled();
        if (battery != null) {
            battery.periodic(disabled);
        }
        batteryVoltage.periodic(
                swerve.supplyVoltage(), disabled, org.wpilib.system.Timer.getTimestamp());
    }

    private void localizationPeriodic() {
        if (localization != null) {
            localization.periodic(vision, org.wpilib.system.Timer.getTimestamp());
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
