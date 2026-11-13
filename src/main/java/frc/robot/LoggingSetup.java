package frc.robot;

import frc.spectrumLib.telemetry.Alert;
import frc.spectrumLib.telemetry.DashboardReceiver;
import frc.spectrumLib.telemetry.Telemetry;
import java.io.File;
import java.util.UUID;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedOpModeRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.wpilib.util.Alert.Level;

/**
 * Starts AdvantageKit: metadata, where the log goes, and in replay where it comes from. Runs before
 * anything records an input.
 *
 * <p>Every log records the git SHA, branch and dirty flag so a replay can be matched to the code
 * that wrote it (strict replay: see docs/architecture.md).
 */
final class LoggingSetup {
    private LoggingSetup() {}

    /** USB stick if one is mounted, otherwise SystemCore's own storage. */
    private static final String USB_LOGS = "/U/logs";

    private static final String LOCAL_LOGS = "/home/systemcore/logs";

    static void start(LoggedOpModeRobot robot) {
        Logger.recordMetadata("Robot", "MiniSwerve");
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        // SystemCore has no battery-backed clock; a random id per boot tells logs apart.
        Logger.recordMetadata("BootId", UUID.randomUUID().toString());
        Logger.recordMetadata("RunMode", RunMode.CURRENT.name());
        Logger.recordMetadata("MainLoopSeconds", Double.toString(RunMode.MAIN_LOOP_PERIOD_SECONDS));
        Logger.recordMetadata("TuningMode", Boolean.toString(RunMode.TUNING_MODE));
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        Logger.recordMetadata(
                "GitDirty",
                switch (BuildConstants.DIRTY) {
                    case 0 -> "All changes committed";
                    case 1 -> "Uncommitted changes";
                    default -> "Unknown";
                });

        switch (RunMode.CURRENT) {
            case REAL -> {
                // The first folder that can really be written: a full or read-only stick used to
                // mean a match with no log and nothing saying so.
                boolean stick = new File("/U").isDirectory();
                String folder =
                        stick && writable(USB_LOGS)
                                ? USB_LOGS
                                : writable(LOCAL_LOGS) ? LOCAL_LOGS : null;
                Logger.recordMetadata("LogFolder", folder == null ? "none" : folder);
                if (folder != null) {
                    Logger.addDataReceiver(new WPILOGWriter(folder));
                }
                if (folder == null) {
                    new Alert("No log folder can be written: this run is not logged", Level.HIGH)
                            .set(true);
                } else if (stick && !folder.equals(USB_LOGS)) {
                    new Alert(
                                    "USB stick cannot be written: logging to the robot's storage",
                                    Level.MEDIUM)
                            .set(true);
                }
                Logger.addDataReceiver(new DashboardReceiver(new NT4Publisher(), ntEveryN()));
            }
            case SIM -> {
                Logger.addDataReceiver(
                        new WPILOGWriter(
                                System.getenv().getOrDefault("MINISWERVE_SIM_LOG_DIR", "logs")));
                Logger.addDataReceiver(new DashboardReceiver(new NT4Publisher(), ntEveryN()));
            }
            case REPLAY -> {
                robot.setUseTiming(false);
                String path = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(path));
                Logger.addDataReceiver(
                        new WPILOGWriter(LogFileUtil.addPathSuffix(path, "_replay")));
            }
        }
        Logger.start();
        // Sim mirrors every output to NT for AdvantageScope; the robot sends dashboard keys only.
        Telemetry.startDashboard(RunMode.CURRENT == RunMode.Mode.SIM);
    }

    /** Whether a file can be made in {@code folder} (made if missing). Boot only: file I/O. */
    private static boolean writable(String folder) {
        try {
            File dir = new File(folder);
            if (!dir.isDirectory() && !dir.mkdirs()) {
                return false;
            }
            File probe = File.createTempFile("probe", ".tmp", dir);
            return probe.delete() || !probe.exists();
        } catch (java.io.IOException | RuntimeException e) {
            return false;
        }
    }

    /** Main loops per NetworkTables publish, for about 50 Hz at any loop rate. */
    private static int ntEveryN() {
        return Math.max(1, (int) Math.round(0.02 / RunMode.MAIN_LOOP_PERIOD_SECONDS));
    }
}
