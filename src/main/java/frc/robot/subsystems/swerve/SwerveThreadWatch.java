package frc.robot.subsystems.swerve;

import frc.spectrumLib.framework.RobotLoop;
import frc.spectrumLib.swerve.SwerveInputs;
import frc.spectrumLib.swerve.SwerveThread;
import frc.spectrumLib.telemetry.Alert;
import org.littletonrobotics.junction.Logger;
import org.wpilib.util.Alert.Level;

/**
 * The main loop's watch on the swerve thread: without that thread nothing drives. If it has ended
 * it is started again (it re-plans from the measured wheels); if it is alive but not cycling, or it
 * has had to stop itself or drop a non-finite request, an alert says so.
 *
 * <p>Reads logged inputs, so replay shows the same alerts; only the restart itself needs hardware.
 * Logged under {@code Swerve/Thread/}: {@code Restarts}, {@code Stalled}.
 */
final class SwerveThreadWatch {
    /** No completed swerve cycle for this long means the thread is stuck (s). */
    private static final double STALL_SECONDS = 0.25;

    private final SwerveThread thread;
    private final boolean hasHardware;
    private long watchedCycles = -1;
    private int loopsWithoutCycle;
    private long restarts;
    private long alertedRestarts;
    private long alertedStale;
    private long alertedNonFinite;
    private final Alert restartAlert = new Alert("", Level.HIGH);
    private final Alert stallAlert =
            new Alert("Swerve thread is not cycling: the robot cannot drive", Level.HIGH);
    private final Alert staleRequestAlert = new Alert("", Level.MEDIUM);
    private final Alert nonFiniteAlert = new Alert("", Level.MEDIUM);

    SwerveThreadWatch(SwerveThread thread, boolean hasHardware) {
        this.thread = thread;
        this.hasHardware = hasHardware;
    }

    /** Once per main loop, after the swerve inputs are processed. */
    void update(SwerveInputs inputs) {
        if (inputs.threadCycles != watchedCycles) {
            watchedCycles = inputs.threadCycles;
            loopsWithoutCycle = 0;
        } else {
            loopsWithoutCycle++;
        }
        boolean stalled =
                loopsWithoutCycle * RobotLoop.periodSeconds() > STALL_SECONDS
                        && inputs.threadCycles > 0;
        if (!inputs.threadAlive && hasHardware) {
            thread.start();
            restarts++;
        }
        // Times the swerve thread ended and was started again, since boot (count).
        Logger.recordOutput("Swerve/Thread/Restarts", restarts);
        // The swerve thread has not completed a cycle for 0.25 s (bool).
        Logger.recordOutput("Swerve/Thread/Stalled", stalled);
        if (restarts != alertedRestarts) {
            alertedRestarts = restarts;
            restartAlert.setText("Swerve thread ended and was restarted: " + restarts);
        }
        restartAlert.set(restarts > 0);
        stallAlert.set(stalled);
        if (inputs.staleRequestCycles != alertedStale) {
            alertedStale = inputs.staleRequestCycles;
            staleRequestAlert.setText(
                    "Main loop stalled: swerve stopped itself for "
                            + inputs.staleRequestCycles
                            + " cycles");
        }
        staleRequestAlert.set(inputs.staleRequestCycles > 0);
        if (inputs.nonFiniteDropped != alertedNonFinite) {
            alertedNonFinite = inputs.nonFiniteDropped;
            nonFiniteAlert.setText(
                    "Swerve dropped non-finite requests or plans: " + inputs.nonFiniteDropped);
        }
        nonFiniteAlert.set(inputs.nonFiniteDropped > 0);
    }
}
