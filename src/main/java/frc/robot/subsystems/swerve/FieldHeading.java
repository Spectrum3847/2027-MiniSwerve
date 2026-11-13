package frc.robot.subsystems.swerve;

import frc.spectrumLib.swerve.HeadingMemory;
import frc.spectrumLib.swerve.SwerveInputs;
import frc.spectrumLib.swerve.SwerveOdometry;
import frc.spectrumLib.swerve.SwerveSample;
import frc.spectrumLib.telemetry.Alert;
import org.littletonrobotics.junction.Logger;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;

/**
 * Which way "away from the driver" is, and keeping odometry's heading right about it without the
 * driver's help where we can:
 *
 * <ul>
 *   <li><b>Alliance:</b> the last alliance the Driver Station gave is kept, so a moment without one
 *       (a reconnect) cannot turn a red driver's sticks around.
 *   <li><b>At the start:</b> until the robot is first enabled (or the heading is set some other
 *       way) the robot is taken to face away from its own driver: heading 0 on blue, pi on red.
 *       Before this a red robot booted at 0 and drove backwards until Back was pressed.
 *   <li><b>After a code restart:</b> the heading saved by {@link HeadingMemory} is put back, if the
 *       gyro has not turned more than the robot could coast while the code was down (a gyro that
 *       lost power starts again near zero). The saved values are logged inputs, so replay restores
 *       the same heading.
 *   <li><b>Back button:</b> {@link #seed} makes the way the robot faces now "away from the driver".
 * </ul>
 *
 * <p>Every jump it puts into odometry's heading is summed in {@link #resets}, so the pose estimator
 * can be fed a heading without them.
 *
 * <p>Main thread. Logged under {@code Swerve/FieldHeading/}.
 */
final class FieldHeading {
    /**
     * Most the gyro may have turned while the code was down for a saved heading to be believed
     * (rad): two full turns. The robot is disabled then, so it only coasts or is pushed.
     */
    private static final double MAX_TURN_WHILE_DOWN = 4 * Math.PI;

    /** Loops to wait for a good gyro sample before giving up on the saved heading. */
    private static final int RESTORE_LOOPS = 1000;

    private final SwerveOdometry odometry;
    private final HeadingMemory memory;
    private final Runnable onJump;
    private final Alert restoredAlert =
            new Alert("Field heading restored after a code restart", Alert.Level.LOW);
    private Alliance alliance = Alliance.BLUE;
    private double assumedForward;
    private boolean headingSet;
    private boolean restoreDone;
    private int restoreLoops;
    private String restore = "none saved";
    private double resets;
    private double lastGoodYaw = Double.NaN;

    /**
     * @param memory where the heading is kept across a code restart
     * @param onJump run whenever the heading is moved (heading hold must let go)
     */
    FieldHeading(SwerveOdometry odometry, HeadingMemory memory, Runnable onJump) {
        this.odometry = odometry;
        this.memory = memory;
        this.onJump = onJump;
    }

    /**
     * CTRE's {@code setOperatorForwardDirection}, from the alliance: the field heading (rad) that
     * is "away from the driver". 0 on blue, pi on red; blue until an alliance has been seen.
     * Alliance is a Driver Station input, so replay sees the same value.
     */
    double operatorForward() {
        var now = MatchState.getAlliance();
        if (now.isPresent()) {
            alliance = now.get();
        }
        return alliance == Alliance.RED ? Math.PI : 0.0;
    }

    /** Sum of every jump put into odometry's heading (rad). */
    double resets() {
        return resets;
    }

    /**
     * Once per main loop, after odometry has taken this loop's samples.
     *
     * @param gyroOk the gyro is connected and configured (logged input)
     * @param enabled the robot is enabled (Driver Station input)
     * @param save this is a loop to save the heading on (a few times a second, hardware only)
     */
    void update(SwerveInputs inputs, boolean gyroOk, boolean enabled, boolean save) {
        if (inputs.count > 0) {
            SwerveSample last = inputs.samples[inputs.count - 1];
            if (gyroOk && last.gyroOk && Double.isFinite(last.yaw)) {
                lastGoodYaw = last.yaw;
                if (!restoreDone) {
                    restoreDone = true;
                    tryRestore(inputs, last.yaw);
                }
            }
        }
        if (!restoreDone && ++restoreLoops > RESTORE_LOOPS) {
            restoreDone = true;
            if (Double.isFinite(inputs.restoreYawOffset)) {
                restore = "not used: no gyro reading";
            }
        }
        if (!headingSet) {
            if (enabled) {
                headingSet = true; // from here on the heading is the driver's and the gyro's
            } else {
                double forward = operatorForward();
                if (forward != assumedForward) {
                    jumpTo(odometry.heading() + forward - assumedForward);
                    assumedForward = forward;
                }
            }
        }
        // Only a heading someone has stood behind is worth bringing back.
        if (save && headingSet && restoreDone && gyroOk && Double.isFinite(lastGoodYaw)) {
            memory.save(odometry.yawOffset(), lastGoodYaw);
        }
        // The saved heading from before a code restart: "restored", or why it was not used.
        Logger.recordOutput("Swerve/FieldHeading/Restore", restore);
        // The field heading that is "away from the driver" (rad): 0 blue, pi red.
        Logger.recordOutput("Swerve/FieldHeading/OperatorForward", assumedOrSeen());
        // The heading has been set by an enable, the Back button or a restore (bool); false means
        // it is still the starting guess (facing away from the driver).
        Logger.recordOutput("Swerve/FieldHeading/Set", headingSet);
    }

    private double assumedOrSeen() {
        return alliance == Alliance.RED ? Math.PI : 0.0;
    }

    private void tryRestore(SwerveInputs inputs, double yaw) {
        if (!Double.isFinite(inputs.restoreYawOffset) || !Double.isFinite(inputs.restoreYaw)) {
            return;
        }
        if (Math.abs(yaw - inputs.restoreYaw) > MAX_TURN_WHILE_DOWN) {
            restore = "not used: gyro yaw moved too far (gyro restarted?)";
            return;
        }
        jumpTo(yaw + inputs.restoreYawOffset);
        headingSet = true;
        restore = "restored";
        restoredAlert.set(true);
    }

    /** The Back button: the way the robot faces now becomes "away from the driver". */
    void seed() {
        jumpTo(operatorForward());
        headingSet = true;
    }

    private void jumpTo(double heading) {
        double before = odometry.heading();
        odometry.resetPose(odometry.x(), odometry.y(), heading);
        resets += odometry.heading() - before;
        onJump.run();
    }
}
