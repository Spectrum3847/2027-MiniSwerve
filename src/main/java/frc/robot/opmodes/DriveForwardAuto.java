package frc.robot.opmodes;

import frc.robot.subsystems.swerve.Swerve;
import org.wpilib.opmode.OpMode;
import org.wpilib.system.Timer;

/**
 * The simplest real auto: drive straight ahead, robot-relative, for a fixed time, then stop. A
 * placeholder until the Choreo auto builder makes match autos; also a quick check that auto runs.
 */
public final class DriveForwardAuto implements OpMode {
    private static final String NAME = "driveForwardAuto";
    private static final double SPEED = 1.0; // m/s
    private static final double SECONDS = 1.5;

    private final Swerve swerve;
    private double start;

    public DriveForwardAuto(Swerve swerve) {
        this.swerve = swerve;
    }

    @Override
    public void start() {
        // Robot clock: a logged input, so replay ends the drive on the same loop.
        start = Timer.getTimestamp();
    }

    @Override
    public void periodic() {
        if (Timer.getTimestamp() - start < SECONDS) {
            swerve.driveRobotRelative(SPEED, 0, 0, NAME);
        } else {
            swerve.stopRobotRelative(NAME);
        }
    }

    @Override
    public void end() {
        swerve.stopRobotRelative(NAME);
    }
}
