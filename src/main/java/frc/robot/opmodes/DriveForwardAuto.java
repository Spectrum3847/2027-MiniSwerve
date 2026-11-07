package frc.robot.opmodes;

import frc.robot.subsystems.swerve.Swerve;
import frc.spectrumLib.framework.OpModeLoop;
import org.wpilib.opmode.OpMode;
import org.wpilib.system.Timer;

/**
 * The simplest real auto: drive straight ahead, robot-relative, for a fixed time, then hold still
 * until auto ends. A placeholder until the Choreo auto builder makes match autos; also a quick
 * check that auto runs.
 */
public final class DriveForwardAuto implements OpMode {
    private static final String NAME = "driveForwardAuto";
    private static final double SPEED = 1.0; // m/s
    private static final double SECONDS = 1.5;

    private final Swerve swerve;
    private final OpModeLoop loop;
    private final Runnable step = this::step;
    private double start;

    public DriveForwardAuto(Swerve swerve, OpModeLoop loop) {
        this.swerve = swerve;
        this.loop = loop;
    }

    @Override
    public void start() {
        // Robot clock: a logged input, so replay ends the drive on the same loop.
        start = Timer.getTimestamp();
        loop.set(step);
    }

    private void step() {
        double speed = Timer.getTimestamp() - start < SECONDS ? SPEED : 0;
        swerve.driveRobotRelative(speed, 0, 0, NAME);
    }

    @Override
    public void end() {
        loop.clear(step);
        swerve.stopRobotRelative(NAME);
    }
}
