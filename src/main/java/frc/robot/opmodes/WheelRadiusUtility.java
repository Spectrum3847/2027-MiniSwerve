package frc.robot.opmodes;

import frc.robot.subsystems.swerve.Swerve;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.opmode.OpMode;

/**
 * Pit utility: on enable, spins in place to measure the wheel radius, until disabled. The estimate
 * is logged at Swerve/WheelRadius/Estimate and printed when it ends; copy it into the setting
 * swerve/wheelRadius. Run it on carpet, with room to spin.
 */
public final class WheelRadiusUtility implements OpMode {
    private final Command spin;

    public WheelRadiusUtility(Swerve swerve) {
        spin = swerve.characterizeWheelRadius();
    }

    @Override
    public void start() {
        CommandScheduler.getInstance().schedule(spin);
    }

    @Override
    public void end() {
        spin.cancel();
    }
}
