package frc.robot.pilot;

import frc.robot.subsystems.swerve.Swerve;
import frc.spectrumLib.robotsettings.DoubleSetting;
import frc.spectrumLib.robotsettings.RobotSettings;
import frc.spectrumLib.robotsettings.SettingGroup;
import org.wpilib.command2.button.CommandGamepad;
import org.wpilib.command2.button.RobotModeTriggers;
import org.wpilib.command2.button.Trigger;
import org.wpilib.driverstation.Gamepad;

/**
 * The driver: one Xbox controller on the 2027 Driver Station mapping, port 0. Face buttons use
 * WPILib's position names (A = faceDown, B = faceRight, X = faceLeft, Y = faceUp).
 *
 * <p>Bindings (teleop only; in auto and utility the sticks and buttons do nothing):
 *
 * <ul>
 *   <li>Left stick: drive, field-relative. Right stick X: turn.
 *   <li>Right bumper (hold): slow mode.
 *   <li>X (faceLeft, hold): wheels in an X.
 *   <li>Back: the way the robot faces now becomes "away from the driver" (seedFieldCentric).
 * </ul>
 *
 * <p>The left stick uses a radial deadband: the deadband and squaring apply to how far the stick is
 * pushed, not to each axis, so a diagonal keeps its direction. Stick values are Driver Station
 * inputs, which AdvantageKit logs, so replay sees the same sticks.
 */
public final class Pilot {
    private static final SettingGroup G = RobotSettings.group("pilot");

    /** Stick deadband, as a fraction of full travel. */
    private static final DoubleSetting DEADBAND = G.number("stickDeadband", 0.08, "", 0, 0.3);

    /** Slow mode scale on translation and rotation. */
    private static final DoubleSetting SLOW = G.number("slowScale", 0.35, "", 0.05, 1);

    private final CommandGamepad pad = new CommandGamepad(0);
    private final Gamepad gamepad = pad.getGamepad();
    private final double deadband = DEADBAND.get();
    private final double slow = SLOW.get();

    public Pilot(Swerve swerve) {
        // Our own deadband below; Gamepad's per-axis one would square off the corners.
        gamepad.setLeftXDeadband(0);
        gamepad.setLeftYDeadband(0);
        gamepad.setRightXDeadband(0);
        gamepad.setRightYDeadband(0);
        swerve.setTeleopInputs(this::driverForward, this::driverLeft, this::turn);
        // Pilot buttons act only in teleop; autos and utilities own the robot otherwise.
        Trigger teleop = RobotModeTriggers.teleop();
        teleop.and(pad.faceLeft()).whileTrue(swerve.hold(Swerve.State.X_LOCK, "pilotXLock"));
        teleop.and(pad.back())
                .onTrue(swerve.runOnce(swerve::seedFieldCentric).withName("SeedFieldCentric"));
    }

    /** Away from the driver, from stick up, -1..1 (Swerve turns it into the field frame). */
    private double driverForward() {
        return radial(-gamepad.getLeftY(), -gamepad.getLeftX()) * scale();
    }

    /** Driver's left, from stick left, -1..1. */
    private double driverLeft() {
        return radial(-gamepad.getLeftX(), -gamepad.getLeftY()) * scale();
    }

    /** CCW from stick left, -1..1. */
    private double turn() {
        return shape(-gamepad.getRightX()) * scale();
    }

    private double scale() {
        return gamepad.getRightBumperButton() ? slow : 1.0;
    }

    /**
     * One axis of the left stick after a radial deadband and squaring of the stick's magnitude.
     *
     * @param axis this axis
     * @param other the other axis
     */
    private double radial(double axis, double other) {
        double mag = Math.hypot(axis, other);
        if (mag <= deadband) {
            return 0;
        }
        double shaped = shape(Math.min(1, mag));
        return axis / mag * shaped;
    }

    /** Deadband then square, keeping the sign; full travel stays 1. */
    private double shape(double v) {
        double a = Math.abs(v);
        if (a <= deadband) {
            return 0;
        }
        double scaled = (Math.min(1, a) - deadband) / (1 - deadband);
        return Math.copySign(scaled * scaled, v);
    }
}
