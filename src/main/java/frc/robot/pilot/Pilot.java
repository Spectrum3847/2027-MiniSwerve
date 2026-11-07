package frc.robot.pilot;

import frc.robot.subsystems.swerve.Swerve;
import org.wpilib.command2.button.CommandGamepad;
import org.wpilib.driverstation.Gamepad;

/**
 * The driver: one Xbox controller on the 2027 Driver Station mapping, port 0. Face buttons use
 * WPILib's position names (A = faceDown, B = faceRight, X = faceLeft, Y = faceUp).
 *
 * <p>Bindings:
 *
 * <ul>
 *   <li>Left stick: drive, field-relative. Right stick X: turn.
 *   <li>Right bumper (hold): slow mode.
 *   <li>X (faceLeft, hold): wheels in an X.
 *   <li>Back: the way the robot faces now becomes downfield (heading 0).
 * </ul>
 *
 * Stick values are Driver Station inputs, which AdvantageKit logs, so replay sees the same sticks.
 */
public final class Pilot {
    /** Slow mode scale on translation and rotation. */
    private static final double SLOW = 0.35;

    private final CommandGamepad pad = new CommandGamepad(0);
    private final Gamepad gamepad = pad.getGamepad();

    public Pilot(Swerve swerve) {
        swerve.setTeleopInputs(this::fieldX, this::fieldY, this::turn);
        pad.faceLeft().whileTrue(swerve.hold(Swerve.State.X_LOCK, "pilotXLock"));
        pad.back().onTrue(swerve.runOnce(() -> swerve.resetHeading(0)).withName("ResetHeading"));
    }

    /** Field +x (downfield) from stick up, -1..1. */
    private double fieldX() {
        return shape(-gamepad.getLeftY()) * scale();
    }

    /** Field +y (left) from stick left, -1..1. */
    private double fieldY() {
        return shape(-gamepad.getLeftX()) * scale();
    }

    /** CCW from stick left, -1..1. */
    private double turn() {
        return shape(-gamepad.getRightX()) * scale();
    }

    private double scale() {
        return gamepad.getRightBumperButton() ? SLOW : 1.0;
    }

    /** Squared response keeps fine control near center; Gamepad already applies the deadband. */
    private static double shape(double v) {
        return Math.copySign(v * v, v);
    }
}
