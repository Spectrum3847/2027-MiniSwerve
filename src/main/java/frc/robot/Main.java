package frc.robot;

import org.wpilib.framework.RobotBase;

/** Entry point. Keep it empty: no statics, no setup. Boot work belongs in {@link Robot}. */
public final class Main {
    private Main() {}

    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }
}
