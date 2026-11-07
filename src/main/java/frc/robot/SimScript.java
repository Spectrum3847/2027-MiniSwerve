package frc.robot;

import org.littletonrobotics.junction.Logger;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.GamepadSim;
import org.wpilib.system.Timer;

/**
 * Scripted simulation: plays the Driver Station and the pilot's controller so a sim run needs no
 * person. It drives through the real path (Driver Station inputs, Pilot bindings, wanted states),
 * so it tests what the robot will do.
 *
 * <p>Run with {@code MINISWERVE_SIM_SCRIPT=drive ./gradlew run -PnoSimGui}; it exits when done.
 * Then compare {@code Swerve/Odometry/Pose} with {@code Sim/TruthPose} in the log.
 */
final class SimScript {
    private record Step(double seconds, String what, double leftX, double leftY, double rightX) {}

    private static final Step[] DRIVE = {
        new Step(1.0, "disabled", 0, 0, 0),
        new Step(2.0, "forward", 0, -1.0, 0),
        new Step(1.5, "strafe left", -0.8, 0, 0),
        new Step(2.0, "forward while spinning", 0, -0.7, -0.8),
        new Step(1.0, "stop", 0, 0, 0),
        new Step(1.0, "back and right", 0.5, 0.5, 0),
        new Step(1.0, "stop", 0, 0, 0),
    };

    private final GamepadSim pad = new GamepadSim(0);
    private double start = Double.NaN;
    private int lastStep = -1;

    /** The script named by MINISWERVE_SIM_SCRIPT, or null for none (or not in sim). */
    static SimScript fromEnvironment() {
        String name = System.getenv("MINISWERVE_SIM_SCRIPT");
        if (RunMode.CURRENT != RunMode.Mode.SIM || name == null || name.isBlank()) {
            return null;
        }
        if (!name.equals("drive")) {
            throw new IllegalArgumentException("Unknown MINISWERVE_SIM_SCRIPT: " + name);
        }
        return new SimScript();
    }

    /** Called first thing each main loop. */
    @SuppressWarnings("SystemExitOutsideMain") // the script ends the sim run on purpose
    void periodic() {
        double now = Timer.getTimestamp();
        if (Double.isNaN(start)) {
            start = now; // first loop: the robot clock is only meaningful once logging runs
        }
        double t = now - start;
        int i = 0;
        double end = 0;
        while (i < DRIVE.length && t >= end + DRIVE[i].seconds()) {
            end += DRIVE[i].seconds();
            i++;
        }
        if (i >= DRIVE.length) {
            Logger.end();
            System.exit(0);
        }
        Step step = DRIVE[i];
        if (i != lastStep) {
            lastStep = i;
            // Current sim script step.
            Logger.recordOutput("Sim/ScriptStep", step.what());
        }
        boolean enabled = i > 0;
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setRobotMode(RobotMode.TELEOPERATED);
        DriverStationSim.setEnabled(enabled);
        pad.setLeftX(step.leftX());
        pad.setLeftY(step.leftY());
        pad.setRightX(step.rightX());
        DriverStationSim.notifyNewData();
    }
}
