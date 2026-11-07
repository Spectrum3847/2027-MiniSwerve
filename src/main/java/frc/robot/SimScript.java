package frc.robot;

import static frc.spectrumLib.hardware.SimFaults.Fault.UNHEARD;
import static frc.spectrumLib.hardware.SimFaults.Fault.UNPLUGGED;

import frc.spectrumLib.hardware.SimFaults;
import java.util.function.Consumer;
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
 * <p>Run with {@code MINISWERVE_SIM_SCRIPT=drive ./gradlew run -PnoSimGui} ({@code radius} for the
 * wheel-radius utility, {@code auto} for the drive-forward auto then demo driving, {@code faults}
 * to unplug and reboot devices mid-drive); it exits when done. Then compare {@code
 * Swerve/Odometry/Pose} with {@code Sim/TruthPose} in the log.
 */
final class SimScript {
    private record Step(
            double seconds,
            String what,
            double leftX,
            double leftY,
            double rightX,
            RobotMode mode,
            String opMode,
            Consumer<SimFaults> faults) {
        Step(double seconds, String what, double leftX, double leftY, double rightX) {
            this(seconds, what, leftX, leftY, rightX, RobotMode.TELEOPERATED, "Drive", null);
        }

        /** A step with no sticks in this mode and OpMode (the Driver Station's selection). */
        Step(double seconds, String what, RobotMode mode, String opMode) {
            this(seconds, what, 0, 0, 0, mode, opMode, null);
        }

        /** A teleop step that changes simulated faults when it starts. */
        Step(
                double seconds,
                String what,
                double leftX,
                double leftY,
                double rightX,
                Consumer<SimFaults> faults) {
            this(seconds, what, leftX, leftY, rightX, RobotMode.TELEOPERATED, "Drive", faults);
        }
    }

    private static final Step[] DRIVE = {
        new Step(1.0, "disabled", 0, 0, 0),
        new Step(2.0, "forward", 0, -1.0, 0),
        new Step(1.5, "strafe left", -0.8, 0, 0),
        new Step(2.0, "forward while spinning", 0, -0.7, -0.8),
        new Step(1.0, "stop", 0, 0, 0),
        new Step(1.0, "back and right", 0.5, 0.5, 0),
        new Step(1.0, "stop", 0, 0, 0),
    };

    /** Utility OpMode "Wheel radius": spins in place while enabled. */
    private static final Step[] RADIUS = {
        new Step(1.0, "disabled", RobotMode.UTILITY, "Wheel radius"),
        new Step(8.0, "characterize wheel radius", RobotMode.UTILITY, "Wheel radius"),
        new Step(1.0, "disabled", RobotMode.UTILITY, "Wheel radius"),
    };

    /** Auto OpMode "Drive forward", then teleop "Demo" driving at full stick (capped). */
    private static final Step[] AUTO = {
        new Step(1.0, "disabled", RobotMode.AUTONOMOUS, "Drive forward"),
        new Step(3.0, "auto drive forward", RobotMode.AUTONOMOUS, "Drive forward"),
        new Step(0.5, "disabled", RobotMode.TELEOPERATED, "Demo"),
        new Step(2.0, "demo forward", 0, -1.0, 0, RobotMode.TELEOPERATED, "Demo", null),
        new Step(1.5, "demo spin", 0, 0, -1.0, RobotMode.TELEOPERATED, "Demo", null),
    };

    /**
     * Drives while devices fail and come back: the robot must keep driving the whole time and every
     * module must return to NORMAL. Device names as in PhoenixDeviceSet (FL=0 FR=1 BL=2 BR=3).
     */
    private static final Step[] FAULTS = {
        new Step(1.0, "disabled", 0, 0, 0),
        new Step(1.0, "forward", 0, -0.8, 0),
        new Step(2.0, "FR drive unplugged", 0, -0.8, 0, f -> f.set("Drive1", UNPLUGGED)),
        new Step(1.5, "FR drive back; strafe", -0.6, 0, 0, f -> f.clear("Drive1")),
        new Step(2.0, "BL steer unplugged; drive", 0, 0.6, 0, f -> f.set("Steer2", UNPLUGGED)),
        new Step(1.5, "BL steer back", 0, 0.6, 0, f -> f.clear("Steer2")),
        new Step(
                2.0,
                "BR CANcoder unplugged; spin",
                0,
                -0.5,
                -0.6,
                f -> f.set("Encoder3", UNPLUGGED)),
        new Step(1.0, "BR CANcoder back", 0, 0, 0, f -> f.clear("Encoder3")),
        new Step(1.5, "FL drive unheard (still obeys)", 0.5, 0, 0, f -> f.set("Drive0", UNHEARD)),
        new Step(1.0, "FL drive heard again", 0, 0, 0, f -> f.clear("Drive0")),
        new Step(1.5, "Pigeon unplugged; drive", 0, -0.6, 0.3, f -> f.set("Pigeon", UNPLUGGED)),
        new Step(1.0, "Pigeon back", 0, 0, 0, f -> f.clear("Pigeon")),
        new Step(
                1.5,
                "FR steer reboots, config fails twice",
                0,
                -0.6,
                0,
                f -> {
                    f.failConfigs("Steer1", 2);
                    f.reboot("Steer1");
                }),
        new Step(2.0, "recovered", 0, 0, 0),
    };

    private final Step[] steps;
    private final GamepadSim pad = new GamepadSim(0);
    private double start = Double.NaN;
    private int lastStep = -1;

    /** The script named by MINISWERVE_SIM_SCRIPT, or null for none (or not in sim). */
    static SimScript fromEnvironment(SimFaults faults) {
        String name = System.getenv("MINISWERVE_SIM_SCRIPT");
        if (RunMode.CURRENT != RunMode.Mode.SIM || name == null || name.isBlank()) {
            return null;
        }
        return switch (name) {
            case "drive" -> new SimScript(DRIVE, faults);
            case "radius" -> new SimScript(RADIUS, faults);
            case "faults" -> new SimScript(FAULTS, faults);
            case "auto" -> new SimScript(AUTO, faults);
            default -> throw new IllegalArgumentException("Unknown MINISWERVE_SIM_SCRIPT: " + name);
        };
    }

    private final SimFaults faults;

    private SimScript(Step[] steps, SimFaults faults) {
        this.steps = steps;
        this.faults = faults;
    }

    /** The Driver Station id of the OpMode the robot published with this name, or 0 for none. */
    private static long opModeId(RobotMode mode, String name) {
        for (var option : DriverStationSim.getOpModeOptions()) {
            if (option.getMode() == mode && option.name.equals(name)) {
                return option.id;
            }
        }
        return 0;
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
        while (i < steps.length && t >= end + steps[i].seconds()) {
            end += steps[i].seconds();
            i++;
        }
        if (i >= steps.length) {
            Logger.end();
            System.exit(0);
        }
        Step step = steps[i];
        if (i != lastStep) {
            lastStep = i;
            // Current sim script step.
            Logger.recordOutput("Sim/ScriptStep", step.what());
            if (step.faults() != null && faults != null) {
                step.faults().accept(faults);
            }
        }
        // A disabled step between enabled ones, so the Driver Station starts the next OpMode.
        boolean enabled = !step.what().equals("disabled");
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setRobotMode(step.mode());
        DriverStationSim.setOpMode(opModeId(step.mode(), step.opMode()));
        DriverStationSim.setEnabled(enabled);
        pad.setLeftX(step.leftX());
        pad.setLeftY(step.leftY());
        pad.setRightX(step.rightX());
        DriverStationSim.notifyNewData();
        if (faults != null) {
            // Simulated device faults active now ("device: fault").
            Logger.recordOutput("Sim/Faults", faults.describe());
        }
    }
}
