package frc.robot;

import static frc.spectrumLib.hardware.SimFaults.Fault.UNHEARD;
import static frc.spectrumLib.hardware.SimFaults.Fault.UNPLUGGED;

import frc.spectrumLib.hardware.SimFaults;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;
import org.wpilib.driverstation.Gamepad;
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
 * wheel-radius utility, {@code auto} for the drive-forward auto then demo driving, {@code walls}
 * for driving into the field walls, {@code buttons} for the pilot's buttons, {@code faults} to
 * unplug and reboot devices mid-drive); it exits when done. Then compare {@code
 * Swerve/Odometry/Pose} with {@code Sim/TruthPose} in the log.
 */
final class SimScript {
    /**
     * Inputs a step sets each loop, given how far into the step it is ({@code seconds}), so a press
     * can last as long as a person's: a tap is several loops, a hold many.
     */
    interface Inputs {
        void set(GamepadSim pad, double seconds);
    }

    private static final int UNPLUGGED_PAD = 1;
    private static final int DS_LOST = 2;

    /** How long a person's quick tap holds a button down (s): several loops, not one. */
    static final double TAP_SECONDS = 0.12;

    /** A quick tap at the start of the step: down for {@link #TAP_SECONDS}, then up. */
    static Inputs tap(Gamepad.Button button) {
        return (pad, t) -> pad.setButton(button, t < TAP_SECONDS);
    }

    /** The button held down the whole step. */
    static Inputs hold(Gamepad.Button button) {
        return (pad, t) -> pad.setButton(button, true);
    }

    /**
     * One step of a script.
     *
     * @param faults changes simulated faults when the step starts, or null
     * @param inputs sets any other controller input (buttons, triggers, d-pad, right stick Y) or
     *     Driver Station input (alliance, FMS attached, match time, game data, via {@code
     *     DriverStationSim}) every loop of the step, or null. Each loop starts from released
     *     buttons, centred unset sticks, zero triggers and the run's starting Driver Station
     *     values, so a step sets only what it names and nothing carries into the next step. A
     *     button held through a step is one press; two steps in a row pressing the same button are
     *     one long press (put a step between them for two).
     */
    private record Step(
            double seconds,
            String what,
            double leftX,
            double leftY,
            double rightX,
            RobotMode mode,
            String opMode,
            Consumer<SimFaults> faults,
            Inputs inputs,
            int events) {
        Step(double seconds, String what, double leftX, double leftY, double rightX) {
            this(
                    seconds,
                    what,
                    leftX,
                    leftY,
                    rightX,
                    RobotMode.TELEOPERATED,
                    "Drive",
                    null,
                    null,
                    0);
        }

        /** A step with no sticks in this mode and OpMode (the Driver Station's selection). */
        Step(double seconds, String what, RobotMode mode, String opMode) {
            this(seconds, what, 0, 0, 0, mode, opMode, null, null, 0);
        }

        /** A step with sticks and faults in this mode and OpMode. */
        Step(
                double seconds,
                String what,
                double leftX,
                double leftY,
                double rightX,
                RobotMode mode,
                String opMode,
                Consumer<SimFaults> faults) {
            this(seconds, what, leftX, leftY, rightX, mode, opMode, faults, null, 0);
        }

        /** A teleop step that changes simulated faults when it starts. */
        Step(
                double seconds,
                String what,
                double leftX,
                double leftY,
                double rightX,
                Consumer<SimFaults> faults) {
            this(
                    seconds,
                    what,
                    leftX,
                    leftY,
                    rightX,
                    RobotMode.TELEOPERATED,
                    "Drive",
                    faults,
                    null,
                    0);
        }

        /** This step with other inputs set every loop (buttons, triggers, ...). */
        Step press(Inputs set) {
            return new Step(seconds, what, leftX, leftY, rightX, mode, opMode, faults, set, events);
        }

        /** The controller's USB is unplugged for this step: the robot hears no controller. */
        Step unplugged() {
            return new Step(
                    seconds,
                    what,
                    leftX,
                    leftY,
                    rightX,
                    mode,
                    opMode,
                    faults,
                    inputs,
                    events | UNPLUGGED_PAD);
        }

        /** The Driver Station is gone for this step (radio drop): the robot disables. */
        Step dsLost() {
            return new Step(
                    seconds,
                    what,
                    leftX,
                    leftY,
                    rightX,
                    mode,
                    opMode,
                    faults,
                    inputs,
                    events | DS_LOST);
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

    /**
     * Auto "Drive forward" with the sticks pushed (ignored), then "Demo" at full stick (capped).
     */
    private static final Step[] AUTO = {
        new Step(1.0, "disabled", RobotMode.AUTONOMOUS, "Drive forward"),
        // Sticks pushed hard left during auto: they must not move the robot.
        new Step(
                3.0, "auto drive forward", -1.0, 0, 0, RobotMode.AUTONOMOUS, "Drive forward", null),
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

    /**
     * Field walls (SPECTRUM_SIM_FIELD=empty, start 2, 2): strafes right into the side wall, drives
     * along it while pushing into it, backs into the alliance wall, and pulls away. The robot must
     * stop at each wall, slide along it, and keep driving.
     */
    private static final Step[] WALLS = {
        new Step(1.0, "disabled", 0, 0, 0),
        new Step(2.5, "strafe right into the side wall", 0.8, 0, 0),
        new Step(2.0, "drive along the wall, pushing into it", 0.3, -0.8, 0),
        new Step(1.5, "drive along the wall", 0, -0.8, 0),
        new Step(3.0, "back into the alliance wall", 0, 0.8, 0),
        new Step(1.5, "pull away", -0.5, -0.5, 0),
        new Step(0.5, "stop", 0, 0, 0),
    };

    /**
     * The pilot's buttons (Pilot): turns 40 deg, then Back (field-centric re-zero) while driving,
     * which must not move TagEKF's pose; X held with the stick pushed (wheels in an X: the robot
     * holds still); right bumper held (slow mode: slower). Check Pose/Robot against Sim/FieldPose.
     */
    private static final Step[] BUTTONS = {
        new Step(1.0, "disabled", 0, 0, 0),
        new Step(1.0, "forward", 0, -0.6, 0),
        new Step(0.6, "turn left", 0, 0, -0.5),
        new Step(0.3, "Back tapped: re-zero field-centric while driving", 0, -0.6, 0)
                .press(tap(Gamepad.Button.BACK)),
        new Step(1.5, "forward after the re-zero", 0, -0.6, 0),
        new Step(1.0, "X held, stick pushed: wheels in an X", 0, -0.8, 0)
                .press(hold(Gamepad.Button.FACE_LEFT)),
        new Step(1.0, "full stick", -0.8, 0, 0),
        new Step(1.0, "right bumper held: slow mode", 0.8, 0, 0)
                .press(hold(Gamepad.Button.RIGHT_BUMPER)),
        new Step(0.5, "stop", 0, 0, 0),
    };

    /**
     * A real driver's inputs and the ways they fail: hands off (stick drift alone must not move the
     * robot), the controller unplugged mid-drive with the stick still pushed (the robot must stop,
     * then drive again when it is back), and the Driver Station lost (the robot disables, then
     * drives again).
     */
    private static final Step[] DRIVER = {
        new Step(1.0, "disabled", 0, 0, 0),
        new Step(2.0, "hands off: stick drift only", 0, 0, 0),
        new Step(1.5, "forward", 0, -0.8, 0),
        new Step(1.5, "controller unplugged, stick still pushed", 0, -0.8, 0).unplugged(),
        new Step(1.5, "controller back: forward", 0, -0.8, 0),
        new Step(1.0, "Driver Station lost, stick pushed", 0, -0.8, 0).dsLost(),
        new Step(1.5, "Driver Station back: forward", 0, -0.8, 0),
        new Step(0.5, "stop", 0, 0, 0),
    };

    private final Step[] steps;
    private final GamepadSim pad = new GamepadSim(0);
    private final SimDriver driver = new SimDriver();
    private final double[] want = new double[3];
    private double lastLoop = Double.NaN;

    /** What a plugged-in gamepad reports: 6 axes, 26 buttons (bit masks), 1 POV. */
    private static final int AXES = (1 << 6) - 1;

    private static final long BUTTONS_MASK = (1L << 26) - 1;
    private org.wpilib.hardware.hal.AllianceStationID baseAlliance;
    private boolean baseFms;
    private double baseMatchTime;
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
            case "walls" -> new SimScript(WALLS, faults);
            case "buttons" -> new SimScript(BUTTONS, faults);
            case "driver" -> new SimScript(DRIVER, faults);
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
        boolean dsLost = (step.events() & DS_LOST) != 0;
        boolean enabled = !step.what().equals("disabled") && !dsLost;
        DriverStationSim.setDsAttached(!dsLost);
        DriverStationSim.setRobotMode(step.mode());
        DriverStationSim.setOpMode(opModeId(step.mode(), step.opMode()));
        DriverStationSim.setEnabled(enabled);
        // The driver's hands: what they do reaches the robot SimDriver.LINK_SECONDS late.
        double tInput = driver.delayed() ? Math.max(0, t - SimDriver.LINK_SECONDS) : t;
        int k = 0;
        double inputStart = 0;
        while (k < steps.length - 1 && tInput >= inputStart + steps[k].seconds()) {
            inputStart += steps[k].seconds();
            k++;
        }
        Step input = steps[k];
        want[0] = input.leftX();
        want[1] = input.leftY();
        want[2] = input.rightX();
        double dt = Double.isNaN(lastLoop) ? 0.02 : now - lastLoop;
        lastLoop = now;
        double[] sticks = driver.sticks(k, want, dt);
        pad.setLeftX(sticks[0]);
        pad.setLeftY(sticks[1]);
        pad.setRightX(sticks[2]);
        // Everything else starts released each loop; the step presses what it names.
        for (Gamepad.Button b : Gamepad.Button.values()) {
            pad.setButton(b, false);
        }
        pad.setLeftTriggerAxis(0);
        pad.setRightTriggerAxis(0);
        pad.setRightY(0); // sticks a step does not set are centred
        if (baseAlliance == null) {
            // The run's starting Driver Station values: every loop starts from them.
            baseAlliance = DriverStationSim.getAllianceStationId();
            baseFms = DriverStationSim.getFmsAttached();
            baseMatchTime = DriverStationSim.getMatchTime();
        }
        DriverStationSim.setAllianceStationId(baseAlliance);
        DriverStationSim.setFmsAttached(baseFms);
        DriverStationSim.setMatchTime(baseMatchTime);
        DriverStationSim.setGameData("");
        if (input.inputs() != null) {
            input.inputs().set(pad, tInput - inputStart);
        }
        // A controller is connected when it reports axes, buttons or POVs (WPILib); unplugged
        // reports none.
        boolean unplugged = (input.events() & UNPLUGGED_PAD) != 0;
        pad.setAxesAvailable(unplugged ? 0 : AXES);
        pad.setButtonsAvailable(unplugged ? 0 : BUTTONS_MASK);
        pad.setPOVsAvailable(unplugged ? 0 : 1);
        DriverStationSim.notifyNewData();
        if (faults != null) {
            // Simulated device faults active now ("device: fault").
            Logger.recordOutput("Sim/Faults", faults.describe());
        }
    }
}
