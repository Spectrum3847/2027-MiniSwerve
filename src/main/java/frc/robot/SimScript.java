package frc.robot;

import frc.spectrumLib.hardware.SimFaults;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;
import org.wpilib.driverstation.Gamepad;
import org.wpilib.hardware.hal.AllianceStationID;
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

    static final int UNPLUGGED_PAD = 1;
    static final int DS_LOST = 2;

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
     *     DriverStationSim}) every loop of the step, or null. It follows the driver's timeline, 30
     *     ms behind the robot's (SimDriver): sticks, buttons and anything it sets arrive late, like
     *     a person's; mode, enable, faults, unplug and Driver Station loss do not. Each loop starts
     *     from released buttons, centred unset sticks, zero triggers and the run's starting Driver
     *     Station values, so a step sets only what it names and nothing carries into the next step.
     *     A button held through a step is one press; two steps in a row pressing the same button
     *     are one long press (put a step between them for two).
     */
    record Step(
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

        /**
         * The Driver Station is gone for this step (radio drop): the robot disables and hears no
         * controller. (WPILib's sim keeps reporting the Driver Station attached.)
         */
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

    private final Step[] steps;
    private final GamepadSim pad = new GamepadSim(0);
    private final SimDriver driver = new SimDriver();
    private final double[] want = new double[3];
    private double lastLoop = Double.NaN;

    /** What a plugged-in gamepad reports as available (bit masks): 6 axes. */
    private static final int AXES = (1 << 6) - 1;

    /** ...and 26 buttons (and 1 POV, set where used). */
    private static final long BUTTONS_MASK = (1L << 26) - 1;

    private AllianceStationID baseAlliance;
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
        SimScript.Step[] steps = SimScripts.byName(name);
        if (steps == null) {
            throw new IllegalArgumentException("unknown MINISWERVE_SIM_SCRIPT " + name);
        }
        return new SimScript(steps, faults);
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
        // Not hand actions, so on the robot's timeline: a USB unplug, and a radio drop (which stops
        // controller data too: WPILib's sim keeps the Driver Station attached, so it shows as a
        // disabled robot with no controller).
        boolean unplugged = (step.events() & (UNPLUGGED_PAD | DS_LOST)) != 0;
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
