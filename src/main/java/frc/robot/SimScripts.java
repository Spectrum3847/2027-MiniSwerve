package frc.robot;

import static frc.spectrumLib.hardware.SimFaults.Fault.UNHEARD;
import static frc.spectrumLib.hardware.SimFaults.Fault.UNPLUGGED;

import org.wpilib.driverstation.Gamepad;
import org.wpilib.hardware.hal.RobotMode;

/**
 * The sim scripts {@link SimScript} plays, by name ({@code MINISWERVE_SIM_SCRIPT}). Each is a list
 * of timed steps: sticks, buttons, the Driver Station's mode and OpMode, faults.
 */
final class SimScripts {
    private SimScripts() {}

    static final SimScript.Step[] DRIVE = {
        new SimScript.Step(1.0, "disabled", 0, 0, 0),
        new SimScript.Step(2.0, "forward", 0, -1.0, 0),
        new SimScript.Step(1.5, "strafe left", -0.8, 0, 0),
        new SimScript.Step(2.0, "forward while spinning", 0, -0.7, -0.8),
        new SimScript.Step(1.0, "stop", 0, 0, 0),
        new SimScript.Step(1.0, "back and right", 0.5, 0.5, 0),
        new SimScript.Step(1.0, "stop", 0, 0, 0),
    };

    /** Utility OpMode "Wheel radius": spins in place while enabled. */
    static final SimScript.Step[] RADIUS = {
        new SimScript.Step(1.0, "disabled", RobotMode.UTILITY, "Wheel radius"),
        new SimScript.Step(8.0, "characterize wheel radius", RobotMode.UTILITY, "Wheel radius"),
        new SimScript.Step(1.0, "disabled", RobotMode.UTILITY, "Wheel radius"),
    };

    /**
     * Auto "Drive forward" with the sticks pushed (ignored), then "Demo" at full stick (capped).
     */
    static final SimScript.Step[] AUTO = {
        new SimScript.Step(1.0, "disabled", RobotMode.AUTONOMOUS, "Drive forward"),
        // Sticks pushed hard left during auto: they must not move the robot.
        new SimScript.Step(
                3.0, "auto drive forward", -1.0, 0, 0, RobotMode.AUTONOMOUS, "Drive forward", null),
        new SimScript.Step(0.5, "disabled", RobotMode.TELEOPERATED, "Demo"),
        new SimScript.Step(2.0, "demo forward", 0, -1.0, 0, RobotMode.TELEOPERATED, "Demo", null),
        new SimScript.Step(1.5, "demo spin", 0, 0, -1.0, RobotMode.TELEOPERATED, "Demo", null),
    };

    /**
     * Drives while devices fail and come back: the robot must keep driving the whole time and every
     * module must return to NORMAL. Device names as in PhoenixDeviceSet (FL=0 FR=1 BL=2 BR=3).
     */
    static final SimScript.Step[] FAULTS = {
        new SimScript.Step(1.0, "disabled", 0, 0, 0),
        new SimScript.Step(1.0, "forward", 0, -0.8, 0),
        new SimScript.Step(2.0, "FR drive unplugged", 0, -0.8, 0, f -> f.set("Drive1", UNPLUGGED)),
        new SimScript.Step(1.5, "FR drive back; strafe", -0.6, 0, 0, f -> f.clear("Drive1")),
        new SimScript.Step(
                2.0, "BL steer unplugged; drive", 0, 0.6, 0, f -> f.set("Steer2", UNPLUGGED)),
        new SimScript.Step(1.5, "BL steer back", 0, 0.6, 0, f -> f.clear("Steer2")),
        new SimScript.Step(
                2.0,
                "BR CANcoder unplugged; spin",
                0,
                -0.5,
                -0.6,
                f -> f.set("Encoder3", UNPLUGGED)),
        new SimScript.Step(1.0, "BR CANcoder back", 0, 0, 0, f -> f.clear("Encoder3")),
        new SimScript.Step(
                1.5, "FL drive unheard (still obeys)", 0.5, 0, 0, f -> f.set("Drive0", UNHEARD)),
        new SimScript.Step(1.0, "FL drive heard again", 0, 0, 0, f -> f.clear("Drive0")),
        new SimScript.Step(
                1.5, "Pigeon unplugged; drive", 0, -0.6, 0.3, f -> f.set("Pigeon", UNPLUGGED)),
        new SimScript.Step(1.0, "Pigeon back", 0, 0, 0, f -> f.clear("Pigeon")),
        new SimScript.Step(
                1.5,
                "FR steer reboots, config fails twice",
                0,
                -0.6,
                0,
                f -> {
                    f.failConfigs("Steer1", 2);
                    f.reboot("Steer1");
                }),
        new SimScript.Step(2.0, "recovered", 0, 0, 0),
    };

    /**
     * Field walls (SPECTRUM_SIM_FIELD=empty, start 2, 2): strafes right into the side wall, drives
     * along it while pushing into it, backs into the alliance wall, and pulls away. The robot must
     * stop at each wall, slide along it, and keep driving.
     */
    static final SimScript.Step[] WALLS = {
        new SimScript.Step(1.0, "disabled", 0, 0, 0),
        new SimScript.Step(2.5, "strafe right into the side wall", 0.8, 0, 0),
        new SimScript.Step(2.0, "drive along the wall, pushing into it", 0.3, -0.8, 0),
        new SimScript.Step(1.5, "drive along the wall", 0, -0.8, 0),
        new SimScript.Step(3.0, "back into the alliance wall", 0, 0.8, 0),
        new SimScript.Step(1.5, "pull away", -0.5, -0.5, 0),
        new SimScript.Step(0.5, "stop", 0, 0, 0),
    };

    /**
     * The pilot's buttons (Pilot): turns 40 deg, then Back (field-centric re-zero) while driving,
     * which must not move TagEKF's pose; X held with the stick pushed (wheels in an X: the robot
     * holds still); right bumper held (slow mode: slower). Check Pose/Robot against Sim/FieldPose.
     */
    static final SimScript.Step[] BUTTONS = {
        new SimScript.Step(1.0, "disabled", 0, 0, 0),
        new SimScript.Step(1.0, "forward", 0, -0.6, 0),
        new SimScript.Step(0.6, "turn left", 0, 0, -0.5),
        new SimScript.Step(0.3, "Back tapped: re-zero field-centric while driving", 0, -0.6, 0)
                .press(SimScript.tap(Gamepad.Button.BACK)),
        new SimScript.Step(1.5, "forward after the re-zero", 0, -0.6, 0),
        new SimScript.Step(1.0, "X held, stick pushed: wheels in an X", 0, -0.8, 0)
                .press(SimScript.hold(Gamepad.Button.FACE_LEFT)),
        new SimScript.Step(1.0, "full stick", -0.8, 0, 0),
        new SimScript.Step(1.0, "right bumper held: slow mode", 0.8, 0, 0)
                .press(SimScript.hold(Gamepad.Button.RIGHT_BUMPER)),
        new SimScript.Step(0.5, "stop", 0, 0, 0),
    };

    /**
     * A real driver's inputs and the ways they fail: hands off (stick drift alone must not move the
     * robot), the controller unplugged mid-drive with the stick still pushed (the robot must stop,
     * then drive again when it is back), and the Driver Station lost (the robot disables, then
     * drives again).
     */
    static final SimScript.Step[] DRIVER = {
        new SimScript.Step(1.0, "disabled", 0, 0, 0),
        new SimScript.Step(2.0, "hands off: stick drift only", 0, 0, 0),
        new SimScript.Step(1.5, "forward", 0, -0.8, 0),
        new SimScript.Step(1.5, "controller unplugged, stick still pushed", 0, -0.8, 0).unplugged(),
        new SimScript.Step(1.5, "controller back: forward", 0, -0.8, 0),
        new SimScript.Step(1.0, "Driver Station lost, stick pushed", 0, -0.8, 0).dsLost(),
        new SimScript.Step(1.5, "Driver Station back: forward", 0, -0.8, 0),
        new SimScript.Step(0.5, "stop", 0, 0, 0),
    };

    /**
     * Over a bump and back: run with {@code SPECTRUM_SIM_FIELD=2026-rebuilt} and {@code
     * SPECTRUM_SIM_START=2.5,2.511,0} to cross a 2026 BUMP, then watch {@code Sim/FieldPose3d} and
     * {@code Pose/Robot3d} in AdvantageScope's 3D field.
     */
    static final SimScript.Step[] BUMP = {
        new SimScript.Step(1.0, "disabled", 0, 0, 0),
        new SimScript.Step(3.0, "forward over the bump", 0, -0.6, 0),
        new SimScript.Step(1.0, "stop", 0, 0, 0),
        new SimScript.Step(3.0, "back over the bump", 0, 0.6, 0),
        new SimScript.Step(1.0, "stop", 0, 0, 0),
    };

    /** The script with this name, or null. */
    static SimScript.Step[] byName(String name) {
        return switch (name) {
            case "drive" -> DRIVE;
            case "radius" -> RADIUS;
            case "faults" -> FAULTS;
            case "auto" -> AUTO;
            case "walls" -> WALLS;
            case "buttons" -> BUTTONS;
            case "driver" -> DRIVER;
            case "bump" -> BUMP;
            default -> null;
        };
    }
}
