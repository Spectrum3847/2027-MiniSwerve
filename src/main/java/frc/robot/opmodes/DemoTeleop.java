package frc.robot.opmodes;

import frc.robot.subsystems.swerve.Swerve;
import frc.spectrumLib.framework.OpModeLoop;
import frc.spectrumLib.robotsettings.DoubleSetting;
import frc.spectrumLib.robotsettings.RobotSettings;
import frc.spectrumLib.robotsettings.SettingGroup;
import org.wpilib.opmode.OpMode;

/**
 * Demos and visitors driving: the pilot's normal bindings with speed, acceleration and turn rate
 * capped every loop, so a new driver cannot hit anyone hard.
 */
public final class DemoTeleop implements OpMode {
    /**
     * Demo caps, read from settings once at boot (settings are made at boot, not when a mode is
     * picked).
     */
    public record Limits(
            DoubleSetting speed, DoubleSetting accel, DoubleSetting turn, DoubleSetting turnAccel) {
        public static Limits fromSettings() {
            SettingGroup g = RobotSettings.group("demo");
            return new Limits(
                    // Top wheel speed in demo mode (m/s).
                    g.number("maxSpeed", 1.5, "m/s", 0.2, 5),
                    // Wheel acceleration in demo mode (m/s^2).
                    g.number("maxAccel", 3.0, "m/s^2", 0.5, 20),
                    // Turn rate in demo mode (rad/s).
                    g.number("maxTurn", 3.0, "rad/s", 0.5, 15),
                    // Turn acceleration in demo mode (rad/s^2).
                    g.number("maxTurnAccel", 6.0, "rad/s^2", 0.5, 60));
        }
    }

    private final Swerve swerve;
    private final Limits limits;
    private final OpModeLoop loop;
    private final Runnable step = this::step;

    public DemoTeleop(Swerve swerve, Limits limits, OpModeLoop loop) {
        this.swerve = swerve;
        this.limits = limits;
        this.loop = loop;
    }

    @Override
    public void start() {
        loop.set(step);
    }

    @Override
    public void end() {
        loop.clear(step);
    }

    private void step() {
        swerve.limitDrive(
                limits.speed().get(),
                limits.accel().get(),
                limits.turn().get(),
                limits.turnAccel().get());
    }
}
