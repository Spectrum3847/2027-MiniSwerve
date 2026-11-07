package frc.robot.subsystems.pose;

import frc.spectrumLib.localization.TagEkf;
import frc.spectrumLib.robotsettings.DoubleSetting;
import frc.spectrumLib.robotsettings.RobotSettings;
import frc.spectrumLib.robotsettings.SettingGroup;

/**
 * TagEKF's noise model as Robot Settings (logged, so replay uses the same values). Defaults are
 * TagEkf's own, tuned on FM's TXMAN1 replays, except the bearing: 0.5 deg did better than 0.25 on
 * the harder match (SpectrumJetson robot-ekf, Q24: held-out p90 0.88 vs 2.33 m).
 */
public final class PoseSettings {
    private PoseSettings() {}

    /** The noise model from settings (boot, after RobotSettings.start). */
    public static TagEkf.Noise noise() {
        SettingGroup g = RobotSettings.group("tagekf");
        TagEkf.Noise d = TagEkf.Noise.DEFAULT;
        // Odometry variance growth per metre driven (m^2/m).
        DoubleSetting odom = g.number("odomVarPerMeter", d.odomVarPerMeter(), "m^2/m", 0, 0.1);
        // Extra variance per metre when modules disagree (slip) (m^2/m).
        DoubleSetting slip = g.number("slipVarPerMeter", d.slipVarPerMeter(), "m^2/m", 0, 1);
        // Weight of the wheel-vs-gyro rotation gap in the slip term.
        DoubleSetting slipYaw = g.number("slipYawWeight", d.slipYawWeight(), "", 0, 10);
        // Heading random walk (rad^2/s).
        DoubleSetting headSec =
                g.number("headingVarPerSecond", d.headingVarPerSecond(), "rad^2/s", 0, 0.1);
        // Heading variance per radian turned (rad^2/rad).
        DoubleSetting headRad =
                g.number("headingVarPerRadian", d.headingVarPerRadian(), "rad^2/rad", 0, 0.1);
        // Bearing noise per tag (deg).
        DoubleSetting bearing = g.number("bearingStdDeg", 0.5, "deg", 0.01, 5);
        // Range noise at 0 m (m).
        DoubleSetting rangeBase = g.number("rangeStdBaseM", d.rangeStdBaseM(), "m", 0, 0.5);
        // Range noise growth with distance squared (m per m^2).
        DoubleSetting rangePerM2 = g.number("rangeStdPerM2", d.rangeStdPerM2(), "1/m", 0, 0.5);
        // Capture-time uncertainty (s), times speed.
        DoubleSetting timing = g.number("timingStdSeconds", d.timingStdSeconds(), "s", 0, 0.05);
        // Chi-squared gate per tag (2 dof).
        DoubleSetting gate = g.number("gateChi2", d.gateChi2(), "", 1, 100);
        return new TagEkf.Noise(
                odom.get(),
                slip.get(),
                slipYaw.get(),
                headSec.get(),
                headRad.get(),
                Math.toRadians(bearing.get()),
                rangeBase.get(),
                rangePerM2.get(),
                timing.get(),
                gate.get());
    }

    /**
     * Self-checking uncertainty: mean NIS above which a frame grows the covariance, and how hard.
     * (10, 1) as on FM; off lost the robot 41% of a match in replay.
     */
    public static double[] adaptive() {
        SettingGroup g = RobotSettings.group("tagekf");
        // Mean NIS over a frame's tags above which the covariance grows; 0 turns it off.
        double nis = g.number("adaptiveNis", 10, "", 0, 100).get();
        // How hard the covariance grows past adaptiveNis.
        double gain = g.number("adaptiveGain", 1, "", 0, 10).get();
        return new double[] {nis, gain};
    }
}
