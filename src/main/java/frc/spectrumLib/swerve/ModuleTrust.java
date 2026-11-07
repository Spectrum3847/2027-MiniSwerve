package frc.spectrumLib.swerve;

import org.littletonrobotics.junction.Logger;

/**
 * Which modules odometry can trust, sample by sample, from logged inputs only (so replay agrees).
 *
 * <p>A module counts when its drive and steer signals were fresh in that sample, its devices are
 * connected and configured, its CANcoder magnet reads, its steer motor is fusing the CANcoder, and
 * its absolute angle has been good at least once since boot. A CANcoder that drops out later does
 * not lose the angle immediately (the steer motor keeps counting from its rotor), but a
 * fused-sensor fault does.
 *
 * <p>This is the groundwork for degraded driving (a dead drive motor with seven motors left, or a
 * dead steer motor dragging); control still goes neutral on any fault until those modes are tested.
 *
 * <p>Logged under {@code <prefix>/}: {@code Trusted}, {@code AbsoluteAngleKnown}.
 */
public final class ModuleTrust {
    private final String prefix;
    private final int n;
    private final double[] weight;
    private final boolean[] trusted;
    private final boolean[] absoluteKnown;

    public ModuleTrust(String prefix, int modules) {
        this.prefix = prefix;
        n = modules;
        weight = new double[n];
        trusted = new boolean[n];
        absoluteKnown = new boolean[n];
    }

    /** Updates the slow, per-loop part from health. Call once per main loop, after inputs. */
    public void update(SwerveHealth h) {
        for (int i = 0; i < n; i++) {
            boolean devices =
                    h.configured[i]
                            && h.driveConnected[i]
                            && h.steerConnected[i]
                            && h.encoderConnected[i]
                            && h.magnetOk[i];
            if (devices && h.fusedOk[i]) {
                absoluteKnown[i] = true;
            }
            trusted[i] = devices && h.fusedOk[i] && absoluteKnown[i];
        }
        // Modules odometry may use (health only; each sample also needs fresh signals).
        Logger.recordOutput(prefix + "/Trusted", trusted);
        // Each module's absolute angle has been good at least once since boot.
        Logger.recordOutput(prefix + "/AbsoluteAngleKnown", absoluteKnown);
    }

    /** Per-module weights for one sample: 1 if trusted and fresh, else 0. Reused array. */
    public double[] weights(SwerveSample s) {
        for (int i = 0; i < n; i++) {
            weight[i] = trusted[i] && s.moduleOk[i] ? 1.0 : 0.0;
        }
        return weight;
    }

    /** Whether every module is trusted (what driving requires, for now). */
    public boolean allTrusted() {
        for (boolean t : trusted) {
            if (!t) {
                return false;
            }
        }
        return true;
    }
}
