package frc.spectrumLib.swerve;

import org.littletonrobotics.junction.Logger;

/**
 * Holds the robot's heading while the driver is not turning, without fighting a hit.
 *
 * <p>Plain heading hold steers the robot back after contact, which drivers dislike. This one:
 *
 * <ul>
 *   <li>captures a heading only after the driver releases the turn stick <em>and</em> the robot's
 *       yaw rate has settled, so it holds where the robot came to rest, not where the stick was
 *       released;
 *   <li>only corrects while the robot is translating (a stopped robot does not drift);
 *   <li>re-captures instead of correcting when the error jumps past a threshold (the robot was hit
 *       or pushed: accept the new heading);
 *   <li>limits its correction rate, so it can only trim drift.
 * </ul>
 *
 * Off by default (setting); main thread; reads only logged values, so replay matches.
 *
 * <p>Logged under {@code <prefix>/}: {@code Holding}, {@code Target}, {@code Error}, {@code
 * Correction}, {@code Releases}.
 */
public final class HeadingHold {
    /** Settings for {@link HeadingHold}. */
    public record Config(
            double kP,
            double maxCorrection,
            double settleSeconds,
            double settleRate,
            double releaseError,
            double minTranslation) {}

    private final String prefix;
    private boolean holding;
    private double target;
    private double settledFor;
    private long releases;

    public HeadingHold(String prefix) {
        this.prefix = prefix;
    }

    /**
     * One main loop.
     *
     * @param driverTurning the driver is commanding rotation
     * @param translationSpeed commanded translation (m/s)
     * @param heading field heading (rad, continuous)
     * @param yawRate measured yaw rate (rad/s)
     * @param dt loop period (s)
     * @return rotation to add (rad/s), 0 when not holding
     */
    public double calculate(
            Config c,
            boolean driverTurning,
            double translationSpeed,
            double heading,
            double yawRate,
            double dt) {
        double correction = 0;
        if (driverTurning) {
            holding = false;
            settledFor = 0;
        } else if (!holding) {
            settledFor = Math.abs(yawRate) < c.settleRate() ? settledFor + dt : 0;
            if (settledFor >= c.settleSeconds()) {
                holding = true;
                target = heading;
            }
        } else {
            double err = SwerveMath.angleModulus(target - heading);
            if (Math.abs(err) > c.releaseError()) {
                // Hit or pushed: take the new heading instead of steering back.
                target = heading;
                releases++;
                err = 0;
            }
            if (translationSpeed >= c.minTranslation()) {
                correction =
                        Math.max(-c.maxCorrection(), Math.min(c.maxCorrection(), c.kP() * err));
            }
            // Heading error held against (rad).
            Logger.recordOutput(prefix + "/Error", err);
        }
        // Holding a captured heading.
        Logger.recordOutput(prefix + "/Holding", holding);
        // Captured heading (rad).
        Logger.recordOutput(prefix + "/Target", target);
        // Rotation added (rad/s).
        Logger.recordOutput(prefix + "/Correction", correction);
        // Times a large error re-captured the heading instead of correcting (hits), since boot.
        Logger.recordOutput(prefix + "/Releases", releases);
        return correction;
    }

    /** Forget the held heading (e.g. after seedFieldCentric or a pose reset). */
    public void reset() {
        holding = false;
        settledFor = 0;
    }
}
