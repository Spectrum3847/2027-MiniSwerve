package frc.spectrumLib.swerve;

/**
 * Whether the drive motors should brake or coast. Brake while enabled and while the robot is still
 * moving after a disable (so it does not roll at the end of a match); coast once it has been
 * disabled and still for a while, so the pit crew can push it. Team 6328's approach.
 *
 * <p>Once coasting it stays coasting until the next enable: pushing or shaking a disabled robot
 * must not brake it again (that fights the person moving it, and every switch is a config apply on
 * all four drive motors).
 *
 * <p>Pure decision from logged values (enabled state, wheel speeds), so replay agrees.
 */
public final class CoastOnDisable {
    private final double stillSpeed;
    private final double delaySeconds;
    private double stillFor;
    private boolean coasting;

    /**
     * @param stillSpeed wheel speed below which the robot counts as still (m/s)
     * @param delaySeconds how long disabled and still before coasting (s)
     */
    public CoastOnDisable(double stillSpeed, double delaySeconds) {
        this.stillSpeed = stillSpeed;
        this.delaySeconds = delaySeconds;
    }

    /**
     * Brake or coast for this loop.
     *
     * @param enabled the robot is enabled
     * @param maxWheelSpeed fastest wheel right now (m/s)
     * @param dt loop period (s)
     * @return true for brake, false for coast
     */
    public boolean brake(boolean enabled, double maxWheelSpeed, double dt) {
        if (enabled) {
            stillFor = 0;
            coasting = false;
            return true;
        }
        if (coasting) {
            return false;
        }
        stillFor = maxWheelSpeed < stillSpeed ? stillFor + dt : 0;
        coasting = stillFor >= delaySeconds;
        return !coasting;
    }
}
