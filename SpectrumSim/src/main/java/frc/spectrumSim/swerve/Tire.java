package frc.spectrumSim.swerve;

/**
 * The tire on carpet: force from slip speed. Force rises linearly with slip to its peak at {@link
 * #PEAK_SLIP}, then falls toward sliding friction ({@link #SLIDING_SHARE} of peak) as the tire
 * slides. Slip along and across the wheel is one vector, and the force opposes it: its size from
 * the curve at the total slip speed, its direction the slip's (an isotropic tire). So a wheel
 * scrubbing sideways at a few cm/s while it drives still pushes mostly along its slip, not half
 * sideways. Peak force is the tire's friction coefficient times the load on that wheel.
 */
final class Tire {
    private Tire() {}

    /** Slip speed at which the tire gives its most grip (m/s): a few percent of driving speed. */
    static final double PEAK_SLIP = 0.1;

    /** Grip once fully sliding, as a share of peak (kinetic below static friction). */
    static final double SLIDING_SHARE = 0.85;

    /** Tire force for a slip speed, as a signed share of peak grip. */
    static double gripCurve(double slip) {
        double x = Math.abs(slip) / PEAK_SLIP;
        double share = x <= 1 ? x : slidingFactor(Math.abs(slip));
        return Math.copySign(share, slip);
    }

    /** Most grip at a total slip speed: full at the peak, sliding friction well past it. */
    static double slidingFactor(double slip) {
        double x = slip / PEAK_SLIP;
        return x <= 1 ? 1.0 : SLIDING_SHARE + (1 - SLIDING_SHARE) * Math.exp(-(x - 1));
    }
}
