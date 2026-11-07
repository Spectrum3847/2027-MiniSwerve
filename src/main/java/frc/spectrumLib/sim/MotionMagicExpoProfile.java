package frc.spectrumLib.sim;

/**
 * The profile a TalonFX's Motion Magic Expo request follows, for {@link TalonFXModel}: the target
 * moves the way a DC motor would at {@link #PROFILE_VOLTS}, speeding up as {@code (12 - kV v) / kA}
 * and braking in time to stop on the goal.
 *
 * <p>CTRE does not publish the exact profile math. This matches its documented shape (kV sets the
 * top speed, kA how fast it gets there, a nonzero cruise velocity caps the speed); {@code
 * tools/talon_crosscheck} measures how far it is from Phoenix's simulator.
 */
public final class MotionMagicExpoProfile {
    /** Voltage the Expo profile is planned at (V). */
    public static final double PROFILE_VOLTS = 12.0;

    private double position;
    private double velocity;
    private double acceleration;

    /** Starts the profile from where the mechanism is now. */
    public void reset(double position, double velocity) {
        this.position = position;
        this.velocity = velocity;
        this.acceleration = 0;
    }

    /**
     * Moves the profile one step toward {@code goal}.
     *
     * @param kV MotionMagicExpo_kV (V per mechanism rps)
     * @param kA MotionMagicExpo_kA (V per mechanism rps/s)
     * @param cruise MotionMagicCruiseVelocity (rps); 0 for "as fast as kV allows"
     * @param wrap ContinuousWrap: take the short way round one rotation
     */
    public void step(double goal, double kV, double kA, double cruise, boolean wrap, double dt) {
        double ka = Math.max(kA, 1e-6);
        double error = goal - position;
        if (wrap) {
            error = Math.IEEEremainder(error, 1.0);
        }
        double dir = Math.signum(error);
        if (dir == 0) {
            dir = 1;
        }
        double u = velocity * dir; // speed toward the goal
        double distance = Math.abs(error);
        double a;
        if (u < 0) {
            a = (PROFILE_VOLTS + kV * -u) / ka; // heading away: brake hard
        } else if (distance <= stopDistance(u, kV, ka) + u * dt) {
            // Brake just hard enough to land on the goal.
            a = distance > 1e-9 ? -u * u / (2 * distance) : -u / dt;
        } else {
            a = (PROFILE_VOLTS - kV * u) / ka;
            if (cruise > 0 && u + a * dt > cruise) {
                a = (cruise - u) / dt;
            }
        }
        double next = u + a * dt;
        if (u >= 0 && next < 0) {
            next = 0;
        }
        double move = (u + next) / 2 * dt;
        if (u >= 0 && move >= distance) {
            position += error; // arrived
            next = 0;
        } else {
            position += dir * move;
        }
        acceleration = dir * (next - u) / dt;
        velocity = dir * next;
    }

    /** Distance to stop from speed {@code u} braking at full profile voltage (rotations). */
    static double stopDistance(double u, double kV, double kA) {
        if (u <= 0) {
            return 0;
        }
        if (kV < 1e-9) {
            return kA * u * u / (2 * PROFILE_VOLTS);
        }
        double x = kV * u / PROFILE_VOLTS;
        return kA / kV * (u - PROFILE_VOLTS / kV * Math.log1p(x));
    }

    public double position() {
        return position;
    }

    public double velocity() {
        return velocity;
    }

    public double acceleration() {
        return acceleration;
    }
}
