package frc.spectrumLib.swerve;

/**
 * The drive force each wheel needs to follow the setpoint generator's plan: robot mass times the
 * planned acceleration (and turning inertia times the planned turn acceleration), shared among the
 * wheels that can push, plus what speeds up each wheel's own rotor. Sent as torque-current
 * feedforward, so the drive velocity loop corrects only what is left; without it a wheel must fall
 * behind its setpoint before kP supplies the current.
 *
 * <p>Only the part of each wheel's force along the wheel is returned: the rest (the sideways force
 * of a turn) comes from the tire, not the drive motor. Allocation-free; swerve thread.
 */
public final class DriveAccelerationFeedforward {
    private final double[] mx;
    private final double[] my;
    private final double mass;
    private final double turningInertia;
    private final double rotorMass;
    private final double gain;
    private final double[] fx;
    private final double[] fy;

    /**
     * @param mx module x positions (m)
     * @param my module y positions (m)
     * @param massKg robot weight ready to play (kg)
     * @param turningInertia robot inertia about the vertical (kg m^2)
     * @param rotorMassKg one drive rotor's inertia as a mass at the wheel's tread (kg): rotor
     *     inertia times (gear ratio / wheel radius) squared
     * @param gain share of the computed force to send (1: all of it)
     */
    public DriveAccelerationFeedforward(
            double[] mx,
            double[] my,
            double massKg,
            double turningInertia,
            double rotorMassKg,
            double gain) {
        this.mx = mx.clone();
        this.my = my.clone();
        mass = massKg;
        this.turningInertia = turningInertia;
        rotorMass = rotorMassKg;
        this.gain = gain;
        fx = new double[mx.length];
        fy = new double[mx.length];
    }

    /**
     * Each wheel's feedforward force along where it points (N), for this cycle's plan.
     *
     * @param ax planned robot acceleration along robot x (m/s^2), as the floor sees it
     * @param ay planned robot acceleration along robot y (m/s^2)
     * @param alpha planned turn acceleration (rad/s^2)
     * @param angle module directions (rad)
     * @param wheelAcceleration each wheel's planned speed change along where it points (m/s^2)
     * @param pushing which wheels can push (null: all); the others get 0
     * @param out receives the forces; all 0 if fewer than two wheels push or a value is not finite
     */
    public void calculate(
            double ax,
            double ay,
            double alpha,
            double[] angle,
            double[] wheelAcceleration,
            boolean[] pushing,
            double[] out) {
        java.util.Arrays.fill(out, 0.0);
        if (!SwerveMath.distributeWrench(
                mass * ax, mass * ay, turningInertia * alpha, mx, my, pushing, fx, fy)) {
            return;
        }
        boolean finite = true;
        for (int i = 0; i < out.length; i++) {
            if (pushing != null && !pushing[i]) {
                continue;
            }
            double along = fx[i] * Math.cos(angle[i]) + fy[i] * Math.sin(angle[i]);
            out[i] = gain * (along + rotorMass * wheelAcceleration[i]);
            finite &= Double.isFinite(out[i]);
        }
        if (!finite) {
            // A NaN here would become a NaN motor command: send no feedforward this cycle.
            java.util.Arrays.fill(out, 0.0);
        }
    }
}
