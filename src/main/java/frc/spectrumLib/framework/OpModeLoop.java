package frc.spectrumLib.framework;

/**
 * The running OpMode's per-loop work, run by the robot at a fixed place in its loop (before the
 * scheduler and before subsystems apply their wanted states).
 *
 * <p>The base class calls an OpMode's {@code periodic()} after {@code robotPeriodic()}, so anything
 * an OpMode asks for there reaches the subsystems one loop late. OpModes instead {@link #set} their
 * step in {@code start()} (which runs before {@code robotPeriodic()}) and {@link #clear} it in
 * {@code end()}.
 */
public final class OpModeLoop {
    private Runnable step;

    /** Makes {@code step} the running OpMode's work, replacing any other. */
    public void set(Runnable step) {
        this.step = step;
    }

    /** Stops {@code step}, if it is still the running one. */
    public void clear(Runnable step) {
        if (this.step == step) {
            this.step = null;
        }
    }

    /** Runs the running OpMode's step, if any. */
    public void run() {
        Runnable s = step;
        if (s != null) {
            s.run();
        }
    }
}
