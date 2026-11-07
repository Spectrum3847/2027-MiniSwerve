package frc.spectrumSim.field;

/**
 * Another robot on the sim field: a partner to drive around or an opponent to push against. It
 * drives like a swerve on carpet, simply: it accelerates toward what its {@link Behavior} wants, no
 * harder than its tires allow ({@link #friction} g), so in a pushing match the one with more
 * traction and weight wins, and a parked robot slides only when pushed harder than its tires hold.
 */
public final class SimRobot {
    private static final double GRAVITY = 9.81;

    /** What the robot wants to do this step: a field-frame velocity. */
    public interface Behavior {
        /**
         * Fills {@code out} with the wanted field velocity: vx, vy (m/s), omega (rad/s).
         *
         * @param self this robot
         * @param target the robot being simulated (ours), e.g. for a defender to chase
         * @param time seconds since the arena started
         */
        void want(Arena.Body self, Arena.Body target, double time, double[] out);
    }

    public final Arena.Body body = new Arena.Body();
    final Behavior behavior;

    /** Tire friction coefficient: most acceleration (and push) is this many g. */
    public final double friction;

    /** Top speed (m/s) and turn rate (rad/s). */
    public final double maxSpeed;

    public final double maxTurn;

    private final double[] want = new double[3];

    /**
     * @param x start on the field (m)
     * @param y start on the field (m)
     * @param heading start heading (rad)
     * @param massKg weight ready to play
     * @param length bumper outline (m)
     * @param width bumper outline (m)
     */
    public SimRobot(
            double x,
            double y,
            double heading,
            double massKg,
            double length,
            double width,
            double friction,
            double maxSpeed,
            double maxTurn,
            Behavior behavior) {
        body.x = x;
        body.y = y;
        body.heading = heading;
        body.mass = massKg;
        // A box robot: about the inertia of a uniform plate.
        body.inertia = massKg * (length * length + width * width) / 12;
        body.length = length;
        body.width = width;
        this.friction = friction;
        this.maxSpeed = maxSpeed;
        this.maxTurn = maxTurn;
        this.behavior = behavior;
    }

    /** One step: tires accelerate toward the wanted motion, then the body moves. */
    void step(Arena.Body target, double time, double dt) {
        behavior.want(body, target, time, want);
        double wx = want[0], wy = want[1];
        double speed = Math.hypot(wx, wy);
        if (speed > maxSpeed) {
            wx *= maxSpeed / speed;
            wy *= maxSpeed / speed;
        }
        double dvx = wx - body.vx, dvy = wy - body.vy;
        double dv = Math.hypot(dvx, dvy);
        double most = friction * GRAVITY * dt; // the tires' limit this step
        if (dv > most) {
            dvx *= most / dv;
            dvy *= most / dv;
        }
        body.vx += dvx;
        body.vy += dvy;
        double wOmega = Math.max(-maxTurn, Math.min(maxTurn, want[2]));
        // Turning is traction-limited too: about the same push at the wheels' radius.
        double radius = Math.hypot(body.length, body.width) / 2;
        double mostTurn = most / radius;
        body.omega += Math.max(-mostTurn, Math.min(mostTurn, wOmega - body.omega));
        body.x += body.vx * dt;
        body.y += body.vy * dt;
        body.heading += body.omega * dt;
    }

    /** Holds a spot (brakes): only moves when pushed harder than its tires hold. */
    public static Behavior parked() {
        return (self, target, time, out) -> {
            out[0] = 0;
            out[1] = 0;
            out[2] = 0;
        };
    }

    /**
     * Drives back and forth between two points at {@code speed}, turning nowhere: a robot crossing
     * the field.
     */
    public static Behavior shuttle(double x1, double y1, double x2, double y2, double speed) {
        double len = Math.hypot(x2 - x1, y2 - y1);
        return (self, target, time, out) -> {
            double period = 2 * len / speed;
            double phase = (time % period) / period;
            double tx = phase < 0.5 ? x2 : x1, ty = phase < 0.5 ? y2 : y1;
            double dx = tx - self.x, dy = ty - self.y;
            double d = Math.hypot(dx, dy);
            double v = Math.min(speed, 3 * d); // slow down near the end
            out[0] = d > 1e-6 ? dx / d * v : 0;
            out[1] = d > 1e-6 ? dy / d * v : 0;
            out[2] = 0;
        };
    }

    /**
     * Plays defense: drives at the simulated robot at {@code speed} and keeps pushing into it,
     * turning to face it.
     */
    public static Behavior defender(double speed) {
        return (self, target, time, out) -> {
            double dx = target.x - self.x, dy = target.y - self.y;
            double d = Math.hypot(dx, dy);
            out[0] = d > 1e-6 ? dx / d * speed : 0;
            out[1] = d > 1e-6 ? dy / d * speed : 0;
            double face = Math.atan2(dy, dx);
            out[2] = 4 * Math.IEEEremainder(face - self.heading, 2 * Math.PI);
        };
    }
}
