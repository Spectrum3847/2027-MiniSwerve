package frc.spectrumSim.field;

import java.util.List;

/**
 * The field the sim robot drives on: its walls and field elements as fixed convex outlines, and the
 * robot's bumpers hitting them. Each physics step, after the robot moves, {@link #resolve} pushes
 * the bumpers back out of anything they overlap and takes the impulse out of the robot's motion: a
 * bounce off the bumpers' padding ({@link #RESTITUTION}) and sliding friction along the wall
 * ({@link #FRICTION}). The tires then fight that like on a real field: a robot driven into a wall
 * stops, spins its wheels, and slides along it.
 *
 * <p>Field frame: the field's origin and axes (as AdvantageScope's field view). The sim's true pose
 * starts at (0, 0, 0), so the arena places that origin at {@link #startX}, {@link #startY}, {@link
 * #startHeading} on the field.
 *
 * <p>Deterministic and allocation-free per step; one thread steps it.
 */
public final class Arena {
    /** FRC field length (m): the 2026 field, used until a season's arena replaces it. */
    public static final double FIELD_LENGTH = 16.541;

    /** FRC field width (m). */
    public static final double FIELD_WIDTH = 8.069;

    /** Share of the closing speed bumpers give back in a hit: padded, so little bounce. */
    static final double RESTITUTION = 0.2;

    /** Bumper fabric on the field wall or an element: sliding friction coefficient. */
    static final double FRICTION = 0.4;

    /** Closing speed (m/s) below which bumpers do not bounce: resting contact. */
    private static final double BOUNCE_SPEED = 0.1;

    /** Overlap left uncorrected (m), so a robot resting on a wall does not jitter. */
    private static final double SLOP = 0.002;

    private final ConvexPolygon[] obstacles;
    public final double startX;
    public final double startY;
    public final double startHeading;
    private final Contact contact = new Contact();
    private final java.util.ArrayList<SimRobot> robots = new java.util.ArrayList<>();
    private double time;

    public Arena(List<ConvexPolygon> obstacles, double startX, double startY, double startHeading) {
        this.obstacles = obstacles.toArray(new ConvexPolygon[0]);
        this.startX = startX;
        this.startY = startY;
        this.startHeading = startHeading;
    }

    /** An empty FRC field: the four walls only. */
    public static Arena emptyField(double startX, double startY, double startHeading) {
        double t = 1.0; // wall thickness, outside the field
        return new Arena(
                List.of(
                        new ConvexPolygon(-t, -t, FIELD_LENGTH + t, -t, FIELD_LENGTH + t, 0, -t, 0),
                        new ConvexPolygon(
                                -t,
                                FIELD_WIDTH,
                                FIELD_LENGTH + t,
                                FIELD_WIDTH,
                                FIELD_LENGTH + t,
                                FIELD_WIDTH + t,
                                -t,
                                FIELD_WIDTH + t),
                        new ConvexPolygon(-t, 0, 0, 0, 0, FIELD_WIDTH, -t, FIELD_WIDTH),
                        new ConvexPolygon(
                                FIELD_LENGTH,
                                0,
                                FIELD_LENGTH + t,
                                0,
                                FIELD_LENGTH + t,
                                FIELD_WIDTH,
                                FIELD_LENGTH,
                                FIELD_WIDTH)),
                startX,
                startY,
                startHeading);
    }

    private Terrain terrain = Terrain.FLAT;

    /** Sets the floor's bumps and ramps (default flat). */
    public void setTerrain(Terrain terrain) {
        this.terrain = terrain;
    }

    /** The floor's bumps and ramps. */
    public Terrain terrain() {
        return terrain;
    }

    /** Adds another robot (partner or opponent). Before the sim starts. */
    public void add(SimRobot robot) {
        robots.add(robot);
    }

    /** The other robots on the field. */
    public List<SimRobot> robots() {
        return robots;
    }

    /**
     * One step for everything but the simulated robot: the other robots drive, then every robot is
     * pushed out of walls and out of each other, the simulated one included.
     *
     * @param ours the simulated robot, in the field frame; changed in place
     * @return how many contacts the simulated robot had this step
     */
    public int step(Body ours, double dt) {
        time += dt;
        for (SimRobot r : robots) {
            r.step(ours, time, dt);
        }
        // Each contact is solved on its own, so a robot pinned between another and a wall would be
        // pushed out of one into the other: a few passes settle the stack.
        int n = 0;
        for (int pass = 0; pass < PASSES; pass++) {
            for (SimRobot r : robots) {
                walls(r.body);
            }
            for (int a = 0; a < robots.size(); a++) {
                for (int b = a + 1; b < robots.size(); b++) {
                    bodies(robots.get(a).body, robots.get(b).body);
                }
            }
            n = walls(ours);
            for (SimRobot r : robots) {
                n += bodies(r.body, ours);
            }
        }
        return n;
    }

    /** Pushes a robot out of every wall and field element it overlaps; returns the contacts. */
    public int resolve(Body b) {
        return walls(b);
    }

    private int walls(Body b) {
        int n = 0;
        b.outline.setBox(b.x, b.y, b.length, b.width, b.heading);
        for (ConvexPolygon o : obstacles) {
            if (!contact.test(o, b.outline)) {
                continue;
            }
            n++;
            push(null, b, contact);
            b.outline.setBox(b.x, b.y, b.length, b.width, b.heading);
        }
        return n;
    }

    /** Two robots' bumpers; returns 1 if they touched. */
    private int bodies(Body a, Body b) {
        a.outline.setBox(a.x, a.y, a.length, a.width, a.heading);
        b.outline.setBox(b.x, b.y, b.length, b.width, b.heading);
        if (!contact.test(a.outline, b.outline)) {
            return 0;
        }
        push(a, b, contact);
        return 1;
    }

    /** Passes over all contacts per step, so stacked contacts (robot, robot, wall) settle. */
    private static final int PASSES = 3;

    /** Solver passes over a contact's points per step: enough for two points to settle. */
    private static final int ITERATIONS = 12;

    private final double[] accN = new double[2];
    private final double[] accT = new double[2];
    private final double[] bounce = new double[2];

    /**
     * One contact between {@code a} (null: a fixed wall) and {@code b}, normal from a into b:
     * positional correction shared by inverse mass, then normal and friction impulses at each touch
     * point, solved together (sequential impulses; each point's normal impulse never pulls, and its
     * friction is capped by its own normal impulse).
     */
    private void push(Body a, Body b, Contact c) {
        double nx = c.normalX, ny = c.normalY;
        double ima = a == null ? 0 : 1 / a.mass, imb = 1 / b.mass;
        double iia = a == null ? 0 : 1 / a.inertia, iib = 1 / b.inertia;
        double move = Math.max(0, c.depth - SLOP) / (ima + imb);
        b.x += nx * move * imb;
        b.y += ny * move * imb;
        if (a != null) {
            a.x -= nx * move * ima;
            a.y -= ny * move * ima;
        }
        double tx = -ny, ty = nx;
        for (int k = 0; k < c.count; k++) {
            accN[k] = 0;
            accT[k] = 0;
            double vn = normalSpeed(a, b, c, k, nx, ny);
            // Bounce only on a real hit; pressed together, contact just stops the push.
            bounce[k] = vn < -BOUNCE_SPEED ? -RESTITUTION * vn : 0;
        }
        for (int it = 0; it < ITERATIONS; it++) {
            for (int k = 0; k < c.count; k++) {
                double rbx = c.pointX[k] - b.x, rby = c.pointY[k] - b.y;
                double rax = a == null ? 0 : c.pointX[k] - a.x;
                double ray = a == null ? 0 : c.pointY[k] - a.y;
                double rbn = rbx * ny - rby * nx, ran = rax * ny - ray * nx;
                double kn = ima + imb + ran * ran * iia + rbn * rbn * iib;
                double vn = normalSpeed(a, b, c, k, nx, ny);
                double next = Math.max(0, accN[k] - (vn - bounce[k]) / kn);
                double dj = next - accN[k];
                accN[k] = next;
                apply(b, dj * nx, dj * ny, rbx, rby);
                apply(a, -dj * nx, -dj * ny, rax, ray);
                // Bumper friction along the contact, capped by this point's normal impulse.
                double vx = relX(a, b, ray, rby), vy = relY(a, b, rax, rbx);
                double vt = vx * tx + vy * ty;
                double rbt = rbx * ty - rby * tx, rat = rax * ty - ray * tx;
                double kt = ima + imb + rat * rat * iia + rbt * rbt * iib;
                double cap = FRICTION * accN[k];
                double nextT = Math.max(-cap, Math.min(cap, accT[k] - vt / kt));
                double djt = nextT - accT[k];
                accT[k] = nextT;
                apply(b, djt * tx, djt * ty, rbx, rby);
                apply(a, -djt * tx, -djt * ty, rax, ray);
            }
        }
    }

    /** Closing speed of b against a at touch point k, along the normal (negative: closing). */
    private static double normalSpeed(Body a, Body b, Contact c, int k, double nx, double ny) {
        double rbx = c.pointX[k] - b.x, rby = c.pointY[k] - b.y;
        double rax = a == null ? 0 : c.pointX[k] - a.x, ray = a == null ? 0 : c.pointY[k] - a.y;
        return relX(a, b, ray, rby) * nx + relY(a, b, rax, rbx) * ny;
    }

    /** x velocity of b's bumper point against a's (a null: a fixed wall). */
    private static double relX(Body a, Body b, double ray, double rby) {
        double v = b.vx - b.omega * rby;
        return a == null ? v : v - (a.vx - a.omega * ray);
    }

    /** y velocity of b's bumper point against a's. */
    private static double relY(Body a, Body b, double rax, double rbx) {
        double v = b.vy + b.omega * rbx;
        return a == null ? v : v - (a.vy + a.omega * rax);
    }

    private static void apply(Body b, double jx, double jy, double rx, double ry) {
        if (b == null) {
            return;
        }
        b.vx += jx / b.mass;
        b.vy += jy / b.mass;
        b.omega += (rx * jy - ry * jx) / b.inertia;
    }

    /** A robot as the arena sees it: a box of bumpers with mass, in the field frame. */
    public static final class Body {
        public double x, y, heading; // m, m, rad
        public double vx, vy, omega; // field frame: m/s, m/s, rad/s
        public double mass; // kg
        public double inertia; // kg m^2 about the vertical
        public double length, width; // bumper outline (m)
        final ConvexPolygon outline = ConvexPolygon.box(0, 0, 1, 1, 0);
    }
}
