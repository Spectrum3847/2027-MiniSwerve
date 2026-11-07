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
    private int contacts;

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

    /**
     * Pushes a robot out of every obstacle it overlaps and applies the contact impulses.
     *
     * @param b the robot, in the field frame; changed in place
     * @return how many obstacles it touched this step
     */
    public int resolve(Body b) {
        contacts = 0;
        b.outline.setBox(b.x, b.y, b.length, b.width, b.heading);
        for (ConvexPolygon o : obstacles) {
            if (!contact.test(o, b.outline)) {
                continue;
            }
            contacts++;
            push(b, contact);
            b.outline.setBox(b.x, b.y, b.length, b.width, b.heading);
        }
        return contacts;
    }

    /** One contact: positional correction, then normal and friction impulses at the touch point. */
    private static void push(Body b, Contact c) {
        double nx = c.normalX, ny = c.normalY;
        double move = Math.max(0, c.depth - SLOP);
        b.x += nx * move;
        b.y += ny * move;
        double rx = c.pointX - b.x, ry = c.pointY - b.y;
        // Velocity of the bumper at the touch point.
        double vx = b.vx - b.omega * ry, vy = b.vy + b.omega * rx;
        double vn = vx * nx + vy * ny;
        if (vn >= 0) {
            return; // already separating
        }
        double rn = rx * ny - ry * nx;
        // Bounce only on a real hit; pressed against a wall, contact just stops the push.
        double e = vn < -BOUNCE_SPEED ? RESTITUTION : 0;
        double j = -(1 + e) * vn / (1 / b.mass + rn * rn / b.inertia);
        apply(b, j * nx, j * ny, rx, ry);
        // Friction along the wall, at most FRICTION times the normal impulse.
        vx = b.vx - b.omega * ry;
        vy = b.vy + b.omega * rx;
        double tx = -ny, ty = nx;
        double vt = vx * tx + vy * ty;
        double rt = rx * ty - ry * tx;
        double jt = -vt / (1 / b.mass + rt * rt / b.inertia);
        jt = Math.max(-FRICTION * j, Math.min(FRICTION * j, jt));
        apply(b, jt * tx, jt * ty, rx, ry);
    }

    private static void apply(Body b, double jx, double jy, double rx, double ry) {
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
