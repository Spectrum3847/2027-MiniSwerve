package frc.spectrumSim.field;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class ArenaTest {
    private static Arena.Body robot(double x, double y, double vx, double vy) {
        var b = new Arena.Body();
        b.x = x;
        b.y = y;
        b.vx = vx;
        b.vy = vy;
        b.mass = 50;
        b.inertia = 2;
        b.length = 0.8;
        b.width = 0.8;
        return b;
    }

    @Test
    void boxesApartDoNotTouch() {
        var c = new Contact();
        assertFalse(c.test(ConvexPolygon.box(0, 0, 1, 1, 0), ConvexPolygon.box(2, 0, 1, 1, 0.3)));
    }

    @Test
    void overlapGivesNormalFromObstacleAndDepth() {
        var c = new Contact();
        assertTrue(c.test(ConvexPolygon.box(0, 0, 1, 1, 0), ConvexPolygon.box(0.9, 0, 1, 1, 0)));
        assertEquals(1, c.normalX, 1e-9);
        assertEquals(0.1, c.depth, 1e-9);
    }

    @Test
    void aRobotDrivingIntoTheWallIsPushedOutAndBouncesALittle() {
        var arena = Arena.emptyField(0, 0, 0);
        var b = robot(0.35, 4, -2, 0); // bumper 0.05 m into the alliance wall, closing at 2 m/s
        assertEquals(1, arena.resolve(b));
        assertTrue(b.x >= 0.4 - 0.003, "pushed out: " + b.x);
        assertEquals(2 * Arena.RESTITUTION, b.vx, 1e-9); // bounces back at 20%
        assertEquals(0, b.omega, 1e-9); // flat face: no spin
    }

    @Test
    void slidingAlongAWallLosesOnlyFrictionAndRestingContactDoesNotBounce() {
        var arena = Arena.emptyField(0, 0, 0);
        var b = robot(5, 0.399, 3, -0.01); // pressed lightly against the side wall
        arena.resolve(b);
        assertEquals(0, b.vy, 1e-9); // resting contact: no bounce
        assertTrue(b.vx > 2.99 && b.vx < 3, "friction takes a little: " + b.vx);
    }

    @Test
    void aCornerHitSpinsTheRobot() {
        var arena = Arena.emptyField(0, 0, 0);
        var b = robot(0.3, 4, -2, 0);
        b.heading = Math.toRadians(30); // a corner leads
        arena.resolve(b);
        assertTrue(Math.abs(b.omega) > 0.1, "spin " + b.omega);
    }

    @Test
    void anOpenFieldLeavesTheRobotAlone() {
        var arena = Arena.emptyField(0, 0, 0);
        var b = robot(8, 4, 3, 1);
        assertEquals(0, arena.resolve(b));
        assertEquals(3, b.vx, 0);
    }
}
