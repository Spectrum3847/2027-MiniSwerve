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
        // Bounces back at 20% (the iterated solver lands within a few percent).
        assertEquals(2 * Arena.RESTITUTION, b.vx, 0.02);
        assertEquals(0, b.omega, 0.01); // flat face: no spin
    }

    @Test
    void slidingAlongAWallLosesOnlyFrictionAndRestingContactDoesNotBounce() {
        var arena = Arena.emptyField(0, 0, 0);
        var b = robot(5, 0.399, 3, -0.01); // pressed lightly against the side wall
        arena.resolve(b);
        assertEquals(0, b.vy, 1e-4); // resting contact: no bounce
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
    void aNearlyFlatHitDoesNotSpinTheRobot() {
        for (double deg : new double[] {0, 0.05, 0.2, 1.0}) {
            var arena = Arena.emptyField(0, 0, 0);
            var b = robot(0.39, 4, -2, 0);
            b.heading = Math.toRadians(deg);
            arena.resolve(b);
            assertTrue(Math.abs(b.omega) < 0.2, deg + " deg: spin " + b.omega);
            assertTrue(b.vx > 0, deg + " deg: bounced " + b.vx);
        }
    }

    @Test
    void aFasterRobotPushesAParkedOneOnlyPastItsTraction() {
        var arena = Arena.emptyField(0, 0, 0);
        var parked = new SimRobot(5, 4, 0, 60, 0.9, 0.9, 1.1, 4.5, 10, SimRobot.parked());
        arena.add(parked);
        var ours = robot(4.16, 4, 1, 0); // bumpers just touching, rolling into it at 1 m/s
        for (int k = 0; k < 500; k++) {
            ours.vx = 1; // ours keeps pushing at 1 m/s whatever it takes
            ours.x += ours.vx * 0.001; // the swerve physics moves ours in the sim
            arena.step(ours, 0.001);
        }
        assertTrue(parked.body.x > 5.1, "pushed along: " + parked.body.x);
        assertTrue(ours.x < parked.body.x - 0.8, "bumpers not through each other");
    }

    @Test
    void aDefenderDrivesAtUs() {
        var arena = Arena.emptyField(0, 0, 0);
        var d = new SimRobot(10, 4, Math.PI, 60, 0.9, 0.9, 1.1, 4.5, 10, SimRobot.defender(3));
        arena.add(d);
        var ours = robot(5, 4, 0, 0);
        for (int k = 0; k < 2000; k++) {
            ours.x += ours.vx * 0.001; // ours rolls freely: no tires in this test
            arena.step(ours, 0.001);
        }
        assertTrue(d.body.x < 6.1, "reached us: " + d.body.x);
        assertTrue(ours.x < 5, "and pushed us back: " + ours.x);
    }

    @Test
    void anOpenFieldLeavesTheRobotAlone() {
        var arena = Arena.emptyField(0, 0, 0);
        var b = robot(8, 4, 3, 1);
        assertEquals(0, arena.resolve(b));
        assertEquals(3, b.vx, 0);
    }
}
