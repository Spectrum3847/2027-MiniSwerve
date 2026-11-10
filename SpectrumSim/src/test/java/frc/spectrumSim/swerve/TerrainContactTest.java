package frc.spectrumSim.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.spectrumSim.field.Terrain;
import org.junit.jupiter.api.Test;

class TerrainContactTest {
    private static final double H = 0.25;
    private static final double[] MX = {H, H, -H, -H};
    private static final double[] MY = {H, -H, H, -H};

    /** A 15 degree ramp rising toward +x from x = 0, wide in y. */
    private static final Terrain RAMP = new Terrain(new double[][] {{5, 0, 10, 10, 5, 15, 0}});

    @Test
    void facingUpARampIsNoseUpAndPulledBack() {
        TerrainContact c = new TerrainContact(MX, MY);
        c.update(RAMP, 1.0, 0, 0);
        double slope = Math.toRadians(15);
        assertEquals(-slope, c.pitch(), 1e-9); // nose up is negative pitch (WPILib)
        assertEquals(0, c.roll(), 1e-9);
        assertEquals(1.0 * Math.tan(slope), c.height(), 1e-9);
        assertEquals(-9.81 * Math.sin(slope), c.ax(), 1e-9);
        assertEquals(0, c.ay(), 1e-9);
    }

    @Test
    void sidewaysOnARampRollsTowardTheLowSide() {
        TerrainContact c = new TerrainContact(MX, MY);
        // Facing +y: the ramp rises to the robot's right, so the left side is low.
        c.update(RAMP, 1.0, 0, Math.PI / 2);
        assertTrue(c.roll() < 0, "roll " + c.roll());
        assertEquals(0, c.pitch(), 1e-9);
        assertTrue(c.ay() > 0, "pulled left, downhill");
    }

    @Test
    void aWheelOnARampRollsTheFloorsLength() {
        TerrainContact c = new TerrainContact(MX, MY);
        c.update(RAMP, 1.0, 0, 0);
        double slope = Math.toRadians(15);
        assertEquals(1 / Math.cos(slope), c.stretch(0, 1, 0), 1e-6); // along the ramp
        assertEquals(1, c.stretch(0, 0, 1), 1e-6); // across it
        assertEquals(Math.cos(slope), c.normalShare(), 1e-9);
    }

    @Test
    void theLipAddsNoSlope() {
        // A ramp with a 2 cm lip at its foot: a wheel right on the lip sees the ramp's slope, not
        // the step's.
        Terrain lipped = new Terrain(new double[][] {{5, 0, 10, 10, 5, 15, 0, 0.02}});
        TerrainContact c = new TerrainContact(MX, MY);
        c.update(lipped, H + 0.001, 0, 0); // back wheels 1 mm onto the ramp, on the lip
        assertEquals(1 / Math.cos(Math.toRadians(15)), c.stretch(0, 1, 0), 1e-6);
        assertEquals(1 / Math.cos(Math.toRadians(15)), c.stretch(2, 1, 0), 1e-6);
    }

    @Test
    void diagonalRampTurnsUpOntoThePlaneNormal() {
        // Facing 30 deg off the ramp's rise: it slopes both ahead and to the side.
        TerrainContact c = new TerrainContact(MX, MY);
        double heading = Math.toRadians(30);
        c.update(RAMP, 1.0, 0, heading);
        var up =
                new org.wpilib.math.geometry.Translation3d(0, 0, 1)
                        .rotateBy(new org.wpilib.math.geometry.Rotation3d(c.roll(), c.pitch(), 0));
        // The ramp's normal in the robot frame: rise tan 15 deg along field x.
        double s = Math.tan(Math.toRadians(15));
        double gx = s * Math.cos(heading), gy = -s * Math.sin(heading);
        double norm = Math.sqrt(1 + gx * gx + gy * gy);
        assertEquals(-gx / norm, up.getX(), 1e-9);
        assertEquals(-gy / norm, up.getY(), 1e-9);
        assertEquals(1 / norm, up.getZ(), 1e-9);
    }

    @Test
    void flatFloorIsFlat() {
        TerrainContact c = new TerrainContact(MX, MY);
        c.update(Terrain.FLAT, 3, 3, 1);
        assertEquals(0, c.height() + c.roll() + c.pitch() + c.ax() + c.ay(), 0);
    }
}
