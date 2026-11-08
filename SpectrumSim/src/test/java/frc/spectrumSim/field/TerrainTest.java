package frc.spectrumSim.field;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class TerrainTest {
    @Test
    void aBumpRampsUpAtItsAngleAndCapsAtItsHeight() {
        Terrain t = new Terrain(new double[][] {{5, 3, 2.0, 1.0, 0.1, 15, 0}});
        assertEquals(0, t.height(3.9, 3), 1e-12); // off the footprint
        // 0.1 m from the edge: 0.1 * tan 15.
        assertEquals(0.1 * Math.tan(Math.toRadians(15)), t.height(4.1, 3), 1e-12);
        assertEquals(0.1, t.height(5, 3), 1e-12); // flat top
        assertEquals(0, t.height(5, 3.6), 1e-12); // past its width
    }

    @Test
    void aTurnedBumpRunsAlongItsHeading() {
        Terrain t = new Terrain(new double[][] {{0, 0, 2.0, 0.5, 0.1, 15, 90}});
        assertEquals(0.1, t.height(0, 0.5), 1e-9);
        assertEquals(0, t.height(0.9, 0), 1e-12);
    }

    @Test
    void the2026FieldLoadsFourBumpsAndFlatIsTheDefault() {
        Terrain t = Terrain.load("2026-rebuilt");
        assertEquals(4, t.bumps());
        // A 2026 BUMP is 6.513 in tall but its 15 degree ramps meet before that over 44.4 in.
        double peak = t.height(4.626, 2.511);
        assertTrue(peak > 0.14 && peak <= 0.1654, "peak " + peak);
        assertEquals(0, Terrain.load(null).bumps());
        assertEquals(0, Terrain.load("flat").bumps());
    }
}
