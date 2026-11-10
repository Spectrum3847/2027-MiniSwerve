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
    void aLipStartsTheRampAboveTheCarpet() {
        Terrain t = new Terrain(new double[][] {{0, 0, 1.0, 1.0, 0.5, 15, 0, 0.02}});
        assertEquals(0.02 + 0.1 * Math.tan(Math.toRadians(15)), t.height(-0.4, 0), 1e-12);
    }

    @Test
    void the2026FieldMatchesTheDrawings() {
        FieldMap f = FieldMap.load("2026-rebuilt");
        assertEquals(651.22 * 0.0254, f.length(), 1e-9);
        assertEquals(317.69 * 0.0254, f.width(), 1e-9);
        assertEquals(4, f.terrain().bumps());
        assertEquals(6, f.elements().size()); // two hubs, four trench posts
        // GE-26100: 6.51 in peak at the middle; 0.61 in lip at each ramp's foot.
        assertEquals(6.51 * 0.0254, f.terrain().height(182.11 * 0.0254, 98.84 * 0.0254), 1e-9);
        assertEquals(
                0.61 * 0.0254, f.terrain().height((182.11 - 22.19) * 0.0254, 98.84 * 0.0254), 2e-4);
        // Off the bump, beside the hub, the floor is flat.
        assertEquals(0, f.terrain().height(182.11 * 0.0254, 158.84 * 0.0254), 0);
        FieldMap am = FieldMap.load("2026-rebuilt-andymark");
        assertTrue(am.length() < f.length());
    }
}
