package frc.spectrumLib.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

/** The thread's guards: a stale request becomes a stop, and a NaN never stays in the plan. */
class SwerveThreadGuardsTest {
    private static final double H = 0.1778;
    private static final double[] MX = {H, H, -H, -H};
    private static final double[] MY = {H, -H, H, -H};

    private static final class RecordingIO implements SwerveIO {
        double[] angle = new double[4];
        double[] velocity = new double[4];

        @Override
        public void waitForSample(SwerveSample out) {}

        @Override
        public void setModules(double[] a, double[] v, double[] sv, double[] f, ModuleCommands c) {
            angle = a.clone();
            velocity = v.clone();
        }

        @Override
        public void setNeutral() {}

        @Override
        public void updateHealth(SwerveHealth out) {}
    }

    private static SwerveThread thread(RecordingIO io) {
        return new SwerveThread(io, new SampleQueue(4, 4), MX, MY, 4.5, 0.004, 0);
    }

    @Test
    void aRequestTheMainLoopStopsRenewingBecomesAStop() {
        SwerveThread t = thread(new RecordingIO());
        DriveRequest drive = DriveRequest.velocity(2, 0, 0, false, 0, true);
        t.setRequest(drive);
        int limit = (int) Math.ceil(SwerveThread.STALE_REQUEST_SECONDS / 0.004);
        // Obeyed for the whole allowance...
        for (int i = 0; i <= limit; i++) {
            assertSame(drive, t.freshRequest());
        }
        // ...then a stop, counted, until the main loop speaks again.
        DriveRequest stale = t.freshRequest();
        assertEquals(DriveRequest.Mode.VELOCITY, stale.mode());
        assertEquals(0.0, stale.vx(), 0.0);
        assertEquals(1, t.staleRequestCycles());
        t.setRequest(drive);
        assertSame(drive, t.freshRequest());
    }

    @Test
    void neutralAndXLockNeverGoStale() {
        SwerveThread t = thread(new RecordingIO());
        t.setRequest(DriveRequest.X_LOCK);
        for (int i = 0; i < 500; i++) {
            assertSame(DriveRequest.X_LOCK, t.freshRequest());
        }
        assertEquals(0, t.staleRequestCycles());
    }

    @Test
    void aNonFiniteRequestIsDroppedAndThePlanRecovers() {
        for (boolean generator : new boolean[] {false, true}) {
            RecordingIO io = new RecordingIO();
            SwerveThread t = thread(io);
            t.setSetpointGenerator(generator, 10, 20, 300);
            t.sampleForTest().timestamp = 1.0;
            t.computeTargets(DriveRequest.velocity(Double.NaN, 0, 0, false, 0, true));
            assertEquals(1, t.nonFinite());
            for (int i = 0; i < 4; i++) {
                assertTrue(Double.isFinite(io.velocity[i]) && Double.isFinite(io.angle[i]));
                assertEquals(0.0, io.velocity[i], 1e-9);
            }
            // The next good request drives: nothing non-finite was kept.
            for (int k = 0; k < 200; k++) {
                t.computeTargets(DriveRequest.velocity(1, 0, 0, false, 0, true));
            }
            for (int i = 0; i < 4; i++) {
                assertEquals(1.0, Math.abs(io.velocity[i]), 1e-6);
            }
        }
    }

    @Test
    void aNonFiniteHeadingIsDropped() {
        RecordingIO io = new RecordingIO();
        SwerveThread t = thread(io);
        t.computeTargets(DriveRequest.velocity(1, 0, 0, true, Double.NaN, false));
        assertEquals(1, t.nonFinite());
        for (int i = 0; i < 4; i++) {
            assertTrue(Double.isFinite(io.velocity[i]));
        }
    }
}
