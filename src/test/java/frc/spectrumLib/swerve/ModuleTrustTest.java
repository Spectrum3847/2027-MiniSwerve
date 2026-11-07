package frc.spectrumLib.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Arrays;
import org.junit.jupiter.api.Test;

/** A module fault degrades that module only; nothing here can stop the drivetrain. */
class ModuleTrustTest {
    private static SwerveHealth healthy() {
        SwerveHealth h = new SwerveHealth(4);
        Arrays.fill(h.driveConnected, true);
        Arrays.fill(h.steerConnected, true);
        Arrays.fill(h.encoderConnected, true);
        Arrays.fill(h.driveConfigured, true);
        Arrays.fill(h.steerConfigured, true);
        Arrays.fill(h.encoderConfigured, true);
        Arrays.fill(h.fusedOk, true);
        Arrays.fill(h.magnetOk, true);
        return h;
    }

    @Test
    void eachFaultDegradesOnlyItsModule() {
        ModuleTrust t = new ModuleTrust("Test/Modules", 4);
        SwerveHealth h = healthy();
        t.update(h);
        h.driveConnected[0] = false; // drive motor out
        h.steerConnected[1] = false; // steer motor out
        h.encoderConnected[2] = false; // CANcoder lost after boot
        h.fusedOk[2] = false;
        t.update(h);
        ModuleMode[] m = t.modes();
        assertEquals(ModuleMode.STEER_ONLY, m[0]);
        assertEquals(ModuleMode.COAST, m[1]);
        assertEquals(ModuleMode.NORMAL, m[2], "angle known since boot: keep driving on the rotor");
        assertEquals(ModuleMode.NORMAL, m[3]);
        SwerveSample s = new SwerveSample(4);
        Arrays.fill(s.moduleOk, true);
        double[] w = t.weights(s);
        assertEquals(0.0, w[0]);
        assertEquals(0.0, w[1]);
        assertEquals(0.0, w[2], "odometry stays strict about the absolute angle");
        assertEquals(1.0, w[3]);
    }

    @Test
    void angleNeverKnownCoastsAndRecoveryReturnsToNormal() {
        ModuleTrust t = new ModuleTrust("Test/Modules", 4);
        SwerveHealth h = healthy();
        h.encoderConnected[0] = false; // CANcoder out from boot: angle never known
        h.driveConfigured[3] = false; // a config apply that has not landed yet
        t.update(h);
        assertEquals(ModuleMode.COAST, t.modes()[0]);
        assertEquals(ModuleMode.STEER_ONLY, t.modes()[3]);
        int v = t.version();
        h.encoderConnected[0] = true; // connector reseated, device rebooted, config retried
        h.driveConfigured[3] = true;
        t.update(h);
        assertEquals(ModuleMode.NORMAL, t.modes()[0]);
        assertEquals(ModuleMode.NORMAL, t.modes()[3]);
        assertEquals(v + 1, t.version());
    }

    @Test
    void aDisconnectedButConfiguredMotorKeepsItsCommandAtTheSlowRate() {
        ModuleTrust t = new ModuleTrust("Test/Modules", 4);
        SwerveHealth h = healthy();
        t.update(h);
        h.driveConnected[0] = false; // lost its status frames; may still be listening
        h.driveConfigured[1] = false; // config never applied: invert unknown, must coast
        t.update(h);
        ModuleCommands c = t.commands();
        assertEquals(true, c.drive()[0]);
        assertEquals(true, c.driveSlow()[0]);
        assertEquals(false, c.drive()[1]);
        assertEquals(false, c.driveSlow()[1]);
        assertEquals(true, c.steer()[0]);
        assertEquals(true, c.drive()[2]);
    }

    @Test
    void aFusedSensorFaultKeepsTheModuleDrivingButOutOfOdometry() {
        ModuleTrust t = new ModuleTrust("Test/Modules", 4);
        SwerveHealth h = healthy();
        t.update(h);
        h.fusedOk[0] = false; // rotor and CANcoder disagree; we don't know which is wrong
        t.update(h);
        assertEquals(ModuleMode.NORMAL, t.modes()[0]);
        assertEquals(true, t.commands().drive()[0]);
        SwerveSample s = new SwerveSample(4);
        Arrays.fill(s.moduleOk, true);
        assertEquals(0.0, t.weights(s)[0]);
    }
}
