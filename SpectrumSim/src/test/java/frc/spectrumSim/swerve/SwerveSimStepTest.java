package frc.spectrumSim.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.spectrumLib.swerve.ModuleCommands;
import frc.spectrumLib.swerve.PhoenixSwerveConfig;
import frc.spectrumLib.swerve.SwerveIOPhoenix;
import frc.spectrumLib.swerve.SwerveSample;
import frc.spectrumSim.MotorModel;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.simulation.DriverStationSim;

/**
 * Runs the real Phoenix IO against the physics sim: every module must steer to a target and drive
 * at a speed. Catches sign, invert, coupling and status-frame mistakes that unit math cannot.
 */
class SwerveSimStepTest {
    @org.junit.jupiter.params.ParameterizedTest(name = "steerVoltage={0} motionMagicExpo={1}")
    @org.junit.jupiter.params.provider.CsvSource({
        "true,false",
        "true,true",
        "false,false",
        "false,true"
    })
    void modulesSteerAndDrive(boolean steerVoltage, boolean motionMagicExpo) {
        HAL.initialize();
        DriverStationSim.setEnabled(true);
        DriverStationSim.setDsAttached(true);
        DriverStationSim.notifyNewData();
        double h = 0.1778;
        double[] mx = {h, h, -h, -h};
        double[] my = {h, -h, h, -h};
        double driveRatio = (50.0 / 14) * (17.0 / 27) * (45.0 / 15);
        var cfg =
                new PhoenixSwerveConfig(
                        "",
                        1,
                        new int[] {1, 2, 3, 4},
                        new int[] {11, 12, 13, 14},
                        new int[] {1, 2, 3, 4},
                        new double[] {0.1, -0.2, 0.3, -0.4},
                        new boolean[] {false, true, false, true},
                        true,
                        driveRatio,
                        150.0 / 7,
                        50.0 / 14,
                        0.0508,
                        80,
                        40,
                        new PhoenixSwerveConfig.Gains(10, 0, 0, 3, 0, 0),
                        steerVoltage
                                ? new PhoenixSwerveConfig.Gains(100, 0, 0.5, 0.1, 2.66, 0)
                                : new PhoenixSwerveConfig.Gains(1000, 0, 30, 2, 0, 0),
                        250,
                        new double[3],
                        motionMagicExpo,
                        0.12 * 150.0 / 7,
                        1.2 / (150.0 / 7),
                        5.84 / 304,
                        true,
                        steerVoltage,
                        0,
                        0);
        var physics =
                new SwervePhysicsSim(
                        MotorModel.FALCON_500_FOC,
                        MotorModel.FALCON_500_FOC,
                        18,
                        1.1,
                        0.004,
                        0.01,
                        mx,
                        my);
        var io = PhoenixSwerveSim.create(cfg, physics, null);
        var sample = new SwerveSample(4);
        // 1. Robot still: every module steers to its own angle.
        double[] angle = {1.0, -1.0, 2.0, -2.5};
        double[] still = new double[4];
        settle(io, sample, angle, still, ModuleCommands.allNormal(4), s -> anglesAt(s, angle));
        for (int i = 0; i < 4; i++) {
            assertEquals(angle[i], sample.angle[i], 0.02, "module " + i + " angle");
        }
        // 2. A motion a rigid robot can do: all wheels one way at 1 m/s.
        double[] same = {1.0, 1.0, 1.0, 1.0};
        double[] velocity = {1.0, 1.0, 1.0, 1.0};
        settle(
                io,
                sample,
                same,
                velocity,
                ModuleCommands.allNormal(4),
                s -> anglesAt(s, same) && speedsAt(s, velocity));
        for (int i = 0; i < 4; i++) {
            assertEquals(1.0, sample.angle[i], 0.02, "module " + i + " angle");
            assertEquals(1.0, sample.velocity[i], 0.1, "module " + i + " velocity");
        }
        physics.stop();
    }

    /** Samples a module must stay within tolerance for: 0.1 s at 250 Hz. */
    private static final int HOLD = 25;

    /** Longest a phase may take to settle: 4 s at 250 Hz. */
    private static final int MOST = 1000;

    /**
     * Commands the modules until {@code done} has held for {@link #HOLD} samples in a row, or for
     * {@link #MOST} samples. Phoenix's simulator runs on the wall clock, so a busy machine (CI) is
     * slower to settle; waiting for the result instead of a fixed time keeps the test about signs
     * and inverts, not machine speed. A real sign mistake never settles and still fails the
     * asserts.
     */
    private static void settle(
            SwerveIOPhoenix io,
            SwerveSample sample,
            double[] angle,
            double[] velocity,
            ModuleCommands modes,
            java.util.function.Predicate<SwerveSample> done) {
        int held = 0;
        for (int k = 0; k < MOST && held < HOLD; k++) {
            DriverStationSim.notifyNewData();
            io.waitForSample(sample);
            io.setModules(angle, velocity, new double[4], new double[4], modes);
            held = done.test(sample) ? held + 1 : 0;
        }
    }

    private static boolean anglesAt(SwerveSample s, double[] angle) {
        for (int i = 0; i < angle.length; i++) {
            if (Math.abs(Math.IEEEremainder(s.angle[i] - angle[i], 2 * Math.PI)) > 0.02) {
                return false;
            }
        }
        return true;
    }

    private static boolean speedsAt(SwerveSample s, double[] velocity) {
        for (int i = 0; i < velocity.length; i++) {
            if (Math.abs(s.velocity[i] - velocity[i]) > 0.1) {
                return false;
            }
        }
        return true;
    }

    @org.junit.jupiter.api.Test
    void degradedModulesCoastWhileTheRestStillSteerAndDrive() {
        HAL.initialize();
        DriverStationSim.setEnabled(true);
        DriverStationSim.setDsAttached(true);
        DriverStationSim.notifyNewData();
        double h = 0.1778;
        double driveRatio = (50.0 / 14) * (17.0 / 27) * (45.0 / 15);
        var cfg =
                new PhoenixSwerveConfig(
                        "",
                        1,
                        new int[] {21, 22, 23, 24},
                        new int[] {31, 32, 33, 34},
                        new int[] {21, 22, 23, 24},
                        new double[4],
                        new boolean[] {false, true, false, true},
                        true,
                        driveRatio,
                        150.0 / 7,
                        50.0 / 14,
                        0.0508,
                        80,
                        40,
                        new PhoenixSwerveConfig.Gains(10, 0, 0, 3, 0, 0),
                        new PhoenixSwerveConfig.Gains(100, 0, 0.5, 0.1, 2.66, 0),
                        250,
                        new double[3],
                        false,
                        0.12 * 150.0 / 7,
                        1.2 / (150.0 / 7),
                        5.84 / 304,
                        true,
                        true,
                        0,
                        0);
        var physics =
                new SwervePhysicsSim(
                        MotorModel.FALCON_500_FOC,
                        MotorModel.FALCON_500_FOC,
                        18,
                        1.1,
                        0.004,
                        0.01,
                        new double[] {h, h, -h, -h},
                        new double[] {h, -h, h, -h});
        var io = PhoenixSwerveSim.create(cfg, physics, null);
        var sample = new SwerveSample(4);
        double[] angle = {1.0, 1.0, 1.0, 1.0};
        double[] velocity = {1.0, 1.0, 1.0, 1.0};
        // Module 0: drive coasting, still steered. Module 1: both coasting. Modules 2-3 healthy.
        ModuleCommands modes =
                new ModuleCommands(
                        new boolean[] {false, false, true, true},
                        new boolean[] {true, false, true, true},
                        new boolean[4],
                        new boolean[4]);
        settle(
                io,
                sample,
                angle,
                velocity,
                modes,
                s ->
                        Math.abs(s.angle[0] - 1.0) < 0.02
                                && Math.abs(s.angle[2] - 1.0) < 0.02
                                && Math.abs(s.angle[3] - 1.0) < 0.02
                                && s.velocity[2] > 0.5
                                && s.velocity[3] > 0.5
                                && Math.abs(s.velocity[2] - s.velocity[0]) < 0.25);
        // Steer-only module still turns to its angle, and its coasting wheel rolls with the robot.
        assertEquals(1.0, sample.angle[0], 0.02);
        assertEquals(sample.velocity[2], sample.velocity[0], 0.25);
        // Coasting module is not steered (it stays where it was) and drags.
        assertEquals(0.0, sample.angle[1], 0.05);
        // Healthy modules still steer to target and drive the robot.
        for (int i = 2; i < 4; i++) {
            assertEquals(1.0, sample.angle[i], 0.02);
            assertTrue(sample.velocity[i] > 0.5, "module " + i + " drives");
        }
        physics.stop();
    }
}
