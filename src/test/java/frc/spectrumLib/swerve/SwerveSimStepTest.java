package frc.spectrumLib.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

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
                        steerVoltage);
        var io = new SwerveIOPhoenix(cfg, new SwervePhysicsSim(18, 1.1, 0.004, 0.01, mx, my), null);
        var sample = new SwerveSample(4);
        // 1. Robot still: every module steers to its own angle.
        double[] angle = {1.0, -1.0, 2.0, -2.5};
        double[] still = new double[4];
        for (int k = 0; k < 250; k++) {
            DriverStationSim.notifyNewData();
            io.waitForSample(sample);
            io.setModules(angle, still, new double[4], new double[4], ModuleCommands.allNormal(4));
        }
        for (int i = 0; i < 4; i++) {
            assertEquals(angle[i], sample.angle[i], 0.02, "module " + i + " angle");
        }
        // 2. A motion a rigid robot can do: all wheels one way at 1 m/s.
        double[] same = {1.0, 1.0, 1.0, 1.0};
        double[] velocity = {1.0, 1.0, 1.0, 1.0};
        for (int k = 0; k < 375; k++) {
            DriverStationSim.notifyNewData();
            io.waitForSample(sample);
            io.setModules(
                    same, velocity, new double[4], new double[4], ModuleCommands.allNormal(4));
        }
        for (int i = 0; i < 4; i++) {
            assertEquals(1.0, sample.angle[i], 0.02, "module " + i + " angle");
            assertEquals(1.0, sample.velocity[i], 0.1, "module " + i + " velocity");
        }
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
                        true);
        var io =
                new SwerveIOPhoenix(
                        cfg,
                        new SwervePhysicsSim(
                                18,
                                1.1,
                                0.004,
                                0.01,
                                new double[] {h, h, -h, -h},
                                new double[] {h, -h, h, -h}),
                        null);
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
        for (int k = 0; k < 250; k++) {
            DriverStationSim.notifyNewData();
            io.waitForSample(sample);
            io.setModules(angle, velocity, new double[4], new double[4], modes);
        }
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
    }
}
