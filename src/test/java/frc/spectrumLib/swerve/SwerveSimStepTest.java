package frc.spectrumLib.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.wpilib.hardware.hal.HAL;
import org.wpilib.simulation.DriverStationSim;

/**
 * Runs the real Phoenix IO against the physics sim: every module must steer to a target and drive
 * at a speed. Catches sign, invert, coupling and status-frame mistakes that unit math cannot.
 */
class SwerveSimStepTest {
    @org.junit.jupiter.params.ParameterizedTest(name = "steerMotionMagicExpo={0}")
    @org.junit.jupiter.params.provider.ValueSource(booleans = {false, true})
    void modulesSteerAndDrive(boolean motionMagicExpo) {
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
                        new PhoenixSwerveConfig.Gains(1000, 0, 30, 2, 0, 0),
                        250,
                        new double[3],
                        motionMagicExpo,
                        0.12 * 150.0 / 7,
                        1.2 / (150.0 / 7),
                        5.84 / 304);
        var io = new SwerveIOPhoenix(cfg, new SwervePhysicsSim(18, 0.004, 0.01, mx, my));
        var sample = new SwerveSample(4);
        double[] angle = {1.0, -1.0, 2.0, -2.5};
        double[] velocity = {1.0, 1.0, 1.0, 1.0};
        for (int k = 0; k < 250; k++) {
            DriverStationSim.notifyNewData();
            io.waitForSample(sample);
            io.setModules(angle, velocity, new double[4]);
        }
        for (int i = 0; i < 4; i++) {
            assertEquals(angle[i], sample.angle[i], 0.02, "module " + i + " angle");
            assertEquals(1.0, sample.velocity[i], 0.1, "module " + i + " velocity");
        }
    }
}
