package frc.spectrumLib.swerve;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;

import frc.spectrumLib.hardware.SimFaults;
import frc.spectrumLib.sim.MotorModel;
import org.junit.jupiter.api.Test;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.simulation.DriverStationSim;

/** The deterministic drivetrain sim: it steers and drives, repeats exactly, and honors faults. */
class SwerveIOSimTest {
    private static final double H = 0.1778;

    private static PhoenixSwerveConfig config(boolean steerVoltage) {
        double steerRatio = 150.0 / 7;
        return new PhoenixSwerveConfig(
                "",
                1,
                new int[] {1, 2, 3, 4},
                new int[] {11, 12, 13, 14},
                new int[] {1, 2, 3, 4},
                new double[4],
                new boolean[] {false, true, false, true},
                true,
                (50.0 / 14) * (17.0 / 27) * (45.0 / 15),
                steerRatio,
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
                false,
                0.12 * steerRatio,
                1.2 / steerRatio,
                MotorModel.FALCON_500_FOC.kT(),
                false,
                steerVoltage,
                40,
                20);
    }

    /**
     * Runs a steer-then-drive sequence; returns module 0's angle and the true x after each step.
     */
    private static double[] drive(boolean steerVoltage, SimFaults faults) {
        HAL.initialize();
        DriverStationSim.setEnabled(true);
        DriverStationSim.setDsAttached(true);
        DriverStationSim.notifyNewData();
        var physics =
                new SwervePhysicsSim(
                        MotorModel.FALCON_500_FOC,
                        MotorModel.FALCON_500_FOC,
                        18,
                        1.1,
                        0.004,
                        0.01,
                        new double[] {H, H, -H, -H},
                        new double[] {H, -H, H, -H});
        var io = new SwerveIOSim(config(steerVoltage), physics, faults, null);
        var sample = new SwerveSample(4);
        double[] angle = {1.0, 1.0, 1.0, 1.0};
        double[] zero = new double[4];
        double[] speed = {1.0, 1.0, 1.0, 1.0};
        double[] out = new double[500 + 2];
        for (int k = 0; k < 500; k++) {
            io.waitForSample(sample);
            io.setModules(angle, k < 150 ? zero : speed, zero, zero, ModuleCommands.allNormal(4));
            out[k] = sample.angle[0];
        }
        out[500] = sample.velocity[0];
        out[501] = physics.truthX();
        return out;
    }

    @Test
    void steersAndDrivesWithEitherSteerOutput() {
        for (boolean voltage : new boolean[] {true, false}) {
            double[] r = drive(voltage, null);
            assertEquals(1.0, r[499], 0.02, "angle");
            assertEquals(1.0, r[500], 0.1, "wheel speed");
        }
    }

    @Test
    void twoRunsAreIdentical() {
        assertArrayEquals(drive(true, null), drive(true, null));
    }

    @Test
    void anUnpluggedDriveMakesNoTorque() {
        var faults = new SimFaults();
        for (int i = 0; i < 4; i++) {
            faults.set("Drive" + i, SimFaults.Fault.UNPLUGGED);
        }
        double[] r = drive(false, faults);
        assertEquals(0, r[501], 0.01, "robot does not move");
        assertFalse(faults.configFails("Steer0"), "steer motors still configured");
    }
}
