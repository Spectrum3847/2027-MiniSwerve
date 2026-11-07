package frc.spectrumSim.tools;

import frc.spectrumLib.swerve.ModuleCommands;
import frc.spectrumLib.swerve.PhoenixSwerveConfig;
import frc.spectrumLib.swerve.SwerveIO;
import frc.spectrumLib.swerve.SwerveSample;
import frc.spectrumSim.MotorModel;
import frc.spectrumSim.swerve.PhoenixSwerveSim;
import frc.spectrumSim.swerve.SwerveIOSim;
import frc.spectrumSim.swerve.SwervePhysicsSim;
import java.util.Locale;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.simulation.DriverStationSim;

/**
 * Checks our TalonFX model against Phoenix's simulator: the same drivetrain, physics and commands
 * run once on each (Phoenix in real time, ours self-stepped), and the tool prints how far module
 * 0's angle and wheel speed differ, plus each one's step response. Run after changing {@code
 * TalonFXModel} or the physics:
 *
 * <p>{@code ./gradlew talonCrosscheck}
 *
 * <p>Commands: settle, steer 90 degrees, drive 2 m/s, steer back to 0 while driving, stop. Steer
 * variants: PositionVoltage, PositionTorqueCurrentFOC, and Motion Magic Expo of each.
 */
public final class TalonCrosscheck {
    private TalonCrosscheck() {}

    private static final int HZ = Integer.getInteger("crosscheck.hz", 250);
    private static final double[][] PHASES = { // seconds, angle (rad), wheel speed (m/s)
        {0.4, 0, 0}, {0.6, Math.PI / 2, 0}, {1.0, Math.PI / 2, 2}, {0.6, 0, 2}, {0.6, 0, 0}
    };

    public static void main(String[] args) {
        HAL.initialize();
        DriverStationSim.setEnabled(true);
        DriverStationSim.setDsAttached(true);
        DriverStationSim.notifyNewData();
        int id = 0;
        for (boolean voltage : new boolean[] {true, false}) {
            for (boolean expo : new boolean[] {false, true}) {
                String name =
                        (voltage ? "PositionVoltage" : "PositionTorqueFOC") + (expo ? " Expo" : "");
                // Phoenix devices need unique CAN IDs (0-62); the model makes none.
                double[][] phoenix = run(config(voltage, expo, id += 10), false);
                double[][] model = run(config(voltage, expo, id), true);
                report(name, phoenix, model);
            }
        }
        System.exit(0);
    }

    static PhoenixSwerveConfig config(boolean voltage, boolean expo, int id) {
        double steerRatio = 150.0 / 7;
        return new PhoenixSwerveConfig(
                "",
                id,
                new int[] {id + 1, id + 2, id + 3, id + 4},
                new int[] {id + 5, id + 6, id + 7, id + 8},
                new int[] {id + 1, id + 2, id + 3, id + 4},
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
                voltage
                        ? new PhoenixSwerveConfig.Gains(100, 0, 0.5, 0.1, 2.66, 0)
                        : new PhoenixSwerveConfig.Gains(1000, 0, 30, 2, 0, 0),
                HZ,
                new double[3],
                expo,
                0.12 * steerRatio,
                1.2 / steerRatio,
                MotorModel.FALCON_500_FOC.kT(),
                false,
                voltage,
                40,
                20);
    }

    /** Runs the command phases; returns per sample {time, angle0, velocity0}. */
    static double[][] run(PhoenixSwerveConfig cfg, boolean model) {
        double h = 0.1778;
        var physics =
                new SwervePhysicsSim(
                        MotorModel.FALCON_500_FOC,
                        MotorModel.FALCON_500_FOC,
                        30,
                        1.1,
                        0.004,
                        0.01,
                        new double[] {h, h, -h, -h},
                        new double[] {h, -h, h, -h});
        SwerveIO io =
                model
                        ? new SwerveIOSim(cfg, physics, null, null)
                        : PhoenixSwerveSim.create(cfg, physics, null);
        var sample = new SwerveSample(4);
        int total = 0;
        for (double[] p : PHASES) {
            total += (int) Math.round(p[0] * HZ);
        }
        double[][] out = new double[total][];
        double[] angle = new double[4];
        double[] velocity = new double[4];
        double[] zero = new double[4];
        int k = 0;
        for (double[] p : PHASES) {
            java.util.Arrays.fill(angle, p[1]);
            java.util.Arrays.fill(velocity, p[2]);
            for (int j = 0; j < (int) Math.round(p[0] * HZ); j++) {
                DriverStationSim.notifyNewData();
                io.waitForSample(sample);
                io.setModules(angle, velocity, zero, zero, ModuleCommands.allNormal(4));
                out[k] = new double[] {(double) k / HZ, sample.angle[0], sample.velocity[0]};
                k++;
            }
        }
        physics.stop();
        return out;
    }

    static void report(String name, double[][] phoenix, double[][] model) {
        double angleSq = 0, speedSq = 0, angleMax = 0;
        for (int k = 0; k < phoenix.length; k++) {
            double da =
                    Math.toDegrees(Math.IEEEremainder(phoenix[k][1] - model[k][1], 2 * Math.PI));
            double dv = phoenix[k][2] - model[k][2];
            angleSq += da * da;
            speedSq += dv * dv;
            angleMax = Math.max(angleMax, Math.abs(da));
        }
        int n = phoenix.length;
        System.out.printf(
                Locale.ROOT,
                "%-24s angle diff RMS %5.2f deg (max %5.1f)   speed diff RMS %5.3f m/s%n",
                name,
                Math.sqrt(angleSq / n),
                angleMax,
                Math.sqrt(speedSq / n));
        System.out.printf(
                Locale.ROOT,
                "    steer 0->90 deg: within 2 deg after  phoenix %5.3f s  model %5.3f s;"
                        + " overshoot phoenix %4.1f model %4.1f deg%n",
                settle(phoenix, 0.4, Math.PI / 2),
                settle(model, 0.4, Math.PI / 2),
                overshoot(phoenix, 0.4, 1.0, Math.PI / 2),
                overshoot(model, 0.4, 1.0, Math.PI / 2));
        System.out.printf(
                Locale.ROOT,
                "    drive 0->2 m/s: 90%% after  phoenix %5.3f s  model %5.3f s;"
                        + " mean speed 1.5-2.0 s phoenix %5.3f model %5.3f%n",
                rise(phoenix, 1.0, 1.8),
                rise(model, 1.0, 1.8),
                mean(phoenix, 1.5, 2.0),
                mean(model, 1.5, 2.0));
    }

    private static double settle(double[][] t, double from, double target) {
        double last = from;
        for (double[] s : t) {
            if (s[0] >= from
                    && s[0] < from + 0.6
                    && Math.abs(Math.IEEEremainder(s[1] - target, 2 * Math.PI))
                            > Math.toRadians(2)) {
                last = s[0];
            }
        }
        return last - from;
    }

    private static double overshoot(double[][] t, double from, double to, double target) {
        double most = 0;
        for (double[] s : t) {
            if (s[0] >= from && s[0] < to) {
                most = Math.max(most, Math.IEEEremainder(s[1] - target, 2 * Math.PI));
            }
        }
        return Math.toDegrees(most);
    }

    private static double rise(double[][] t, double from, double level) {
        for (double[] s : t) {
            if (s[0] >= from && s[2] >= level) {
                return s[0] - from;
            }
        }
        return Double.NaN;
    }

    private static double mean(double[][] t, double from, double to) {
        double sum = 0;
        int c = 0;
        for (double[] s : t) {
            if (s[0] >= from && s[0] < to) {
                sum += s[2];
                c++;
            }
        }
        return c > 0 ? sum / c : Double.NaN;
    }
}
