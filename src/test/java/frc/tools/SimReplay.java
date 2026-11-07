package frc.tools;

import com.ctre.phoenix6.unmanaged.Unmanaged;
import frc.spectrumLib.swerve.ModuleCommands;
import frc.spectrumLib.swerve.MotorModel;
import frc.spectrumLib.swerve.PhoenixSwerveConfig;
import frc.spectrumLib.swerve.SwerveIOPhoenix;
import frc.spectrumLib.swerve.SwervePhysicsSim;
import frc.spectrumLib.swerve.SwerveSample;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.PrintWriter;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.simulation.DriverStationSim;

/**
 * Replays a real robot's module commands through our physics sim and writes what the sim did, for
 * tools/sim_vs_real.py. The sim is configured as FM (2026): MK5n R2 modules, Kraken X60 FOC,
 * TorqueCurrentFOC drive and steer with FM's gains and limits.
 *
 * <p>{@code ./gradlew simReplay -Pin=build/simreal/q32.csv -Pout=build/simreal/q32_sim.csv}
 * (optional {@code -Pmass=60 -Pfriction=1.1}). Runs in real time: Phoenix's simulator runs the
 * motor loops on the wall clock.
 */
public final class SimReplay {
    private SimReplay() {}

    /** FM 2026 (2026-FM-SystemCore SwerveConfig): what its logs come from. */
    static PhoenixSwerveConfig fmConfig() {
        return new PhoenixSwerveConfig(
                "",
                40,
                new int[] {41, 42, 43, 44},
                new int[] {51, 52, 53, 54},
                new int[] {41, 42, 43, 44},
                new double[4],
                new boolean[] {false, true, false, true},
                true,
                6.03,
                26.09,
                54.0 / 14.0,
                0.0499,
                80,
                60,
                new PhoenixSwerveConfig.Gains(10, 0, 0, 4, 0, 0),
                new PhoenixSwerveConfig.Gains(500, 0, 20, 0.15, 1.0, 0),
                250,
                new double[3],
                false,
                1.0,
                0.05,
                MotorModel.KRAKEN_X60_FOC.kT(),
                false,
                false,
                30);
    }

    private static List<double[]> read(Path in) throws IOException {
        List<double[]> rows = new ArrayList<>();
        try (BufferedReader r = Files.newBufferedReader(in)) {
            r.readLine();
            for (String line; (line = r.readLine()) != null; ) {
                String[] p = line.split(",");
                double[] v = new double[p.length];
                for (int i = 0; i < p.length; i++) {
                    v[i] = Double.parseDouble(p[i]);
                }
                rows.add(v);
            }
        }
        return rows;
    }

    @SuppressWarnings("SystemExitOutsideMain") // a command-line tool
    public static void main(String... args) throws IOException {
        List<double[]> rows = read(Path.of(args[0]));
        Path out = Path.of(args[1]);
        double massKg = args.length > 2 ? Double.parseDouble(args[2]) : 60.0;
        double friction = args.length > 3 ? Double.parseDouble(args[3]) : 1.1;
        double steerInertia = args.length > 4 ? Double.parseDouble(args[4]) : 0.01;
        double steerFriction = args.length > 5 ? Double.parseDouble(args[5]) : 0.3;
        double gearEfficiency = args.length > 6 ? Double.parseDouble(args[6]) : 0.9;
        double gearDrag = args.length > 7 ? Double.parseDouble(args[7]) : 0.1;
        HAL.initialize();
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        double h = 21.75 * 0.0254 / 2;
        var sim =
                new SwervePhysicsSim(
                        MotorModel.KRAKEN_X60_FOC,
                        MotorModel.KRAKEN_X60_FOC,
                        massKg,
                        friction,
                        steerInertia,
                        steerFriction,
                        new double[] {h, h, -h, -h},
                        new double[] {h, -h, h, -h});
        sim.setGearLosses(gearEfficiency, gearDrag);
        var io = new SwerveIOPhoenix(fmConfig(), sim, null);
        var sample = new SwerveSample(4);
        double[] angle = new double[4];
        double[] velocity = new double[4];
        double[] zero = new double[4];
        var commands = ModuleCommands.allNormal(4);
        double end = rows.get(rows.size() - 1)[0];
        long startNanos = System.nanoTime();
        double[] lastV = sim.bodyVelocity();
        double lastT = 0;
        double nextWrite = 0;
        int row = 0;
        try (PrintWriter w = new PrintWriter(Files.newBufferedWriter(out))) {
            w.println("t,sv0,sv1,sv2,sv3,sa0,sa1,sa2,sa3,sax,say,si0,si1,si2,si3");
            while (true) {
                double t = (System.nanoTime() - startNanos) * 1e-9;
                if (t > end) {
                    break;
                }
                while (row + 1 < rows.size() && rows.get(row + 1)[0] <= t) {
                    row++;
                }
                double[] cmd = rows.get(row);
                for (int i = 0; i < 4; i++) {
                    velocity[i] = cmd[1 + i];
                    angle[i] = cmd[5 + i];
                }
                Unmanaged.feedEnable(100);
                DriverStationSim.notifyNewData();
                io.waitForSample(sample);
                io.setModules(angle, velocity, zero, zero, commands);
                if (t >= nextWrite) {
                    double[] v = sim.bodyVelocity();
                    double dt = Math.max(1e-3, t - lastT);
                    // Body-frame acceleration in g, as a Pigeon reports it (incl. the omega x v
                    // term).
                    double axG = ((v[0] - lastV[0]) / dt - v[2] * v[1]) / 9.81;
                    double ayG = ((v[1] - lastV[1]) / dt + v[2] * v[0]) / 9.81;
                    lastV = v;
                    lastT = t;
                    double[] amps = sim.driveCurrents();
                    w.printf(
                            Locale.ROOT,
                            "%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%.2f%n",
                            t,
                            sample.velocity[0],
                            sample.velocity[1],
                            sample.velocity[2],
                            sample.velocity[3],
                            sample.angle[0],
                            sample.angle[1],
                            sample.angle[2],
                            sample.angle[3],
                            axG,
                            ayG,
                            Math.abs(amps[0]),
                            Math.abs(amps[1]),
                            Math.abs(amps[2]),
                            Math.abs(amps[3]));
                    nextWrite += 0.01;
                }
            }
        }
        System.out.println(
                "SimReplay wrote "
                        + out
                        + String.format(Locale.ROOT, " (physics %.0f Hz)", sim.steps() / end));
        System.exit(0);
    }
}
