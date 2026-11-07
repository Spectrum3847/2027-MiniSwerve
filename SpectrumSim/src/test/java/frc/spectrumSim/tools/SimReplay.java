package frc.spectrumSim.tools;

import com.ctre.phoenix6.unmanaged.Unmanaged;
import frc.spectrumLib.swerve.ModuleCommands;
import frc.spectrumLib.swerve.PhoenixSwerveConfig;
import frc.spectrumLib.swerve.SwerveIO;
import frc.spectrumLib.swerve.SwerveSample;
import frc.spectrumSim.MotorModel;
import frc.spectrumSim.swerve.PhoenixSwerveSim;
import frc.spectrumSim.swerve.SwerveIOSim;
import frc.spectrumSim.swerve.SwervePhysicsSim;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.PrintWriter;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.simulation.DriverStationSim;

/**
 * Replays a real robot's module commands through our physics sim and writes what the sim did, for
 * tools/sim_vs_real.py and tools/fit_sim.py. The sim is configured as FM (2026): MK5n R2 modules,
 * Kraken X60 FOC, TorqueCurrentFOC drive and steer with FM's gains and limits.
 *
 * <p>{@code ./gradlew simReplay -Pin=in.csv -Pout=out.csv -Pparams="mass=60 latency=0.01"}. Params
 * (defaults in {@link #DEFAULTS}): mass (kg), friction, steerInertia (kg m^2), steerFriction (N m
 * per rad/s), gearEff, gearDrag (N m), inertia (turning inertia factor), cog (center of gravity
 * height, m), latency (s from command to motor), realBattery (1: use the log's battery voltage).
 * {@code -Psweep=file} runs one line of params per output ({@code name key=value ...}, writing
 * {@code <out>_<name>.csv}) in one JVM.
 *
 * <p>Runs on our TalonFX models, self-stepped: repeatable and much faster than real time. {@code
 * phoenix=1} runs Phoenix's simulator instead, in real time.
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
                30,
                0);
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

    /** Parameters and their defaults. */
    static final Map<String, Double> DEFAULTS = new LinkedHashMap<>();

    static {
        DEFAULTS.put("mass", 60.0);
        DEFAULTS.put("friction", 1.1);
        DEFAULTS.put("steerInertia", 0.01);
        DEFAULTS.put("steerFriction", 0.3);
        DEFAULTS.put("gearEff", 0.9);
        DEFAULTS.put("gearDrag", 0.1);
        DEFAULTS.put("inertia", 0.6);
        DEFAULTS.put("cog", 0.25);
        DEFAULTS.put("latency", 0.0);
        DEFAULTS.put("realBattery", 1.0);
        DEFAULTS.put("phoenix", 0.0);
    }

    static Map<String, Double> params(String text) {
        Map<String, Double> p = new LinkedHashMap<>(DEFAULTS);
        for (String kv : text.trim().split("\\s+")) {
            if (kv.isEmpty()) {
                continue;
            }
            String[] a = kv.split("=");
            if (!DEFAULTS.containsKey(a[0])) {
                throw new IllegalArgumentException(
                        "unknown param " + a[0] + "; known " + DEFAULTS.keySet());
            }
            p.put(a[0], Double.parseDouble(a[1]));
        }
        return p;
    }

    @SuppressWarnings("SystemExitOutsideMain") // a command-line tool
    public static void main(String... args) throws IOException {
        List<double[]> rows = read(Path.of(args[0]));
        Path out = Path.of(args[1]);
        HAL.initialize();
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        if (args.length > 3 && !args[3].isBlank()) {
            String base = out.toString().replaceFirst("\\.csv$", "");
            for (String line : Files.readAllLines(Path.of(args[3]))) {
                String l = line.trim();
                if (l.isEmpty() || l.startsWith("#")) {
                    continue;
                }
                String[] parts = l.split("\\s+", 2);
                run(
                        rows,
                        Path.of(base + "_" + parts[0] + ".csv"),
                        params(parts.length > 1 ? parts[1] : ""));
            }
        } else {
            run(rows, out, params(args.length > 2 ? args[2] : ""));
        }
        System.exit(0);
    }

    /** CSV column of the log's battery voltage (fm_extract.py). */
    private static final int VBAT_COLUMN = 23;

    static void run(List<double[]> rows, Path out, Map<String, Double> p) throws IOException {
        double h = 21.75 * 0.0254 / 2;
        var sim =
                new SwervePhysicsSim(
                        MotorModel.KRAKEN_X60_FOC,
                        MotorModel.KRAKEN_X60_FOC,
                        p.get("mass"),
                        p.get("friction"),
                        p.get("steerInertia"),
                        p.get("steerFriction"),
                        new double[] {h, h, -h, -h},
                        new double[] {h, -h, h, -h});
        sim.setGearLosses(p.get("gearEff"), p.get("gearDrag"));
        sim.setInertiaFactor(p.get("inertia"));
        sim.setCogHeight(p.get("cog"));
        boolean phoenix = p.get("phoenix") > 0;
        boolean realBattery = p.get("realBattery") > 0;
        double latency = p.get("latency");
        SwerveIO io =
                phoenix
                        ? PhoenixSwerveSim.create(fmConfig(), sim, null)
                        : new SwerveIOSim(fmConfig(), sim, null, null);
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
        double t = 0;
        int row = 0;
        try (PrintWriter w = new PrintWriter(Files.newBufferedWriter(out))) {
            w.println("t,sv0,sv1,sv2,sv3,sa0,sa1,sa2,sa3,sax,say,si0,si1,si2,si3,svbat,syaw");
            while (t <= end) {
                // Commands reach the motors latency seconds after the log shows them.
                while (row + 1 < rows.size() && rows.get(row + 1)[0] <= t - latency) {
                    row++;
                }
                double[] cmd = rows.get(row);
                for (int i = 0; i < 4; i++) {
                    velocity[i] = t < latency ? 0 : cmd[1 + i];
                    angle[i] = cmd[5 + i];
                }
                if (realBattery && cmd.length > VBAT_COLUMN) {
                    sim.setBatteryOverride(cmd[VBAT_COLUMN]);
                }
                if (phoenix) {
                    Unmanaged.feedEnable(100);
                }
                DriverStationSim.notifyNewData();
                io.waitForSample(sample);
                io.setModules(angle, velocity, zero, zero, commands);
                t = phoenix ? (System.nanoTime() - startNanos) * 1e-9 : sample.timestamp;
                if (t >= nextWrite) {
                    double[] v = sim.bodyVelocity();
                    double dt = Math.max(1e-3, t - lastT);
                    // Body-frame acceleration in g, as a Pigeon reports it (incl. omega x v).
                    double axG = ((v[0] - lastV[0]) / dt - v[2] * v[1]) / 9.81;
                    double ayG = ((v[1] - lastV[1]) / dt + v[2] * v[0]) / 9.81;
                    lastV = v;
                    lastT = t;
                    double[] amps = sim.driveCurrents();
                    w.printf(
                            Locale.ROOT,
                            "%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
                                    + "%.2f,%.2f,%.2f,%.2f,%.3f,%.4f%n",
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
                            Math.abs(amps[3]),
                            sim.batteryVolts(),
                            v[2]);
                    nextWrite += 0.01;
                }
            }
        }
        sim.stop();
        System.out.println("SimReplay wrote " + out + " " + p);
    }
}
