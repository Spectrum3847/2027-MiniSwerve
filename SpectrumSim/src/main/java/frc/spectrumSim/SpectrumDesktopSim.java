package frc.spectrumSim;

import frc.spectrumLib.framework.DesktopSim;
import frc.spectrumLib.hardware.SimFaults;
import frc.spectrumLib.swerve.PhoenixSwerveConfig;
import frc.spectrumLib.swerve.SimulatedSwerve;
import frc.spectrumLib.swerve.SwerveIO;
import frc.spectrumLib.swerve.SwerveSimSpec;
import frc.spectrumSim.field.Arena;
import frc.spectrumSim.field.FieldMap;
import frc.spectrumSim.field.SimRobot;
import frc.spectrumSim.swerve.PhoenixSwerveSim;
import frc.spectrumSim.swerve.SwerveIOSim;
import frc.spectrumSim.swerve.SwervePhysicsSim;
import frc.spectrumSim.vision.CameraSim;
import org.wpilib.fields.Field;
import org.wpilib.math.geometry.Transform3d;

/**
 * SpectrumSim's {@link DesktopSim}, found by robot code through {@code ServiceLoader}
 * (META-INF/services). Runs everything on one {@link SimClock} with TalonFX models, so runs repeat.
 *
 * <p>Environment: {@code SPECTRUM_SIM_SPEED} is a multiple of real time or {@code max} (default max
 * for a scripted run, else real time); {@code SPECTRUM_SIM_DEVICES=phoenix} runs the drivetrain on
 * Phoenix's own simulator on the wall clock instead (not repeatable). {@code SPECTRUM_SIM_FIELD} is
 * {@code empty} (default: the field's walls), {@code none} (open floor), or a season's field file
 * by name or path (walls, elements and bumps, e.g. {@code 2026-rebuilt}; see {@link FieldMap});
 * {@code SPECTRUM_SIM_START} is the robot's start on the field, {@code x,y,degrees} (default {@code
 * 2,2,0}: near the blue wall, clear of the walls for the scripted drives). {@code
 * SPECTRUM_SIM_ROBOTS} adds other robots, separated by {@code ;}: {@code parked:x,y,deg}, {@code
 * shuttle:x1,y1,x2,y2,speed} (drives back and forth), {@code defender:x,y,speed} (chases and pushes
 * ours; starts facing -x, toward the blue wall). Each is a 60 kg robot with 0.9 m square bumpers
 * and 1.1 tire friction.
 */
public final class SpectrumDesktopSim implements DesktopSim {
    private SimClock clock;
    private SwervePhysicsSim physics;

    @Override
    public void init(boolean scripted) {
        if (phoenixDevices()) {
            return; // Phoenix's simulator runs on the wall clock; no sim clock.
        }
        String speed = System.getenv("SPECTRUM_SIM_SPEED");
        if (speed == null || speed.isBlank()) {
            speed = scripted ? "max" : "1";
        }
        clock = new SimClock("max".equals(speed) ? 0 : Double.parseDouble(speed));
    }

    private static boolean phoenixDevices() {
        return "phoenix".equals(System.getenv("SPECTRUM_SIM_DEVICES"));
    }

    @Override
    public SimulatedSwerve swerve(PhoenixSwerveConfig cfg, SwerveSimSpec spec, SimFaults faults) {
        physics =
                new SwervePhysicsSim(
                        MotorModel.byName(spec.driveMotor()),
                        MotorModel.byName(spec.steerMotor()),
                        spec.massKg(),
                        spec.tireFriction(),
                        spec.steerInertia(),
                        spec.steerFriction(),
                        spec.mx(),
                        spec.my());
        SwerveIO io =
                clock != null
                        ? new SwerveIOSim(cfg, physics, faults, clock)
                        : PhoenixSwerveSim.create(cfg, physics, faults);
        physics.setCogHeight(spec.cogHeight());
        var arena = arena();
        if (arena != null) {
            physics.setArena(arena, spec.bumperLength(), spec.bumperWidth());
        }
        if (!Double.isNaN(spec.trueWheelRadius())) {
            physics.setTrueWheelRadius(spec.trueWheelRadius());
        }
        return new SimulatedSwerve() {
            @Override
            public SwerveIO io() {
                return io;
            }

            @Override
            public void logTruth(double odometryX, double odometryY) {
                physics.logTruth(odometryX, odometryY);
            }
        };
    }

    /** The field from the environment, or null for open floor. */
    private static Arena arena() {
        String field = System.getenv("SPECTRUM_SIM_FIELD");
        if ("none".equals(field)) {
            return null;
        }
        // empty (default): the four walls; else a season's field file (walls, elements, bumps).
        FieldMap map =
                field == null || field.isBlank() || "empty".equals(field)
                        ? null
                        : FieldMap.load(field);
        double length = map != null ? map.length() : Arena.FIELD_LENGTH;
        double width = map != null ? map.width() : Arena.FIELD_WIDTH;
        String start = System.getenv("SPECTRUM_SIM_START");
        double[] s = {2, 2, 0};
        if (start != null && !start.isBlank()) {
            String[] p = start.split(",");
            if (p.length != 3) {
                throw new IllegalArgumentException(
                        "SPECTRUM_SIM_START is x,y,degrees on the field, not " + start);
            }
            for (int i = 0; i < 3; i++) {
                s[i] = Double.parseDouble(p[i].trim());
            }
            if (s[0] < 0.5 || s[0] > length - 0.5 || s[1] < 0.5 || s[1] > width - 0.5) {
                throw new IllegalArgumentException(
                        "SPECTRUM_SIM_START " + start + " is not on the field, clear of the walls");
            }
        }
        double heading = Math.toRadians(s[2]);
        Arena arena =
                map != null
                        ? Arena.field(map, s[0], s[1], heading)
                        : Arena.emptyField(s[0], s[1], heading);
        String robots = System.getenv("SPECTRUM_SIM_ROBOTS");
        if (robots != null && !robots.isBlank()) {
            for (String r : robots.split(";")) {
                arena.add(robot(r.trim()));
            }
        }
        return arena;
    }

    /** One other robot from {@code kind:numbers}. */
    static SimRobot robot(String spec) {
        String[] kv = spec.split(":", 2);
        String[] p = kv.length > 1 ? kv[1].split(",") : new String[0];
        double[] v = new double[p.length];
        for (int i = 0; i < p.length; i++) {
            v[i] = Double.parseDouble(p[i].trim());
        }
        int need =
                switch (kv[0]) {
                    case "parked", "defender" -> 3;
                    case "shuttle" -> 5;
                    default ->
                            throw new IllegalArgumentException(
                                    "SPECTRUM_SIM_ROBOTS: parked, shuttle or defender, not "
                                            + kv[0]);
                };
        if (v.length != need) {
            throw new IllegalArgumentException(
                    "SPECTRUM_SIM_ROBOTS: " + kv[0] + " takes " + need + " numbers: " + spec);
        }
        checkOnField(v[0], v[1], spec);
        if (kv[0].equals("shuttle")) {
            checkOnField(v[2], v[3], spec);
        }
        return switch (kv[0]) {
            case "parked" -> other(v[0], v[1], Math.toRadians(v[2]), SimRobot.parked());
            case "shuttle" -> other(v[0], v[1], 0, SimRobot.shuttle(v[0], v[1], v[2], v[3], v[4]));
            case "defender" -> other(v[0], v[1], Math.PI, SimRobot.defender(v[2]));
            default ->
                    throw new IllegalArgumentException(
                            "SPECTRUM_SIM_ROBOTS: parked, shuttle or defender, not " + kv[0]);
        };
    }

    private static void checkOnField(double x, double y, String what) {
        if (x < 0.5 || x > Arena.FIELD_LENGTH - 0.5 || y < 0.5 || y > Arena.FIELD_WIDTH - 0.5) {
            throw new IllegalArgumentException(what + " is not on the field, clear of the walls");
        }
    }

    private static SimRobot other(double x, double y, double heading, SimRobot.Behavior b) {
        return new SimRobot(x, y, heading, 60, 0.9, 0.9, 1.1, 4.5, 10, b);
    }

    @Override
    public boolean cameras(Field field, String[] names, Transform3d[] robotToCamera) {
        if (clock == null || physics == null) {
            return false; // Phoenix's simulator on the wall clock: no repeatable cameras
        }
        SwervePhysicsSim truth = physics;
        for (int i = 0; i < names.length; i++) {
            var camera =
                    new CameraSim(
                            names[i],
                            robotToCamera[i],
                            CameraSim.Model.thriftiestCam(),
                            field,
                            truth::fieldPose3d);
            clock.addStepper(camera::step);
        }
        return true;
    }

    @Override
    public void start() {
        if (clock != null) {
            clock.start();
        }
    }

    @Override
    public void loopStarted() {
        if (clock != null) {
            clock.loopStarted();
        }
    }

    @Override
    public void loopDone() {
        if (clock != null) {
            clock.loopDone();
        }
    }
}
