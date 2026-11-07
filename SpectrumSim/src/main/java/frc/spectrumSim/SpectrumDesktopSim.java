package frc.spectrumSim;

import frc.spectrumLib.framework.DesktopSim;
import frc.spectrumLib.hardware.SimFaults;
import frc.spectrumLib.swerve.PhoenixSwerveConfig;
import frc.spectrumLib.swerve.SimulatedSwerve;
import frc.spectrumLib.swerve.SwerveIO;
import frc.spectrumLib.swerve.SwerveSimSpec;
import frc.spectrumSim.field.Arena;
import frc.spectrumSim.swerve.PhoenixSwerveSim;
import frc.spectrumSim.swerve.SwerveIOSim;
import frc.spectrumSim.swerve.SwervePhysicsSim;

/**
 * SpectrumSim's {@link DesktopSim}, found by robot code through {@code ServiceLoader}
 * (META-INF/services). Runs everything on one {@link SimClock} with TalonFX models, so runs repeat.
 *
 * <p>Environment: {@code SPECTRUM_SIM_SPEED} is a multiple of real time or {@code max} (default max
 * for a scripted run, else real time); {@code SPECTRUM_SIM_DEVICES=phoenix} runs the drivetrain on
 * Phoenix's own simulator on the wall clock instead (not repeatable). {@code SPECTRUM_SIM_FIELD} is
 * {@code empty} (default: the field's walls) or {@code none} (open floor); {@code
 * SPECTRUM_SIM_START} is the robot's start on the field, {@code x,y,degrees} (default {@code
 * 2,2,0}: near the blue wall, clear of the walls for the scripted drives).
 */
public final class SpectrumDesktopSim implements DesktopSim {
    private SimClock clock;

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
        var physics =
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
        if (field != null && !field.isBlank() && !"empty".equals(field)) {
            throw new IllegalArgumentException("SPECTRUM_SIM_FIELD is empty or none, not " + field);
        }
        String start = System.getenv("SPECTRUM_SIM_START");
        double[] s = {2, 2, 0};
        if (start != null && !start.isBlank()) {
            String[] p = start.split(",");
            for (int i = 0; i < 3; i++) {
                s[i] = Double.parseDouble(p[i].trim());
            }
        }
        return Arena.emptyField(s[0], s[1], Math.toRadians(s[2]));
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
