package frc.spectrumSim;

import frc.spectrumLib.framework.DesktopSim;
import frc.spectrumLib.hardware.SimFaults;
import frc.spectrumLib.swerve.PhoenixSwerveConfig;
import frc.spectrumLib.swerve.SimulatedSwerve;
import frc.spectrumLib.swerve.SwerveIO;
import frc.spectrumLib.swerve.SwerveSimSpec;
import frc.spectrumSim.swerve.PhoenixSwerveSim;
import frc.spectrumSim.swerve.SwerveIOSim;
import frc.spectrumSim.swerve.SwervePhysicsSim;

/**
 * SpectrumSim's {@link DesktopSim}, found by robot code through {@code ServiceLoader}
 * (META-INF/services). Runs everything on one {@link SimClock} with TalonFX models, so runs repeat.
 *
 * <p>Environment: {@code SPECTRUM_SIM_SPEED} is a multiple of real time or {@code max} (default max
 * for a scripted run, else real time); {@code SPECTRUM_SIM_DEVICES=phoenix} runs the drivetrain on
 * Phoenix's own simulator on the wall clock instead (not repeatable).
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
