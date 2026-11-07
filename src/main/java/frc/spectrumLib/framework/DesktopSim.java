package frc.spectrumLib.framework;

import frc.spectrumLib.hardware.SimFaults;
import frc.spectrumLib.swerve.PhoenixSwerveConfig;
import frc.spectrumLib.swerve.SimulatedSwerve;
import frc.spectrumLib.swerve.SwerveSimSpec;
import java.util.ServiceLoader;

/**
 * The desktop simulation as robot code sees it. SpectrumSim implements it; robot code and
 * spectrumLib never compile against SpectrumSim, so the robot jar carries none of it. It is found
 * on the desktop classpath with {@link ServiceLoader}.
 *
 * <p>Order: {@link #load} first in robot init (it fixes sim time before logging reads it), build
 * subsystems with their sim parts ({@link #swerve}), {@link #start} last in init, and call {@link
 * #loopStarted} / {@link #loopDone} around every main loop.
 */
public interface DesktopSim {
    /**
     * Starts SpectrumSim if it is on the classpath.
     *
     * @param scripted a sim script runs: default to as fast as possible instead of real time
     * @return the sim, or null when SpectrumSim is not on the classpath
     */
    static DesktopSim load(boolean scripted) {
        DesktopSim sim;
        try {
            sim = ServiceLoader.load(DesktopSim.class).findFirst().orElse(null);
        } catch (java.util.ServiceConfigurationError e) {
            throw new IllegalStateException(
                    "SpectrumSim is on the classpath but would not load", e);
        }
        if (sim != null) {
            sim.init(scripted);
        }
        return sim;
    }

    /** Called once by {@link #load}: pauses and fixes sim time, picks the pace. */
    void init(boolean scripted);

    /**
     * A simulated drivetrain: the IO the swerve thread uses plus the physics behind it.
     *
     * @param faults simulated device faults the sim honors
     */
    SimulatedSwerve swerve(PhoenixSwerveConfig cfg, SwerveSimSpec spec, SimFaults faults);

    /**
     * Cameras on the simulated robot: each sends frames ({@code VisionFrame}) on its NetworkTables
     * topic, seeing the field's AprilTags from the true pose. Call after {@link #swerve}.
     *
     * @param names camera names (their topics)
     * @param robotToCamera each camera's mount on the robot
     * @return whether the cameras are simulated (not with Phoenix's wall-clock simulator)
     */
    boolean cameras(
            org.wpilib.fields.Field field,
            String[] names,
            org.wpilib.math.geometry.Transform3d[] robotToCamera);

    /** Starts sim time. Last thing in robot init. */
    void start();

    /** Main thread, first thing each main loop. */
    void loopStarted();

    /** Main thread, last thing each main loop. */
    void loopDone();
}
