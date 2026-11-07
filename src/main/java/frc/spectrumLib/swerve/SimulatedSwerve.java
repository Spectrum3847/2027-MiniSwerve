package frc.spectrumLib.swerve;

/** A drivetrain in the desktop sim (SpectrumSim): its IO and the true pose behind it. */
public interface SimulatedSwerve {
    /** The IO the swerve thread uses, as on the robot. */
    SwerveIO io();

    /**
     * Main thread: logs the sim's true pose and odometry's error against it. Sim truth, for judging
     * odometry; robot logic never reads it.
     */
    void logTruth(double odometryX, double odometryY);
}
