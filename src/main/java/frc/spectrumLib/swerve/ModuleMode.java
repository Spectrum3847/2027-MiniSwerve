package frc.spectrumLib.swerve;

/**
 * What one swerve module can still do. The drivetrain keeps driving with whatever modules still
 * work (AGENTS.md "Finish the match"); a fault only changes that module's mode.
 */
public enum ModuleMode {
    /** Drives and steers. */
    NORMAL,
    /**
     * Drive motor out (disconnected or not configured): the drive coasts, and the wheel is still
     * steered along the direction the robot's motion rolls it, so it does not scrub.
     */
    STEER_ONLY,
    /**
     * Steer motor out, or the module's angle has never been known since boot: drive and steer
     * coast, and the wheel drags. The other modules carry the robot.
     */
    COAST
}
