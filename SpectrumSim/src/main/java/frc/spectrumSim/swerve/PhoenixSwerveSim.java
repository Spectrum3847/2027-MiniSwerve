package frc.spectrumSim.swerve;

import frc.spectrumLib.hardware.SimFaults;
import frc.spectrumLib.swerve.PhoenixSwerveConfig;
import frc.spectrumLib.swerve.SwerveIOPhoenix;

/**
 * The drivetrain on Phoenix's own simulator: the robot's real {@link SwerveIOPhoenix}, with {@link
 * SwervePhysicsSim} moving its simulated devices on a 1 kHz wall-clock thread. Not repeatable (CTRE
 * runs the motor loops on the wall clock); kept to check that path and our TalonFX models against
 * CTRE's ({@code ./gradlew talonCrosscheck}).
 */
public final class PhoenixSwerveSim {
    private PhoenixSwerveSim() {}

    /** Physics rate (Hz): as fast as a motor's own control loop. */
    private static final double PHYSICS_HZ = 1000;

    /** Builds the IO and starts the physics behind it. */
    public static SwerveIOPhoenix create(
            PhoenixSwerveConfig cfg, SwervePhysicsSim physics, SimFaults faults) {
        var io = new SwerveIOPhoenix(cfg, faults);
        var d = io.simDevices();
        physics.attach(
                cfg,
                new PhoenixSimDevices(
                        cfg, physics.steerMotor(), d.drive(), d.steer(), d.encoder(), d.pigeon()));
        physics.setSimFaults(faults);
        // Physics on its own 1 kHz thread, so the simulated motors' 1 kHz loops see fresh
        // positions as real ones do (stepping at 250 Hz with this thread made them lag).
        physics.start(PHYSICS_HZ);
        io.setSimPacer(start -> physics.waitForNextPeriod(start, cfg.signalHz()));
        return io;
    }
}
