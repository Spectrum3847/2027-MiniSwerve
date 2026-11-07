package frc.robot.subsystems.swerve;

import frc.spectrumLib.swerve.CoastOnDisable;
import frc.spectrumLib.swerve.SwerveIO;
import frc.spectrumLib.swerve.SwerveInputs;
import frc.spectrumLib.swerve.WheelRadiusCharacterizer;
import org.littletonrobotics.junction.Logger;

/**
 * The drivetrain's housekeeping beside driving: brake/coast on disable and wheel-radius
 * measurement. Kept out of {@link Swerve} so that file stays about driving.
 *
 * <p>Main thread; decisions use logged inputs only.
 */
final class SwerveUtilities {
    private final SwerveIO io;
    private final boolean coastOnDisable = SwerveSettings.COAST_ON_DISABLE.get();
    private final CoastOnDisable coast =
            new CoastOnDisable(0.05, SwerveSettings.COAST_DELAY_SECONDS.get());
    private final WheelRadiusCharacterizer radius;

    /** Boot configures brake, so nothing is sent until the first change. */
    private boolean brakeRequested = true;

    private double characterizingFor;
    private double sinceBrakeRequest;
    private boolean radiusStarted;

    /** Seconds of spinning before the wheel-radius measurement starts. */
    private static final double SETTLE_SECONDS = 1.0;

    SwerveUtilities(SwerveIO io, double[] mx, double[] my) {
        this.io = io;
        radius = new WheelRadiusCharacterizer(mx, my);
    }

    /**
     * Once per main loop, after inputs.
     *
     * @param enabled the robot is enabled (Driver Station input)
     * @param characterizing the wheel-radius state is active this loop
     */
    void update(SwerveInputs inputs, boolean enabled, boolean characterizing, double dt) {
        double fastest = 0;
        if (inputs.count > 0) {
            for (double v : inputs.samples[inputs.count - 1].velocity) {
                fastest = Math.max(fastest, Math.abs(v));
            }
        }
        boolean brake = !coastOnDisable || coast.brake(enabled, fastest, dt);
        sinceBrakeRequest += dt;
        // Request on a change, and again (about once a second) while the applied state does not
        // match: an apply can fail, and a rebooted drive motor comes back in its boot mode.
        boolean mismatch = inputs.health.driveBrake != brake;
        if (brake != brakeRequested || (mismatch && sinceBrakeRequest >= 1.0)) {
            // A config apply: SwerveIO does it on its own thread.
            io.setDriveBrake(brake);
            brakeRequested = brake;
            sinceBrakeRequest = 0;
        }
        // Drive motors should brake (false: coast); Health/DriveBrake is what was applied.
        Logger.recordOutput("Swerve/DriveBrakeWanted", brake);

        if (characterizing) {
            characterizingFor += dt;
            // Measure only once the spin is steady: the wheels turning to their spin angles and
            // the ramp-up would bias the ratio (team 6328 does the same).
            double yawRate =
                    inputs.count > 0 ? inputs.samples[inputs.count - 1].angularVelocityZWorld : 0;
            boolean steady =
                    Math.abs(yawRate) >= 0.9 * Math.abs(SwerveSettings.WHEEL_RADIUS_SPIN.get());
            if (characterizingFor >= SETTLE_SECONDS && (steady || radiusStarted)) {
                radiusStarted = true;
                double r = SwerveSettings.WHEEL_RADIUS.get();
                for (int s = 0; s < inputs.count; s++) {
                    radius.update(inputs.samples[s], r);
                }
            }
            // Measured wheel radius (m) from spinning in place; copy into swerve/wheelRadius.
            Logger.recordOutput("Swerve/WheelRadius/Estimate", radius.estimate());
            // Gyro turn so far in the measurement (rad).
            Logger.recordOutput("Swerve/WheelRadius/Turned", radius.turned());
        } else {
            characterizingFor = 0;
            radiusStarted = false;
            radius.stop();
        }
    }

    /** The current wheel-radius estimate (m), NaN until enough turn. */
    double wheelRadiusEstimate() {
        return radius.estimate();
    }
}
