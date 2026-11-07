package frc.robot.subsystems.swerve;

import frc.robot.RunMode;
import frc.robot.config.Hardware;
import frc.spectrumLib.framework.RobotLoop;
import frc.spectrumLib.statemachine.WantedState;
import frc.spectrumLib.swerve.DriveRequest;
import frc.spectrumLib.swerve.HeadingHold;
import frc.spectrumLib.swerve.ModuleTrust;
import frc.spectrumLib.swerve.SampleQueue;
import frc.spectrumLib.swerve.SwerveIO;
import frc.spectrumLib.swerve.SwerveIOPhoenix;
import frc.spectrumLib.swerve.SwerveInputs;
import frc.spectrumLib.swerve.SwerveOdometry;
import frc.spectrumLib.swerve.SwervePhysicsSim;
import frc.spectrumLib.swerve.SwerveThread;
import frc.spectrumLib.telemetry.Alert;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.Command;
import org.wpilib.command2.Subsystem;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.util.Alert.Level;

/**
 * The drivetrain: wanted states on the main loop, our 250 Hz swerve thread underneath.
 *
 * <p>Main loop: {@link #updateInputs()} (drain samples, log, odometry) runs before the command
 * scheduler; commands {@link #request} states; {@link #applyWantedState()} after the scheduler
 * turns the wanted state into a {@link DriveRequest} for the thread.
 *
 * <p>Index: {@link State} | constructor | {@link #updateInputs} | {@link #applyWantedState} |
 * commands | pose
 */
public final class Swerve implements Subsystem {
    public enum State {
        /** Motors neutral. */
        IDLE,
        /** Field-relative driving from the pilot's sticks. */
        TELEOP_DRIVE,
        /** Wheels in an X. */
        X_LOCK,
        /** Spin in place to measure the wheel radius (utility mode). */
        WHEEL_RADIUS_CHARACTERIZATION
    }

    private static final int MODULES = 4;
    private static final int MAX_SAMPLES_PER_LOOP = 16;
    private static final int SWERVE_THREAD_PRIORITY = 1;

    private final ModuleTrust trust = new ModuleTrust("Swerve/Modules", MODULES);
    private final HeadingHold headingHold = new HeadingHold("Swerve/HeadingHold");
    private final boolean headingHoldEnabled = SwerveSettings.HEADING_HOLD.get();
    private final HeadingHold.Config headingHoldConfig =
            new HeadingHold.Config(
                    SwerveSettings.HEADING_HOLD_KP.get(),
                    SwerveSettings.HEADING_HOLD_MAX_CORRECTION.get(),
                    SwerveSettings.HEADING_HOLD_SETTLE_SECONDS.get(),
                    SwerveSettings.HEADING_HOLD_SETTLE_RATE.get(),
                    Math.toRadians(SwerveSettings.HEADING_HOLD_RELEASE_DEG.get()),
                    SwerveSettings.HEADING_HOLD_MIN_TRANSLATION.get());
    private final double[] velocityError = new double[MODULES];
    private final SwerveUtilities utilities;
    private double limitVelocity = Double.NaN;
    private double limitAcceleration = Double.NaN;
    private final WantedState<State> state = new WantedState<>("Swerve", State.TELEOP_DRIVE);
    private final SwerveIO io;
    private final SwerveThread thread;
    private final SampleQueue queue;
    private final SwerveInputs inputs = new SwerveInputs(MODULES, MAX_SAMPLES_PER_LOOP);
    private final SwerveOdometry odometry;
    private final SwervePhysicsSim sim;
    private final double maxDriveVelocity;
    private final double maxAngularVelocity;
    private final Alert disconnectedAlert = new Alert("", Level.HIGH);
    private final Alert threadFaultAlert = new Alert("", Level.HIGH);
    private final Alert gyroAlert =
            new Alert("Swerve: Pigeon down, heading from wheels only", Level.HIGH);

    private DoubleSupplier teleopX = () -> 0;
    private DoubleSupplier teleopY = () -> 0;
    private DoubleSupplier teleopOmega = () -> 0;

    public Swerve() {
        double half = SwerveSettings.TRACK.get() / 2;
        double[] mx = {half, half, -half, -half};
        double[] my = {half, -half, half, -half};
        maxDriveVelocity = SwerveSettings.MAX_DRIVE_VELOCITY.get();
        maxAngularVelocity = SwerveSettings.MAX_ANGULAR_VELOCITY.get();
        odometry = new SwerveOdometry(mx, my);
        queue = new SampleQueue(64, MODULES);
        if (RunMode.hasHardware()) {
            sim =
                    RunMode.CURRENT == RunMode.Mode.SIM
                            ? new SwervePhysicsSim(
                                    SwerveSettings.SIM_MASS.get(), 0.004, 0.01, mx, my)
                            : null;
            io = new SwerveIOPhoenix(SwerveConfigs.phoenix(MODULES), sim);
            if (sim != null) {
                // The sim's wheels are the nominal size; a wrong wheelRadius setting then shows up
                // as odometry error, and the wheel-radius routine should find the nominal value.
                sim.setTrueWheelRadius(Hardware.MK4iL2.NOMINAL_WHEEL_RADIUS_METERS);
            }
        } else {
            sim = null;
            io = SwerveIO.NONE;
        }
        utilities = new SwerveUtilities(io, mx, my);
        thread =
                new SwerveThread(
                        io,
                        queue,
                        mx,
                        my,
                        SwerveSettings.MAX_WHEEL_VELOCITY.get(),
                        1.0 / SwerveSettings.LOOP_HZ.get(),
                        RunMode.CURRENT == RunMode.Mode.REAL ? SWERVE_THREAD_PRIORITY : 0);
        thread.setRotationLookahead(SwerveSettings.ROTATION_LOOKAHEAD.get());
        thread.setSetpointGenerator(
                SwerveSettings.SETPOINT_GENERATOR.get(),
                SwerveSettings.MAX_DRIVE_ACCELERATION.get(),
                SwerveSettings.MAX_STEER_VELOCITY.get());
        if (io.hasHardware()) {
            thread.start();
        }
    }

    /** Where the pilot's sticks come from: -1..1 field x, field y, rotation. */
    public void setTeleopInputs(DoubleSupplier x, DoubleSupplier y, DoubleSupplier omega) {
        teleopX = x;
        teleopY = y;
        teleopOmega = omega;
    }

    /** Main loop, before the scheduler: drain the 250 Hz samples, log them, run odometry. */
    public void updateInputs() {
        if (io.hasHardware()) {
            inputs.count = queue.drainTo(inputs.samples);
            inputs.droppedSamples = queue.dropped();
            inputs.threadFaults = thread.faults();
            inputs.lastThreadFault = thread.lastFault();
            inputs.staleCycles = thread.staleCycles();
            if (RobotLoop.everySeconds(0.1)) {
                io.updateHealth(inputs.health);
            }
        }
        Logger.processInputs("Swerve", inputs);
        trust.update(inputs.health);
        for (int s = 0; s < inputs.count; s++) {
            var sample = inputs.samples[s];
            odometry.update(sample, gyroOk() && sample.gyroOk, trust.weights(sample));
        }
        utilities.update(
                inputs,
                RobotState.isEnabled(),
                state.current() == State.WHEEL_RADIUS_CHARACTERIZATION,
                RobotLoop.periodSeconds());
        logOutputs();
    }

    /** Main loop, after the scheduler: turn the wanted state into a request for the thread. */
    public void applyWantedState() {
        State wanted = state.wanted();
        DriveRequest request =
                switch (wanted) {
                    case IDLE -> DriveRequest.NEUTRAL;
                    case X_LOCK -> DriveRequest.X_LOCK;
                    // Robot-relative spin; the setpoint generator ramps it up and down.
                    case WHEEL_RADIUS_CHARACTERIZATION ->
                            DriveRequest.velocity(
                                    0, 0, SwerveSettings.WHEEL_RADIUS_SPIN.get(), false, 0, true);
                    case TELEOP_DRIVE -> {
                        // Sticks are in the driver's frame; turn them into the field frame.
                        double fwd = operatorForwardDirection();
                        double x = teleopX.getAsDouble() * maxDriveVelocity;
                        double y = teleopY.getAsDouble() * maxDriveVelocity;
                        double turn = teleopOmega.getAsDouble();
                        double omega = turn * maxAngularVelocity;
                        // Always computed and logged, so a drive session shows what heading hold
                        // would do before anyone turns it on; added only when enabled.
                        double hold =
                                headingHold.calculate(
                                        headingHoldConfig,
                                        Math.abs(turn) > 0.02,
                                        Math.hypot(x, y),
                                        odometry.heading(),
                                        lastYawRate(),
                                        RobotLoop.periodSeconds());
                        if (headingHoldEnabled) {
                            omega += hold;
                        }

                        yield DriveRequest.velocity(
                                x * Math.cos(fwd) - y * Math.sin(fwd),
                                x * Math.sin(fwd) + y * Math.cos(fwd),
                                omega,
                                true,
                                gyroOk() ? odometry.yawOffset() : odometry.heading(),
                                gyroOk());
                    }
                };
        boolean devicesOk = allConnected();
        if (!devicesOk) {
            // Never drive on a module we cannot read.
            request = DriveRequest.NEUTRAL;
        }
        if (Double.isFinite(limitVelocity) || Double.isFinite(limitAcceleration)) {
            request = request.withLimits(limitVelocity, limitAcceleration);
        }
        // This cycle's lowered limits (NaN: configured), e.g. from a raised mechanism.
        Logger.recordOutput(
                "Swerve/Request/Limits", new double[] {limitVelocity, limitAcceleration});
        limitVelocity = Double.NaN;
        limitAcceleration = Double.NaN;
        thread.setRequest(request);
        state.setCurrent(devicesOk ? wanted : State.IDLE);
        // The request sent to the swerve thread: vx, vy (m/s), omega (rad/s).
        Logger.recordOutput("Swerve/Request/Mode", request.mode());
        Logger.recordOutput(
                "Swerve/Request/Velocity",
                new double[] {request.vx(), request.vy(), request.omega()});
    }

    /** The Pigeon is connected and configured (logged input). */
    private boolean gyroOk() {
        return inputs.health.pigeonConnected;
    }

    /**
     * Every module is trusted: connected, configured, magnet readable, CANcoder fused (logged
     * inputs only, so replay decides the same way). False until the first health read, so the robot
     * never drives on devices it has not checked.
     */
    private boolean allConnected() {
        return trust.allTrusted();
    }

    /** Yaw rate (rad/s): the gyro's when good, else the wheels' estimate. */
    private double lastYawRate() {
        if (inputs.count > 0 && gyroOk() && inputs.samples[inputs.count - 1].gyroOk) {
            return inputs.samples[inputs.count - 1].angularVelocityZWorld;
        }
        return odometry.robotVelocity()[2];
    }

    private void logOutputs() {
        // Heading hold's correction is being applied (false: computed and logged only).
        Logger.recordOutput("Swerve/HeadingHold/Applied", headingHoldEnabled);
        if (inputs.count > 0) {
            var last = inputs.samples[inputs.count - 1];
            for (int i = 0; i < MODULES; i++) {
                velocityError[i] = last.targetVelocity[i] - last.velocity[i];
            }
            // Commanded minus measured wheel speed per module (m/s): slip or a weak module.
            Logger.recordOutput("Swerve/Modules/VelocityError", velocityError);
        }
        // Each module's disagreement with the fitted motion over the last sample (m).
        Logger.recordOutput("Swerve/Modules/Slip", odometry.moduleSlip());
        // Modules used by odometry in the last sample.
        Logger.recordOutput("Swerve/Odometry/ModulesUsed", odometry.modulesUsed());
        // Measured robot-frame chassis velocity from the last sample: vx, vy (m/s), omega (rad/s).
        // Compare with Swerve/Request/Velocity to tune lookahead and acceleration limits.
        Logger.recordOutput("Swerve/Odometry/RobotVelocity", odometry.robotVelocity());
        // Wheel-and-gyro odometry pose (field frame).
        Logger.recordOutput(
                "Swerve/Odometry/Pose",
                new Pose2d(
                        odometry.x(),
                        odometry.y(),
                        org.wpilib.math.geometry.Rotation2d.fromRadians(odometry.heading())));
        // RMS module disagreement over the last sample (m): slip or a bad module.
        Logger.recordOutput("Swerve/Odometry/Slip", odometry.slip());
        // Heading came from the gyro (false: from the wheels).
        Logger.recordOutput("Swerve/Odometry/UsedGyro", odometry.usedGyro());
        boolean connected = allConnected();
        disconnectedAlert.setText(
                "Swerve: a drive, steer or CANcoder is disconnected or failed to configure");
        disconnectedAlert.set(!connected);
        if (inputs.threadFaults > 0 && !threadFaultAlert.get()) {
            threadFaultAlert.setText("Swerve thread faults: " + inputs.lastThreadFault);
        }
        threadFaultAlert.set(inputs.threadFaults > 0);
        gyroAlert.set(!gyroOk());
        if (sim != null) {
            // Sim only: the true pose, for judging odometry. Never read by logic.
            Pose2d truth =
                    new Pose2d(
                            sim.truthX(),
                            sim.truthY(),
                            org.wpilib.math.geometry.Rotation2d.fromRadians(sim.truthHeading()));
            Logger.recordOutput("Sim/TruthPose", truth);
            // Odometry position error against truth (m).
            Logger.recordOutput(
                    "Sim/OdometryError",
                    Math.hypot(odometry.x() - sim.truthX(), odometry.y() - sim.truthY()));
        }
    }

    /**
     * Lowers the top speed and acceleration for this cycle only (NaN keeps the configured value);
     * call every loop while needed, e.g. while an elevator is raised. The lowest of several calls
     * in one loop wins.
     *
     * @param maxWheelVelocity m/s
     * @param maxDriveAcceleration m/s^2
     */
    public void limitDrive(double maxWheelVelocity, double maxDriveAcceleration) {
        limitVelocity = lower(limitVelocity, maxWheelVelocity);
        limitAcceleration = lower(limitAcceleration, maxDriveAcceleration);
    }

    private static double lower(double a, double b) {
        if (!Double.isFinite(a)) {
            return b;
        }
        return Double.isFinite(b) ? Math.min(a, b) : a;
    }

    /**
     * Spins in place while scheduled and logs the measured wheel radius
     * (Swerve/WheelRadius/Estimate); prints it when done. Run in utility mode on carpet, then copy
     * the value into setting swerve/wheelRadius.
     */
    public Command characterizeWheelRadius() {
        String name = "wheelRadius";
        return startEnd(
                        () -> state.request(State.WHEEL_RADIUS_CHARACTERIZATION, name),
                        () -> {
                            state.release(name);
                            System.out.println(
                                    "Swerve wheel radius estimate: "
                                            + utilities.wheelRadiusEstimate()
                                            + " m (setting swerve/wheelRadius is "
                                            + SwerveSettings.WHEEL_RADIUS.get()
                                            + ")");
                        })
                .withName("Swerve.CharacterizeWheelRadius");
    }

    /** Ask for a state; see {@link WantedState#request}. */
    public void request(State s, String requester) {
        state.request(s, requester);
    }

    /** A command that holds a state while scheduled and releases it after. */
    public Command hold(State s, String name) {
        return startEnd(() -> state.request(s, name), () -> state.release(name))
                .withName("Swerve." + name);
    }

    /**
     * CTRE's {@code setOperatorForwardDirection}, set from the alliance: the field heading (rad)
     * that is "away from the driver". 0 on blue, pi on red; blue when the alliance is unknown.
     * Alliance is a Driver Station input, so replay sees the same value.
     */
    public double operatorForwardDirection() {
        return MatchState.getAlliance().orElse(Alliance.BLUE) == Alliance.RED ? Math.PI : 0.0;
    }

    /**
     * CTRE's {@code seedFieldCentric}: the way the robot faces now becomes "away from the driver",
     * without moving the pose.
     */
    public void seedFieldCentric() {
        headingHold.reset();
        odometry.resetPose(odometry.x(), odometry.y(), operatorForwardDirection());
    }

    public Pose2d odometryPose() {
        return new Pose2d(
                odometry.x(),
                odometry.y(),
                org.wpilib.math.geometry.Rotation2d.fromRadians(odometry.heading()));
    }
}
