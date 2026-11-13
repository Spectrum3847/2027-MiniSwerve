package frc.robot.subsystems.swerve;

import frc.robot.RunMode;
import frc.robot.config.Hardware;
import frc.spectrumLib.framework.DesktopSim;
import frc.spectrumLib.framework.RobotLoop;
import frc.spectrumLib.hardware.SimFaults;
import frc.spectrumLib.statemachine.WantedState;
import frc.spectrumLib.swerve.DriveRequest;
import frc.spectrumLib.swerve.HeadingHold;
import frc.spectrumLib.swerve.HeadingMemory;
import frc.spectrumLib.swerve.ModuleTrust;
import frc.spectrumLib.swerve.SampleQueue;
import frc.spectrumLib.swerve.SimulatedSwerve;
import frc.spectrumLib.swerve.SwerveIO;
import frc.spectrumLib.swerve.SwerveIOPhoenix;
import frc.spectrumLib.swerve.SwerveInputs;
import frc.spectrumLib.swerve.SwerveOdometry;
import frc.spectrumLib.swerve.SwerveSample;
import frc.spectrumLib.swerve.SwerveSimSpec;
import frc.spectrumLib.swerve.SwerveThread;
import frc.spectrumLib.telemetry.Alert;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.Command;
import org.wpilib.command2.Subsystem;
import org.wpilib.driverstation.RobotState;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.util.Units;
import org.wpilib.util.Alert.Level;

/**
 * The drivetrain: wanted states on the main loop, our 250 Hz swerve thread underneath.
 *
 * <p>Main loop: {@link #updateInputs()} (drain samples, log, odometry) runs before the command
 * scheduler; commands {@link #request} states; {@link #applyWantedState()} after the scheduler
 * turns the wanted state into a {@link DriveRequest} for the thread.
 *
 * <p>Over 400 lines: the wanted states, the request each becomes and the logging of both are kept
 * together so the path from "what was asked" to "what the thread was sent" reads in one file. The
 * field heading ({@link FieldHeading}), the thread watch ({@link SwerveThreadWatch}) and
 * housekeeping ({@link SwerveUtilities}) are their own files.
 *
 * <p>Index: {@link State} | constructor | {@link #updateInputs} | {@link #applyWantedState} |
 * {@link #applyLimp} | commands | pose
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
        WHEEL_RADIUS_CHARACTERIZATION,
        /** Robot-relative velocity from an auto or utility ({@link #driveRobotRelative}). */
        ROBOT_VELOCITY
    }

    private static final int MODULES = 4;

    /** Sim steer friction (N m per rad/s at the module), fitted to FM's real steer overshoot. */
    private static final double SIM_STEER_FRICTION = 0.3;

    /**
     * Samples taken per main loop: 0.5 s at 250 Hz, so a long stall (GC, a settings reload) loses
     * none; memory is no concern on SystemCore. The queue holds 1 s.
     */
    private static final int MAX_SAMPLES_PER_LOOP = 128;

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
    private double limitOmega = Double.NaN;
    private double limitAlpha = Double.NaN;
    private final WantedState<State> state = new WantedState<>("Swerve", State.TELEOP_DRIVE);
    private final SwerveIO io;
    private final SwerveThread thread;
    private final SampleQueue queue;
    private final SwerveInputs inputs = new SwerveInputs(MODULES, MAX_SAMPLES_PER_LOOP);
    private final SwerveOdometry odometry;
    private final SimulatedSwerve sim;
    private final double maxDriveVelocity;
    private final double maxAngularVelocity;
    private final Alert disconnectedAlert = new Alert("", Level.HIGH);
    private static final String[] MODULE_NAMES = {"FL", "FR", "BL", "BR"};
    private String lastDegraded = "";
    private int sentModesVersion = -1;
    private final Alert threadFaultAlert = new Alert("", Level.HIGH);
    private final Alert droppedAlert = new Alert("", Level.MEDIUM);
    private long alertedDropped;
    private final Alert gyroAlert =
            new Alert("Swerve: Pigeon down, heading from wheels only", Level.HIGH);

    private double robotVx;
    private double robotVy;
    private double robotOmega;

    private DoubleSupplier teleopX = () -> 0;
    private DoubleSupplier teleopY = () -> 0;
    private DoubleSupplier teleopOmega = () -> 0;

    /**
     * @param simFaults simulated device faults (sim only; null otherwise)
     * @param desktopSim the desktop sim (SpectrumSim); null on the robot and in replay
     */
    public Swerve(SimFaults simFaults, DesktopSim desktopSim) {
        this.simFaults = simFaults;
        double half = SwerveSettings.TRACK.get() / 2;
        double[] mx = {half, half, -half, -half};
        double[] my = {half, -half, half, -half};
        moduleX = mx;
        moduleY = my;
        maxDriveVelocity = SwerveSettings.MAX_DRIVE_VELOCITY.get();
        maxAngularVelocity = SwerveSettings.MAX_ANGULAR_VELOCITY.get();
        odometry = new SwerveOdometry(mx, my);
        queue = new SampleQueue(2 * MAX_SAMPLES_PER_LOOP, MODULES);
        if (RunMode.hasHardware()) {
            sim =
                    desktopSim != null
                            ? desktopSim.swerve(
                                    SwerveConfigs.phoenix(MODULES),
                                    new SwerveSimSpec(
                                            "Falcon500FOC",
                                            "Falcon500FOC",
                                            SwerveSettings.robotMassKg(),
                                            SwerveSettings.TIRE_FRICTION.get(),
                                            0.004,
                                            SIM_STEER_FRICTION,
                                            mx,
                                            my,
                                            // Nominal-size sim wheels: a wrong wheelRadius setting
                                            // shows up as odometry error.
                                            Hardware.MK4iL2.NOMINAL_WHEEL_RADIUS_METERS,
                                            Units.inchesToMeters(
                                                    SwerveSettings.COG_HEIGHT_INCHES.get()),
                                            Units.inchesToMeters(
                                                    SwerveSettings.BUMPER_LENGTH_INCHES.get()),
                                            Units.inchesToMeters(
                                                    SwerveSettings.BUMPER_WIDTH_INCHES.get())),
                                    simFaults)
                            : null;
            io =
                    sim != null
                            ? sim.io()
                            : new SwerveIOPhoenix(SwerveConfigs.phoenix(MODULES), simFaults);
        } else {
            sim = null;
            io = SwerveIO.NONE;
        }
        utilities = new SwerveUtilities(io, mx, my);
        // The heading saved by the code that ran before this one (a code restart), as inputs.
        HeadingMemory memory =
                new HeadingMemory(RunMode.CURRENT == RunMode.Mode.REAL ? HEADING_FILE : null);
        HeadingMemory.Saved saved = memory.load();
        if (saved != null) {
            inputs.restoreYawOffset = saved.yawOffset();
            inputs.restoreYaw = saved.yaw();
            inputs.restoreAgeSeconds = saved.ageSeconds();
        }
        fieldHeading = new FieldHeading(odometry, memory, headingHold::reset);
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
                SwerveConfigs.plannedAccelerationLimit(),
                SwerveSettings.MAX_STEER_VELOCITY.get(),
                SwerveSettings.MAX_STEER_ACCELERATION.get());
        thread.setMotorLimit(SwerveConfigs.motorLimit(mx.length));
        SwerveConfigs.logTraction();
        threadWatch = new SwerveThreadWatch(thread, io.hasHardware());
        if (io.hasHardware()) {
            thread.start();
        }
    }

    /**
     * Where the field heading is kept across a code restart: memory-backed, so it is gone after the
     * robot is powered off (see {@link HeadingMemory}).
     */
    private static final java.nio.file.Path HEADING_FILE =
            java.nio.file.Path.of("/dev/shm/miniswerve-field-heading");

    private final FieldHeading fieldHeading;
    private final SwerveThreadWatch threadWatch;

    /**
     * Receives each 250 Hz sample, in order, with a continuous heading after it (rad: odometry's,
     * unmoved by field-centric re-zeroing) and whether that heading came from the gyro.
     */
    public interface SampleListener {
        void accept(SwerveSample sample, double heading, boolean fromGyro);
    }

    private SampleListener sampleListener;
    private final SimFaults simFaults;
    private final double[] moduleX;
    private final double[] moduleY;

    /** The pose estimator's feed: every sample, from the main loop's {@link #updateInputs}. */
    public void setSampleListener(SampleListener listener) {
        sampleListener = listener;
    }

    /** Module positions from the robot's center (m), FL FR BL BR. */
    public Translation2d[] moduleLocations() {
        Translation2d[] out = new Translation2d[moduleX.length];
        for (int i = 0; i < moduleX.length; i++) {
            out[i] = new Translation2d(moduleX[i], moduleY[i]);
        }
        return out;
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
            inputs.rejectedJumps = thread.rejectedJumps();
            inputs.threadCycles = thread.cycles();
            inputs.threadAlive = thread.alive();
            inputs.staleRequestCycles = thread.staleRequestCycles();
            inputs.nonFiniteDropped = thread.nonFinite();
            if (simFaults != null && simFaults.consumeReset("Code/SwerveThread")) {
                thread.dieForSim(); // sim script: the thread ends, as if it had died
            }
            if (RobotLoop.everySeconds(0.1)) {
                io.updateHealth(inputs.health);
            }
        }
        Logger.processInputs("Swerve", inputs);
        threadWatch.update(inputs);
        thread.setBusVoltage(inputs.health.supplyVoltage);
        trust.update(inputs.health);
        if (trust.version() != sentModesVersion) {
            sentModesVersion = trust.version();
            thread.setModuleCommands(trust.commands());
        }
        if (io.hasHardware() && RobotLoop.everySeconds(0.5)) {
            // Keep retrying: a device that rebooted or missed its config comes back on its own.
            io.retryFailedConfigs();
        }
        for (int s = 0; s < inputs.count; s++) {
            var sample = inputs.samples[s];
            odometry.update(sample, gyroOk() && sample.gyroOk, trust.weights(sample));
            if (sampleListener != null) {
                // Odometry's heading (the gyro's when healthy, the wheels' when not) without the
                // jumps of field-centric re-zeroing: a pose estimator would read them as turning.
                sampleListener.accept(
                        sample,
                        odometry.heading() - fieldHeading.resets(),
                        gyroOk() && sample.gyroOk);
            }
        }
        fieldHeading.update(
                inputs,
                gyroOk(),
                RobotState.isEnabled(),
                io.hasHardware() && RobotLoop.everySeconds(0.1));
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
                    case ROBOT_VELOCITY ->
                            DriveRequest.velocity(robotVx, robotVy, robotOmega, false, 0, true);
                    // Sticks drive only in teleop: in auto and utility they hold still unless an
                    // OpMode asks for something.
                    case TELEOP_DRIVE -> {
                        if (!RobotState.isTeleop()) {
                            yield DriveRequest.NEUTRAL;
                        }
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
        if (RobotState.isDisabled()) {
            // The motors are off whatever is asked. Neutral keeps the thread's plan on the real
            // wheels, so enabling with a stick already pushed ramps up from rest instead of
            // starting at a speed the plan reached while nothing moved.
            request = DriveRequest.NEUTRAL;
        }
        if (Double.isFinite(limitVelocity)
                || Double.isFinite(limitAcceleration)
                || Double.isFinite(limitOmega)
                || Double.isFinite(limitAlpha)) {
            request = request.withLimits(limitVelocity, limitAcceleration, limitOmega, limitAlpha);
        }
        // Robot-relative velocity an OpMode asked for: forward, left (m/s), CCW (rad/s).
        Logger.recordOutput(
                "Swerve/Request/RobotVelocity", new double[] {robotVx, robotVy, robotOmega});
        // This cycle's lowered limits: wheel speed (m/s), wheel acceleration (m/s^2), turn rate
        // (rad/s), turn acceleration (rad/s^2); NaN means configured. E.g. from a raised mechanism.
        Logger.recordOutput(
                "Swerve/Request/Limits",
                new double[] {limitVelocity, limitAcceleration, limitOmega, limitAlpha});
        limitVelocity = Double.NaN;
        limitAcceleration = Double.NaN;
        limitOmega = Double.NaN;
        limitAlpha = Double.NaN;
        thread.setRequest(request);
        // Module faults never stop the drivetrain: each module degrades on its own (ModuleTrust).
        state.setCurrent(wanted);
        // The request sent to the swerve thread: vx, vy (m/s), omega (rad/s).
        Logger.recordOutput("Swerve/Request/Mode", request.mode());
        Logger.recordOutput("Swerve/Request/Velocity", requestVelocity(request));
    }

    /**
     * The shortest path from the sticks to the wheels, for when {@link #applyWantedState} throws:
     * robot-relative (stick forward is the robot's front), no heading hold, no limits from other
     * mechanisms, nothing from odometry. Neutral outside teleop. Worse to drive, but it drives.
     */
    public void applyLimp() {
        DriveRequest request = DriveRequest.NEUTRAL;
        if (RobotState.isTeleop() && RobotState.isEnabled()) {
            request =
                    DriveRequest.velocity(
                            teleopX.getAsDouble() * maxDriveVelocity,
                            teleopY.getAsDouble() * maxDriveVelocity,
                            teleopOmega.getAsDouble() * maxAngularVelocity,
                            false,
                            0,
                            true);
        }
        thread.setRequest(request);
        // The request sent to the swerve thread by the limp path (see Stages/SwerveApply).
        Logger.recordOutput("Swerve/Request/Mode", request.mode());
        Logger.recordOutput("Swerve/Request/Velocity", requestVelocity(request));
    }

    private final double[] requestVelocityLog = new double[3];

    private double[] requestVelocity(DriveRequest r) {
        requestVelocityLog[0] = r.vx();
        requestVelocityLog[1] = r.vy();
        requestVelocityLog[2] = r.omega();
        return requestVelocityLog;
    }

    /** Battery voltage at the drive motors (V, logged input); 0 when none is reporting. */
    public double supplyVoltage() {
        return inputs.health.supplyVoltage;
    }

    /**
     * The pose in 3D for AdvantageScope's 3D field: field x, y and heading from {@code pose}, roll
     * and pitch from the Pigeon (logged inputs; flat while it is not connected). Height is not
     * measured, so the robot stays on the carpet. For display only.
     */
    public Pose3d pose3d(Pose2d pose) {
        return new Pose3d(
                pose.getX(),
                pose.getY(),
                0.0,
                new Rotation3d(
                        inputs.health.roll, inputs.health.pitch, pose.getRotation().getRadians()));
    }

    /** The Pigeon is connected and configured (logged input). */
    private boolean gyroOk() {
        return inputs.health.pigeonConnected;
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
        Pose2d odometryPose =
                new Pose2d(
                        odometry.x(),
                        odometry.y(),
                        org.wpilib.math.geometry.Rotation2d.fromRadians(odometry.heading()));
        // Wheel-and-gyro odometry pose (field frame).
        Logger.recordOutput("Swerve/Odometry/Pose", odometryPose);
        // The same pose tilted by the Pigeon's roll and pitch, for AdvantageScope's 3D field.
        Logger.recordOutput("Swerve/Odometry/Pose3d", pose3d(odometryPose));
        // RMS module disagreement over the last sample (m): slip or a bad module.
        Logger.recordOutput("Swerve/Odometry/Slip", odometry.slip());
        // Heading came from the gyro (false: from the wheels).
        Logger.recordOutput("Swerve/Odometry/UsedGyro", odometry.usedGyro());
        String degraded = trust.degradedSummary(MODULE_NAMES);
        if (!degraded.equals(lastDegraded)) {
            lastDegraded = degraded;
            disconnectedAlert.setText("Swerve degraded, still driving: " + degraded);
        }
        disconnectedAlert.set(!degraded.isEmpty());
        if (inputs.droppedSamples != alertedDropped) {
            alertedDropped = inputs.droppedSamples;
            droppedAlert.setText(
                    "Swerve samples dropped (main loop stalled): " + inputs.droppedSamples);
        }
        droppedAlert.set(inputs.droppedSamples > 0);
        if (inputs.threadFaults > 0 && !threadFaultAlert.get()) {
            threadFaultAlert.setText("Swerve thread faults: " + inputs.lastThreadFault);
        }
        threadFaultAlert.set(inputs.threadFaults > 0);
        gyroAlert.set(!gyroOk());
        if (sim != null) {
            sim.logTruth(odometry.x(), odometry.y());
        }
    }

    /**
     * Lowers the top speed, acceleration, turn rate and turn acceleration for this cycle only (NaN
     * keeps the configured value); call every loop while needed, e.g. while an elevator is raised.
     * The lowest of several calls in one loop wins. Calls made in a command's execute (after this
     * subsystem's update) apply from the next loop. Speed is clamped at 0; acceleration at 0.5
     * m/s^2 so the robot can still slow down.
     *
     * @param maxWheelVelocity m/s
     * @param maxDriveAcceleration m/s^2
     * @param maxAngularVelocity turn rate, rad/s (clamped at 0)
     * @param maxAngularAcceleration turn acceleration, rad/s^2 (clamped at 0.5 so it can stop)
     */
    public void limitDrive(
            double maxWheelVelocity,
            double maxDriveAcceleration,
            double maxAngularVelocity,
            double maxAngularAcceleration) {
        limitVelocity = lower(limitVelocity, maxWheelVelocity);
        limitAcceleration = lower(limitAcceleration, maxDriveAcceleration);
        limitOmega = lower(limitOmega, maxAngularVelocity);
        limitAlpha = lower(limitAlpha, maxAngularAcceleration);
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

    /**
     * Drives robot-relative at this velocity until {@link #stopRobotRelative}. Call every loop: the
     * newest call's velocity is used.
     *
     * @param vx forward, m/s
     * @param vy left, m/s
     * @param omega counterclockwise, rad/s
     * @param requester who asks, for the wanted-state log
     */
    public void driveRobotRelative(double vx, double vy, double omega, String requester) {
        robotVx = vx;
        robotVy = vy;
        robotOmega = omega;
        state.request(State.ROBOT_VELOCITY, requester);
    }

    /** Releases {@link #driveRobotRelative}; the swerve goes back to its default state. */
    public void stopRobotRelative(String requester) {
        robotVx = 0;
        robotVy = 0;
        robotOmega = 0;
        state.release(requester);
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

    /** The field heading (rad) that is "away from the driver"; see {@link FieldHeading}. */
    public double operatorForwardDirection() {
        return fieldHeading.operatorForward();
    }

    /**
     * CTRE's {@code seedFieldCentric}: the way the robot faces now becomes "away from the driver",
     * without moving the pose.
     */
    public void seedFieldCentric() {
        fieldHeading.seed();
    }

    public Pose2d odometryPose() {
        return new Pose2d(
                odometry.x(),
                odometry.y(),
                org.wpilib.math.geometry.Rotation2d.fromRadians(odometry.heading()));
    }
}
