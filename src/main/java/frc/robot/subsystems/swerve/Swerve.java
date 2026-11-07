package frc.robot.subsystems.swerve;

import frc.robot.RunMode;
import frc.robot.config.Hardware;
import frc.robot.config.ImuSettings;
import frc.spectrumLib.framework.RobotLoop;
import frc.spectrumLib.statemachine.WantedState;
import frc.spectrumLib.swerve.DriveRequest;
import frc.spectrumLib.swerve.PhoenixSwerveConfig;
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
        X_LOCK
    }

    private static final int MODULES = 4;
    private static final int MAX_SAMPLES_PER_LOOP = 16;
    private static final int SWERVE_THREAD_PRIORITY = 1;

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
            io = new SwerveIOPhoenix(phoenixConfig(), sim);
        } else {
            sim = null;
            io = SwerveIO.NONE;
        }
        thread =
                new SwerveThread(
                        io,
                        queue,
                        mx,
                        my,
                        SwerveSettings.MAX_WHEEL_VELOCITY.get(),
                        1.0 / SwerveSettings.LOOP_HZ.get(),
                        RunMode.CURRENT == RunMode.Mode.REAL ? SWERVE_THREAD_PRIORITY : 0);
        if (io.hasHardware()) {
            thread.start();
        }
    }

    private static PhoenixSwerveConfig phoenixConfig() {
        int[] drive = new int[MODULES];
        int[] steer = new int[MODULES];
        int[] encoder = new int[MODULES];
        double[] offsets = new double[MODULES];
        for (Hardware.Module m : Hardware.Module.values()) {
            int i = m.ordinal();
            drive[i] = m.driveId;
            steer[i] = m.steerId;
            encoder[i] = m.encoderId;
            offsets[i] = SwerveSettings.MAGNET_OFFSET[i].get();
        }
        return new PhoenixSwerveConfig(
                Hardware.CAN_BUS,
                Hardware.PIGEON_ID,
                drive,
                steer,
                encoder,
                offsets,
                Hardware.driveInverted(),
                Hardware.STEER_INVERTED,
                Hardware.MK4iL2.DRIVE_RATIO,
                Hardware.MK4iL2.STEER_RATIO,
                Hardware.MK4iL2.COUPLE_RATIO,
                SwerveSettings.WHEEL_RADIUS.get(),
                Hardware.DRIVE_SLIP_CURRENT_AMPS,
                Hardware.STEER_STATOR_LIMIT_AMPS,
                new PhoenixSwerveConfig.Gains(
                        SwerveSettings.DRIVE_KP.get(), 0, 0, SwerveSettings.DRIVE_KS.get(), 0, 0),
                new PhoenixSwerveConfig.Gains(
                        SwerveSettings.STEER_KP.get(),
                        0,
                        SwerveSettings.STEER_KD.get(),
                        SwerveSettings.STEER_KS.get(),
                        0,
                        0),
                SwerveSettings.LOOP_HZ.get(),
                new double[] {
                    ImuSettings.PIGEON_MOUNT_YAW.get(),
                    ImuSettings.PIGEON_MOUNT_PITCH.get(),
                    ImuSettings.PIGEON_MOUNT_ROLL.get()
                },
                SwerveSettings.STEER_MOTION_MAGIC_EXPO.get(),
                SwerveSettings.STEER_EXPO_KV.get(),
                SwerveSettings.STEER_EXPO_KA.get(),
                Hardware.MK4iL2.DRIVE_KT);
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
        for (int s = 0; s < inputs.count; s++) {
            odometry.update(inputs.samples[s], gyroOk() && inputs.samples[s].allGood);
        }
        logOutputs();
    }

    /** Main loop, after the scheduler: turn the wanted state into a request for the thread. */
    public void applyWantedState() {
        State wanted = state.wanted();
        DriveRequest request =
                switch (wanted) {
                    case IDLE -> DriveRequest.NEUTRAL;
                    case X_LOCK -> DriveRequest.X_LOCK;
                    case TELEOP_DRIVE -> {
                        // Sticks are in the driver's frame; turn them into the field frame.
                        double fwd = operatorForwardDirection();
                        double x = teleopX.getAsDouble() * maxDriveVelocity;
                        double y = teleopY.getAsDouble() * maxDriveVelocity;
                        yield DriveRequest.velocity(
                                x * Math.cos(fwd) - y * Math.sin(fwd),
                                x * Math.sin(fwd) + y * Math.cos(fwd),
                                teleopOmega.getAsDouble() * maxAngularVelocity,
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
     * Every module's devices are connected and took their config (logged inputs only, so replay
     * decides the same way). False until the first health read, so the robot never drives on
     * devices it has not checked.
     */
    private boolean allConnected() {
        var h = inputs.health;
        for (int i = 0; i < MODULES; i++) {
            if (!h.configured[i]
                    || !h.driveConnected[i]
                    || !h.steerConnected[i]
                    || !h.encoderConnected[i]) {
                return false;
            }
        }
        return true;
    }

    private void logOutputs() {
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
        odometry.resetPose(odometry.x(), odometry.y(), operatorForwardDirection());
    }

    public Pose2d odometryPose() {
        return new Pose2d(
                odometry.x(),
                odometry.y(),
                org.wpilib.math.geometry.Rotation2d.fromRadians(odometry.heading()));
    }
}
