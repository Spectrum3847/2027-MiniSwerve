package frc.spectrumLib.swerve;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * Everything the drivetrain logic reads, as AdvantageKit inputs: the 250 Hz samples drained this
 * main loop (flattened, sample-major: {@code [s * modules + m]}) and the slow health readings.
 * Replay restores both, so odometry and every decision rerun on exactly what the robot saw.
 *
 * <p>Index: {@link #samples} | {@link #toLog} | {@link #fromLog}
 */
public final class SwerveInputs implements LoggableInputs {
    private final int modules;

    /** Samples drained this loop; only the first {@link #count} are valid. */
    public final SwerveSample[] samples;

    public int count;
    public final SwerveHealth health;

    /** Samples the swerve thread lost because the main loop fell behind, since boot. */
    public long droppedSamples;

    /** Swerve thread cycles that threw, since boot, and the last message. */
    public long threadFaults;

    public String lastThreadFault = "";

    /**
     * Swerve thread cycles in a row with stale or failed signals (logged only: stale modules are
     * planned from their last target).
     */
    public long staleCycles;

    /** Readings the swerve thread rejected as impossible jumps (device reboots), since boot. */
    public long rejectedJumps;

    public SwerveInputs(int modules, int maxSamplesPerLoop) {
        this.modules = modules;
        samples = new SwerveSample[maxSamplesPerLoop];
        for (int i = 0; i < samples.length; i++) {
            samples[i] = new SwerveSample(modules);
        }
        health = new SwerveHealth(modules);
    }

    /** One set of log arrays sized for a sample count. */
    private static final class LogArrays {
        final double[] ts;
        final double[] yaw;
        final double[] yawRate;
        final boolean[] good;
        final boolean[] gyroOk;
        final double[] fraction;
        final boolean[] moduleOk;
        final double[] distance;
        final double[] velocity;
        final double[] angle;
        final double[] targetAngle;
        final double[] targetVelocity;

        LogArrays(int n, int modules) {
            ts = new double[n];
            yaw = new double[n];
            yawRate = new double[n];
            good = new boolean[n];
            gyroOk = new boolean[n];
            fraction = new double[n];
            moduleOk = new boolean[n * modules];
            distance = new double[n * modules];
            velocity = new double[n * modules];
            angle = new double[n * modules];
            targetAngle = new double[n * modules];
            targetVelocity = new double[n * modules];
        }
    }

    /**
     * Log arrays by sample count, made the first time a count is seen and reused after: the logged
     * arrays must match this loop's count, and AdvantageKit copies what it is given.
     */
    private LogArrays[] byCount;

    // Main thread only (toLog runs inside Logger.processInputs).
    @Override
    public void toLog(LogTable t) {
        int n = count;
        if (byCount == null) {
            byCount = new LogArrays[samples.length + 1];
        }
        if (byCount[n] == null) {
            byCount[n] = new LogArrays(n, modules);
        }
        LogArrays a = byCount[n];
        double[] ts = a.ts;
        double[] yaw = a.yaw;
        double[] yawRate = a.yawRate;
        boolean[] good = a.good;
        boolean[] gyroOk = a.gyroOk;
        double[] fraction = a.fraction;
        boolean[] moduleOk = a.moduleOk;
        double[] distance = a.distance;
        double[] velocity = a.velocity;
        double[] angle = a.angle;
        double[] targetAngle = a.targetAngle;
        double[] targetVelocity = a.targetVelocity;
        for (int s = 0; s < n; s++) {
            SwerveSample x = samples[s];
            ts[s] = x.timestamp;
            yaw[s] = x.yaw;
            yawRate[s] = x.angularVelocityZWorld;
            good[s] = x.allGood;
            gyroOk[s] = x.gyroOk;
            fraction[s] = x.setpointFraction;
            System.arraycopy(x.moduleOk, 0, moduleOk, s * modules, modules);
            System.arraycopy(x.distance, 0, distance, s * modules, modules);
            System.arraycopy(x.velocity, 0, velocity, s * modules, modules);
            System.arraycopy(x.angle, 0, angle, s * modules, modules);
            System.arraycopy(x.targetAngle, 0, targetAngle, s * modules, modules);
            System.arraycopy(x.targetVelocity, 0, targetVelocity, s * modules, modules);
        }
        // Capture time of each 250 Hz sample this loop (s).
        t.put("Samples/Timestamp", ts);
        // Gyro yaw per sample (rad, continuous).
        t.put("Samples/Yaw", yaw);
        // Gyro yaw rate per sample (rad/s).
        t.put("Samples/AngularVelocityZWorld", yawRate);
        // Every device signal fresh and OK, per sample.
        t.put("Samples/AllGood", good);
        // Gyro signals fresh and OK, per sample.
        t.put("Samples/GyroOk", gyroOk);
        // Fraction of the request the setpoint generator allowed, per sample (1 = not limited).
        t.put("Samples/SetpointFraction", fraction);
        // Each module's drive and steer signals fresh and OK, [sample * 4 + module].
        t.put("Samples/ModuleOk", moduleOk);
        // Wheel travel per sample and module (m), [sample * 4 + module].
        t.put("Samples/Distance", distance);
        // Wheel speed per sample and module (m/s).
        t.put("Samples/Velocity", velocity);
        // Module direction per sample and module (rad).
        t.put("Samples/Angle", angle);
        // Commanded module direction (rad).
        t.put("Samples/TargetAngle", targetAngle);
        // Commanded wheel speed (m/s).
        t.put("Samples/TargetVelocity", targetVelocity);

        SwerveHealth h = health;
        // Device connected flags, FL FR BL BR.
        t.put("Health/DriveConnected", h.driveConnected);
        t.put("Health/SteerConnected", h.steerConnected);
        t.put("Health/EncoderConnected", h.encoderConnected);
        t.put("Health/PigeonConnected", h.pigeonConnected);
        // Robot tilt from the Pigeon (rad, WPILib Rotation3d): roll left-side-up, pitch nose-down.
        t.put("Health/Roll", h.roll);
        t.put("Health/Pitch", h.pitch);
        // Each device took its config (boot or a later retry).
        t.put("Health/DriveConfigured", h.driveConfigured);
        t.put("Health/SteerConfigured", h.steerConfigured);
        t.put("Health/EncoderConfigured", h.encoderConfigured);
        // Steer motors fusing their CANcoder (no fused-sensor faults).
        t.put("Health/FusedOk", h.fusedOk);
        // CANcoder magnets readable.
        t.put("Health/MagnetOk", h.magnetOk);
        // CANivore timesync in use.
        t.put("Health/Timesync", h.timesync);
        // Drivetrain device reboots detected since boot.
        t.put("Health/DeviceResets", h.deviceResets);
        // Signal rates set on every device (false after a reboot until redone).
        t.put("Health/SignalsSet", h.signalsSet);
        // Drive motors in brake (false: coast), as last applied.
        t.put("Health/DriveBrake", h.driveBrake);
        // Swerve thread cycles in a row with stale or failed signals.
        t.put("Thread/StaleCycles", staleCycles);
        // Motor currents (A).
        t.put("Health/DriveStatorCurrent", h.driveStatorCurrent);
        t.put("Health/SteerStatorCurrent", h.steerStatorCurrent);
        t.put("Health/DriveSupplyCurrent", h.driveSupplyCurrent);
        t.put("Health/SteerSupplyCurrent", h.steerSupplyCurrent);
        // Motor temperatures (C).
        t.put("Health/DriveDeviceTemp", h.driveDeviceTemp);
        t.put("Health/SteerDeviceTemp", h.steerDeviceTemp);
        // Supply voltage at the drive motors (V).
        t.put("Health/SupplyVoltage", h.supplyVoltage);
        // Supply voltage at each drive and steer motor (V), FL FR BL BR: wiring drop per motor.
        t.put("Health/DriveSupplyVoltage", h.driveSupplyVoltage);
        t.put("Health/SteerSupplyVoltage", h.steerSupplyVoltage);
        // Samples dropped because the main loop fell behind, since boot.
        t.put("Thread/DroppedSamples", droppedSamples);
        // Swerve thread cycles that threw, since boot.
        t.put("Thread/Faults", threadFaults);
        // Readings rejected as jumps no robot can make, e.g. a device reboot, since boot (count).
        t.put("Thread/RejectedJumps", rejectedJumps);
        // Last swerve thread exception message.
        t.put("Thread/LastFault", lastThreadFault);
    }

    @Override
    public void fromLog(LogTable t) {
        double[] ts = t.get("Samples/Timestamp", new double[0]);
        int n = Math.min(ts.length, samples.length);
        double[] yaw = t.get("Samples/Yaw", new double[n]);
        double[] yawRate = t.get("Samples/AngularVelocityZWorld", new double[n]);
        boolean[] good = t.get("Samples/AllGood", new boolean[n]);
        boolean[] gyroOk = t.get("Samples/GyroOk", good);
        double[] fraction = t.get("Samples/SetpointFraction", new double[n]);
        boolean[] moduleOk = t.get("Samples/ModuleOk", new boolean[n * modules]);
        double[] distance = t.get("Samples/Distance", new double[n * modules]);
        double[] velocity = t.get("Samples/Velocity", new double[n * modules]);
        double[] angle = t.get("Samples/Angle", new double[n * modules]);
        double[] targetAngle = t.get("Samples/TargetAngle", new double[n * modules]);
        double[] targetVelocity = t.get("Samples/TargetVelocity", new double[n * modules]);
        for (int s = 0; s < n; s++) {
            SwerveSample x = samples[s];
            x.timestamp = ts[s];
            x.yaw = yaw[s];
            x.angularVelocityZWorld = yawRate[s];
            x.allGood = good[s];
            x.gyroOk = gyroOk[s];
            x.setpointFraction = s < fraction.length ? fraction[s] : 1.0;
            System.arraycopy(moduleOk, s * modules, x.moduleOk, 0, modules);
            System.arraycopy(distance, s * modules, x.distance, 0, modules);
            System.arraycopy(velocity, s * modules, x.velocity, 0, modules);
            System.arraycopy(angle, s * modules, x.angle, 0, modules);
            System.arraycopy(targetAngle, s * modules, x.targetAngle, 0, modules);
            System.arraycopy(targetVelocity, s * modules, x.targetVelocity, 0, modules);
        }
        count = n;

        SwerveHealth h = health;
        copy(t.get("Health/DriveConnected", h.driveConnected), h.driveConnected);
        copy(t.get("Health/SteerConnected", h.steerConnected), h.steerConnected);
        copy(t.get("Health/EncoderConnected", h.encoderConnected), h.encoderConnected);
        h.pigeonConnected = t.get("Health/PigeonConnected", h.pigeonConnected);
        h.roll = t.get("Health/Roll", h.roll);
        h.pitch = t.get("Health/Pitch", h.pitch);
        copy(t.get("Health/DriveConfigured", h.driveConfigured), h.driveConfigured);
        copy(t.get("Health/SteerConfigured", h.steerConfigured), h.steerConfigured);
        copy(t.get("Health/EncoderConfigured", h.encoderConfigured), h.encoderConfigured);
        copy(t.get("Health/FusedOk", h.fusedOk), h.fusedOk);
        copy(t.get("Health/MagnetOk", h.magnetOk), h.magnetOk);
        h.timesync = t.get("Health/Timesync", h.timesync);
        h.driveBrake = t.get("Health/DriveBrake", h.driveBrake);
        h.deviceResets = t.get("Health/DeviceResets", h.deviceResets);
        h.signalsSet = t.get("Health/SignalsSet", h.signalsSet);
        staleCycles = t.get("Thread/StaleCycles", staleCycles);
        copy(t.get("Health/DriveStatorCurrent", h.driveStatorCurrent), h.driveStatorCurrent);
        copy(t.get("Health/SteerStatorCurrent", h.steerStatorCurrent), h.steerStatorCurrent);
        copy(t.get("Health/DriveSupplyCurrent", h.driveSupplyCurrent), h.driveSupplyCurrent);
        copy(t.get("Health/SteerSupplyCurrent", h.steerSupplyCurrent), h.steerSupplyCurrent);
        copy(t.get("Health/DriveDeviceTemp", h.driveDeviceTemp), h.driveDeviceTemp);
        copy(t.get("Health/SteerDeviceTemp", h.steerDeviceTemp), h.steerDeviceTemp);
        h.supplyVoltage = t.get("Health/SupplyVoltage", h.supplyVoltage);
        copy(t.get("Health/DriveSupplyVoltage", h.driveSupplyVoltage), h.driveSupplyVoltage);
        copy(t.get("Health/SteerSupplyVoltage", h.steerSupplyVoltage), h.steerSupplyVoltage);
        droppedSamples = t.get("Thread/DroppedSamples", droppedSamples);
        threadFaults = t.get("Thread/Faults", threadFaults);
        rejectedJumps = t.get("Thread/RejectedJumps", rejectedJumps);
        lastThreadFault = t.get("Thread/LastFault", lastThreadFault);
    }

    private static void copy(double[] from, double[] to) {
        System.arraycopy(from, 0, to, 0, Math.min(from.length, to.length));
    }

    private static void copy(boolean[] from, boolean[] to) {
        System.arraycopy(from, 0, to, 0, Math.min(from.length, to.length));
    }
}
