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

    /** Swerve thread cycles in a row with stale or failed signals (it goes neutral at 5). */
    public long staleCycles;

    public SwerveInputs(int modules, int maxSamplesPerLoop) {
        this.modules = modules;
        samples = new SwerveSample[maxSamplesPerLoop];
        for (int i = 0; i < samples.length; i++) {
            samples[i] = new SwerveSample(modules);
        }
        health = new SwerveHealth(modules);
    }

    // Allocates its arrays each main loop (100 Hz, not the 250 Hz thread): AdvantageKit copies
    // what it is given anyway, and the arrays must match this loop's sample count.
    @Override
    public void toLog(LogTable t) {
        int n = count;
        double[] ts = new double[n];
        double[] yaw = new double[n];
        double[] yawRate = new double[n];
        boolean[] good = new boolean[n];
        boolean[] gyroOk = new boolean[n];
        boolean[] moduleOk = new boolean[n * modules];
        double[] distance = new double[n * modules];
        double[] velocity = new double[n * modules];
        double[] angle = new double[n * modules];
        double[] targetAngle = new double[n * modules];
        double[] targetVelocity = new double[n * modules];
        for (int s = 0; s < n; s++) {
            SwerveSample x = samples[s];
            ts[s] = x.timestamp;
            yaw[s] = x.yaw;
            yawRate[s] = x.angularVelocityZWorld;
            good[s] = x.allGood;
            gyroOk[s] = x.gyroOk;
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
        // Each module's devices took their config at boot.
        t.put("Health/Configured", h.configured);
        // Steer motors fusing their CANcoder (no fused-sensor faults).
        t.put("Health/FusedOk", h.fusedOk);
        // CANcoder magnets readable.
        t.put("Health/MagnetOk", h.magnetOk);
        // CANivore timesync in use.
        t.put("Health/Timesync", h.timesync);
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
        // Samples dropped because the main loop fell behind, since boot.
        t.put("Thread/DroppedSamples", droppedSamples);
        // Swerve thread cycles that threw, since boot.
        t.put("Thread/Faults", threadFaults);
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
        copy(t.get("Health/Configured", h.configured), h.configured);
        copy(t.get("Health/FusedOk", h.fusedOk), h.fusedOk);
        copy(t.get("Health/MagnetOk", h.magnetOk), h.magnetOk);
        h.timesync = t.get("Health/Timesync", h.timesync);
        staleCycles = t.get("Thread/StaleCycles", staleCycles);
        copy(t.get("Health/DriveStatorCurrent", h.driveStatorCurrent), h.driveStatorCurrent);
        copy(t.get("Health/SteerStatorCurrent", h.steerStatorCurrent), h.steerStatorCurrent);
        copy(t.get("Health/DriveSupplyCurrent", h.driveSupplyCurrent), h.driveSupplyCurrent);
        copy(t.get("Health/SteerSupplyCurrent", h.steerSupplyCurrent), h.steerSupplyCurrent);
        copy(t.get("Health/DriveDeviceTemp", h.driveDeviceTemp), h.driveDeviceTemp);
        copy(t.get("Health/SteerDeviceTemp", h.steerDeviceTemp), h.steerDeviceTemp);
        h.supplyVoltage = t.get("Health/SupplyVoltage", h.supplyVoltage);
        droppedSamples = t.get("Thread/DroppedSamples", droppedSamples);
        threadFaults = t.get("Thread/Faults", threadFaults);
        lastThreadFault = t.get("Thread/LastFault", lastThreadFault);
    }

    private static void copy(double[] from, double[] to) {
        System.arraycopy(from, 0, to, 0, Math.min(from.length, to.length));
    }

    private static void copy(boolean[] from, boolean[] to) {
        System.arraycopy(from, 0, to, 0, Math.min(from.length, to.length));
    }
}
