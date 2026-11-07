package frc.spectrumLib.swerve;

/**
 * One 250 Hz reading of the whole drivetrain, plus what the thread commanded from it. Field names
 * follow WPILib's {@code SwerveModulePosition} (distance, angle) and Phoenix's Pigeon 2 signals
 * (yaw, angularVelocityZWorld).
 *
 * <p>Mutable and reused: the swerve thread fills one, {@link SampleQueue} copies it, the main loop
 * copies it again into logged inputs. Nothing here allocates after construction.
 */
public final class SwerveSample {
    /**
     * When the readings were captured, seconds, on the robot's clock (RobotController's monotonic
     * time, the camera frames' clock): the mean of the signals' own Phoenix timestamps
     * (CANivore-synchronized with timesync, otherwise time of receipt), shifted by the Phoenix-to-
     * robot clock offset measured as they were read. Not yet checked on SystemCore.
     */
    public double timestamp;

    /** Wheel travel per module (m), with steer coupling removed. */
    public final double[] distance;

    /** Wheel speed per module (m/s). */
    public final double[] velocity;

    /** Module direction (rad, CCW positive, 0 = robot forward). */
    public final double[] angle;

    /** Gyro yaw (rad, CCW positive, continuous: not wrapped). */
    public double yaw;

    /** Gyro yaw rate (rad/s). */
    public double angularVelocityZWorld;

    /** Every device's signals in this sample were fresh and OK. */
    public boolean allGood;

    /** Each module's drive and steer signals were fresh and OK in this sample. */
    public final boolean[] moduleOk;

    /** The gyro's yaw and yaw rate were fresh and OK in this sample. */
    public boolean gyroOk;

    /**
     * Fraction of the request the setpoint generator allowed this cycle (1 = not limited). Shows
     * how often maxDriveAcceleration is holding the robot back.
     */
    public double setpointFraction = 1.0;

    /** Commanded module direction this cycle (rad). */
    public final double[] targetAngle;

    /** Commanded wheel speed this cycle (m/s), after optimize and cosine scale. */
    public final double[] targetVelocity;

    public SwerveSample(int modules) {
        distance = new double[modules];
        velocity = new double[modules];
        angle = new double[modules];
        targetAngle = new double[modules];
        targetVelocity = new double[modules];
        moduleOk = new boolean[modules];
    }

    public int modules() {
        return distance.length;
    }

    /** Copies every field of {@code from} into this sample. */
    public void copyFrom(SwerveSample from) {
        timestamp = from.timestamp;
        yaw = from.yaw;
        angularVelocityZWorld = from.angularVelocityZWorld;
        allGood = from.allGood;
        gyroOk = from.gyroOk;
        setpointFraction = from.setpointFraction;
        System.arraycopy(from.moduleOk, 0, moduleOk, 0, moduleOk.length);
        System.arraycopy(from.distance, 0, distance, 0, distance.length);
        System.arraycopy(from.velocity, 0, velocity, 0, velocity.length);
        System.arraycopy(from.angle, 0, angle, 0, angle.length);
        System.arraycopy(from.targetAngle, 0, targetAngle, 0, targetAngle.length);
        System.arraycopy(from.targetVelocity, 0, targetVelocity, 0, targetVelocity.length);
    }
}
