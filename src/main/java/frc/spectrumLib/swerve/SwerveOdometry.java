package frc.spectrumLib.swerve;

/**
 * Wheel-and-gyro odometry, sample by sample, without allocating. The same job as WPILib's {@code
 * SwerveDriveOdometry}, with three differences: it takes our 250 Hz samples directly, it uses the
 * gyro for heading while the gyro is good and the wheels' own estimate otherwise, and it reports
 * how much the modules disagree (a slip signal).
 *
 * <p>Pose is field frame: x, y (m), heading (rad, CCW). Main thread only; runs identically in
 * replay because it reads only logged samples.
 *
 * <p>Index: {@link #resetPose} | {@link #update} | getters
 */
public final class SwerveOdometry {
    private final double[] mx;
    private final double[] my;
    private final double[] dx;
    private final double[] dy;
    private final double[] lastDistance;
    private final double[] lastAngle;
    private final double[] twist = new double[3];
    private final double[] residual;
    private int modulesUsed;
    private final double[] robotVelocity = new double[3];
    private final double[] vxm;
    private final double[] vym;

    private double x;
    private double y;
    private double heading;
    private double yawOffset; // heading = yaw + yawOffset
    private boolean primed;
    private double lastYaw;
    private double slip;
    private boolean usedGyro;

    public SwerveOdometry(double[] mx, double[] my) {
        this.mx = mx.clone();
        this.my = my.clone();
        int n = mx.length;
        dx = new double[n];
        dy = new double[n];
        lastDistance = new double[n];
        lastAngle = new double[n];
        residual = new double[n];
        vxm = new double[n];
        vym = new double[n];
    }

    /** Puts the robot at a pose; the next sample's yaw becomes this heading. */
    public void resetPose(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
        if (primed) {
            yawOffset = heading - lastYaw;
        } else {
            yawOffset = Double.NaN; // set from the first sample
        }
    }

    /** Advances the pose by one sample using every module. */
    public void update(SwerveSample s, boolean gyroGood) {
        update(s, gyroGood, null);
    }

    /**
     * Advances the pose by one sample.
     *
     * @param gyroGood use the sample's yaw for heading (Pigeon connected and the sample all-good)
     * @param weight per-module weight, 1 use and 0 leave out (dead drive, untrusted angle); null
     *     for all. With fewer than two usable modules, translation holds and only heading moves.
     */
    public void update(SwerveSample s, boolean gyroGood, double[] weight) {
        int n = mx.length;
        if (!primed) {
            System.arraycopy(s.distance, 0, lastDistance, 0, n);
            System.arraycopy(s.angle, 0, lastAngle, 0, n);
            lastYaw = s.yaw;
            if (Double.isNaN(yawOffset)) {
                yawOffset = heading - s.yaw;
            }
            primed = true;
            return;
        }
        for (int i = 0; i < n; i++) {
            double d = s.distance[i] - lastDistance[i];
            // The wheel turned from one angle to the other over the sample: use the midpoint.
            double a = lastAngle[i] + SwerveMath.angleModulus(s.angle[i] - lastAngle[i]) / 2.0;
            dx[i] = d * Math.cos(a);
            dy[i] = d * Math.sin(a);
            lastDistance[i] = s.distance[i];
            lastAngle[i] = s.angle[i];
        }
        slip = SwerveMath.toChassisVelocities(dx, dy, mx, my, weight, twist, residual);
        modulesUsed = 0;
        for (int i = 0; i < n; i++) {
            modulesUsed += weight == null || weight[i] > 0 ? 1 : 0;
        }
        if (Double.isNaN(slip) && !gyroGood) {
            // Nothing trustworthy moved us this sample: hold the pose.
            lastYaw = s.yaw;
            usedGyro = false;
            java.util.Arrays.fill(robotVelocity, 0.0);
            return;
        }
        double dTheta;
        usedGyro = gyroGood;
        if (gyroGood) {
            dTheta = SwerveMath.angleModulus(s.yaw - lastYaw);
        } else {
            dTheta = twist[2];
            // Keep the gyro offset following the wheels, so heading does not jump when the gyro
            // comes back.
            yawOffset += twist[2] - SwerveMath.angleModulus(s.yaw - lastYaw);
        }
        // Measured chassis velocity straight from the wheels' measured speeds (no differencing of
        // timestamps, which jitter), with the gyro's yaw rate when it is good.
        for (int i = 0; i < n; i++) {
            vxm[i] = s.velocity[i] * Math.cos(s.angle[i]);
            vym[i] = s.velocity[i] * Math.sin(s.angle[i]);
        }
        SwerveMath.toChassisVelocities(vxm, vym, mx, my, weight, robotVelocity, null);
        if (gyroGood) {
            robotVelocity[2] = s.angularVelocityZWorld;
        }
        lastYaw = s.yaw;
        // Integrate the robot-frame twist along an arc (Pose2d.exp), then rotate into the field.
        double sin = Math.sin(dTheta);
        double cos = Math.cos(dTheta);
        double sinOver, oneMinusCosOver;
        if (Math.abs(dTheta) < 1e-9) {
            sinOver = 1.0 - dTheta * dTheta / 6.0;
            oneMinusCosOver = 0.5 * dTheta;
        } else {
            sinOver = sin / dTheta;
            oneMinusCosOver = (1 - cos) / dTheta;
        }
        double rx = twist[0] * sinOver - twist[1] * oneMinusCosOver;
        double ry = twist[0] * oneMinusCosOver + twist[1] * sinOver;
        double c = Math.cos(heading);
        double sh = Math.sin(heading);
        x += rx * c - ry * sh;
        y += rx * sh + ry * c;
        heading = gyroGood ? s.yaw + yawOffset : heading + dTheta;
    }

    public double x() {
        return x;
    }

    public double y() {
        return y;
    }

    /** Field heading (rad, continuous). */
    public double heading() {
        return heading;
    }

    /** Field heading minus gyro yaw (rad): what the swerve thread adds to its live yaw. */
    public double yawOffset() {
        return Double.isNaN(yawOffset) ? 0.0 : yawOffset;
    }

    /** RMS module disagreement over the last sample (m): slip, or a bad module. */
    public double slip() {
        return slip;
    }

    /**
     * Each module's disagreement with the fitted motion over the last sample (m): a module that
     * keeps disagreeing is slipping or bad. Includes modules left out of the fit.
     */
    public double[] moduleSlip() {
        return residual;
    }

    /** Measured robot-frame velocity over the last sample: vx, vy (m/s), omega (rad/s). */
    public double[] robotVelocity() {
        return robotVelocity;
    }

    /** Modules used in the last sample's fit. */
    public int modulesUsed() {
        return modulesUsed;
    }

    /** Whether the last sample's heading came from the gyro. */
    public boolean usedGyro() {
        return usedGyro;
    }
}
