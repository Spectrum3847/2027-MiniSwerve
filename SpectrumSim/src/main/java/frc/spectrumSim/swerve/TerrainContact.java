package frc.spectrumSim.swerve;

import frc.spectrumSim.field.Terrain;

/**
 * How the floor under the wheels tilts and lifts the robot: the floor height at each wheel, a plane
 * fitted through them (the chassis is rigid and has no suspension), and the part of gravity along
 * that plane, which rolls the robot down a ramp and slows it climbing one.
 *
 * <p>Quasi-static: the robot sits on the fitted plane at once, with no bounce, and a four-wheel
 * robot on uneven floor rests on the best-fit plane rather than on three wheels. Good enough to see
 * the robot ride a bump in AdvantageScope and to feel a bump's effect on speed and odometry; not
 * for landing loads. The plane fit is exact for wheels laid out symmetrically about the center
 * (every swerve we build). Allocation-free.
 */
final class TerrainContact {
    private static final double GRAVITY = 9.81;

    private final double[] mx;
    private final double[] my;
    private final double sumX2;
    private final double sumY2;
    private double height;
    private double roll;
    private double pitch;
    private double ax;
    private double ay;

    TerrainContact(double[] mx, double[] my) {
        this.mx = mx.clone();
        this.my = my.clone();
        double sx = 0, sy = 0;
        for (int i = 0; i < mx.length; i++) {
            sx += mx[i] * mx[i];
            sy += my[i] * my[i];
        }
        sumX2 = sx > 0 ? sx : 1;
        sumY2 = sy > 0 ? sy : 1;
    }

    /** Fits the floor under the wheels for the robot at a field pose (m, rad). */
    void update(Terrain terrain, double x, double y, double heading) {
        double c = Math.cos(heading), s = Math.sin(heading);
        double mean = 0, alongX = 0, alongY = 0;
        int n = mx.length;
        for (int i = 0; i < n; i++) {
            double z = terrain.height(x + mx[i] * c - my[i] * s, y + mx[i] * s + my[i] * c);
            mean += z;
            alongX += z * mx[i];
            alongY += z * my[i];
        }
        // Plane z = height + gx * forward + gy * left, in the robot frame.
        double gx = alongX / sumX2;
        double gy = alongY / sumY2;
        height = mean / n;
        // WPILib Rotation3d: floor rising ahead (nose up) is negative pitch; left side up is
        // positive roll.
        pitch = -Math.atan(gx);
        roll = Math.atan(gy);
        // Gravity along the plane, as a robot-frame acceleration: downhill.
        double norm = Math.sqrt(1 + gx * gx + gy * gy);
        ax = -GRAVITY * gx / norm;
        ay = -GRAVITY * gy / norm;
    }

    /** Height of the floor plane under the robot's center (m). */
    double height() {
        return height;
    }

    /** Roll (rad, WPILib: positive left side up). */
    double roll() {
        return roll;
    }

    /** Pitch (rad, WPILib: positive nose down). */
    double pitch() {
        return pitch;
    }

    /** Gravity's pull along the floor, robot frame forward (m/s^2). */
    double ax() {
        return ax;
    }

    /** Gravity's pull along the floor, robot frame left (m/s^2). */
    double ay() {
        return ay;
    }
}
