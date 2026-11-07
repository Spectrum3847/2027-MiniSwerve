package frc.robot.config;

import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;

/**
 * Every CAN ID, bus and fixed mechanical fact of the MiniSwerve. HARD STOPS: see AGENTS.md before
 * changing anything here.
 *
 * <p>ID scheme: module N has drive N, steer 10+N and CANcoder N. Phoenix IDs are unique per device
 * type, so drive 1, CANcoder 1 and the Pigeon 1 do not collide.
 *
 * <p>Index: {@link #CAN_BUS} · {@link #PIGEON_ID} · {@link Module} · {@link MK4iL2}
 */
public final class Hardware {
    private Hardware() {}

    /** The training robot has everything on one bus: SystemCore's built-in CAN. */
    public static final String CAN_BUS = "";

    public static final int PIGEON_ID = 1;

    /** Modules in the order every array in the swerve code uses: FL, FR, BL, BR. */
    public enum Module {
        FRONT_LEFT(1),
        FRONT_RIGHT(2),
        BACK_LEFT(3),
        BACK_RIGHT(4);

        public final int driveId;
        public final int steerId;
        public final int encoderId;

        Module(int n) {
            this.driveId = n;
            this.steerId = 10 + n;
            this.encoderId = n;
        }
    }

    /** SDS MK4i, L2 ratio, Falcon 500 on drive and steer. */
    public static final class MK4iL2 {
        private MK4iL2() {}

        /** Drive motor rotations per wheel rotation: (50/14)(17/27)(45/15). */
        public static final double DRIVE_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);

        /** Steer motor rotations per module rotation: 150/7. */
        public static final double STEER_RATIO = 150.0 / 7.0;

        /**
         * Drive motor rotations per module rotation from the steer gearing: 50/14, the same sign in
         * every drive motor's own frame (as Phoenix's coupling ratio is applied).
         */
        public static final double COUPLE_RATIO = 50.0 / 14.0;

        /** Falcon 500 with FOC torque constant (N m per A): 5.84 N m / 304 A stall. */
        public static final double DRIVE_KT = 5.84 / 304.0;

        /** Nominal 4 in wheel (m). The calibrated radius is a Robot Setting. */
        public static final double NOMINAL_WHEEL_RADIUS_METERS = Inches.of(2).in(Meters);
    }

    /** Nominal 14 in square module spacing, center to center (m). Calibrated value is a setting. */
    public static final double NOMINAL_TRACK_METERS = Inches.of(14).in(Meters);

    /**
     * Drive slip current (A): CTRE's SlipCurrent, used as both the stator current limit and the
     * peak torque current, so the wheels stop short of slipping. HARD STOP.
     */
    public static final double DRIVE_SLIP_CURRENT_AMPS = 80.0;

    /** Steer motor stator current limit (A). HARD STOP. */
    public static final double STEER_STATOR_LIMIT_AMPS = 40.0;

    /**
     * Drive motors: left side not inverted, right side inverted. Same on every MK4i (Phoenix
     * Tuner's generated MK4i constants: kInvertLeftSide false, kInvertRightSide true). HARD STOP.
     */
    private static final boolean[] DRIVE_INVERTED = {false, true, false, true};

    /** A copy of {@link #DRIVE_INVERTED}, FL FR BL BR. */
    public static boolean[] driveInverted() {
        return DRIVE_INVERTED.clone();
    }

    /**
     * MK4i steer motors run opposite the module: Clockwise_Positive, CANcoder
     * CounterClockwise_Positive (Tuner's kSteerMotorReversed true; team 1710's COTS table). HARD
     * STOP.
     */
    public static final boolean STEER_INVERTED = true;
}
