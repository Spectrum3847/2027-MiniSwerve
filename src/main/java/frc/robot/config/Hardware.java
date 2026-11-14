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
 * <p>Index: {@link #CAN_BUS} · {@link #PIGEON_ID} · {@link #PDP_ID} · {@link Module} · {@link
 * MK4iL2}
 */
public final class Hardware {
    private Hardware() {}

    /**
     * The training robot has everything on one bus, SystemCore's bus 0. Phoenix needs its Linux
     * name: "" does not map to it on SystemCore. HARD STOP.
     */
    public static final String CAN_BUS = "can_s0";

    public static final int PIGEON_ID = 1;

    /**
     * The power distribution board: a CTRE PDP at CAN ID 0 on bus 0 (read by WPILib, not Phoenix).
     */
    public static final org.wpilib.hardware.bus.CANPort PDP_BUS =
            org.wpilib.hardware.bus.CANPort.CAN_S0;

    public static final int PDP_ID = 0;

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

        /** Falcon 500 with FOC stall torque at 12 V (N m). */
        public static final double DRIVE_STALL_TORQUE = 5.84;

        /** Falcon 500 with FOC stall current at 12 V (A). */
        public static final double DRIVE_STALL_CURRENT = 304.0;

        /** Falcon 500 with FOC free speed at 12 V (rpm). */
        public static final double DRIVE_FREE_RPM = 6080.0;

        /** Nominal 4 in wheel (m). The calibrated radius is a Robot Setting. */
        public static final double NOMINAL_WHEEL_RADIUS_METERS = Inches.of(2).in(Meters);
    }

    /** Nominal 14 in square module spacing, center to center (m). Calibrated value is a setting. */
    public static final double NOMINAL_TRACK_METERS = Inches.of(14).in(Meters);

    /**
     * Upper cap on the drive slip current (A). The slip current itself (CTRE's SlipCurrent: stator
     * limit and peak torque current) is computed from robotMass, tireFriction and the gearing, just
     * under the point the wheels slip (SwerveConfigs.driveSlipCurrent); this only caps it. HARD
     * STOP.
     */
    public static final double DRIVE_SLIP_CURRENT_AMPS = 80.0;

    /**
     * Drive motor supply (battery-side) current limit (A): keeps eight motors from browning out the
     * battery while still allowing full stator current at low speed. HARD STOP.
     */
    public static final double DRIVE_SUPPLY_LIMIT_AMPS = 40.0;

    /** Steer motor supply current limit (A). HARD STOP. */
    public static final double STEER_SUPPLY_LIMIT_AMPS = 20.0;

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
