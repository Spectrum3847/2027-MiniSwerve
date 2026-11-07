package frc.spectrumLib.swerve;

/**
 * The physical drivetrain a desktop sim models, beyond what {@link PhoenixSwerveConfig} says.
 *
 * @param driveMotor drive motor, as SpectrumSim names it: "KrakenX60FOC" or "Falcon500FOC"
 * @param steerMotor steer motor, same names
 * @param massKg robot weight ready to play (kg)
 * @param tireFriction tire-to-carpet friction coefficient
 * @param steerInertia module inertia about its steer axis (kg m^2)
 * @param steerFriction steer friction (N m per rad/s)
 * @param mx module x positions (m)
 * @param my module y positions (m)
 * @param trueWheelRadius the sim's real wheel radius (m); NaN for the configured one
 */
public record SwerveSimSpec(
        String driveMotor,
        String steerMotor,
        double massKg,
        double tireFriction,
        double steerInertia,
        double steerFriction,
        double[] mx,
        double[] my,
        double trueWheelRadius) {}
