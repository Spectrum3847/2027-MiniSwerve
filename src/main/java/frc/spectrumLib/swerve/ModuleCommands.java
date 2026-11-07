package frc.spectrumLib.swerve;

/**
 * How the swerve thread commands each module's two motors, decided by {@link ModuleTrust} from
 * health. Immutable: the main thread builds a new one when anything changes.
 *
 * <p>The rule is "keep commanding, but don't flood the bus": a motor that is disconnected (no
 * status frames) may still be alive and listening, so it keeps getting its real command if its
 * config is known, but only at a slow rate. A motor only coasts when its real command could do harm
 * (its config never applied, so its invert is unknown; or a drive whose wheel angle is unknown).
 *
 * @param drive per module: send the drive its velocity command (false: coast)
 * @param steer per module: send the steer its position command (false: coast)
 * @param driveSlow per module: the drive is disconnected, send to it at the slow rate only
 * @param steerSlow per module: the steer is disconnected, send to it at the slow rate only
 */
public record ModuleCommands(
        boolean[] drive, boolean[] steer, boolean[] driveSlow, boolean[] steerSlow) {

    /** Every motor commanded normally, at full rate. */
    public static ModuleCommands allNormal(int modules) {
        boolean[] yes = new boolean[modules];
        java.util.Arrays.fill(yes, true);
        return new ModuleCommands(yes, yes.clone(), new boolean[modules], new boolean[modules]);
    }
}
