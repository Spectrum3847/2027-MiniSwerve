package frc.spectrumLib.robotsettings;

/** Where a setting's current value came from. Later layers win. */
public enum SettingSource {
    /** The default in the declaration. */
    DEFAULT,
    /** {@code settings/<group>.json} in the deploy folder. */
    DEPLOY,
    /** {@code settings/robots/<robot>/<group>.json} in the deploy folder. */
    ROBOT,
    /** {@code <group>.json} in the robot-local folder, which a deploy does not wipe. */
    LOCAL,
    /** Changed over the transport (NetworkTables) while disabled, with live tuning on. */
    LIVE
}
