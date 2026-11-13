# subsystems/swerve

| File | What |
|---|---|
| `Swerve` | Wanted states (IDLE, TELEOP_DRIVE, X_LOCK), builds IO and thread, odometry, logging |
| `FieldHeading` | Which way is "away from the driver": alliance kept through dropouts, start heading from the alliance, heading restored after a code restart, Back-button re-zero |
| `SwerveThreadWatch` | Main loop's watch on the swerve thread: restarts it if it ends, alerts on stalls |
| `SwerveConfigs` | Builds the Phoenix drivetrain config from Hardware and settings; traction limit |
| `SwerveUtilities` | Brake/coast on disable and the wheel-radius measurement |
| `SwerveSettings` | Robot Settings: magnet offsets, wheel radius, track, speeds, gains |
