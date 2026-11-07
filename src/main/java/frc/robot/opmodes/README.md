# frc.robot.opmodes

The Driver Station lists these and runs the one picked. `Robot` registers them and passes each its
dependencies; nothing here is static. An OpMode's per-loop work goes through `OpModeLoop` (set in
`start()`, cleared in `end()`), so it runs before the scheduler and swerve on the same loop; WPILib's
own `periodic()` runs after them.

| File | Mode | What |
|---|---|---|
| `DriveTeleop` | Teleop | Match driving at full speed (pilot bindings) |
| `DemoTeleop` | Teleop | Pilot driving with speed and turn capped (settings `demo/*`) |
| `DoNothingAuto` | Auto | Sits still |
| `DriveForwardAuto` | Auto | Drives ahead at 1 m/s for 1.5 s, then holds still; placeholder for Choreo autos |
| `WheelRadiusUtility` | Utility | Spins in place to measure the wheel radius |
