# frc.robot.opmodes

The Driver Station lists these and runs the one picked. `Robot` registers them and passes each its
dependencies; nothing here is static.

| File | Mode | What |
|---|---|---|
| `DriveTeleop` | Teleop | Match driving at full speed (pilot bindings) |
| `DemoTeleop` | Teleop | Pilot driving with speed and turn capped (settings `demo/*`) |
| `DoNothingAuto` | Auto | Sits still |
| `DriveForwardAuto` | Auto | Drives straight ahead 1.5 m, then stops; placeholder for Choreo autos |
| `WheelRadiusUtility` | Utility | Spins in place to measure the wheel radius |
