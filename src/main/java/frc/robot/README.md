# frc.robot

| File | What |
|---|---|
| `Main` | Entry point only |
| `Robot` | Boot order and the main loop order; owns subsystems as fields |
| `RunMode` | REAL / SIM / REPLAY, loop periods, RT priority, tuning mode |
| `LoggingSetup` | AdvantageKit metadata and receivers |
| `SimScript` | Scripted sim driving (`MINISWERVE_SIM_SCRIPT=drive|faults|radius|auto|walls|buttons|driver`): sticks, buttons (tap, hold), Driver Station inputs, unplugged controller, Driver Station lost |
| `SimDriver` | The person on the scripted controller: sticks that take time to move, imperfect pushes, drift and jitter (`MINISWERVE_SIM_DRIVER=perfect` turns it off) |
| `config/Hardware` | CAN IDs, bus, gear ratios, inverts, current limits: hard stops |
| `subsystems/swerve/` | The drivetrain |
| `subsystems/vision/` | The cameras |
| `subsystems/pose/` | Where the robot is: TagEKF, with WPILib's estimator as a shadow |
| `pilot/` | Driver controller |
| `opmodes/` | What the Driver Station can pick: autos, teleop and utility modes |
