# frc.robot

| File | What |
|---|---|
| `Main` | Entry point only |
| `Robot` | Boot order and the main loop order; owns subsystems as fields |
| `RunMode` | REAL / SIM / REPLAY, loop periods, RT priority, tuning mode |
| `LoggingSetup` | AdvantageKit metadata and receivers |
| `SimScript` | Scripted sim driving (`MINISWERVE_SIM_SCRIPT=drive|faults|radius|auto`) |
| `config/Hardware` | CAN IDs, bus, gear ratios, inverts, current limits: hard stops |
| `subsystems/swerve/` | The drivetrain |
| `subsystems/vision/` | The cameras |
| `pilot/` | Driver controller |
| `opmodes/` | What the Driver Station can pick: autos, teleop and utility modes |
