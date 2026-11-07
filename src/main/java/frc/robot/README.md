# frc.robot

| File | What |
|---|---|
| `Main` | Entry point only |
| `Robot` | Boot order and the main loop order; owns subsystems as fields |
| `RunMode` | REAL / SIM / REPLAY, loop periods, RT priority, tuning mode |
| `LoggingSetup` | AdvantageKit metadata and receivers |
| `config/Hardware` | CAN IDs, bus, gear ratios: hard stops |
