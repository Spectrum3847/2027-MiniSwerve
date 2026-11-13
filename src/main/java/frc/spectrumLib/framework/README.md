# framework

| File | What |
|---|---|
| `OpModeLoop` | The running OpMode's per-loop work, run before the scheduler |
| `SpectrumRobot` | AdvantageKit LoggedOpModeRobot base (runs the Driver Station's OpMode) with quiet overrun warnings |
| `LoopStage` | Runs one stage of the robot loop (or an optional boot step) so a throw in it is counted and alerted instead of ending the robot program |
| `RobotLoop` | Loop counter and `everySeconds` cadence helper |
| `DesktopSim` | The desktop sim as robot code sees it; SpectrumSim implements it, found with `ServiceLoader` |
