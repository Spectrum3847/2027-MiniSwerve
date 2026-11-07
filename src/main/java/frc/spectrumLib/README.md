# spectrumLib

Reusable Spectrum code. It never imports `frc.robot`. It will move to its own repo with a vendordep.

| Package | What |
|---|---|
| `framework/` | `SpectrumRobot` (AdvantageKit LoggedOpModeRobot base), `RobotLoop` (loop counter, `everySeconds`), `DesktopSim` (how robot code reaches SpectrumSim) |
| `robotsettings/` | Robot Settings: layered JSON values, live tuning while disabled, saving, hardware stamps |
| `hardware/` | `PhoenixBackground`: the one thread blocking Phoenix calls run on after boot |
| `swerve/` | Our swerve: 250 Hz thread, Phoenix Pro IO, odometry |
| `localization/` | TagEKF (bearing/range per tag, history replay, gating), mount estimator, odometry guard |
| `vision/` | Camera frames from the coprocessor: the compact wire format, NetworkTables IO, logged inputs |
| `statemachine/` | `WantedState`: wanted vs current state, logged |
| `telemetry/` | `Telemetry` (log facade), `Alert` (logged alerts), `DashboardReceiver` (NT dashboard filter) |
