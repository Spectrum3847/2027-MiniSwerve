# spectrumLib

Reusable Spectrum code. It never imports `frc.robot`. It will move to its own repo with a vendordep.

| Package | What |
|---|---|
| `framework/` | `SpectrumRobot` (AdvantageKit LoggedRobot base), `RobotLoop` (loop counter, `everySeconds`) |
| `robotsettings/` | Robot Settings: layered JSON values, live tuning while disabled, saving, hardware stamps |
| `telemetry/` | `Telemetry` (log facade), `Alert` (logged alerts), `DashboardReceiver` (NT dashboard filter) |
