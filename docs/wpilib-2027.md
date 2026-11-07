# WPILib 2027: what we use, what we avoid

Checked against the alpha-8 jars and the 2027 docs changelog on 2026-11-06. Re-check at each alpha.

## Decisions for this repo

| Topic | Decision | Why |
|---|---|---|
| Robot base | `SpectrumRobot extends LoggedOpModeRobot` (our AdvantageKit fork). | `OpModeRobot` extends `RobotBase`, not TimedRobot, so AdvantageKit's `LoggedRobot` cannot host OpModes; the fork's `LoggedOpModeRobot` can. |
| Auto / utility selection | OpModes (`@Autonomous`, `@Teleop`, `@Utility`, `addAnnotatedOpModeClasses`). No SendableChooser. | Chooser shim is removed before kickoff. OpMode API still changes each alpha: keep OpModes thin wrappers that request wanted states. |
| Commands | v2 (`org.wpilib.command2`) for bindings. Never `Command.schedule()`. | v3 is still being designed. Prototype it on a branch only. |
| Controller | `org.wpilib.driverstation.Gamepad` / `XboxController`, face-button names (A = faceDown, B = faceRight, X = faceLeft, Y = faceUp). Triggers via `getLeftTrigger()`. | New DS mapping. `NiDs*` classes are for the old NI DS only. |
| Swerve thread | Our own thread or `Notifier.startPeriodic(Hertz.of(250))`, priority set from inside it with `Threads.setCurrentThreadPriority`. | |
| Time | `long` times are **nanoseconds** (`RobotController.getTime`, `nt::Now`). `double` times are seconds. DataLog / NT stay microseconds. Never hard-code a µs conversion. | Changed silently in alpha-7. |
| Live tuning | Our `NetworkTablesTransport` today; WPILib Tunables (`org.wpilib.tunable`) can replace it behind `SettingTransport`. | Tunables are new in alpha-7; the transport is pluggable on purpose. |
| Dashboards | Logged outputs via AdvantageKit; no `SmartDashboard` / `Sendable` in new code. | Being replaced by `org.wpilib.telemetry`. `Alert` still mirrors to `/SmartDashboard/Alerts` for Elastic. |
| IMU | Pigeon 2 primary. `OnboardIMU` is a SystemCore-only API; not present on our Pi. | |
| CAN | One bus. WPILib's `CANPort.CAN_S0..S4`; Phoenix uses its own bus names. | |
| Init | Constructors only; `robotInit` is gone. Utility, not test. | |

## Watch list before beta

- OpMode API (changed every alpha since 5).
- AdvantageKit support for OpModes (our fork).
- `Selectable` shim removal.
- Commands v3 docs and API.
- DS simulation support.
