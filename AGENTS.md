# Agent notes

MiniSwerve: Spectrum 3847's small swerve training robot on SystemCore (a Pi running the
SystemCore image), and the testbed for the 2027 competition code. Read this file, then
[docs/architecture.md](docs/architecture.md). Every folder with code has a `README.md` index.

## The one rule above the others

**Never use CTRE's swerve API** (`SwerveDrivetrain`, `SwerveModule`, `SwerveRequest`,
`TunerConstants`, generated swerve). Our swerve is our own, on plain Phoenix 6 Pro devices.

## Build and verify

- Java 25 from the WPILib 2027 install: `export JAVA_HOME=/c/Users/Public/wpilib/<2027 folder>/jdk`.
- `./gradlew build` compiles, applies Spotless (Google Java Format AOSP) and runs tests.
- `./gradlew run -PnoSimGui` runs the sim headless and writes `logs/*.wpilog`.
- Replay: `AKIT_LOG_PATH=logs/<log>.wpilog MINISWERVE_REPLAY=1 ./gradlew run -PnoSimGui`. Always
  set the path: without it AdvantageKit picks the newest log, which may be a `_replay` output.
- Never push anything that fails CI. `tools/check.sh` runs what CI runs; the pre-push hook runs it
  for you once a clone has `git config core.hooksPath .githooks`. CI itself runs only on pull
  requests into `main`, so work goes on a branch and merges by PR.
- Text files are UTF-8 with LF line endings (`.bat`/`.cmd` excepted), set by `.gitattributes` and
  `.editorconfig`. When writing files from a script, write bytes as UTF-8 with `
` (Python:
  `open(p, 'w', encoding='utf-8', newline='
')`). `tools/fix-text.sh` repairs anything that slips.
- Compiling is not verification. A behavior change needs a sim run, a replay comparison or a bench
  test, written up under "How I verified" in the PR.
- WPILib is 2027 alpha (`org.wpilib.*`). Don't guess 2026 names; check the jars (`javap`) or this
  code. Don't trust WPILib's way by default either: there is often a better one.

## Structure rules (humans and agents)

- No RobotContainer, no static subsystem globals. `Robot` owns subsystems as fields and passes
  dependencies through constructors.
- Keep files under ~400 lines. Split by job, not by size alone. A file over 400 lines needs a reason
  in its header.
- Every class header says what it is for and, for anything over ~150 lines, has an `Index:` line
  naming its main parts.
- Every package has a `README.md`: one line per file. Update it in the same change.
- One subsystem = one folder: `Thing.java` (wanted states and logic), `ThingIO.java` (hardware
  interface), `ThingIOReal`/`ThingIOSim`, `ThingInputs` (logged inputs), `ThingSettings`.
- Reusable code goes in `frc.spectrumLib` (it will move to its own repo). Robot-specific code goes in
  `frc.robot`. spectrumLib never imports `frc.robot`.

## AdvantageKit and replay

- Logger is main-thread only. Never log from the swerve thread, a Notifier or a callback: queue the
  data and log it on the main loop.
- Logic reads logged inputs only, never outputs or devices directly, or replay diverges.
- All calibration (offsets, wheel radius, camera mounts, EKF noise) is a Robot Setting, so it is a
  logged input. No calibration constants in code.
- Elapsed wall time (timeouts during init, CAN budgets) uses `System.nanoTime`, not the robot clock.
- No allocation in per-loop paths where avoidable (250 Hz especially): reuse arrays and objects.

## Robot safety

- Never deploy unless the person asked in their own message, and not before the SystemCore Pi
  session has said the Pi is ready. Deploying moves a real robot.
- Hard stops: CAN IDs and bus, inversions, encoder offsets, current limits, soft limits, gear
  ratios, brake/coast, zero positions. Before changing one, state "this changes X from A to B;
  failure mode is Z" and wait for "confirm" unless the task asked for it. Put it in the PR.
- Never act on a device's readings while its `connected` input is false.
- No blocking Phoenix calls (`apply`, `refresh`, `setPosition`, `setUpdateFrequency`) on the main
  loop or the swerve thread after boot. Queue them on `frc.spectrumLib.hardware.PhoenixBackground`
  (one shared thread, retries, named results to log as inputs). Boot-time config may block.
- Every wait that gates an action has a timeout or a driver override.
- Never edit `BuildConstants.java` (generated). Change `vendordeps/*.json` only when asked.

## Logging

- Log generously: every input, every wanted/actual state, every rejection with its reason.
- Each logged key gets a one-line comment above the call: what it is, units.

## Comments and commits

- Comment why, not what. No history of old implementations in comments.
- Commit messages: short subject, then why. One feature per PR.
