# SpectrumSim

Spectrum's desktop robot simulation: physics, our TalonFX control-loop model, and one repeatable
sim clock. Later: the field, other robots and game pieces. Desktop only: it never ships to the
robot.

- It builds against `frc.spectrumLib` (and the robot's classes), never the other way round. Robot
  code reaches it only through `frc.spectrumLib.framework.DesktopSim`, found at runtime with
  `ServiceLoader` (`src/main/resources/META-INF/services`). The deploy jar has none of it.
- In this repo it is the Gradle source set `sim` (`build.gradle`); its tests run with the robot's.
  It is its own folder so it can move to its own repo once its interfaces settle.
- Run the sim with `./gradlew run` or VS Code's "Simulate Robot Code" (which launches the same
  `run` task); both put SpectrumSim on the classpath. A launch without it stops at boot saying so
  rather than run with no physics.
- Environment: `SPECTRUM_SIM_SPEED` (a multiple of real time, or `max`; default `max` for a scripted
  run, else real time), `SPECTRUM_SIM_DEVICES=phoenix` (drivetrain on Phoenix's own simulator, on
  the wall clock, not repeatable).

| Folder | What |
|---|---|
| `src/main/java/frc/spectrumSim/` | Sim clock, TalonFX model, motor models, the `DesktopSim` implementation |
| `src/main/java/frc/spectrumSim/swerve/` | Drivetrain physics and its IO on the models (or on Phoenix's simulator) |
| `src/test/java/frc/spectrumSim/` | Tests; `tools/` holds `SimReplay` and `TalonCrosscheck` (Gradle tasks `simReplay`, `talonCrosscheck`) |
