# spectrumSim

Desktop simulation pieces any Spectrum robot can use. Nothing here knows about `frc.robot`.

| File | What |
|---|---|
| `SpectrumDesktopSim` | The `DesktopSim` robot code loads: builds the sim clock and each simulated subsystem |
| `SimClock` | One clock for the whole sim: pauses WPILib timing and steps physics, the robot loop and helper threads in a fixed order, 1 ms at a time; real time or as fast as possible |
| `TalonFXModel` | A TalonFX's own control loop, from the same `TalonFXConfiguration` and control requests the robot sends; returns stator current |
| `MotionMagicExpoProfile` | The Motion Magic Expo profile `TalonFXModel` follows |
| `MotorModel` | DC motor constants (Kraken X60 FOC, Falcon 500 FOC): torque, back-EMF, resistance, current limits |

Adding a control request to `TalonFXModel`: one more `instanceof` branch in `setControl`, a unit
test in `TalonFXModelTest`, and a case in `./gradlew talonCrosscheck` if it can be compared with
Phoenix's simulator. A request the model does not know coasts that motor and warns once.
