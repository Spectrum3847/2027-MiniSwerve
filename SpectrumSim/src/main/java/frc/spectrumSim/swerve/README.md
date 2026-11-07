# spectrumSim.swerve

The drivetrain in the desktop sim.

| File | What |
|---|---|
| `SwervePhysicsSim` | Rigid body, tires (grip curve, friction circle), wheel spin, gear losses, battery; moves the simulated devices and publishes the true pose |
| `Tire` | Tire force from slip: grip curve and friction circle, scaled by the wheel's load |
| `WheelLoads` | Weight transfer: each wheel's load as the robot accelerates, and lifted wheels |
| `SwerveIOSim` | The drivetrain's `SwerveIO` on `TalonFXModel`s and the sim clock: repeatable, faster than real time |
| `SwerveSimDevices` | What the physics moves: Phoenix's simulated devices or our models |
| `PhoenixSimDevices` | `SwerveSimDevices` on Phoenix's own simulator (wall clock) |
| `PhoenixSwerveSim` | The robot's `SwerveIOPhoenix` on Phoenix's simulator, with the physics on a 1 kHz thread |

Phoenix sim sign conventions (measured, pinned by `SwerveSimStepTest`; see `PhoenixSimDevices`):
`getTorqueCurrent()` is in the motor's own (inverted) frame; `getMotorVoltage()`,
`setRawRotorPosition` and `setRotorVelocity` are in the chassis frame, before the invert.
