# swerve

Our swerve, on plain Phoenix 6 Pro devices. **No CTRE swerve API.** Names follow WPILib and
Phoenix (`toSwerveModuleVelocities`, `optimize`, `cosineScale`, `TorqueCurrentFOC`...).

| File | What |
|---|---|
| `SwerveMath` | Allocation-free kinematics on arrays: discretize, module velocities, desaturate, optimize, cosineScale, toChassisVelocities |
| `SwerveSetpointGenerator` | Limits wheel acceleration and steer rate each cycle; gives a real steer-rate feedforward |
| `ModuleTrust` | Each module's mode and how its motors are commanded, and which modules odometry trusts |
| `ModuleMode` | NORMAL / STEER_ONLY / COAST: what a module can still do |
| `ModuleCommands` | Per motor: real command or coast, full or slow (10 Hz) rate |
| `HeadingHold` | Optional heading hold that lets go after a hit (off by default) |
| `CoastOnDisable` | Brake while enabled or moving; coast once disabled and still (pushable in the pit) |
| `WheelRadiusCharacterizer` | Spin-in-place wheel radius measurement (gyro turn vs wheel travel) |
| `SwerveThread` | The 250 Hz loop: read devices, request to module targets, send, queue the sample |
| `SwerveSample` | One 250 Hz reading (distance, velocity, angle, yaw) plus the targets sent |
| `SampleQueue` | Preallocated ring handing samples from the thread to the main loop |
| `DriveRequest` | What the main loop asks the thread for (neutral, velocity, X lock) |
| `SwerveIO` | Hardware interface; `NONE` for replay |
| `SwervePhoenixConfigs` | The TalonFX / CANcoder configs the Phoenix IO applies |
| `DriveNeutralMode` | Drive brake/coast applied in the background; newest request wins |
| `SwerveIOPhoenix` | TalonFX drive/steer, CANcoder, Pigeon 2; on-motor TorqueCurrentFOC loops |
| `SwerveIOSim` | Desktop sim drivetrain on `TalonFXModel`s and the sim clock: repeatable, faster than real time |
| `SwerveRequests` | Module targets to Phoenix control requests, shared by both IOs |
| `SwerveSimDevices` | What the physics moves: Phoenix's simulated devices or our models |
| `PhoenixSimDevices` | `SwerveSimDevices` on Phoenix's own simulator (wall clock), for cross-checks |
| `PhoenixSwerveConfig` | IDs, inverts, ratios, offsets, limits, gains for `SwerveIOPhoenix` |
| `SwervePhysicsSim` | Desktop physics: rigid body, tires, motors; moves the simulated devices and publishes the true pose |
| `SwerveHealth` | Currents, temperatures, connected flags (slow rate) |
| `SwerveInputs` | AdvantageKit inputs: drained samples and health |
| `SwerveOdometry` | Allocation-free wheel + gyro odometry with a slip signal |

Phoenix sim sign conventions (measured, and pinned by `SwerveSimStepTest`; `PhoenixSimDevices`): `getTorqueCurrent()`
is in the motor's own (inverted) frame; `getMotorVoltage()`, `setRawRotorPosition` and
`setRotorVelocity` are in the chassis frame, before the invert.
