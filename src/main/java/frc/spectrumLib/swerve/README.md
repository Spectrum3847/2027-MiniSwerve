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
| `DriveMotorLimit` | How hard a drive motor can speed its wheel up at a wheel speed and battery voltage (back-EMF, slip and supply limits) |
| `DriveNeutralMode` | Drive brake/coast applied in the background; newest request wins |
| `SwerveIOPhoenix` | TalonFX drive/steer, CANcoder, Pigeon 2; on-motor TorqueCurrentFOC loops |
| `SwerveRequests` | Module targets to Phoenix control requests, shared by `SwerveIOPhoenix` and SpectrumSim's sim IO |
| `SimulatedSwerve` | A drivetrain in the desktop sim: its IO and true-pose logging (SpectrumSim implements it) |
| `SwerveSimSpec` | The physical drivetrain the desktop sim models (motors, mass, tires, module positions) |
| `SampleGuard` | Marks a reading stale for one sample when it jumps farther than the robot can move (a device reboot) |
| `PacedSignals` | Paces the swerve thread on the devices that answer; a silent device stops being waited on and rejoins when it returns |
| `PhoenixSwerveConfig` | IDs, inverts, ratios, offsets, limits, gains for `SwerveIOPhoenix` |
| `SwerveHealth` | Currents, temperatures, connected flags (slow rate) |
| `SwerveInputs` | AdvantageKit inputs: drained samples and health |
| `SwerveOdometry` | Allocation-free wheel + gyro odometry with a slip signal |

The drivetrain's simulation (physics, sim IO) lives in SpectrumSim (`SpectrumSim/`).
