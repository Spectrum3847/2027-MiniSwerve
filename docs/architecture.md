# Architecture

How MiniSwerve's code fits together. Status marks: **built**, **next**, **planned**.

## Threads

| Thread | Rate | Owns | Status |
|---|---|---|---|
| Main | 100 Hz, RT priority 1 | settings, logging, wanted states, commands | built |
| Swerve | 250 Hz (setting `swerve/loopHz`), RT | module setpoints, device reads, sample queue (vision ingest planned) | built |
| Settings writer | on demand | saving Robot Settings to disk | built |

The swerve thread never logs. It writes samples into preallocated ring buffers; the main loop
drains them into logged inputs, and replay feeds the same inputs back.

## Main loop order

1. `RobotLoop.next()`
2. `RobotSettings.periodic()`: live tuning (disabled, off-field only), values logged as inputs
3. Subsystem inputs (drain swerve samples, cameras)
4. Pose: TagEKF (drives the robot) + WPILib estimator (shadow) + plain odometry (shadow)
5. Wanted-state machines and commands
6. Alerts, telemetry

## Packages

```
frc.robot                 this robot only
  Robot, Main, RunMode, LoggingSetup
  config/                 Hardware (CAN IDs, ratios: hard stops)
  subsystems/swerve/      drivetrain wanted states              (built)
  pilot/                  driver controller bindings            (built)
frc.spectrumLib           reusable; moves to its own repo later
  framework/              SpectrumRobot, RobotLoop
  robotsettings/          Robot Settings (layered JSON, live tuning, saving)
  telemetry/              Telemetry facade, Alert, DashboardReceiver
  swerve/                 custom swerve on Phoenix 6 Pro          (built)
  statemachine/           WantedState                             (built)
  localization/           TagEKF, odometry guard                  (planned)
  vision/                 our camera frame structs + reader       (planned)
```

## Robot Settings

Values tuned on the robot that survive deploys. Declared in code, resolved from layers (later
wins): code default, `deploy/settings/<group>.json`, `deploy/settings/robots/<robot>/<group>.json`,
the robot-local folder `/home/systemcore/settings/<robot>/`, then live NT edits at
`/RobotSettings/...`. Every value and its layer is a logged input.

Planned: boot reconciliation with three rules plus a git-ancestry check, and Gradle tasks to fetch
the robot's saved values and apply them to git (see the 2026-FM-SystemCore Robot App brief).

## Replay

Strict from day one: anything that changes behavior is a logged input, calibration included.
Logs carry the git SHA, branch and dirty flag.

## Vision

Our PhotonVision fork will publish a compact, versioned per-camera struct (frame header + per-tag
bearing, range, corners, margin) instead of PhotonLib's results. The struct definitions live in
`frc.spectrumLib.vision` and are shared with the fork. Sim produces the same structs.

## Autos and modes

Auto selection and test mode use WPILib 2027 OpModes. Autos ask subsystems for wanted states; the
auto builder (Choreo-based paths, straight-line paths) comes later.

## Swerve

Each module's loops run on the motors at 1 kHz: steer is PositionTorqueCurrentFOC on the fused
CANcoder (or MotionMagicExpoTorqueCurrentFOC with setting `swerve/steerMotionMagicExpo`), drive is
VelocityTorqueCurrentFOC with a torque feedforward from per-wheel forces (CTRE's
WheelForceFeedforwardX/Y). Setpoints go out once per cycle (UpdateFreqHz 0). The drive slip current
is both the stator limit and peak torque current, as CTRE's SlipCurrent. Field-relative driving
follows the alliance (operator forward: 0 blue, pi red); Back seeds field-centric. The 250 Hz thread reads latency-compensated signals,
turns the latest `DriveRequest` into module targets with that fresh reading (field-relative
rotation by the live gyro, discretize, desaturate, optimize, cosine scale, MK4i coupling
compensation, wheel force feedforward) and queues the sample. In sim, `SwervePhysicsSim` moves the same Phoenix devices.

Sim check: `MINISWERVE_SIM_SCRIPT=drive ./gradlew run -PnoSimGui`, then
`python tools/swerve_summary.py logs/<log>.wpilog`. On 2026-11-07: 249 Hz, no dropped samples,
odometry 3 cm mean / 5 cm max from truth over ~16 m; replay reproduces odometry, request and
state exactly.

Safety: the robot drives only when every module's devices are connected and took their config
(logged health); the thread goes neutral after 5 stale-signal cycles (20 ms) and on any fault; with
the Pigeon down, field-relative uses the odometry (wheel) heading.

Open on the robot: CAN bus load at 250 Hz on SystemCore's bus (8 Talons + 4 CANcoders + Pigeon;
may need 200 Hz or a CANivore), whether Phoenix's clock matches WPILib's on SystemCore, and the
four CANcoder magnet offsets. MK4i inverts, ratios and coupling are fixed facts (see `Hardware`).

## Heading sources

The robot has several rotation sources. Each will be its own logged source, compared every loop,
with the best healthy one driving field-relative control and odometry:

1. Pigeon 2 (CAN) - primary. **Built.**
2. SystemCore IMU (`OnboardIMU`); this robot imitates it with a USB gyro. **Planned.**
3. Vision heading from the TagEKF. **Planned** (with the EKF).
4. Wheel odometry. **Built** as the fallback when the Pigeon is down.

Mount orientation for each is a Robot Setting (`config/ImuSettings`): Pigeon MountPose yaw, pitch,
roll; SystemCore IMU `MountOrientation` and yaw.

## Swerve control details

- **Setpoint generator** (`swerve/setpointGenerator`, on): each cycle moves the chassis setpoint
  toward the request as far as every wheel's acceleration allows (`maxDriveAcceleration`), and
  turns modules at most `maxSteerVelocity`. That turn rate goes to the steer motors as a velocity
  feedforward.
- **Rotation lookahead** (`swerve/rotationLookahead`, 0.04 s): field-relative requests are aimed
  with heading + yaw rate x lookahead, so driving while spinning goes straight. Sim: sideways drift
  over the spin-and-drive step fell from 0.61 m to 0.07 m. Tune on the robot.
- **Heading hold** (`swerve/headingHold`, off): captures a heading only after the robot settles,
  corrects only while translating, at most `headingHoldMaxCorrection`, and accepts a new heading
  when the error jumps past `headingHoldReleaseDeg` (a hit) instead of steering back.
- **CANivore timesync**: when the bus is CAN FD (a CANivore), controls and signals use Phoenix
  timesync automatically (`Swerve/Health/Timesync`). Sample timestamps are the mean of the signals'
  own timestamps on every bus.

## Module trust and degraded modes

Every sample records which modules' signals were fresh (`Samples/ModuleOk`); health adds connected,
configured, CANcoder magnet and fused-sensor faults. `ModuleTrust` combines them, plus "absolute
angle good at least once since boot". Odometry uses only trusted modules (weighted least squares;
two are enough), and logs each module's slip (`Swerve/Modules/Slip`) and commanded-minus-measured
speed (`Swerve/Modules/VelocityError`).

Control still goes neutral if any module is untrusted. Planned degraded modes, to build after
testing on the robot:

1. **Drive motor down (7 motors).** Coast that drive, keep steering it along its free-rolling
   direction, and solve kinematics with three driven wheels (any chassis motion is still possible).
2. **Steer motor down.** Coast its drive. The wheel's angle is fixed, so plan chassis motion that
   rolls along it where possible and accept scrub otherwise; account for its drag.
3. **CANcoder down after boot.** The steer motor keeps the angle from its rotor; keep driving,
   alert, and stop trusting the absolute angle after a reboot.
4. **Finding a lost steer angle.** Idea to try: drive the other three straight and steer the bad
   module (or sweep the chassis direction) while watching drive current and slip; the angle with
   the least drag is where the wheel points.
