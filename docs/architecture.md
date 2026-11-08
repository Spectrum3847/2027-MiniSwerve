# Architecture

How MiniSwerve's code fits together. Status marks: **built**, **next**, **planned**.

## Threads

| Thread | Rate | Owns | Status |
|---|---|---|---|
| Main | 100 Hz, RT priority 1 | settings, logging, wanted states, commands | built |
| Swerve | 250 Hz (setting `swerve/loopHz`), RT | module setpoints, device reads, sample queue (vision ingest planned) | built |
| Settings writer | on demand | saving Robot Settings to disk | built |
| Sim clock (sim only) | 1 kHz sim time | physics and TalonFX models; steps the main loop and swerve thread in turn | built |

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

PhotonLib's results carry far more than the pose estimator and object tracking use, so Spectrum's
PhotonVision fork sends only what the robot needs: one compact, versioned binary frame per camera
frame (`frc.spectrumLib.vision.VisionFrame`, v1), on NetworkTables raw topic
`/spectrum/vision/<camera>/frames`. Header: version, kind (tags or objects), count, the camera's
frame counter, capture time on the robot clock (ns). Per tag (50 bytes): id, bearing (yaw, pitch)
and range to its center in the camera frame, decision margin, the four corners in pixels (for mount
calibration). Per object (22 bytes): class, confidence, bearing to the box's bottom center, box
size. `VisionFrame.java` is copied word for word into the fork, which must publish the capture time
on the robot's clock. A camera with nothing in view still sends empty frames, so silence means
trouble. The robot does not use PhotonLib.

`Vision` (robot) reads every frame each loop, logs the raw bytes as the AdvantageKit input (50
bytes a tag: replay decodes exactly what the robot did), decodes them without allocating, counts
dropped (frame counter gaps) and bad frames, logs latency, and raises an alert when an enabled
camera goes quiet for 0.5 s. A dead camera drops out; the robot keeps going on odometry and the
others. Cameras and their mounts are Robot Settings (`vision.<camera>/enabled`, `x`, `y`, `z`,
`rollDeg`, `pitchDeg`, `yawDeg`); MiniSwerve has none yet, so they are off on the robot and on in
sim at placeholder mounts (front and back, 0.3 m up, tilted 15 deg up). The tag layout is WPILib
2027's `Fields.FRC_2026_REBUILT_ANDY_MARK` until the 2027 field is out.

In sim, SpectrumSim's `CameraSim` makes the same frames from the true pose: the field's tags
through a pinhole camera (Thriftiest Cam: 1280x800, about 70 deg, 60 fps, 25 ms), 0.3 px corner
noise, bearing from the noisy corners' center as the coprocessor computes it, range noise 0.005 +
0.005 r^2 m, published on the same topic after the camera's latency, on the sim clock with seeded
noise. Checked on the drive script against the true geometry: bearing error 0.007 deg median, range
error 9 cm median (the noise model at 4 to 8 m), latency 31 ms as the robot reads it; two runs give
identical frames, and replay matches. No lens distortion, blur, occlusion or object detection yet.

Next: the fork publishing `VisionFrame`.

## Pose: TagEKF

TagEKF (`frc.spectrumLib.localization.TagEkf`, FM's, tuned on the TXMAN1 replays) drives the
robot's pose (`Localization`, logged `Pose/Robot`). Every 250 Hz swerve sample predicts it (wheel
travel and odometry's heading: the gyro's when healthy, the wheels' when not); every camera's tags
correct it one at a time, each as a bearing and range from its camera at the frame's capture time.
The heading it predicts with is odometry's without the jumps of field-centric re-zeroing (those
would read as a sudden turn). Swerve sample times are moved onto the robot clock at the IO (the
Phoenix-to-robot offset read with each sample), the clock the camera frames use; each frame's age
against the newest sample is logged, and frames older than 0.3 s or from the future raise an
alert and are not used.
Frames are inserted at their time and later samples replayed over them, so arrival order does not
matter. A tag whose innovation fails a chi-squared gate is rejected; if tags keep failing the
covariance widens so the cameras can pull it back (relocalization); adaptive uncertainty (mean NIS
over 10) grows it when the cameras disagree. Its first pose comes from the first frame with two or
more tags (heading from the line between them). Noise is Robot Settings (`tagekf/*`): TagEkf's
tuned defaults, bearing 0.5 deg.

Seeding is checked: a frame with two or more tags gives a whole pose only if its tags are at least
1 m apart and within 8 m, every pair agrees on heading within 3 deg, and every tag lands within 15
cm of where the pose puts it; a seed also needs every tag within 6 m and the next such frame to
agree (20 cm, 3 deg), and goes in at its capture time. If TagEKF keeps rejecting tags for a second
while checked frames agree on a pose more than 0.5 m away, it is re-seeded there. Logged:
`Pose/Seeds`, `Pose/Reseeds`, `Pose/SeedCheck`, `Pose/UnknownTags`, `Pose/FrameAgeSeconds`,
`Pose/RejectReasons`, `Pose/HeadingFromGyro`, `Pose/Shadow/Delta`.

WPILib's `SwerveDrivePoseEstimator` runs beside it as a logged shadow (`Pose/Shadow/WPILib`), on the
same samples, with each checked multi-tag frame as one pose measurement (never single tags). Its
numbers below measure that feed, not WPILib's estimator at its best. Plain odometry is Swerve's own.

In sim (both cameras, against the true field pose):

| Script | TagEKF median / p90 | WPILib shadow median / p90 | Odometry median |
|---|---|---|---|
| drive | 1.1 / 3.2 cm | 9.4 / 34 cm | 37 cm |
| walls (wheels scrubbing on walls) | 1.6 / 4.8 cm | 39 cm / 3.5 m (few checked frames at the walls) | 3.5 m |
| drive with a defender and a parked robot | 2.4 / 12 cm | 12 / 69 cm | 1.1 m |

TagEKF heading error is 0.01 to 0.02 deg median; it seeds 0.05 s after boot, once; no
relocalizations or re-seeds; frame age 21 ms median.

Scripted driving is a person's, not a step function (`SimDriver`): sticks reach where a step wants
them over about 0.15 s and fall back the same way, each push is 95 to 100% and up to 3 deg off its
axis, sticks rest up to 3% off center and jitter, and everything the driver does reaches the robot
30 ms late (Driver Station and radio). Stick reach is a 0.05 s time constant, 95% in 0.15 s. Seeded: runs repeat. `MINISWERVE_SIM_DRIVER=perfect` gives
exact, instant sticks. Steps can also unplug the controller (`.unplugged()`) or lose the Driver
Station (`.dsLost()`: a radio drop; the robot disables and hears no controller. WPILib's sim keeps
reporting the Driver Station attached, so that flag is not tested). The 30 ms delay is the
driver's only: mode, enable, faults, unplug and Driver Station loss happen on the robot's
timeline. The scripts themselves are in `SimScripts`. `MINISWERVE_SIM_SCRIPT=driver`: hands off for 2 s, the robot does not move
(sticks read about 1.6% off center); the controller unplugged with the stick pushed, the robot
stops and drives again when it is back; the Driver Station lost, the robot disables and stops, then
drives again.

`MINISWERVE_SIM_SCRIPT=buttons` presses the pilot's buttons (sim script steps can press any
button, trigger or d-pad and set Driver Station inputs: `.press(p -> p.setBackButton(true))`):
after a 60 deg turn, Back re-zeroes field-centric while driving. Odometry's heading jumps as
meant; TagEKF's error stays under 1 deg and 1.4 cm. Fed odometry's raw heading instead, the same
re-zero throws TagEKF 63 deg and 1.2 m off. X held with the stick pushed brakes the robot to a
stop, and slow mode drives at 0.55 m/s against 1.87 at full stick.
Runs repeat exactly and replay matches. The sim's cameras have no distortion, blur or occlusion,
so real numbers will be worse; FM's real-match replays are the reference (0.057/0.136 m on Q17).

## Autos and modes

Everything the Driver Station can pick is a WPILib 2027 OpMode, registered in `Robot` with its
dependencies passed through a supplier (`frc.robot.opmodes`). `SpectrumRobot` extends our
AdvantageKit fork's `LoggedOpModeRobot`, so the selected mode and OpMode are Driver Station inputs
(`/DriverStation/RobotMode`, `/DriverStation/OpMode`, `/DriverStation/OpModeId`) and replay runs
the same one.

- Teleop: `Drive` (match, full speed), `Demo` (pilot driving capped by settings `demo/*`: 1.5 m/s,
  3 m/s^2, 3 rad/s, 6 rad/s^2).
- Auto: `Do nothing`, `Drive forward` (1 m/s for 1.5 s, robot-relative, then holds still; a placeholder until the Choreo auto
  builder generates match autos as OpModes).
- Utility: `Wheel radius` (spins in place while enabled, prints the estimate when disabled).

The pilot's sticks and buttons act only in teleop; in auto and utility the swerve holds still
unless an OpMode asks. An OpMode's per-loop work runs through `OpModeLoop`, before the scheduler
and `applyWantedState`, so it takes effect on the same loop (WPILib calls `periodic()` after
`robotPeriodic()`, one loop late).

OpModes ask subsystems for wanted states (`Swerve.driveRobotRelative`, `limitDrive`); they never
command devices. Settings an OpMode uses are made at boot (`DemoTeleop.Limits.fromSettings`), not
when it is picked. Tuning stays a build-time switch (`RunMode.TUNING_MODE`, false on main), not an
OpMode, so a match robot can never be put into tuning from the Driver Station.

`MINISWERVE_SIM_SCRIPT=auto` runs the drive-forward auto, then Demo at full stick: it ends 1.56 m
forward in auto, and Demo's wheel targets stay at 1.5 m/s (measured 1.56).

## Swerve

Each module's loops run on the motors at 1 kHz: steer is PositionVoltage with FOC on the fused
CANcoder by default (`swerve/steerClosedLoopOutput`: "Voltage", CTRE's default and the most common
choice in 2025 team code, or "TorqueCurrentFOC"; either with MotionMagicExpo via
`swerve/steerMotionMagicExpo`), drive is
VelocityTorqueCurrentFOC with a torque feedforward from per-wheel forces (CTRE's
WheelForceFeedforwardX/Y). Setpoints go out once per cycle (UpdateFreqHz 0). The drive slip current
is both the stator limit and peak torque current, as CTRE's SlipCurrent. Field-relative driving
follows the alliance (operator forward: 0 blue, pi red); Back seeds field-centric. The 250 Hz thread reads latency-compensated signals,
turns the latest `DriveRequest` into module targets with that fresh reading (field-relative
rotation by the live gyro, discretize, desaturate, optimize, cosine scale, MK4i coupling
compensation, wheel force feedforward) and queues the sample. In sim, `SwerveIOSim` sends the same
Phoenix control requests to TalonFX models that `SwervePhysicsSim` moves (see "Repeatable sim").

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
- **Motor acceleration limit** (`swerve/motorAccelerationLimit`, on; team 254's idea): a wheel
  speeding up is also held to what its motor can give at its speed and the battery voltage
  (`DriveMotorLimit`: back-EMF, the drive slip current, the supply current limit, robot weight).
  So near top speed or on a sagging battery the plan is one the motors can follow, and the
  wheels stay on their setpoints. Braking stays at the traction limit (back-EMF helps it). The
  voltage is `Swerve/Health/SupplyVoltage`: the highest among connected drive motors.
- **Rotation lookahead** (`swerve/rotationLookahead`, 0.04 s): field-relative requests are aimed
  with heading + yaw rate x lookahead, so driving while spinning goes straight. Sim: sideways drift
  over the spin-and-drive step fell from 0.61 m to 0.07 m. Tune on the robot.
- **Heading hold** (`swerve/headingHold`, off): captures a heading only after the robot settles,
  corrects only while translating, at most `headingHoldMaxCorrection`, and accepts a new heading
  when the error jumps past `headingHoldReleaseDeg` (a hit) instead of steering back.
- **Reboot jumps** (`SampleGuard`): a rebooted TalonFX reports rotor rotations until reconfigured
  and a rebooted Pigeon restarts its yaw at zero, so a reading jumps in one sample. Any reading
  that moves farther than the robot can in one sample is stale for that sample
  (`Swerve/Thread/RejectedJumps`): odometry re-primes that module, heading comes from the wheels
  for that sample, and field-relative driving holds its heading until the main loop's offset
  follows the new yaw. Sim faults script: error added by a steer reboot 0.86 m to under 2 mm.
  Driver seeding and vision only move offsets and the estimator, never device readings, so they
  are untouched; never reset a device's own position or yaw.
- **CANivore timesync**: when the bus is CAN FD (a CANivore), controls and signals use Phoenix
  timesync automatically (`Swerve/Health/Timesync`). Sample timestamps are the mean of the signals'
  own timestamps on every bus.

## Module trust and degraded modes

The rule (AGENTS.md "Finish the match"): a fault degrades only its own module; the drivetrain
always keeps driving with what still works.

Each module has a mode (`Swerve/Modules/Mode`), from logged health:

| Mode | When | What it does |
|---|---|---|
| NORMAL | everything fine | drives and steers |
| STEER_ONLY | drive motor disconnected or its config not applied | wheel still steered along its rolling direction |
| COAST | steer motor out, or angle never known since boot | module rolls free; the others carry the robot |

How each motor is commanded (`ModuleCommands`) is a separate decision: **keep commanding, without
flooding the bus.** A motor that is disconnected (no status frames) may still be alive and
listening, so if its config is known it keeps getting its real command, at 10 Hz instead of the
loop rate. A motor only coasts when its command could do harm: a drive whose config never applied
(invert unknown) or whose wheel angle is unknown.

Recovery is automatic: a device counts as connected again as soon as its status frames arrive
(Phoenix `isConnected`, read at 10 Hz); a reboot (`hasResetOccurred`) marks its config and the
signal rates to be redone; failed configs and signal rates retry on `PhoenixBackground` every 0.5 s
until they succeed; brake/coast is re-requested while the applied state does not match. A CANcoder
lost after boot keeps the module NORMAL (the steer motor counts from its rotor). Late signals never
stop the drivetrain: a stale module is planned from its last target.

Wheels that cannot push (STEER_ONLY, COAST, or a drive only reachable at the slow rate) are left
out of the speed limit, the setpoint generator's acceleration limit and the wheel-force feedforward;
a path follower's force is redistributed to the wheels that can push (`SwerveMath.distributeWrench`,
same totals). Each wheel's speed and angle for a requested motion do not depend on the others, so
the working wheels already get exactly what the motion needs; the robot just has less traction.

The swerve thread holds the last setpoint if one cycle throws (a bug in our code), and goes neutral
only after 25 failing cycles in a row (0.1 s): the one exception to "Finish the match", because no
command can be computed and holding the last one would drive blind.

Odometry stays strict: it uses a module only when NORMAL, its CANcoder fused and readable, and that
sample fresh (weighted least squares; two are enough). Per-module slip and speed error are logged.

A fused-sensor fault (steer rotor and CANcoder disagree) keeps the module driving: the fault does
not say which one is wrong. Phoenix re-syncs to the CANcoder, which is right after a gear or belt
skip (it sits on the module shaft). Odometry stops trusting the module until the fault clears.

**Next: which sensor is wrong.** If the CANcoder is the bad one (BadMagnet / magnet health, jumping
readings, or the module's slip residual says its angle disagrees with the robot's motion while the
rotor-integrated angle agrees), switch that steer motor to rotor-only feedback with the last good
angle as its offset (`FeedbackSensorSource = RotorSensor`, `FeedbackRotorOffset`, applied on
`PhoenixBackground`) and keep pushing. If the rotor is the bad one (skip), stay fused. Decide from
logged inputs so replay agrees; build and test it on the robot.

Next degraded modes, to build after testing on the robot: plan chassis motion around a fixed
(steer-out) wheel instead of letting it scrub; account for its drag; and find a lost steer angle by
driving the other three and watching drive current and slip (the least-drag angle is where the
wheel points).

## Drivetrain utilities

- **Brake/coast** (`swerve/coastOnDisable`, `coastDelaySeconds`): brake while enabled and while the
  robot is still moving after a disable; coast after 0.5 s disabled and still, so it can be pushed.
  The config apply runs on `PhoenixBackground`. Logged: `Swerve/DriveBrakeWanted` and
  `Swerve/Health/DriveBrake` (applied).
- **Per-cycle limits**: `Swerve.limitDrive(maxWheelVelocity, maxDriveAcceleration,
  maxAngularVelocity, maxAngularAcceleration)`, called every loop while needed (e.g. an elevator
  raised), lowers top speed, wheel acceleration, turn rate and turn acceleration for that cycle
  only; it can never raise them. Logged:
  `Swerve/Request/Limits`.
- **Wheel radius**: pick the `Wheel radius` utility OpMode and enable; the robot spins in place (`wheelRadiusSpin`) and
  logs `Swerve/WheelRadius/Estimate` once spinning steadily (after 1 s), printing it when
  released. Copy it into `swerve/wheelRadius`. Run it two or three times on carpet and average.
  The desktop sim reproduces a wrong setting's true radius to within about 1% (its timing is not
  real-time); the math is exact on synthetic data.
- **Stick shaping**: radial deadband (`pilot/stickDeadband`) and squaring on the left stick's
  magnitude, so diagonals keep their direction.

## Planned: calibration drive with a tag rig

A known test rig: two AprilTags a known distance apart at a known height. One recorded session
of driving at, away from and across the tags, plus spins with the tags in view, should calibrate
nearly everything at once, offline from the log (replay makes it repeatable). Built with the
TagEKF and vision work; the rig geometry is a Robot Setting so the solver uses it as truth.

| From the drive | Calibrates | Current home |
|---|---|---|
| Spin in place, tags in view | Wheel radius (vision turn and gyro turn vs wheel travel) | `swerve/wheelRadius` |
| Spin in place | Gyro yaw scale and drift, vs vision heading | `imu` settings (new) |
| Spin and drive, each camera on the tags | Camera mount poses (position, yaw/pitch/roll) | camera settings (new); TagEKF `MountEstimator` |
| Accelerate and brake toward the tags | Camera time offset (latency), vs odometry motion | camera settings (new) |
| Straight drives and strafes | Effective track and wheelbase; CANcoder offset trim (does "straight" go straight) | `swerve/track`, magnet offsets |
| Speed ramps | Drive kS / kV / kA (commanded vs measured wheel speed and current) | swerve gains |
| Whole session | Odometry slip noise and EKF noise values | TagEKF settings |
| Fast spins, drive while spinning | `rotationLookahead`, steer gains, setpoint generator limits | swerve settings |

The tag rig distance and height check the cameras' range scale independently of the field, so a
scaled or miscalibrated camera shows up before it reaches the EKF.

Output: a tool reads the log and writes proposed values into `deploy/settings/*.json` for review
(the same pull-back-to-git path as Robot Settings), never applied silently.

## Battery identity (NFC tags)

From [SpectrumBatteryLogger](https://github.com/Spectrum3847/SpectrumBatteryLogger): every battery
carries an NFC sticker, and a PN532 reader on USB (CH340, `/dev/ttyUSB0`) sits against it when the
battery is installed, so every log records which battery the robot ran on. The code is in
`frc.spectrumLib.battery` (see its README). Proven so far on a laptop and a Pi 4 (300/300 reads,
about 30 ms each, tag present at power-up read first time). **Not yet run on SystemCore**: still to
prove there are the CH340 driver in its kernel, the robot program's permission to open
`/dev/ttyUSB0`, jSerialComm's native library loading, read range through the battery wrap near
metal, reads with the drivetrain running, and swaps while disabled.

How it fits:

- `BatteryTagIOSerial` reads on its own low-priority daemon thread (serial I/O never on the main
  loop or swerve thread): once at boot, then every 2 s while disabled. It only publishes a
  snapshot; the main loop logs it.
- The reading is AdvantageKit **inputs** (`Battery/TagUID`, `Battery/TagName`, `Battery/TagPresent`,
  `Battery/ReaderConnected`, `Battery/Reads`, `Battery/Error`, `Battery/SecondsSinceRead`,
  `Battery/ListName`), so replay and anything that reacts
  to the battery (the pre-match same-battery check, per-battery limits) see the same value.
  The list name is an input too, so replay names the battery as the robot did. Outputs:
  `Battery/Name` (tag name, else the list's name, else the UID) and `Battery/Swaps` (a swap through
  "no tag" counts). The reader polls only while disabled, so in a match the reading is the
  pre-match one; `SecondsSinceRead` says how old.
- Finish the match: no reader, no tag or an unnamed battery is a LOW alert and nothing else. A
  reader that stops answering is reopened every 2 s; a serial library that cannot load is retried
  every 30 s.
- The name written on the tag (a phone app writes it) wins; `src/main/deploy/batteries.csv`
  (`UID,name`, the logger's format) names tags without one. The robot never writes it.
- Sim reads a fixed battery `SIM00001` "sim-battery"; replay reads the log.
- Per-battery history comes from the logs: voltage sag under load, energy used per match, and with
  the swerve's supply currents, internal resistance per battery over the season.
- Low battery: `BatteryVoltageCheck` judges the voltage only after `battery/settleSeconds` (3 s)
  disabled, so normal sag under load never trips it. Below `battery/lowVolts` (12.3 V) it raises a
  MEDIUM alert that stays up through the match. `Battery/RestingVoltage` with `Battery/Name` gives
  each battery's resting voltage history.

## Planned (from other teams' code)

Decided after reading the reports in Other Team's Code Collection (`analysis/`). In order of when:

- **Drive to pose and auto steps that check sensors** (254, 1678, 2910, 6328): with the auto setup.
  Each step has a success check, a timeout and a fallback (retry, skip, re-localize).
- **Collision and stall detection on paths** (1678 speed check, 3256 pause with hysteresis, 6328
  `BeachedUtil`): with the auto and pathfinding work. Commanded vs measured speed, odometry slip,
  Pigeon pitch/roll; pause the path or replan, and raise odometry noise in TagEKF while slipping.
- **Rejecting tags after fast spins** (254 rejects single tags if yaw rate passed 5 rad/s in the
  last 0.3 s): our exposure and latency are short, so the window and rate come from testing on
  the robot, not 254's numbers; it may not be needed at all.
- **Pit self-test in test mode** (6328, 2046, 2910): spin each module and check direction, speed
  and current; CANcoder against rotor; gyro drift at rest; frames from every camera. Pass/fail as
  logged inputs and alerts, so inversions and offsets are caught before a match.
- **Game-piece pickup** (6328, 1678), once object detection exists: a tracker per piece, re-rank
  with hysteresis, ban a spot after a failed pickup, success from the intake sensor.
- **Sim sweeps**, after the above: run the sim scripts headless many times with random seeds,
  camera dropouts and defenders; report pose error and auto success rate as regression numbers.

Not doing: a physical coast/brake switch (2910); `CoastOnDisable` covers it.

## Simulation: rigid body and device faults

The robot's physical description is configuration (Robot Settings), shared by the sim and the real
robot: `swerve/robotWeightLbs` (weigh it ready to play, in pounds) and `swerve/tireFriction` (tread on carpet).
They set the traction limit, friction x g, and the drive current at the slip point: the tire's
share of friction x weight through the gearing plus what spins up the motor rotor, over its torque
constant. `swerve/tractionUse` (default 0.97) takes just under both: the drive slip current
(CTRE's SlipCurrent: stator limit and peak torque current, capped by `Hardware`) and the setpoint
generator's planned acceleration. Logged under `Swerve/Traction/`. At the 55 lb placeholder on MK4i L2 Falcons
the slip current is about 31 A. Measure the real slip point and raise `tractionUse` toward 1. Center-of-gravity height (weight transfer, tipping) can join them later.

`SwervePhysicsSim` treats the robot as one rigid body. Each wheel pushes along its direction with
its motor's torque and grips sideways with tire friction (both limited by friction on carpet, about
1.1 g). Each wheel spins on its own (motor rotor through the gearing plus the wheel), tied to the
ground only by the tire: tire force follows a grip curve of slip speed (peak at 0.1 m/s, falling to
85% once sliding), along and across the wheel in one friction circle. Drive gearing passes 90% of
motor torque with 0.1 N m of drag (typical values, not yet fitted). Physics runs in steps of at most
1 ms and the wheel sub-steps further, so the stiff tire stays stable on heavy or grippy robots. So
wheels spin when the motor gives more than the tire holds, a dead or coasting wheel rolls along, a misaligned wheel scrubs and fights the others, and pushing harder than the tires hold makes
the robot slide, which wheel odometry cannot see. (The first sim moved each wheel on its own and fit
the robot to them, which hid all of that.) Hard maneuvers now show real odometry drift: the drive
script averages about 0.3 m with no acceleration cap and the 80 A slip current: on an 18 kg
robot each tire grips about 48 N, which is only about 19 A on a Falcon through MK4i L2, so the
motors can push far past grip. Measuring and setting the real slip current fixes that. With wheel
spin the encoders see the spin, so sim odometry drift behaves like a real robot's: the drive script
now ends 0.15 m off (mean 0.11 m, heading within 1.3 deg).

Phoenix's simulator cannot disconnect one device, so `SimFaults` (sim only) does it in our code:
`PhoenixDeviceSet` (connected, rebooted, config applies), the IO (frozen readings, ignored commands)
and the physics (no torque) all ask it. Faults: UNPLUGGED (power and data lost; plugging back in
reboots it), UNHEARD (frames lost, still obeys), failed config applies, reboots.

`MINISWERVE_SIM_SCRIPT=faults` drives while unplugging a drive, a steer, a CANcoder, the Pigeon,
making a drive unheard and rebooting a steer whose config then fails twice. Check it with
`python tools/fault_summary.py logs/<log>.wpilog`: the robot must keep moving in every step and end
with every module NORMAL (exit code 0).

Known from the fault scenario: with the Pigeon out, heading comes from the wheels, which scrub, so
odometry drifts (about 0.2 m over a 1.5 s outage while turning) and keeps that heading error when
the gyro returns. The SystemCore/USB gyro as a second heading source, and the TagEKF, are the fix.

## Timing and power from real logs

Fitted from FM's 13 TXMAN1 matches on SystemCore (enabled 155 s each):

| What | Real | In the sim |
|---|---|---|
| CAN sample spacing at 250 Hz | 2 to 8 ms: 3 ms 21%, 4 ms 50%, 5 ms 23%, 6+ ms 4.5% (457,911 samples) | Drawn from that histogram (`SwerveIOSim`); the sim's own run matches it within 1 point per bin |
| Failed reads (stale frames) | 350 of 420,060 | Same rate, as `allGood` false |
| Camera capture to robot | median 15.7 ms, p90 21.5 ms (Orin, 33,396 frames) | Drawn from that histogram (`CameraSim`) |
| Battery open-circuit voltage | falls 0.6 to 1.6 V over a match, about 0.8 V per Ah drawn | 0.8 V per Ah drawn, on top of the 0.012 ohm sag |
| Main loop | median 1.4 ms, p99 3.1 ms; about 2.5 loops a match over 10 ms (max 114 ms) | Not modeled: too rare to matter, and it would need the sim clock to hold the loop |

Fixed with this: sim samples were all stamped with AdvantageKit's loop-start time (what
`Timer.getTimestamp()` returns in the main loop), so every sample in a loop shared one time. They
are now stamped when read, on the robot clock the cameras use; sim odometry error on the drive
script fell from 29 to 9 cm median.

`python tools/fit_timing.py <logs>` reruns these fits.

## Weight transfer

Each wheel's load follows the robot's acceleration (`WheelLoads` in SpectrumSim): speeding up
loads the back wheels, turning loads the outside ones, and each tire grips in proportion to its
load. Height comes from Robot Setting `swerve/cogHeightInches` (6 in placeholder: measure it). A
wheel whose load would go below zero is lifted (`Sim/WheelLifted`); loads are logged as
`Sim/WheelLoads` (N). The chassis is rigid and quasi-static: it does not tilt, so a lifted wheel
means the robot is starting to tip but the tip itself is not modeled yet. In the drive script one
corner goes light for a moment on hard diagonal starts and stops (three wheels carry it), as real
robots do.

Against FM q32 (`simReplay -Pparams="cog=..."`), raising the center of gravity from 0 to 0.25 m moves
wheel-acceleration p90 from 0.82 to 0.92 g (real 1.08) and slows sprints slightly; match data is too
mixed with contact to fit the height, so measure it.

## 3D pose and field terrain

AdvantageScope's 3D field draws a `Pose3d` tilted and raised, so the robot is logged in 3D as well
as 2D: `Pose/Robot3d` (TagEKF) and `Swerve/Odometry/Pose3d` take x, y and heading from their 2D
pose and roll and pitch from the Pigeon (`Swerve/Health/Roll`, `Pitch`; read as its quaternion,
WPILib's frame by construction; flat while it is not connected). Height is not measured, so these
stay on the carpet. Display only: nothing drives on them.

In the sim the floor has height (SpectrumSim `field/Terrain`): bumps and ramps from a field file,
`SpectrumSim/src/main/resources/fields/<name>.json`, picked with `SPECTRUM_SIM_TERRAIN` (default
`flat`, so the other scripts are unchanged). `2026-rebuilt` has the four 2026 BUMPs from the Game
Manual; their positions still need checking against the field drawings and CAD. A new game is a new
file. Each step `TerrainContact` fits a plane through the floor heights under the wheels: the
robot's height, roll and pitch, and gravity along the plane pulling it downhill. The Pigeon
(model and Phoenix sim) reports that tilt, and the true pose is logged as `Sim/FieldPose3d`.

`MINISWERVE_SIM_SCRIPT=bump` with `SPECTRUM_SIM_TERRAIN=2026-rebuilt SPECTRUM_SIM_START=2.5,2.511,0`
crosses a BUMP and back: the robot rises 10 cm, -15 deg pitch climbing and +15 deg coming down, in
both `Sim/FieldPose3d` and `Pose/Robot3d`; replay matches. Not modeled yet: suspension or bounce
(the robot sits on the best-fit plane, not on three wheels), wheels measuring distance along the
slope rather than across the floor (a few cm of odometry error per bump on a real robot), and
cameras tilting with the robot.

## Field walls and bumper contact

The sim robot drives on an FRC field (SpectrumSim `field/`): the four walls, 16.54 x 8.07 m, as
fixed outlines, and the robot's bumpers (Robot Settings `swerve/bumperLengthInches`,
`swerve/bumperWidthInches`, 22 in placeholders) as a box. Each 1 ms step, after the robot moves, the
bumpers are pushed back out of any wall and the contact impulse comes off the robot's motion: a
small bounce on a real hit (20% of the closing speed, none for resting contact), and bumper-on-wall
friction (0.4) while sliding. The tires keep pushing, so a robot driven into a wall stops and
spins or scrubs its wheels, as on a field. `SPECTRUM_SIM_FIELD=none` gives open floor;
`SPECTRUM_SIM_START=x,y,deg` places the robot (default 2, 2, 0, clear of the walls for the scripted
drives). The true pose on the field is logged as `Sim/FieldPose` for AdvantageScope's field view,
contacts as `Sim/Contacts`.

`MINISWERVE_SIM_SCRIPT=walls` strafes into the side wall, drives along it, and backs into the
alliance wall: the bumpers stop 0.278 m out (half the bumper, less 2 mm of slop). Driving along the
wall covers 3.2 m in 1.5 s, but driving along it while also steering 20 deg into it covers only
0.4 m in 2 s: wheels angled into the wall scrub sideways, that scrub pushes the robot into the
wall, and bumper friction against that push cancels the drive. That follows from the model; it is
not yet checked on a real robot.

Other robots (`SPECTRUM_SIM_ROBOTS`, e.g. `defender:9,4,3;parked:6,3.2,0`) are 60 kg boxes with
0.9 m bumpers that drive toward what their behavior wants, no harder than 1.1 g of traction:
`parked` holds a spot and slides only when pushed past its tires, `shuttle` drives back and forth,
`defender` chases our robot and pushes into it. Robot-to-robot contact uses the same bumper impulses
as walls, with both robots moving. Their poses are logged as `Sim/OtherRobots`.

Contacts are solved over up to two touch points at once (iterated impulses), so a bumper face
landing nearly flat on a wall shares the hit along the face: before, anything past about 0.07 deg
off parallel landed on one corner and spun the robot hard. The robot is placed with
`SPECTRUM_SIM_START`, which must be on the field and clear of the walls.

Also fixed with this: the tire force now opposes the combined slip (along and across the wheel as
one vector); before, each direction saturated on its own, which made sideways scrub too strong
whenever a wheel both drove and slid.

## Repeatable sim

Phoenix's simulator runs each motor's control loop on the wall clock, so two runs of the same
script never matched (three drive-script runs ended at headings 122, 131 and 40 deg). The desktop
sim now runs on one clock instead:

- `SimClock` (SpectrumSim, `SpectrumSim/`) pauses WPILib's sim time at a fixed start and steps it 1 ms at a
  time: physics and motor models, then WPILib's clock (which runs the main loop when it is due and
  waits for it), then the 250 Hz swerve thread. Nothing moves while any of them is working.
- `TalonFXModel` runs a TalonFX's control loop from the same `TalonFXConfiguration` and control
  requests the robot sends (`SwervePhoenixConfigs`, `SwerveRequests`), at 1 kHz on that clock.
  Other mechanisms will use it too; new control requests are added as they need them.
- `SwerveIOSim` is the drivetrain on those models; faults (`SimFaults`) act on it as before.

Pace: `SPECTRUM_SIM_SPEED` (multiple of real time, or `max`); default `max` when a script runs,
real time otherwise. `SPECTRUM_SIM_DEVICES=phoenix` runs the old way, on Phoenix's simulator.

All of this is SpectrumSim, its own library in `SpectrumSim/` (Gradle source set `sim`). Robot code
reaches it only through `frc.spectrumLib.framework.DesktopSim`, loaded at runtime, so the deploy
jar carries no simulation code.

Result (2026-11-07): three drive-script runs give identical logs (only boot ID and build time
differ), a 9.5 s script takes about 4 s including Gradle startup, and replay of a sim log matches
it exactly. The auto, wheel-radius and faults scripts pass as before.

`./gradlew talonCrosscheck` runs the same commands through Phoenix's simulator and our model:

| Steer request | Angle diff RMS | Steer 90 deg settle (Phoenix / model) | Overshoot (Phoenix / model) |
|---|---|---|---|
| PositionTorqueCurrentFOC | 1.0 deg | 104 / 104 ms | 2.0 / 1.7 deg |
| MotionMagicExpoTorqueCurrentFOC | 0.35 deg | 164 / 172 ms | 9.7 / 9.3 deg |
| PositionVoltage (FOC) | 7.9 deg | 284 / 136 ms | 27.6 / 0 deg |
| MotionMagicExpoVoltage (FOC) | 10.4 deg | 284 / 172 ms | 23.6 / 10.4 deg |

Drive (VelocityTorqueCurrentFOC) matches within 0.04-0.08 m/s RMS on every run. Voltage steer is
the open one: Phoenix's simulator overshoots about 25 deg on a 90 deg step with CTRE's MK4i
defaults, ours does not. Our guess is that Phoenix's simulator feeds the fused CANcoder to the loop
at the CANcoder's frame rate, where a real TalonFX fuses its rotor sensor at 1 kHz.

Real MK4i robots side with the model. Spectrum's 2024 robot (MotionMagicVoltage steer, kP 100,
`Spectrum-Driver-Station-Logs`, 133 steps over 30 deg) settled within 2 deg in a median 0.12 s with
median overshoot 0.0 deg (p90 1.1 deg); 2023's (Phoenix 5 position PID, `2023-Log-Files`, 72 steps)
0.18 s and 0.0 deg (p90 0.8 deg). Neither ran CTRE's PositionVoltage tune, and no public log that
did was found on GitHub, so MiniSwerve's own first steer steps will settle it. MiniSwerve steers with PositionVoltage, so its drive
script differs between the two sims: the model turns faster while spinning (4.4 vs 3.1 rad/s), and
odometry ends 0.64 m from truth versus 0.13-0.40 m on Phoenix's simulator.

## Sim against real logs

The sim is checked against real robots, not just itself:

```bash
python tools/fm_extract.py <real.wpilog> build/simreal/x.csv --start 0 --seconds 30
./gradlew simReplay -Pin=build/simreal/x.csv -Pout=build/simreal/x_sim.csv [-Pparams="mass=70 friction=1.1"]
python tools/sim_vs_real.py build/simreal/x.csv build/simreal/x_sim.csv
```

`fm_extract` pulls a real match's commanded and measured module states, Pigeon acceleration and
currents; `simReplay` drives the same commands through our sim configured as that robot (FM 2026:
MK5n R2, Kraken X60 FOC, its gains and limits) on the TalonFX models (`phoenix=1` for Phoenix's
simulator in real time); `sim_vs_real` scores steer settling and
overshoot, wheel-speed tracking, sprint rise time, acceleration percentiles and time at the current
limit, real against sim. Change the physics, rerun, keep what shrinks the gaps.

Fitted so far (FM q32 at TXMAN1, 30 s):
- wheel spin: time at the drive current limit 0.1% vs real 0.9% (was 7% before wheel spin);
- acceleration from wheel velocities, scored the same way on both sides: median 0.28 vs real
  0.32 g. The Pigeon-based median (sim 0.41 vs real 0.28 g) compares an accelerometer with filtering
  and vibration against sim velocity differences, so trust the wheel-based median; its p99 is noisy;
- steer friction 0.3 N m per rad/s: overshoot 3.4 vs real 3.2 deg, settle 112 vs 100 ms;
- tire friction about 0.9 (worn tread at TXMAN1): acceleration p99 0.91 vs real 0.89 g. Fresh tread
  is nearer 1.1, so `tireFriction` should be re-measured as tread wears; a match log can estimate it.

Fitted 2026-11-07 (`simReplay` now runs on the TalonFX models, about 6 s per 30 s of match
including Gradle, and `-Psweep` runs many parameter sets in one JVM):
- battery: `python tools/battery_fit.py <logs>` over FM's 13 TXMAN1 matches gives 0.011 to 0.014
  ohm (median 0.012) and 11.6 to 12.3 V open-circuit; the sim's resistance is now 0.012 (was 0.025).
- `python tools/fit_sim.py <logs>` on q1, q27 and q32 (40 s after auto, real battery voltage as
  input, scored only where every real wheel tracks its command within 0.5 m/s):
  - command timing: best at -0.03 s, the logged module targets trail what reached the motors by
    about 30 ms (FM logs them on the main loop after the swerve thread sent them). So FM's targets
    need a 30 ms shift before latency can be fitted; a true command latency is not measurable here.
  - turning inertia factor 0.45 (0.3 to 0.6 within 2% of the best score); mass about 65 kg, but the
    score is flat from 45 to 75 kg.
  These are weak: only 14% of match samples count as free driving (defense, game pieces and walls
  are not in the sim), and where the robot is not free the real gyro turns at about half the rate
  the wheels report. Defaults are unchanged except the battery. A scripted characterization drive
  (sprints, spins, steer steps on open carpet) on MiniSwerve is the data that pins mass, inertia and
  latency.

Not fitted yet: gear losses (`gearEff`, `gearDrag`), the tire curve's peak slip and sliding share,
and load transfer.

Physics so far: Kraken/Falcon motor models, drive supply current limit, battery sag, tire scrub
torque when steering, wheel spin with a grip curve, gear losses, TalonFX control loops modeled at
1 kHz on a repeatable sim clock.
