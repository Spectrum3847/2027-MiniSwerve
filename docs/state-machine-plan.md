# State machine plan

What the wanted-state framework still needs before the 2027 robot's mechanisms and superstructure
are built on it, and in what order. Status marks: **built**, **next** (built with the first
mechanism), **planned**, **maybe**. A piece that needs two mechanisms (limits between them, the
superstructure) builds its one-mechanism part with the first and the rest with the second.
When a piece is built, move its description to [architecture.md](architecture.md) and mark it
built here.

Where these ideas come from: our own `WantedState`, FM's wanted/system states and `ShotGate`
(2026-FM-SystemCore), and Ciprix (a 2026 FRC framework,
[github.com/23Daniel3/Ciprix](https://github.com/23Daniel3/Ciprix)). Ciprix is GPL-3.0: take
ideas from it, never code, or spectrumLib becomes GPL.

## Rules every piece keeps

- **Finish the match.** A timeout or a failed check falls back to something that still works
  (a reduced mode, a fixed setpoint, feeding anyway). It never stops a mechanism or the robot to
  wait for a fix. Every wait has a timeout or a driver override.
- **Replay.** Transition logic reads logged inputs and the robot clock (`Timer.getTimestamp()`)
  only, never devices or outputs. Time is passed in (`now`), so tests and replay control it.
- **Switch statements, not builders.** Transitions are written as a `switch` on the current state
  in the subsystem: the compiler checks every state is handled, a debugger steps through it,
  and grep finds it. No chains of lambdas (Ciprix's `fsm.state(X).onEnter(...).transitionTo(...)`):
  they hide the control flow and can read a device without anyone noticing.
- **Main thread, no per-loop allocation.** Log keys built once, objects reused.
- **Logged generously.** Every piece below lists its keys. Each key gets its comment where it is
  logged, as usual.

## Built: overlapping asks

`WantedState` ([PR #35](https://github.com/Spectrum3847/2027-MiniSwerve/pull/35)): an ask lasts
one loop; the newest asker wins and the one before it takes over when it stops; an asker that
stops renewing lets go by itself. Commands that only ask do not require the subsystem. See
"Wanted states" in architecture.md.

## Next: current state as a real state machine

Today `Swerve` reports current = wanted. A mechanism needs current states that differ from the
wanted one: a wanted `SCORE_HIGH` passes through `RAISING` before `AT_HIGH`; a wanted `SHOOT`
goes `SPINNING_UP`, then `FEEDING`. FM's `handleStateTransitions()` maps wanted to system
one-to-one, so its split does no work; each sequence there (`ShotGate`, the squeeze timer,
`HoodHoming`) is written by hand once.

Shape, per subsystem:

```java
// Wanted: what was asked. Current: where the mechanism is in getting there.
enum Wanted { STOW, INTAKE, SCORE_HIGH }
enum Current { STOWED, RAISING, AT_HIGH, LOWERING, INTAKING, FALLBACK_HIGH }

Current next(Wanted wanted, Current current, double now) {
    return switch (current) {
        case STOWED -> wanted == Wanted.SCORE_HIGH ? Current.RAISING : ...;
        case RAISING -> reachedHigh.update(atHeight(), now) ? Current.AT_HIGH
                : clock.seconds(now) > raiseTimeout ? Current.FALLBACK_HIGH : current;
        ...
    };
}
```

spectrumLib gives the parts that are the same everywhere, in `frc.spectrumLib.statemachine`:

- `StateClock`: time in the current state, reset on a change. Replaces each subsystem's own
  `Timer`.
- Transition reasons: `setCurrent(state, reason)` with a short constant (`"reached"`,
  `"timeout"`, `"asked"`, `"override"`). Logged `<prefix>/State/Transition` (`FROM -> TO`) and
  `TransitionReason`, plus an event.
- One transition per loop, as now. A chain of states costs a loop per step; fine at 100 Hz.
  Write it down where it matters (a state that should be skipped can be).

Done when: the first 2027 mechanism (or a test mechanism in sim) runs a three-state sequence,
logged, with a unit test of `next()` over a scripted `now`.

## Next: "reached its goal"

The one thing a superstructure waits on. Each subsystem says whether its current state is the
goal of the state it was asked for, from logged inputs (position within tolerance, flywheel at
speed), and `WantedState` logs it.

- `setCurrent(state, reason, atGoal)`; `atGoal()` read by the superstructure and LEDs.
- `atGoal(Wanted w)`: true only if `w` is the winning ask and reached. The superstructure asks
  this for the state it asked for, so an operator override (a newer ask) never counts as the
  superstructure's goal being reached. Ciprix keeps this distinction; it matters.
- Tolerances are Robot Settings (they are calibration).
- Logged: `<prefix>/State/AtGoal`, `<prefix>/State/SecondsToGoal` (time from the ask to
  reaching it, last value).

FM's `ShotGate` is the hand-written version of this for one shot: flywheel, hood, aim and range
each vote, with a blocked reason. Keep that idea: `atGoal` false comes with the first reason it
is false (`<prefix>/State/NotAtGoalReason`).

## Next: debounce, timeouts and fallbacks

- `Debounce` (spectrumLib): "true for N seconds without a break", `update(boolean, now)`. Ours,
  not WPILib's `org.wpilib.math.filter.Debouncer`: that one reads
  `MathSharedStore.getTimestamp()` itself, so a test cannot drive it and replay depends on that
  clock being the logged one. Reused object per condition, reset on entering the state.
- Timeouts: a state that should finish (`RAISING`, `HOMING`, `SPINNING_UP`) has a timeout, a
  Robot Setting. On timeout it goes to a fallback current state that still does the job as well
  as it can: score at the height it reached, feed the shot anyway (FM's gate feeds after 1 s),
  hold a homing hood in brake and aim from the assumed zero. Never "stop and wait".
- No retry loop: after a timeout, the same ask from the same requester does not restart the
  sequence until that ask ends (the button is let go, the command ends). A new press tries
  again. With per-loop asks this is "until that requester stops asking". Ciprix blocks the failed
  request the same way; without it a failed homing retries forever.
- Every timeout raises an alert naming the subsystem and state, and is counted.
- Logged: `<prefix>/State/Timeouts` (count), `LastTimeout` (`STATE after N s`),
  `BlockedRequester` (whose ask is held back, empty if none).

## Next: superstructure

A robot-level wanted state that asks the mechanisms for theirs. Built in `frc.robot` (it is this
robot's), on the same `WantedState`.

- Pilot and operator bindings ask the superstructure (`SCORE_HIGH`), not the mechanisms.
- Each loop the superstructure resolves its own ask, steps its current state (same switch shape
  as above), and asks each mechanism for a state. Asking every loop is what the new
  `WantedState` expects, and it is what makes a mechanism fall back to the superstructure when an
  override ends.
- Ordering between mechanisms ("elevator up before the arm swings out") is the superstructure's
  job: it asks the arm to stay stowed until `elevator.atGoal(HIGH)`.
- Operator override: a binding asks a mechanism directly; being the newer ask, it wins. The
  superstructure keeps asking, sees `atGoal(itsState)` false, and does not advance. When the
  override ends the mechanism goes back to the superstructure's ask by itself.
- Loop order: OpMode, scheduler, superstructure, then each mechanism's apply. The superstructure
  reads last loop's `atGoal`, so it reacts a loop after a mechanism arrives. Accept that and say
  so; do not add a second pass.
- Drive limits from mechanisms (a raised elevator slows the robot) go through
  `Swerve.limitDrive`, asked every loop, as now.
- Logged as any `WantedState`, under `SuperStructure/State/`.

Done when: a sim script presses a score button, the log shows the superstructure and both
mechanisms step through their states in order, an override mid-sequence holds it, and letting go
resumes it; replay matches.

## Next: values that belong to their asker

A wanted state often carries a number: a height, a flywheel RPM, a velocity. Today the number is
kept by the subsystem apart from who asked (`Swerve.driveRobotRelative` writes `robotVx`,
`robotVy`, `robotOmega` fields; `WantedState` holds only the enum). Two requesters asking for
`ROBOT_VELOCITY` at different speeds: the newer wins the state, but the speed is whichever wrote
last in the loop.

- An ask carries its value: `request(state, value, requester)`, the value stored in that asker's
  slot, so the winner's value is the one used and an older asker's value comes back with it.
- Values are preallocated per slot (a `double[]` of a fixed width the subsystem declares), so
  asking every loop does not allocate.
- Logged: `<prefix>/State/WantedValue` (the winner's, units in the key's comment) and the value
  in each `Requests` entry.
- `Swerve.driveRobotRelative` moves onto it, so the drivetrain shows the pattern before a
  mechanism uses it.

Done when: a unit test with two askers at different values gets the newer's value, then the
older's after the newer lets go, and Swerve's robot-relative drive uses it with sim and replay
unchanged.

## Next: safety limits that hold through overrides

The superstructure's order of moves makes the normal path smooth, but an operator override asks a
mechanism directly and skips it. Collisions are stopped one layer down, where setpoints are sent.

- Each mechanism clamps its setpoint by limits that read the other mechanisms' measured positions
  (logged inputs): "arm no further out than X while the elevator is under Y", whoever asked.
- A clamped setpoint is reported: `<prefix>/Limit/Active`, `Limit/Reason` (which limit),
  `Limit/AskedValue`, `Limit/SentValue`, and `atGoal` false with reason `"limited"`.
- If a position a limit depends on is not connected, the limit falls back to the conservative but
  usable range (the arm can still move inside the frame), never a stop, and alerts.
- Limits are Robot Settings where they are geometry (calibration), checked against hard stops.
- Unit tests drive every limit with both mechanisms' positions, including the disconnected case.
- One mechanism: its own soft range and the disconnected fallback. Two mechanisms: the limits
  between them.

## Next: states when a sensor is not connected

AGENTS.md: never act on a device's readings while its `connected` input is false. Each mechanism
says what its transitions and `atGoal` do then, so a dead sensor never leaves it stuck "not at
goal" with the superstructure waiting on it.

- Per mechanism, one of: judge arrival by time (the move's measured worst case plus margin, a
  Robot Setting), use another reading (the motor's own encoder for a dead external one), or hold
  a known-safe setpoint. Written in the mechanism's README.
- `atGoal` true by a fallback is still reported: `<prefix>/State/GoalBy` (`"sensor"`, `"time"`,
  `"motorEncoder"`) and an alert while a fallback is in use.
- A sim script unplugs each mechanism sensor mid-sequence (`SimFaults`) and checks the sequence
  still finishes; added to `tools/sim_check.sh`.

## Next: homing and boot state

Every mechanism on a relative encoder boots not knowing where it is. FM wrote `HoodHoming` once
by hand; this is the shared pattern.

- A `NOT_ZEROED` current state at boot. Homing runs on first enable (or on request): drive
  gently to the stop, detect it (current and stall velocity, debounced), set the position, then
  `ZEROED`.
- Homing has a timeout (Robot Setting). On timeout: alert, assume the boot position is the zero,
  and keep working in a reduced range (the safety limits narrow) rather than refusing to move.
  Tried again only when the asker lets go and asks again (the no-retry rule).
- `atGoal` is false with reason `"notZeroed"` until zeroed, unless running on the assumed zero.
- The position write is queued on `PhoenixBackground` (no blocking Phoenix calls after boot), and
  `ZEROED` waits for its result.
- Code restart mid-match: each mechanism's offset is saved to `/dev/shm` the way `FieldHeading`
  saves the heading (age, checksum, motor not moved far since), and restored at boot, so a crash
  does not mean homing again in the middle of a match. A power cycle starts fresh.
- Logged: `<prefix>/Homing/State`, `Homing/Seconds`, `Homing/Result`, `Homing/Restored`.
- Pre-match check: a mechanism that homes when enabled is fine; one that timed out is flagged
  (FM's `HomingCheck`).

## Next: mode changes

What each mechanism and the superstructure do on disable, enable, and auto to teleop, decided
once and written down here rather than per mechanism.

- Disabled: motors are off whatever is asked; current states keep tracking what the mechanism
  actually does (a falling arm is reported), so enabling starts from the truth.
- Enable: each mechanism starts its current state from where it is, not from where it was last
  asked to be; one away from its asked position moves there through its normal sequence.
- Auto to teleop: auto's asks stop with auto. The superstructure keeps the robot's knowledge
  (holding a piece, zeroed) and starts teleop in the state that matches it (`HOLDING`, not
  `STOWED`). Nothing homes again.
- A Driver Station dropout mid-match (disable, then enable) keeps knowledge and zeros as well.
- Logged: an event per mode change with each mechanism's current state.
- A sim script runs auto into teleop holding a piece and drops the Driver Station once.

## Next: latched and exclusive asks

With asks lasting one loop and the newest winning, two kinds of ask need a stated way to work.

- Latched (a toggle, a one-press action that runs to its end): a command that keeps asking until
  it is done or pressed again, or a superstructure state that holds itself. Never a one-off ask
  relying on nobody else asking.
- Exclusive (a climb that has started, a scoring motion past the point of no return): the asker
  marks the ask exclusive; while it is asking, newer asks are logged as refused
  (`<prefix>/State/Refused`, with requester) instead of winning. The pilot keeps an explicit way
  out (a held abort binding that is itself let through), so it is never a trap.
- Unit tests: a newer ask refused during an exclusive one, the abort let through, the exclusive
  one ending and the others taking over.

## Next: robot knowledge as logged state

Superstructure transitions depend on what the robot knows, not only on what was asked: holding a
piece, how many, which kind, zeroed.

- A `RobotKnowledge` (in `frc.robot`) updated once a loop from logged inputs, logged as its own
  keys (`Knowledge/HasPiece`, `Knowledge/PieceCount`, `Knowledge/Source`).
- Each fact has a fallback when its sensor fails: a dead beam break falls back to intake current
  and time, then to "assume held after an intake that ran long enough". `Knowledge/Source` says
  which is in use, with an alert. A driver binding can correct it.
- Autos wait on it the same way they wait on `atGoal`: with a timeout, then carry on with the
  next step (an auto never stops for a missed piece).

## Next: smaller pieces with the first mechanism

- Dashboard: one view per subsystem of wanted, current, requester, transition reason and
  not-at-goal reason (an AdvantageScope layout checked into the repo), so the pit sees why a
  mechanism is not moving.
- Current limits per state scaled by battery voltage (lower limits when the battery sags, a Robot
  Setting curve), applied through `PhoenixBackground` only when they change. This is a hard stop:
  the PR states each limit change and its failure mode.
- A sim test that steps each mechanism through every state from every state it can reach, with
  scripted inputs, and fails if any state has no way out or none reaches its goal.

## Planned: per-state timing

Cheap and useful for match review ("spin-up took 0.4 s every shot but one").

- On leaving a current state: `<prefix>/State/Stats/<STATE>/Count`, `LastSeconds`,
  `AverageSeconds`, `MaxSeconds`.
- Optional expected window per state (code constants: they change no behavior). A visit outside
  it counts in `IrregularCount`; `<prefix>/State/Overtime` is true while the current state runs
  past its max. Never changes the flow; that is what timeouts are for.
- Keys built once per enum value at construction.

## Planned: pit self-test

A utility OpMode asks each mechanism for a `SELF_TEST` state: drive each motor briefly, check it
answers, draws sensible current and moves. Because only the utility OpMode asks, nothing fights
it. On exit the mechanism starts from its boot state again. Results logged under
`SelfTest/<subsystem>/` and raised as alerts. Fits next to `WheelRadiusUtility`.

## Planned: state-driven LEDs and rumble

`Leds` reads mechanisms' current states and `atGoal` (logged inputs to it), rather than commands
setting colors. A small helper, `inState(Current...)`, as a `BooleanSupplier` for triggers.

## Maybe: state diagrams

Ciprix logs a Mermaid diagram generated from its builder. A switch cannot be read that way, so
instead each mechanism's README carries a hand-written Mermaid diagram of its states, updated in
the same change as the switch. Revisit if diagrams keep drifting from the code.

## Not doing

- A generic transition-table or builder state machine class: see the switch rule above.
- Subsystem default commands that copy a request from the superstructure (Ciprix's relay): each
  hop through the scheduler costs a loop. The superstructure asks directly.
- Annotation-processor generated command factories.
- A framework base robot class or subsystem `periodic()` that reads its own inputs: our
  `LoopStage` order and fault handling stay in `Robot`.
- Stop-on-fault "security" states.

## Order

Before the first mechanism, on the drivetrain:

1. Values that belong to their asker (Swerve's robot-relative drive moves onto it).

With the first mechanism, before it goes on a robot:

2. Current state machine, `StateClock`, transition reasons, `atGoal`.
3. Homing and boot state, including restore after a code restart.
4. States when a sensor is not connected; the mechanism's own safety range.
5. `Debounce`, timeouts, fallbacks, no-retry.
6. Latched and exclusive asks.
7. Mode changes.
8. Dashboard view, battery-scaled current limits, the stuck-state sim test.

With the second mechanism:

9. Safety limits between mechanisms.
10. Robot knowledge and the superstructure.

Any time, small: per-state timing. When there are mechanisms to test and show: self-test, LEDs
from state.

Each is its own PR with unit tests, a sim script and a replay check.
