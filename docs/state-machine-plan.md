# State machine plan

What the wanted-state framework still needs before the 2027 robot's mechanisms and superstructure
are built on it, and in what order. Status marks: **built**, **next**, **planned**, **maybe**.
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

1. Current state machine, `StateClock`, transition reasons, `atGoal`: with the first mechanism.
2. `Debounce`, timeouts, fallbacks, no-retry: same mechanism, before it goes on a robot.
3. Superstructure: when there are two mechanisms that must move in order.
4. Per-state timing: any time, small.
5. Self-test, LEDs from state: when there are mechanisms to test and show.

Each is its own PR with unit tests, a sim script and a replay check.
