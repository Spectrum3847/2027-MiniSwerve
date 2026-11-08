# Field files

One season's field for the sim (`frc.spectrumSim.field.FieldMap`): its size, the floor's bumps and
ramps, and the solid elements bumpers hit. Pick one with `SPECTRUM_SIM_FIELD=<name>` (no `.json`),
or give a path to a file outside the jar. A new game is a new file here, not new code.

| File | What |
|---|---|
| `2026-rebuilt.json` | 2026 REBUILT, welded perimeter: four BUMPs, two HUBs, four TRENCH posts, from the FE-2026 rev B drawings |
| `2026-rebuilt-andymark.json` | The same on the AndyMark perimeter (about 1 in shorter and narrower, HUBs 0.55 in closer to the walls) |

Format (`units` is `m` or `in`, for lengths only; angles are degrees):

- `length`, `width`: the field, alliance wall to alliance wall and guardrail to guardrail.
- `bumps`: each a box footprint whose top starts `edge` above the carpet at both ends, rises at
  `rampDeg` and stops at `height`: `x`, `y` (center, WPILib field frame, blue alliance wall at
  x = 0), `length` (along `headingDeg`, default 0: ramps face +x and -x), `width` (across),
  `height`, `rampDeg`, `edge` (default 0). A ridge, a ramped platform or a step all fit.
- `elements`: solid boxes bumpers hit: `x`, `y`, `length`, `width`, `headingDeg` (default 0).
  Steps too steep to drive belong here too.

Not in the 2026 files yet: towers, depots (1 in barriers), outposts, and the trench arms (22.25 in
clearance: a robot taller than that would hit them). Source drawings and CAD: FIRST's playing field
page (FE-2026 field dimension drawings, GE-26100 BUMP, full field STEP).
