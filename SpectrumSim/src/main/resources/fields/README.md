# Field terrain files

The floor's bumps and ramps for the sim (`frc.spectrumSim.field.Terrain`), one JSON file per
field. Pick one with `SPECTRUM_SIM_TERRAIN=<name>` (no `.json`), or give a path to a file outside
the jar. A new game's field is a new file here, not new code.

| File | What |
|---|---|
| `2026-rebuilt.json` | 2026 REBUILT: four BUMPs either side of the two HUBs, from the Game Manual; positions still to check against the field drawings and CAD |

Each entry in `bumps` is a box footprint whose top rises from both ends at `rampDeg` and stops at
`height`: `x`, `y` (center, m, WPILib field frame, blue alliance wall at x = 0), `length` (m,
along `headingDeg`), `width` (m, across), `height` (m), `rampDeg`, `headingDeg` (default 0, ramps
face +x and -x). A ridge, a ramped platform or a single ramp (one long bump cut by a wall) all fit.
Steps too steep to drive belong in `Arena` as walls.
