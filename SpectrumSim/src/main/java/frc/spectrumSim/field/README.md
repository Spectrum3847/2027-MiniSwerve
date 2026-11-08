# spectrumSim.field

The field the sim robot drives on, and what its bumpers hit.

| File | What |
|---|---|
| `Arena` | Walls, field elements and other robots; each step pushes every robot's bumpers out of what they overlap, with a padded bounce and sliding friction solved over up to two touch points |
| `SimRobot` | Another robot (partner or opponent): traction-limited drive toward what its behavior wants (parked, shuttle, defender) |
| `ConvexPolygon` | A convex outline in the field frame (counter-clockwise); a box can be moved in place each step |
| `Terrain` | The floor's height: bumps and ramps from a field file (`src/main/resources/fields/<name>.json`), so a new game is a new file |
| `Contact` | Separating-axis test between two outlines: normal, depth, up to two touch points (no allocation) |
