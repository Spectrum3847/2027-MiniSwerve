# spectrumSim.field

The field the sim robot drives on, and what its bumpers hit.

| File | What |
|---|---|
| `Arena` | Walls and field elements as fixed outlines; pushes a robot's bumpers out of them each step with a padded bounce and sliding friction |
| `ConvexPolygon` | A convex outline in the field frame (counter-clockwise); a box can be moved in place each step |
| `Contact` | Separating-axis test between two outlines: normal, depth, touch point (no allocation) |
