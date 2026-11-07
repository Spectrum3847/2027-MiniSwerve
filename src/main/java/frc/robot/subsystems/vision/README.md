# vision

The robot's cameras.

| File | What |
|---|---|
| `Vision` | Reads, logs and decodes each loop's frames, counts dropped and bad frames, alerts on a silent camera, and hands frames to the pose estimator |
| `VisionSettings` | Which cameras are mounted and where (Robot Settings `vision.<camera>/*`), and the field (tag layout) |
