# spectrumSim.vision

Cameras on the simulated robot.

| File | What |
|---|---|
| `CameraSim` | Sees the field's AprilTags from the true pose through a pinhole camera, with pixel and range noise, frame rate and latency on the sim clock, and publishes `VisionFrame`s on the same topic the coprocessor uses |
