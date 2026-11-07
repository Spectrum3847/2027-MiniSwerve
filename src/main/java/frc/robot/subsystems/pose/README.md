# pose

Where the robot is on the field.

| File | What |
|---|---|
| `Localization` | TagEKF drives the pose from swerve samples and camera frames; WPILib's estimator runs beside it as a logged shadow; seeds from the first two-tag frame |
| `PoseSettings` | TagEKF's noise model and adaptive uncertainty as Robot Settings (`tagekf/*`) |
