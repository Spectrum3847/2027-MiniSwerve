# localization

TagEKF and the pieces around it. Copied unchanged from FM's tested `frc.spectrumLib.localization`
(2026-FM-SystemCore `tagekf-replay-tuner` 47f14a9, via SpectrumJetson `robot-ekf` on `sc/tagekf`),
where it was tuned on the TXMAN1 replays (held-out tags p50/p90 0.057/0.136 m on Q17). Port changes
back by hand.

| File | What |
|---|---|
| `TagEkf` | EKF over (x, y, heading): predicts on every odometry sample, corrects per tag (bearing and range from its camera) at capture time with history replay, chi-squared gate, relocalization, adaptive uncertainty, fixed-lag smoother |
| `TagSighting` | One tag in one frame: camera, capture time, tag's field position, mount, camera-to-tag vector |
| `CornerSighting` | One tag as corner pixels (for the corner-reprojection path; worse than bearing/range in replay) |
| `MountEstimator` | Online camera mount yaw/pitch from tags |
| `OdometryGuard` | Flags odometry that disagrees with the gyro or itself (slip, bumps) |
