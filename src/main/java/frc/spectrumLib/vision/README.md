# vision

Camera frames from the coprocessor, the robot side. The coprocessor (Spectrum's PhotonVision fork)
sends only what the robot uses, as a compact versioned binary frame, not PhotonLib's results.

| File | What |
|---|---|
| `VisionFrame` | The wire format (v1): header + per-tag bearing, range, margin, corners, or per-object class, confidence, bearing, box; encode and allocation-free decode. Copy this file to the fork |
| `VisionIO` | Where frames come from; `NONE` for replay |
| `VisionIONetworkTables` | Every frame each camera published on `/spectrum/vision/<camera>/frames` since the last loop |
| `VisionInputs` | Logged inputs: each camera's raw frame bytes this loop, so replay decodes exactly what the robot did |
