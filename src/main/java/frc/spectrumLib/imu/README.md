# imu

A second IMU read alongside the drivetrain's Pigeon 2, for comparison before it can replace it.

| File | What |
|---|---|
| `ImuIO` | Where readings come from; `NONE` for sim and replay |
| `ImuIOGoku` | Flywoo Goku F7 (GokuIMU firmware) over USB through its vendordep; found by its USB product name, reconnects on its own |
| `ImuInputs` | Logged inputs under `GokuIMU/`: connected, calibrated, moving, yaw/pitch/roll, yaw rate, frame time, temperature, status word, fault bits, device info, error |
