# power

The power distribution board (CTRE PDP or REV PDH), read through WPILib.

| File | What |
|---|---|
| `PowerDistributionIO` | Where readings come from; `NONE` for sim and replay |
| `PowerDistributionIOWpilib` | The board through WPILib's `PowerDistribution` (decodes its own frames, no Phoenix); opened once at boot, readings read 0 with a reason when missing |
| `PowerDistributionInputs` | Logged inputs under `PowerDistribution/`: connected, voltage, total current, temperature, per-channel currents, firmware, error |
