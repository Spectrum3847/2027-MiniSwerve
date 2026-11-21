# leds

The SpectrumShine LED controller (Spectrum's own board, [SpectrumShine](https://github.com/Spectrum3847/SpectrumShine))
on CAN. The board runs the animations; the robot only says what each of its eight slots shows.

| File | What |
|---|---|
| `ShineIO` | One board: `setSlot` what a slot shows, `send` what is due; `NONE` for replay, `simulated()` for sim |
| `ShineIOCan` | The board through WPILib's `CAN`: sends changes, repeats every slot each second, sets LED counts and reads the version when the board appears, sends the robot clock once a second |
| `ShineInputs` | Logged inputs under `Leds/`: connected, faults, current limit reason, temperature, output scale, battery and 5 V rail, LED current, firmware, boots, frame counts |
| `ShineProtocol` | The CAN protocol: API numbers, frame packing, fault names (a copy of the firmware's `proto.h`) |
| `ShineAnimation` | The board's animations and their wire ids (a copy of the firmware's `anim.h`) |
| `LedPattern` | What a slot shows: animation, two colors, speed, param, direction |
| `LedColor` | A color (r, g, b, w) and Spectrum's palette |
