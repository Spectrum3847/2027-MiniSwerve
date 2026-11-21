# leds

The LED strip: 16 LEDs across the middle of the robot, facing up, on a SpectrumShine board (CAN id
`Hardware.LED_ID`). Purple and white mean fine; yellow a warning, orange the battery, red a fault.

| File | What |
|---|---|
| `Leds` | Picks what the strip shows each loop (mode in the middle, trouble at the ends), alerts on a missing or faulted board |
| `LedSettings` | Robot Settings `leds/*`: LED count, end LEDs, brightness, which board output |

| Middle of the strip | Meaning |
|---|---|
| Solid red | E-stopped |
| Orange flashing, whole strip | Disabled and the resting battery is low: swap it |
| Purple eye sweeping | Disabled, no Driver Station |
| Purple breathing | Disabled, Driver Station connected |
| White dots marching on purple | Autonomous |
| Purple and white halves swapping | Utility OpMode |
| White band flowing through purple | Teleop |

Three LEDs at each end (`leds/endLeds`) light only when something needs attention; the mode pattern
then keeps to the middle.

| Ends of the strip | Meaning |
|---|---|
| Red fast flash | A high-level alert is active |
| Orange flash | Enabled on a battery that rested low |
| Yellow slow blink | A medium-level alert is active |
