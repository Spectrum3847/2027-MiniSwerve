# robotsettings

| File | What |
|---|---|
| `RobotSettings` | Entry point: `group`, `start` at boot, `periodic` every loop, save handling |
| `SettingGroup` | Declares settings of one group (one JSON file) |
| `Setting` | Base class: value, default, source layer, version |
| `DoubleSetting` `IntSetting` `BooleanSetting` `StringSetting` | Typed settings with bounds |
| `SettingStore` | Reads layers, validates, applies live values, prepares saves (to be split; over 400 lines) |
| `SettingSource` | Which layer a value came from |
| `RobotSettingsInputs` | AdvantageKit inputs: every value and its source |
| `SettingTransport` / `NetworkTablesTransport` | Where live tuning comes from |
| `HardwareStamp` | Ties a hard-stop group's local file to this robot's hardware |
| `TuningMode` | Global switch for live tuning |
