# battery

Which battery is in the robot, from an NFC sticker on each battery and a PN532 reader on USB
serial (CH340). Ported from [SpectrumBatteryLogger](https://github.com/Spectrum3847/SpectrumBatteryLogger)
at 3b351b1 (the driver files are its code with the package changed; fix bugs there too).

| File | What |
|---|---|
| `BatteryTag` | Main-loop logic: logs the inputs, names the battery (tag name, list, UID), counts swaps, alerts |
| `BatteryTagIO` | Where readings come from; `fixed(uid, name)` for sim, `NONE` for replay |
| `BatteryTagIOSerial` | The real reader on a low-priority daemon thread: reads at boot, then every 2 s while disabled; reports and retries a missing reader |
| `BatteryTagInputs` | Logged inputs under `Battery/`: reader connected, tag present, UID, tag name, reads, error |
| `BatteryNames` | The team list `UID,name` (deploy `batteries.csv`) for tags with no name written on them |
| `Pn532` | Minimal PN532 driver over HSU (UART): find one ISO14443A tag, read an NTAG's name |
| `Pn532Transport` | Byte pipe to the reader (serial, or a fake in tests) |
| `SerialTransport` | `Pn532Transport` over jSerialComm; finds the CH340 port |
| `TagInfo` | One tag: UID, ATQA, SAK, name |
| `NdefText` | Reads the first NDEF Text record (the name a phone app writes) from an NTAG |
