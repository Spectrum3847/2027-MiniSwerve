# 2027-MiniSwerve

Spectrum 3847's mini swerve training robot, and the testbed for our 2027 competition code.

| | |
|---|---|
| Controller | SystemCore image on a Raspberry Pi |
| WPILib | `2027.0.0-alpha-8`, Java 25 |
| Logging / replay | Spectrum AdvantageKit fork |
| Motors | Phoenix 6 Pro: 8 Falcon 500, 4 CANcoder, Pigeon 2 |
| Drive | 4 SDS MK4i L2, custom swerve (no CTRE swerve API) |
| Vision | 2 USB cameras on our PhotonVision fork, TagEKF |

Start with [AGENTS.md](AGENTS.md) and [docs/architecture.md](docs/architecture.md).

```bash
./gradlew build                 # format, compile, test
./gradlew run                   # sim with GUI
./gradlew run -PnoSimGui        # headless sim, writes logs/
```

CAN IDs: module N has drive N, steer 10+N, CANcoder N. Pigeon is 1. One bus.
