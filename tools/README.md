# tools

| File | What |
|---|---|
| `check.sh` | Runs everything CI runs, locally; the pre-push hook calls it |
| `text_check.py` | Reports text files that are not UTF-8 or have CR line endings |
| `wpilog_read.py` | Dependency-free .wpilog reader; `at()` aligns delta-logged keys by time |
| `fm_extract.py` | Real FM match window to CSV (commands, measured states, accel, currents, battery, yaw rate) for sim replay |
| `sim_vs_real.py` | Scores the sim against a real robot on the same commands |
| `fit_sim.py` | Fits sim params (mass, turning inertia, latency, ...) to real logs by coordinate descent over `simReplay` sweeps |
| `fit_timing.py` | Fits CAN sample spacing, failed reads, camera latency, battery voltage over a match and main loop time from real logs |
| `battery_fit.py` | Fits battery open-circuit voltage and resistance from logged battery voltage and total current |
| `sim_check.sh` / `sim_check.py` | Runs the fault sim scripts (guards, driver, red, faults) headless and checks each log: the robot must keep driving; CI and `check.sh` run it |
| `fault_summary.py` | Fault-scenario summary: faults, module modes, motion and odometry per step; exit 0 if all recovered |
| `swerve_summary.py` | Sim drive summary: odometry vs truth, thread rate, drops |
| `fix-text.sh` / `fix_text.py` | Rewrites tracked text as UTF-8 + LF and renormalizes git |
