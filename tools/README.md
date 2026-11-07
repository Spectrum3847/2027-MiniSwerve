# tools

| File | What |
|---|---|
| `check.sh` | Runs everything CI runs, locally; the pre-push hook calls it |
| `text_check.py` | Reports text files that are not UTF-8 or have CR line endings |
| `wpilog_read.py` | Dependency-free .wpilog reader; `at()` aligns delta-logged keys by time |
| `fm_extract.py` | Real FM match window to CSV (commands, measured states, accel, currents) for sim replay |
| `sim_vs_real.py` | Scores the sim against a real robot on the same commands |
| `fault_summary.py` | Fault-scenario summary: faults, module modes, motion and odometry per step; exit 0 if all recovered |
| `swerve_summary.py` | Sim drive summary: odometry vs truth, thread rate, drops |
| `fix-text.sh` / `fix_text.py` | Rewrites tracked text as UTF-8 + LF and renormalizes git |
