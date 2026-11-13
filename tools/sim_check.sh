#!/usr/bin/env bash
# Runs the sim scripts that prove the robot keeps driving through faults, headless, and checks
# each log (tools/sim_check.py). CI runs this; so does tools/check.sh.
#   tools/sim_check.sh            all of them
#   tools/sim_check.sh guards     one
set -euo pipefail
cd "$(dirname "$0")/.."
py=$(command -v python || command -v python3)
scripts=("$@")
if [ ${#scripts[@]} -eq 0 ]; then scripts=(guards driver red faults); fi
for script in "${scripts[@]}"; do
  out="build/simcheck/$script"
  rm -rf "$out"
  mkdir -p "$out"
  echo "sim_check: running $script"
  start=""
  # The red script starts facing away from the red driver, clear of the walls.
  if [ "$script" = "red" ]; then start="10,4,180"; fi
  MINISWERVE_SIM_SCRIPT="$script" MINISWERVE_SIM_LOG_DIR="$out" SPECTRUM_SIM_START="$start" \
    sh ./gradlew run -PnoSimGui -q > "$out/run.txt" 2>&1 || { tail -40 "$out/run.txt"; exit 1; }
  log=$(ls "$out"/*.wpilog | grep -v _replay | head -1)
  "$py" tools/sim_check.py "$script" "$log"
done
echo "sim_check: all passed"
