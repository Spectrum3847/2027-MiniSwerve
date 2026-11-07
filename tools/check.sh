#!/usr/bin/env bash
# Runs what CI runs, locally. The pre-push hook calls it; a push only goes out if it passes.
set -euo pipefail
cd "$(dirname "$0")/.."
if [ -z "${JAVA_HOME:-}" ]; then
  JAVA_HOME=$(ls -d /c/Users/Public/wpilib/2027*/jdk "$HOME"/wpilib/2027*/jdk 2>/dev/null | sort | tail -1 || true)
  export JAVA_HOME
fi
echo "check: workflow YAML"
for f in .github/workflows/*.yml; do
  python -c "import sys,yaml; yaml.safe_load(open(sys.argv[1], encoding='utf-8'))" "$f"
done
echo "check: line endings and encoding"
# Stored files must be LF (.bat/.cmd excepted) and text must be UTF-8. A CRLF or Windows-codepage
# byte here is what breaks Spotless and diffs on other machines; tools/fix-text.sh repairs both.
bad_eol=$(git ls-files --eol | awk '$1 ~ /crlf|mixed/ {print $NF}' | grep -vE '\.(bat|cmd)$' || true)
if [ -n "$bad_eol" ]; then echo "CRLF stored in git (run tools/fix-text.sh):"; echo "$bad_eol"; exit 1; fi
bad_utf8=$(git ls-files -z | xargs -0 python tools/text_check.py)
if [ -n "$bad_utf8" ]; then echo "$bad_utf8"; exit 1; fi
echo "check: TUNING_MODE off"
grep -Eq '^ *public static final boolean TUNING_MODE = false;' src/main/java/frc/robot/RunMode.java
echo "check: no CTRE swerve API"
! grep -rnE "com\.ctre\.phoenix6\.swerve" src
echo "check: spotless + build + tests"
./gradlew spotlessCheck build -q
echo "check: all passed"
