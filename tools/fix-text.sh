#!/usr/bin/env bash
# Rewrites every tracked text file as UTF-8 with LF endings (.bat/.cmd stay CRLF) and renormalizes
# git's index. Run it when tools/check.sh reports line-ending or encoding problems.
set -euo pipefail
cd "$(dirname "$0")/.."
git ls-files -z | xargs -0 python tools/fix_text.py
git add --renormalize .
echo "fixed; review with git status / git diff"
