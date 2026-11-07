# tools

| File | What |
|---|---|
| `check.sh` | Runs everything CI runs, locally; the pre-push hook calls it |
| `text_check.py` | Reports text files that are not UTF-8 or have CR line endings |
| `fix-text.sh` / `fix_text.py` | Rewrites tracked text as UTF-8 + LF and renormalizes git |
