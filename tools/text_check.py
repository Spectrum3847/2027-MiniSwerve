"""Prints each text file that is not valid UTF-8 or has CR line endings in the working tree.

Used by tools/check.sh. Binary files (by .gitattributes or a NUL byte) are skipped.
"""

import sys

BINARY = (".jar", ".glb", ".png", ".jpg", ".wpilog", ".hoot")
CRLF_OK = (".bat", ".cmd")

for path in sys.argv[1:]:
    if path.endswith(BINARY):
        continue
    try:
        data = open(path, "rb").read()
    except OSError:
        continue
    if b"\0" in data:
        continue
    try:
        data.decode("utf-8")
    except UnicodeDecodeError as e:
        print(f"{path}: not UTF-8 at byte {e.start} (run tools/fix-text.sh)")
    if b"\r" in data and not path.endswith(CRLF_OK):
        print(f"{path}: CR line endings in working copy (run tools/fix-text.sh)")
