"""Converts the given text files to UTF-8 with LF endings. Called by tools/fix-text.sh."""

import sys

BINARY = (".jar", ".glb", ".png", ".jpg", ".wpilog", ".hoot")
CRLF_OK = (".bat", ".cmd")

for path in sys.argv[1:]:
    if path.endswith(BINARY):
        continue
    data = open(path, "rb").read()
    if b"\0" in data:
        continue
    try:
        text = data.decode("utf-8")
    except UnicodeDecodeError:
        text = data.decode("cp1252")
    text = text.replace("\r\n", "\n").replace("\r", "\n")
    if path.endswith(CRLF_OK):
        text = text.replace("\n", "\r\n")
    new = text.encode("utf-8")
    if new != data:
        open(path, "wb").write(new)
        print("fixed", path)
