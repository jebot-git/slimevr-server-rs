#!/usr/bin/python3
"""Bounded systemd ExecStartPost readiness check for local Unix listeners."""
import socket
import sys
import time


def main():
    pending = set(sys.argv[1:])
    if not pending:
        raise SystemExit("supply at least one socket path")
    deadline = time.monotonic() + 15
    while pending and time.monotonic() < deadline:
        for path in tuple(pending):
            try:
                with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
                    client.settimeout(0.2)
                    client.connect(path)
                pending.remove(path)
            except OSError:
                pass
        if pending:
            time.sleep(0.05)
    if pending:
        raise SystemExit("listeners did not become ready: " + ", ".join(sorted(pending)))


if __name__ == "__main__":
    main()
