#!/usr/bin/env python3
"""Exec a child after unblocking graceful-shutdown signals.

``tracker_producer`` briefly blocks SIGINT/SIGTERM across ``Popen`` so it can
publish the child PID before its own shutdown handler runs. Signal masks are
inherited across fork/exec, so the intermediate process must explicitly
unblock those two signals before replacing itself with ``live_radio``.
"""

from __future__ import annotations

import os
import signal
import sys


def main(argv: list[str]) -> int:
    if len(argv) < 2:
        print("usage: exec_unblocked.py ABSOLUTE_PROGRAM [ARG ...]", file=sys.stderr)
        return 64
    program = argv[1]
    if not os.path.isabs(program):
        print("exec_unblocked.py: program path must be absolute", file=sys.stderr)
        return 64
    if not hasattr(signal, "pthread_sigmask"):
        print("exec_unblocked.py: pthread_sigmask unavailable", file=sys.stderr)
        return 78
    shutdown_signals = {signal.SIGINT, signal.SIGTERM}
    signal.pthread_sigmask(signal.SIG_UNBLOCK, shutdown_signals)
    current = signal.pthread_sigmask(signal.SIG_BLOCK, set())
    if current.intersection(shutdown_signals):
        print("exec_unblocked.py: shutdown signals remain blocked", file=sys.stderr)
        return 78
    os.execve(program, argv[1:], os.environ)
    return 70  # pragma: no cover - execve returns only by raising


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
