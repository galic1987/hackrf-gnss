#!/usr/bin/env python3
"""Atomic, fail-closed ownership protocol for the production HackRF Pro.

There are two independent objects in ``observations``:

``maintenance.lock``
    An atomically-created JSON gate.  Once present, no normal producer may
    acquire the radio lease.  Creating the gate does not evict the current
    owner; it prevents a new owner from racing in while that owner is being
    stopped.

``pro.radio.lock.d/owner.json``
    The exclusive radio lease.  Directory creation is the atomic operation.
    A client checks the maintenance gate both before and after creating the
    directory, closing the gate-vs-acquire race.

Locks are never reclaimed from PID liveness.  A crash therefore fails closed:
an operator must inspect the radio and the JSON owner before removing the
exact stale object during a maintenance window.

The CLI is deliberately three-phase so a runbook can gate first, stop the
tracker second, and acquire the now-free radio third::

    python3 scripts/pro_lease.py gate --serial 0000000000000000645061de252d6613 \
        --token-file /tmp/pro-maint.token
    # gracefully stop tracker_producer/live_radio here
    python3 scripts/pro_lease.py acquire --token-file /tmp/pro-maint.token --wait-seconds 30
    # perform the pre-staged maintenance operation
    python3 scripts/pro_lease.py release --token-file /tmp/pro-maint.token

``release`` removes the radio lease before the gate, so a partial release
still blocks producers.  Token mismatches and malformed state are fatal.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import uuid


# Production clients must not be able to diverge onto separate lock roots via
# inherited service environments. Tests pass an explicit temporary ``obs``.
DEFAULT_OBS = Path("/Volumes/Radiator 8TB/gnss/observations")
GATE_NAME = "maintenance.lock"
LEASE_DIR_NAME = "pro.radio.lock.d"
OWNER_NAME = "owner.json"
PROTOCOL = 1
PRODUCTION_SERIAL = "0000000000000000645061de252d6613"


class ProLeaseError(RuntimeError):
    """Base class for a fail-closed lease failure."""


class ProLeaseUnavailable(ProLeaseError):
    """The gate or another owner currently excludes this client."""


class ProLeaseProtocolError(ProLeaseError):
    """Persistent lease state is malformed or cannot be changed safely."""


def _paths(obs: Path):
    obs = Path(obs)
    lease_dir = obs / LEASE_DIR_NAME
    return obs / GATE_NAME, lease_dir, lease_dir / OWNER_NAME


def _read_json(path: Path):
    try:
        data = json.loads(path.read_text())
    except Exception as exc:
        raise ProLeaseProtocolError(f"cannot read valid JSON from {path}: {exc}") from exc
    if not isinstance(data, dict):
        raise ProLeaseProtocolError(f"expected a JSON object in {path}")
    return data


def _write_exclusive(path: Path, data: dict):
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
    try:
        fd = os.open(path, flags, 0o600)
    except FileExistsError:
        raise
    except OSError as exc:
        raise ProLeaseProtocolError(f"cannot create {path}: {exc}") from exc
    try:
        with os.fdopen(fd, "w") as f:
            json.dump(data, f, sort_keys=True)
            f.write("\n")
            f.flush()
            os.fsync(f.fileno())
    except Exception:
        try:
            path.unlink()
        except OSError:
            pass
        raise


def _gate_present(gate: Path) -> bool:
    # Any filesystem object with this name is a gate.  A malformed gate must
    # block clients rather than being interpreted as "maintenance finished".
    return gate.exists() or gate.is_symlink()


def maintenance_gate_active(obs: Path) -> bool:
    """True for any gate object, including malformed/dangling state."""
    gate, _lease_dir, _owner_path = _paths(Path(obs))
    return _gate_present(gate)


def _require_production_serial(serial: str):
    if not isinstance(serial, str) or serial.lower() != PRODUCTION_SERIAL:
        raise ProLeaseProtocolError(
            f"lease serial must be the exact production Pro {PRODUCTION_SERIAL}")
    return PRODUCTION_SERIAL


def python_script_pids_from_ps(text: str, script_basename: str,
                               self_pid: int) -> list[int]:
    """Return real Python owners of one script from a macOS ``ps`` snapshot.

    A parent ``/bin/bash -c ... script.py`` contains the child's complete
    launch text, so command-text tools such as ``pgrep -f`` cannot safely be
    used as a signal target.  Require a Python executable and exclude the
    caller plus its complete ancestor chain.  Malformed input is a protocol
    error, never an empty (apparently safe) process list.
    """
    expected = os.path.basename(script_basename)
    if not expected or expected != script_basename:
        raise ProLeaseProtocolError(
            f"script selector must be a basename, got {script_basename!r}")
    rows = {}
    for line in text.splitlines():
        if not line.strip():
            continue
        fields = line.strip().split(None, 3)
        if len(fields) < 3:
            raise ProLeaseProtocolError("malformed process snapshot")
        try:
            pid, ppid = int(fields[0]), int(fields[1])
        except ValueError as exc:
            raise ProLeaseProtocolError("non-numeric pid in process snapshot") from exc
        if pid in rows:
            raise ProLeaseProtocolError(f"duplicate pid {pid} in process snapshot")
        rows[pid] = (ppid, fields[2], fields[3] if len(fields) == 4 else "")

    excluded = set()
    pid = self_pid
    while pid and pid not in excluded:
        excluded.add(pid)
        pid = rows.get(pid, (0, "", ""))[0]

    matches = []
    for pid, (_ppid, comm, args) in rows.items():
        if pid in excluded:
            continue
        executable = os.path.basename(comm).lower()
        if not executable.startswith("python"):
            continue
        argv = [part.strip("'\"") for part in args.split()]
        if any(os.path.basename(part) == expected for part in argv):
            matches.append(pid)
    return sorted(matches)


def running_python_script_pids(script_basename: str, timeout: float = 5.0):
    """Query real Python script processes or fail closed on query errors."""
    try:
        result = subprocess.run(
            ["ps", "-ww", "-axo", "pid=,ppid=,ucomm=,args="],
            capture_output=True, text=True, timeout=timeout)
    except Exception as exc:
        raise ProLeaseProtocolError(f"cannot inspect process owners: {exc}") from exc
    if result.returncode != 0:
        raise ProLeaseProtocolError(
            f"process-owner query failed with status {result.returncode}")
    return python_script_pids_from_ps(
        result.stdout, script_basename, os.getpid())


class ProLease:
    """Token-owned lease; release only removes the object this instance owns."""

    def __init__(self, obs: Path, token: str, role: str, maintenance: bool):
        self.obs = Path(obs)
        self.token = token
        self.role = role
        self.maintenance = maintenance
        self.released = False

    def release(self):
        if self.released:
            return
        _gate, lease_dir, owner_path = _paths(self.obs)
        owner = _read_json(owner_path)
        if owner.get("token") != self.token:
            raise ProLeaseProtocolError(
                f"refusing to release {lease_dir}: ownership token changed")
        try:
            owner_path.unlink()
            lease_dir.rmdir()
        except OSError as exc:
            raise ProLeaseProtocolError(
                f"could not release exact lease {lease_dir}: {exc}") from exc
        self.released = True

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.release()
        return False


def acquire_client(obs: Path, role: str, serial: str) -> ProLease:
    """Try once to acquire the normal production lease.

    The caller decides whether to skip (snapshot producer) or wait/retry
    (tracker).  No timeout here means there is no hidden automatic takeover.
    """
    obs = Path(obs)
    serial = _require_production_serial(serial)
    gate, lease_dir, owner_path = _paths(obs)
    if _gate_present(gate):
        raise ProLeaseUnavailable(f"maintenance gate active at {gate}")
    token = uuid.uuid4().hex
    try:
        lease_dir.mkdir()
    except FileExistsError as exc:
        raise ProLeaseUnavailable(f"production Pro lease busy at {lease_dir}") from exc
    except OSError as exc:
        raise ProLeaseProtocolError(f"cannot create lease directory {lease_dir}: {exc}") from exc

    owner = {
        "protocol": PROTOCOL,
        "token": token,
        "role": role,
        "serial": serial,
        "pid": os.getpid(),
        "maintenance": False,
        "created_unix": time.time(),
    }
    try:
        _write_exclusive(owner_path, owner)
        lease = ProLease(obs, token, role, False)
        # Gate creation can race the mkdir.  If it won at any point before
        # this post-check, surrender the lease and do not open the radio.
        if _gate_present(gate):
            lease.release()
            raise ProLeaseUnavailable(f"maintenance gate won acquisition race at {gate}")
        return lease
    except Exception:
        # If owner.json was never created, only remove our empty directory.
        if lease_dir.exists() and not owner_path.exists():
            try:
                lease_dir.rmdir()
            except OSError:
                pass
        raise


def create_maintenance_gate(obs: Path, role: str, token_file: Path, serial: str):
    obs = Path(obs)
    token_file = Path(token_file)
    serial = _require_production_serial(serial)
    gate, _lease_dir, _owner_path = _paths(obs)
    token = uuid.uuid4().hex
    data = {
        "protocol": PROTOCOL,
        "token": token,
        "role": role,
        "serial": serial,
        "pid": os.getpid(),
        "created_unix": time.time(),
        "state": "gated_waiting_for_radio_lease",
    }
    try:
        _write_exclusive(gate, data)
    except FileExistsError as exc:
        raise ProLeaseUnavailable(f"maintenance is already gated at {gate}") from exc
    try:
        _write_exclusive(token_file, data)
    except Exception:
        # Roll back only the gate with our exact token.
        try:
            if _read_json(gate).get("token") == token:
                gate.unlink()
        except Exception:
            pass
        raise
    return data


def _load_maintenance_token(obs: Path, token_file: Path):
    gate, _lease_dir, _owner_path = _paths(Path(obs))
    token_doc = _read_json(Path(token_file))
    gate_doc = _read_json(gate)
    token = token_doc.get("token")
    if not token or gate_doc.get("token") != token:
        raise ProLeaseProtocolError("maintenance token does not own the active gate")
    if (token_doc.get("serial") != PRODUCTION_SERIAL
            or gate_doc.get("serial") != PRODUCTION_SERIAL):
        raise ProLeaseProtocolError("maintenance gate is not bound to the production serial")
    return token_doc


def acquire_maintenance(obs: Path, token_file: Path, wait_seconds: float) -> ProLease:
    """Acquire the radio after the caller has gated and stopped its owner."""
    obs = Path(obs)
    token_doc = _load_maintenance_token(obs, token_file)
    token = token_doc["token"]
    gate, lease_dir, owner_path = _paths(obs)
    deadline = time.monotonic() + max(0.0, wait_seconds)
    while True:
        # Revalidate the gate on every attempt; losing it is never success.
        if _read_json(gate).get("token") != token:
            raise ProLeaseProtocolError("maintenance gate changed while waiting")
        try:
            lease_dir.mkdir()
            break
        except FileExistsError:
            if time.monotonic() >= deadline:
                raise ProLeaseUnavailable(
                    f"radio lease still owned at {lease_dir}; tracker may not be stopped")
            time.sleep(0.1)
        except OSError as exc:
            raise ProLeaseProtocolError(f"cannot create lease directory {lease_dir}: {exc}") from exc
    owner = {
        "protocol": PROTOCOL,
        "token": token,
        "role": token_doc.get("role", "maintenance"),
        "serial": token_doc.get("serial"),
        "pid": os.getpid(),
        "maintenance": True,
        "created_unix": time.time(),
    }
    try:
        _write_exclusive(owner_path, owner)
        if _read_json(gate).get("token") != token:
            lease = ProLease(obs, token, str(owner["role"]), True)
            lease.release()
            raise ProLeaseProtocolError("maintenance gate changed during lease acquisition")
    except Exception:
        if lease_dir.exists() and not owner_path.exists():
            try:
                lease_dir.rmdir()
            except OSError:
                pass
        raise
    return ProLease(obs, token, str(owner["role"]), True)


def release_maintenance(obs: Path, token_file: Path):
    obs = Path(obs)
    token_file = Path(token_file)
    token_doc = _load_maintenance_token(obs, token_file)
    token = token_doc["token"]
    gate, lease_dir, owner_path = _paths(obs)
    if not owner_path.exists():
        raise ProLeaseProtocolError(
            "maintenance radio lease is absent; refusing to open the gate")
    owner = _read_json(owner_path)
    if owner.get("token") != token or owner.get("maintenance") is not True:
        raise ProLeaseProtocolError(
            "active radio lease is not owned by this maintenance token")
    ProLease(obs, token, str(owner.get("role", "maintenance")), True).release()
    # Clients remain blocked until the verified gate is removed.
    if _read_json(gate).get("token") != token:
        raise ProLeaseProtocolError("maintenance gate token changed before release")
    try:
        # Gate is the last object removed: a crash/failure before that point
        # still excludes every producer.
        token_file.unlink()
        gate.unlink()
    except OSError as exc:
        raise ProLeaseProtocolError(
            f"radio lease released but maintenance gate/token cleanup failed: {exc}") from exc


def cancel_maintenance_gate(obs: Path, token_file: Path):
    """Cancel a gate that never acquired the radio; never evict an owner."""
    obs = Path(obs)
    token_file = Path(token_file)
    token_doc = _load_maintenance_token(obs, token_file)
    token = token_doc["token"]
    gate, lease_dir, _owner_path = _paths(obs)
    if lease_dir.exists() or lease_dir.is_symlink():
        raise ProLeaseProtocolError(
            "radio lease exists; use release after maintenance or inspect it manually")
    if _read_json(gate).get("token") != token:
        raise ProLeaseProtocolError("maintenance gate token changed before cancellation")
    try:
        token_file.unlink()
        gate.unlink()
    except OSError as exc:
        raise ProLeaseProtocolError(
            f"maintenance cancellation remains fail-closed: {exc}") from exc


def status(obs: Path):
    gate, lease_dir, owner_path = _paths(Path(obs))
    lease_present = lease_dir.exists() or lease_dir.is_symlink()
    result = {
        "protocol": PROTOCOL,
        "maintenance_gate": str(gate),
        "maintenance_active": _gate_present(gate),
        "lease_dir": str(lease_dir),
        # A dangling symlink or other malformed object still blocks mkdir and
        # must never be displayed as an available radio.
        "lease_active": lease_present,
    }
    if _gate_present(gate):
        try:
            gate_doc = _read_json(gate)
            gate_doc["token"] = "<redacted>"
            result["gate"] = gate_doc
        except ProLeaseError as exc:
            result["gate_error"] = str(exc)
    if lease_present:
        try:
            owner_doc = _read_json(owner_path)
            owner_doc["token"] = "<redacted>"
            result["owner"] = owner_doc
        except ProLeaseError as exc:
            result["owner_error"] = str(exc)
    return result


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    sub = p.add_subparsers(dest="command", required=True)
    gate_p = sub.add_parser("gate", help="atomically block new production owners")
    gate_p.add_argument("--token-file", type=Path, required=True)
    gate_p.add_argument("--role", default="operator-maintenance")
    gate_p.add_argument("--serial", required=True,
                        help=f"must exactly equal {PRODUCTION_SERIAL}")
    acq_p = sub.add_parser("acquire", help="acquire the gated radio after tracker stop")
    acq_p.add_argument("--token-file", type=Path, required=True)
    acq_p.add_argument("--wait-seconds", type=float, default=0.0)
    rel_p = sub.add_parser("release", help="release radio lease, then maintenance gate")
    rel_p.add_argument("--token-file", type=Path, required=True)
    cancel_p = sub.add_parser("cancel", help="cancel a gate before radio acquisition")
    cancel_p.add_argument("--token-file", type=Path, required=True)
    sub.add_parser("status", help="print gate and lease metadata without changing them")
    args = p.parse_args(argv)
    # Production CLI commands deliberately have no lock-root override.  An
    # alternate root can report a successful gate while leaving the real Pro
    # completely unreserved.  Unit tests exercise the path-parameterized
    # functions directly instead.
    obs = DEFAULT_OBS
    try:
        if args.command == "gate":
            data = create_maintenance_gate(
                obs, args.role, args.token_file, args.serial)
            print(json.dumps({"gated": True, "role": data["role"],
                              "serial": data["serial"],
                              "token_file": str(args.token_file)}))
        elif args.command == "acquire":
            lease = acquire_maintenance(obs, args.token_file, args.wait_seconds)
            # The persistent token owns the reservation after this helper exits.
            print(json.dumps({"acquired": True, "role": lease.role,
                              "serial": PRODUCTION_SERIAL}))
        elif args.command == "release":
            release_maintenance(obs, args.token_file)
            print(json.dumps({"released": True}))
        elif args.command == "cancel":
            cancel_maintenance_gate(obs, args.token_file)
            print(json.dumps({"cancelled": True}))
        else:
            print(json.dumps(status(obs), indent=2, sort_keys=True))
    except ProLeaseError as exc:
        print(f"pro_lease: FATAL: {exc}", file=sys.stderr)
        return 78
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
