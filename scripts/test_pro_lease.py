#!/usr/bin/env python3
"""Offline tests for the production-Pro lease and shadow-only contracts."""

import json
import signal
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

import band_producer
import phase_producer
import pro_lease
import tracker_producer


ROOT = Path(__file__).resolve().parents[1]
SERIAL = "0000000000000000645061de252d6613"


class ProLeaseTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.obs = Path(self.tmp.name) / "observations"
        self.obs.mkdir()
        self.token_file = Path(self.tmp.name) / "maintenance.token"

    def tearDown(self):
        self.tmp.cleanup()

    def test_client_lease_is_exclusive_and_token_owned(self):
        first = pro_lease.acquire_client(self.obs, "tracker", SERIAL)
        with self.assertRaises(pro_lease.ProLeaseUnavailable):
            pro_lease.acquire_client(self.obs, "snapshot", SERIAL)
        owner = json.loads((self.obs / pro_lease.LEASE_DIR_NAME /
                            pro_lease.OWNER_NAME).read_text())
        self.assertEqual(owner["token"], first.token)
        self.assertFalse(owner["maintenance"])
        self.assertEqual(pro_lease.status(self.obs)["owner"]["token"], "<redacted>")
        first.release()
        second = pro_lease.acquire_client(self.obs, "snapshot", SERIAL)
        second.release()

    def test_gate_before_stop_prevents_last_second_snapshot(self):
        tracker = pro_lease.acquire_client(self.obs, "tracker", SERIAL)
        pro_lease.create_maintenance_gate(
            self.obs, "test-maintenance", self.token_file, SERIAL)
        gate_doc = json.loads((self.obs / pro_lease.GATE_NAME).read_text())
        self.assertEqual(gate_doc["serial"], SERIAL)

        # The current owner may finish, but no newcomer can enter after gate.
        with self.assertRaises(pro_lease.ProLeaseUnavailable):
            pro_lease.acquire_client(self.obs, "snapshot", SERIAL)
        with self.assertRaises(pro_lease.ProLeaseUnavailable):
            pro_lease.acquire_maintenance(self.obs, self.token_file, 0)

        tracker.release()
        maintenance = pro_lease.acquire_maintenance(
            self.obs, self.token_file, 0)
        self.assertTrue(maintenance.maintenance)
        with self.assertRaises(pro_lease.ProLeaseUnavailable):
            pro_lease.acquire_client(self.obs, "tracker-restart", SERIAL)

        pro_lease.release_maintenance(self.obs, self.token_file)
        self.assertFalse((self.obs / pro_lease.GATE_NAME).exists())
        after = pro_lease.acquire_client(self.obs, "tracker-restart", SERIAL)
        after.release()

    def test_malformed_gate_fails_closed(self):
        (self.obs / pro_lease.GATE_NAME).write_text("not json")
        with self.assertRaises(pro_lease.ProLeaseUnavailable):
            pro_lease.acquire_client(self.obs, "tracker", SERIAL)

    def test_dangling_lease_symlink_is_reported_busy(self):
        (self.obs / pro_lease.LEASE_DIR_NAME).symlink_to(
            self.obs / "missing-lease-target", target_is_directory=True)
        state = pro_lease.status(self.obs)
        self.assertTrue(state["lease_active"])
        self.assertIn("owner_error", state)

    def test_wrong_token_cannot_release_lease(self):
        lease = pro_lease.acquire_client(self.obs, "tracker", SERIAL)
        wrong = pro_lease.ProLease(self.obs, "wrong-token", "intruder", False)
        with self.assertRaises(pro_lease.ProLeaseProtocolError):
            wrong.release()
        self.assertTrue((self.obs / pro_lease.LEASE_DIR_NAME).exists())
        lease.release()

    def test_gate_can_be_cancelled_only_before_radio_acquisition(self):
        pro_lease.create_maintenance_gate(
            self.obs, "test-maintenance", self.token_file, SERIAL)
        pro_lease.cancel_maintenance_gate(self.obs, self.token_file)
        self.assertFalse((self.obs / pro_lease.GATE_NAME).exists())
        self.assertFalse(self.token_file.exists())

    def test_gate_and_client_reject_any_nonproduction_serial(self):
        wrong = "0000000000000000645061de252d6614"
        with self.assertRaises(pro_lease.ProLeaseProtocolError):
            pro_lease.acquire_client(self.obs, "tracker", wrong)
        with self.assertRaises(pro_lease.ProLeaseProtocolError):
            pro_lease.create_maintenance_gate(
                self.obs, "maintenance", self.token_file, wrong)

    def test_python_script_match_ignores_parent_shell_and_ancestors(self):
        snapshot = "\n".join([
            "100 1 bash /bin/bash -c python3 scripts/sync_producer.py",
            "101 100 python3 python3 scripts/tracker_producer.py",
            "202 1 python3 python3 /Volumes/Radiator 8TB/gnss/"
            "hackrf_gnss/scripts/sync_producer.py",
        ])
        self.assertEqual(
            pro_lease.python_script_pids_from_ps(
                snapshot, "sync_producer.py", self_pid=101),
            [202],
        )


class ProductionSourceContractTests(unittest.TestCase):
    def test_live_radio_rejects_override_before_open_and_has_no_write_call(self):
        source = (ROOT / "examples/live_radio.rs").read_text()
        transport = (ROOT / "vendor/rs-hackrf/src/transport.rs").read_text()
        guard = source.index('var_os("HACKRF_GNSS_ACTUATE")')
        lease_guard = source.index("require_production_lease(serial)")
        radio_open = source.index("HackRf::open_by_serial")
        immutable_identity = source.index("dev.board_partid_serialno()")
        first_mutation = source.index("dev.set_sample_rate")
        self.assertLess(guard, radio_open)
        self.assertLess(lease_guard, radio_open)
        self.assertLess(radio_open, immutable_identity)
        self.assertLess(immutable_identity, first_mutation)
        self.assertIn("pro.radio.lock.d/owner.json", source)
        self.assertIn('Some("tracker_producer/live_radio")', source)
        self.assertIn('owner.get("pid")', source)
        self.assertIn("getppid()", source)
        self.assertNotIn('== "1"', source)
        self.assertNotIn("radio_ctrl.set_clock_corr_ppm(", source)
        self.assertIn('"actuate": false', source)
        self.assertIn('"corr_applied": false', source)
        self.assertIn('"expected_applied_correction_ppm": 0.0', source)
        self.assertIn('"applied_correction_readback": false', source)
        self.assertNotIn('"applied_correction_ppm":', source)
        self.assertNotIn("ClockCorrPpm", transport)
        self.assertNotIn("pub fn set_clock_corr_ppm", transport)
        self.assertNotIn("StreamControl::ClockCorr", transport)
        self.assertNotIn("set_clock_correction_ppm", transport)
        self.assertNotIn("RadioWriteReg", transport)

    def test_both_producers_use_the_shared_lease(self):
        tracker = (ROOT / "scripts/tracker_producer.py").read_text()
        band = (ROOT / "scripts/band_producer.py").read_text()
        self.assertIn("pro_lease.acquire_client", tracker)
        self.assertIn("pro_lease.acquire_client", band)
        self.assertNotIn("def wait_for_pro()", tracker)
        self.assertNotIn("def pro_owned()", band)
        self.assertNotIn(".kill()", tracker)
        self.assertNotIn("— reopening", tracker)
        self.assertIn("automatic reopen forbidden", tracker)

    def test_production_cli_has_no_alternate_lock_root(self):
        source = (ROOT / "scripts/pro_lease.py").read_text()
        self.assertNotIn('add_argument("--obs"', source)

    def test_unstoppable_child_is_reported_not_declared_free(self):
        class Child:
            pid = 123

            def poll(self):
                return None

            def terminate(self):
                pass

            def wait(self, timeout):
                raise TimeoutError("still alive")

        self.assertFalse(tracker_producer.terminate_child(Child()))

    def test_unexpected_failure_gate_blocks_reopen_until_reset_runbook(self):
        with tempfile.TemporaryDirectory() as tmp:
            obs = Path(tmp) / "observations"
            obs.mkdir()
            token_file = obs / "tracker-reset-required.maintenance.token"
            lease = pro_lease.acquire_client(obs, "tracker", SERIAL)
            with (mock.patch.object(pro_lease, "DEFAULT_OBS", obs),
                  mock.patch.object(tracker_producer, "FAILURE_TOKEN_FILE", token_file),
                  mock.patch.object(tracker_producer, "PRO", SERIAL)):
                self.assertTrue(tracker_producer.arm_reset_required_gate())
            lease.release()
            with self.assertRaises(pro_lease.ProLeaseUnavailable):
                pro_lease.acquire_client(obs, "unsafe-auto-reopen", SERIAL)
            maintenance = pro_lease.acquire_maintenance(obs, token_file, 0)
            self.assertTrue(maintenance.maintenance)
            pro_lease.release_maintenance(obs, token_file)

    def test_shutdown_reports_lease_release_failure(self):
        class BadLease:
            def release(self):
                raise pro_lease.ProLeaseProtocolError("persistent lock remains")

        with (mock.patch.object(tracker_producer, "_rust", None),
              mock.patch.object(tracker_producer, "_xfer", None),
              mock.patch.object(tracker_producer, "_pro_lease", BadLease()),
              mock.patch.object(tracker_producer, "arm_reset_required_gate",
                                return_value=True)):
            with self.assertRaises(SystemExit) as caught:
                tracker_producer.shutdown()
        self.assertEqual(caught.exception.code, 78)

    def test_tracker_spawn_is_published_before_shutdown_signals_unblock(self):
        child = mock.Mock(pid=4242)
        lease = mock.Mock()
        lease.token = "0123456789abcdef0123456789abcdef"
        old_mask = {tracker_producer.signal.SIGTERM}
        with (mock.patch.object(tracker_producer, "wait_for_pro_lease",
                               return_value=lease),
              mock.patch.object(tracker_producer, "pause_sync_producer",
                                return_value=[]),
              mock.patch.object(tracker_producer.subprocess, "Popen",
                                return_value=child) as popen,
              mock.patch("builtins.open", mock.mock_open()),
              mock.patch.object(tracker_producer.signal, "pthread_sigmask",
                                side_effect=[old_mask, old_mask]) as mask,
              mock.patch.object(tracker_producer, "_rust", None)):
            _xfer, returned, returned_lease = tracker_producer.open_stream()
            self.assertIs(tracker_producer._rust, child)
        self.assertIs(returned, child)
        self.assertIs(returned_lease, lease)
        child_env = popen.call_args.kwargs["env"]
        self.assertEqual(child_env["HACKRF_PRO_LEASE_TOKEN"], lease.token)
        self.assertEqual(
            popen.call_args.args[0],
            [sys.executable, tracker_producer.UNBLOCKED_EXEC,
             tracker_producer.TRACKER, SERIAL],
        )
        self.assertEqual(mask.call_args_list[-1].args,
                         (tracker_producer.signal.SIG_SETMASK, old_mask))

    @unittest.skipUnless(hasattr(signal, "pthread_sigmask"),
                         "pthread signal masks unavailable")
    def test_exec_launcher_unblocks_shutdown_signals_in_real_child(self):
        probe = (
            "import signal; "
            "m=signal.pthread_sigmask(signal.SIG_BLOCK,set()); "
            "print(int(signal.SIGINT in m),int(signal.SIGTERM in m))"
        )
        previous = signal.pthread_sigmask(
            signal.SIG_BLOCK, {signal.SIGINT, signal.SIGTERM})
        try:
            result = subprocess.run(
                [sys.executable, str(ROOT / "scripts/exec_unblocked.py"),
                 sys.executable, "-c", probe],
                capture_output=True, text=True, timeout=5,
            )
        finally:
            signal.pthread_sigmask(signal.SIG_SETMASK, previous)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout.strip(), "0 0")

    def test_band_legacy_guard_checks_transfer_and_live_radio(self):
        busy = mock.Mock(returncode=0,
                         stdout=f"123 hackrf_transfer -d {SERIAL}\n")
        with mock.patch.object(band_producer.subprocess, "run", return_value=busy) as run:
            self.assertTrue(band_producer.unmanaged_pro_owned())
        self.assertEqual(run.call_args.args[0][-1], "hackrf_transfer")

    def test_band_capture_rejects_nonzero_transfer_exit_and_releases(self):
        lease = mock.Mock()
        failed = mock.Mock(returncode=1, stdout="", stderr="failed")
        with tempfile.TemporaryDirectory() as tmp:
            out = str(Path(tmp) / "capture.iq")
            with (mock.patch.object(pro_lease, "acquire_client", return_value=lease),
                  mock.patch.object(band_producer, "unmanaged_pro_owned",
                                    return_value=False),
                  mock.patch.object(band_producer.subprocess, "run",
                                    return_value=failed)):
                self.assertFalse(band_producer.transfer(
                    band_producer.PRO, 1_575_420_000, 0.001, out))
        lease.release.assert_called_once()

    def test_band_capture_rejects_nonproduction_serial_before_lease(self):
        wrong = "0000000000000000645061de252d6614"
        with mock.patch.object(pro_lease, "acquire_client") as acquire:
            with self.assertRaises(pro_lease.ProLeaseProtocolError):
                band_producer.transfer(wrong, 1_575_420_000, 0.001,
                                       "/tmp/never-open.iq")
        acquire.assert_not_called()

    def test_band_does_not_probe_the_streaming_one(self):
        with mock.patch.object(band_producer.subprocess, "run") as run:
            self.assertIsNone(band_producer.clkin_ok())
        run.assert_not_called()


class ClearStreamProfileTests(unittest.TestCase):
    PROFILE = "clearstream_bias_on_20260901"

    def test_profile_requires_two_explicit_matching_fields(self):
        with self.assertRaises(ValueError):
            phase_producer.clearstream_rf_config({})
        with self.assertRaises(ValueError):
            phase_producer.clearstream_rf_config({
                "HACKRF_ANTENNA_PROFILE": self.PROFILE,
            })

    def test_validated_profile_preserves_bias_on(self):
        config = phase_producer.clearstream_rf_config({
            "HACKRF_ANTENNA_PROFILE": self.PROFILE,
            "HACKRF_RF_PROFILE_ACK": self.PROFILE,
        })
        self.assertTrue(config["bias_tee_requested"])
        self.assertFalse(config["rf_amp_requested"])
        self.assertIn("unresolved", config["electrical_status"])

    def test_phase_source_declares_transmitter_uncertainty(self):
        source = (ROOT / "scripts/phase_producer.py").read_text()
        self.assertIn('"transmitter_discipline_attested": False', source)
        self.assertIn("ATSC transmitter plus One receiver relative frequency",
                      source)

    def test_legacy_single_boolean_override_is_rejected(self):
        with self.assertRaises(ValueError):
            phase_producer.clearstream_rf_config({
                "HACKRF_ANTENNA_PROFILE": self.PROFILE,
                "HACKRF_RF_PROFILE_ACK": self.PROFILE,
                "HACKRF_ANT_POWER": "1",
            })

    def test_reviewed_profile_rejects_silent_gain_mutation(self):
        for key, value in (("HACKRF_LNA", "32"), ("HACKRF_VGA", "46")):
            with self.subTest(key=key), self.assertRaises(ValueError):
                phase_producer.clearstream_rf_config({
                    "HACKRF_ANTENNA_PROFILE": self.PROFILE,
                    "HACKRF_RF_PROFILE_ACK": self.PROFILE,
                    key: value,
                })

    def test_dark_heartbeat_clears_scalar_residual(self):
        with tempfile.TemporaryDirectory() as tmp:
            state_path = Path(tmp) / "state.phase.json"
            phase = {"epoch": 123.0, "lock": False}
            row = {"band": "ATSC ch35", "epoch": 100.0, "value": 0.25}
            with mock.patch.object(phase_producer, "STATE", str(state_path)):
                phase_producer.merge_state(phase, row, 0.25)
            state = json.loads(state_path.read_text())
        self.assertIsNone(state["clock"]["residual_ppm"])
        self.assertFalse(state["clock"]["residual_valid"])
        self.assertEqual(state["sources"][0]["epoch"], 100.0)

    def test_one_process_probe_failure_is_busy(self):
        failed = mock.Mock(returncode=2, stdout="")
        with mock.patch.object(phase_producer.subprocess, "run", return_value=failed):
            self.assertTrue(phase_producer.one_busy())

    def test_phase_parent_shell_launch_text_is_not_an_owner(self):
        snapshot = "\n".join([
            "100 1 /bin/bash /bin/bash -c cd /Volumes/Radiator 8TB/gnss && "
            "python3 scripts/phase_producer.py",
            "101 100 /usr/bin/python3 python3 scripts/phase_producer.py",
        ])
        self.assertFalse(phase_producer._one_busy_from_ps(snapshot, 101))

        old_wrapper = snapshot + "\n" + (
            "202 1 /usr/bin/python3 python3 "
            "/Volumes/Radiator 8TB/gnss/hackrf_gnss/scripts/phase_producer.py")
        self.assertTrue(phase_producer._one_busy_from_ps(old_wrapper, 101))

    def test_phase_wrapper_lock_is_atomic(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = str(Path(tmp) / "phase.lock")
            first = phase_producer.acquire_one_wrapper_lock(path)
            try:
                with self.assertRaises(BlockingIOError):
                    phase_producer.acquire_one_wrapper_lock(path)
            finally:
                __import__("os").close(first)

    def test_phase_startup_failure_proves_child_stopped(self):
        class Child:
            pid = 99
            returncode = None
            terminated = False

            def poll(self):
                return self.returncode

            def terminate(self):
                self.terminated = True
                self.returncode = 0

            def wait(self, timeout):
                return self.returncode

        child = Child()
        config = phase_producer.clearstream_rf_config({
            "HACKRF_ANTENNA_PROFILE": self.PROFILE,
            "HACKRF_RF_PROFILE_ACK": self.PROFILE,
        })
        with tempfile.TemporaryDirectory() as tmp:
            fifo = str(Path(tmp) / "phase.iq")
            with (mock.patch.object(phase_producer, "FIFO", fifo),
                  mock.patch.object(phase_producer, "wait_for_one"),
                  mock.patch.object(phase_producer.subprocess, "Popen",
                                    return_value=child),
                  mock.patch("select.select", return_value=([], [], [])),
                  mock.patch.object(phase_producer, "_proc", None)):
                with self.assertRaises(RuntimeError):
                    phase_producer.open_stream(config)
                self.assertIsNone(phase_producer._proc)
        self.assertTrue(child.terminated)


if __name__ == "__main__":
    unittest.main()
