#!/usr/bin/env python3
"""Unit tests for HIL telnet verdict parsing.

These tests do not connect to hardware. They exercise the parser using compact
samples of Zephyr shell output so HIL pass/fail logic can be changed safely.
"""

from __future__ import annotations

import unittest
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
import hil_telnet


def _args(scenario: str) -> object:
    return hil_telnet.parse_args([scenario, "--no-log"])


class HilTelnetParserTest(unittest.TestCase):
    def test_status_passes_with_idle_no_error_and_clean_encoder_counters(self) -> None:
        results = [
            hil_telnet.ShellResult(
                "motor state status",
                """
Motor Status:
  State: IDLE (7)
  Error: NONE (0)
""",
            ),
            hil_telnet.ShellResult(
                "motor fault snapshot status",
                """
Fault snapshot:
  Latched:    NO
""",
            ),
            hil_telnet.ShellResult(
                "motor encoder acquisition",
                """
Encoder acquisition:
  Errors:   transport=0 frame=0 parity=0 crc=0 status=0 glitch=0
""",
            ),
        ]

        report = hil_telnet.evaluate_results(_args("status"), results, None)

        self.assertEqual(report.verdict, "PASS")

    def test_boot_commission_fails_without_completion_text(self) -> None:
        results = [
            hil_telnet.ShellResult(
                "motor state status",
                "Motor Status:\n  Error: NONE (0)\n",
            ),
            hil_telnet.ShellResult(
                "motor encoder control_status",
                """
Encoder Control Readiness:
  Ready:           YES
  Mapping applied: YES
  Protocol ok:     YES
  Acquisition errors: transport=0 parity=0 crc=0 glitch=0 status=0
""",
            ),
            hil_telnet.ShellResult(
                "motor encoder acquisition",
                "Encoder acquisition:\n  Errors:   transport=0 frame=0 parity=0 crc=0 status=0 glitch=0\n",
            ),
        ]

        report = hil_telnet.evaluate_results(_args("boot-commission"), results, None)

        self.assertEqual(report.verdict, "FAIL")
        self.assertTrue(any(check.name == "boot_commission_complete" for check in report.checks))

    def test_boot_commission_passes_with_ready_mapping_and_completion_text(self) -> None:
        results = [
            hil_telnet.ShellResult(
                "motor commission boot 0.150 0.050 1.000",
                "Boot commissioning complete: sign=-1 commutation_offset=1.0000 deg mechanical outer=PI\n",
            ),
            hil_telnet.ShellResult(
                "motor encoder control_status",
                """
Encoder Control Readiness:
  Ready:           YES
  Mapping applied: YES
  Protocol ok:     YES
  Acquisition errors: transport=0 parity=0 crc=0 glitch=0 status=0
""",
            ),
            hil_telnet.ShellResult(
                "motor encoder acquisition",
                "Encoder acquisition:\n  Errors:   transport=0 frame=0 parity=0 crc=0 status=0 glitch=0\n",
            ),
            hil_telnet.ShellResult(
                "motor state status",
                "Motor Status:\n  Error: NONE (0)\n",
            ),
        ]

        report = hil_telnet.evaluate_results(_args("boot-commission"), results, None)

        self.assertEqual(report.verdict, "PASS")

    def test_encoder_validate_fails_wrong_velocity_sign(self) -> None:
        results = [
            hil_telnet.ShellResult(
                "motor commission validate current 0.030 160",
                """
Current encoder validation: Iq=+/-0.030 A hold=160 ms outer=PI DOB=off detent=off
  +Iq: net=1.500 deg abs=1.500 deg samples=10 warn=0 err=0
  -Iq: net=-1.200 deg abs=1.200 deg samples=10 warn=0 err=0
Current encoder validation complete
""",
            ),
            hil_telnet.ShellResult(
                "motor commission validate velocity 0.050 1000",
                """
Velocity encoder validation: active PI gains, max=0.050 Hz hold=1000 ms DOB=off detent=off
  target=  0.013 Hz ref=  0.013 Hz meas= -0.010 Hz err=  0.023 Hz Iq=0.0100 A Id=0.0000 A warn=0 err=0
""",
            ),
            hil_telnet.ShellResult(
                "motor encoder control_status",
                """
Encoder Control Readiness:
  Ready:           YES
  Mapping applied: YES
  Protocol ok:     YES
  Acquisition errors: transport=0 parity=0 crc=0 glitch=0 status=0
""",
            ),
            hil_telnet.ShellResult(
                "motor encoder acquisition",
                "Encoder acquisition:\n  Errors:   transport=0 frame=0 parity=0 crc=0 status=0 glitch=0\n",
            ),
            hil_telnet.ShellResult(
                "motor state status",
                "Motor Status:\n  Error: NONE (0)\n",
            ),
        ]

        report = hil_telnet.evaluate_results(_args("encoder-validate"), results, None)

        self.assertEqual(report.verdict, "FAIL")
        velocity = next(check for check in report.checks if check.name == "velocity_validation")
        self.assertEqual(velocity.values["wrong_sign_samples"], 1)

    def test_current_validate_passes_clean_opposite_motion(self) -> None:
        results = [
            hil_telnet.ShellResult(
                "motor commission boot 0.150 0.050 1.000",
                "Boot commissioning complete: sign=-1 commutation_offset=1.0000 deg mechanical outer=PI\n",
            ),
            hil_telnet.ShellResult(
                "motor commission validate current 0.030 160",
                """
Current encoder validation: Iq=+/-0.030 A hold=160 ms outer=PI DOB=off detent=off
  +Iq: net=12.000 deg abs=12.000 deg samples=32 warn=0 err=0
  -Iq: net=-10.500 deg abs=10.500 deg samples=32 warn=0 err=0
Current encoder validation complete
""",
            ),
            hil_telnet.ShellResult(
                "motor encoder control_status",
                """
Encoder Control Readiness:
  Ready:           YES
  Mapping applied: YES
  Protocol ok:     YES
  Acquisition errors: transport=0 parity=0 crc=0 glitch=0 status=0
""",
            ),
            hil_telnet.ShellResult(
                "motor encoder acquisition",
                "Encoder acquisition:\n  Errors:   transport=0 frame=0 parity=0 crc=0 status=0 glitch=0\n",
            ),
            hil_telnet.ShellResult(
                "motor state status",
                "Motor Status:\n  Error: NONE (0)\n",
            ),
        ]

        report = hil_telnet.evaluate_results(_args("current-validate"), results, None)

        self.assertEqual(report.verdict, "PASS")

    def test_velocity_validate_fails_excessive_overshoot(self) -> None:
        results = [
            hil_telnet.ShellResult(
                "motor commission boot 0.150 0.050 1.000",
                "Boot commissioning complete: sign=-1 commutation_offset=1.0000 deg mechanical outer=PI\n",
            ),
            hil_telnet.ShellResult(
                "motor commission validate velocity 0.050 1000",
                """
Velocity encoder validation: active PI gains, max=0.050 Hz hold=1000 ms DOB=off detent=off
  target=  0.010 Hz ref=  0.010 Hz meas=  0.500 Hz err= -0.490 Hz Iq_ref=0.1200 A Iq=0.0800 A Id=0.0000 A warn=0 err=0
  target=  0.030 Hz ref=  0.030 Hz meas=  0.600 Hz err= -0.570 Hz Iq_ref=0.1200 A Iq=0.0800 A Id=0.0000 A warn=0 err=0
""",
            ),
            hil_telnet.ShellResult(
                "motor encoder control_status",
                """
Encoder Control Readiness:
  Ready:           YES
  Mapping applied: YES
  Protocol ok:     YES
  Acquisition errors: transport=0 parity=0 crc=0 glitch=0 status=0
""",
            ),
            hil_telnet.ShellResult(
                "motor encoder acquisition",
                "Encoder acquisition:\n  Errors:   transport=0 frame=0 parity=0 crc=0 status=0 glitch=0\n",
            ),
            hil_telnet.ShellResult(
                "motor state status",
                "Motor Status:\n  Error: NONE (0)\n",
            ),
        ]

        report = hil_telnet.evaluate_results(_args("velocity-validate"), results, None)

        self.assertEqual(report.verdict, "FAIL")
        velocity = next(check for check in report.checks if check.name == "velocity_validation")
        self.assertGreater(velocity.values["overshoot_ratio"], 3.0)

    def test_open_loop_trace_passes_with_observer_columns_and_clean_motion(self) -> None:
        results = [
            hil_telnet.ShellResult(
                "motor encoder trace summary",
                """
Encoder raw trace summary:
  Stored:       512 / 512
  Raw delta:    -1045 mdeg, avg -114 mHz
  Ctrl delta:   1045 mdeg, avg 114 mHz
  Counts:       clean=512 fresh=512 ctrl_en=0 warn=0 err=0 io=0
  Delta drops:  0
""",
            ),
            hil_telnet.ShellResult(
                "motor encoder trace dump 32",
                """
idx loop src raw_mdeg ctrl_mdeg obs_mech_mdeg obs_elec_mdeg gen_mech_mdeg gen_elec_mdeg q fresh warn err io status ctrl_en
0 100 0 151397 -151397 206439 291934 206438 291844 0x03 1 0 0 0 0x00 0
""",
            ),
            hil_telnet.ShellResult(
                "motor encoder acquisition",
                "Encoder acquisition:\n  Errors:   transport=0 frame=0 parity=0 crc=0 status=0 glitch=0\n",
            ),
            hil_telnet.ShellResult(
                "motor state status",
                "Motor Status:\n  Error: NONE (0)\n",
            ),
            hil_telnet.ShellResult(
                "motor fault snapshot status",
                "Fault snapshot:\n  Latched:    NO\n",
            ),
        ]

        report = hil_telnet.evaluate_results(_args("encoder-trace-open-loop"), results, None)

        self.assertEqual(report.verdict, "PASS")
        trace = next(check for check in report.checks if check.name == "open_loop_trace")
        self.assertEqual(trace.values["ctrl_delta_mdeg"], 1045)

    def test_open_loop_trace_fails_when_no_motion_was_captured(self) -> None:
        results = [
            hil_telnet.ShellResult(
                "motor encoder trace summary",
                """
Encoder raw trace summary:
  Stored:       512 / 512
  Raw delta:    0 mdeg, avg 0 mHz
  Ctrl delta:   0 mdeg, avg 0 mHz
  Counts:       clean=512 fresh=512 ctrl_en=0 warn=0 err=0 io=0
  Delta drops:  0
""",
            ),
            hil_telnet.ShellResult(
                "motor encoder acquisition",
                "Encoder acquisition:\n  Errors:   transport=0 frame=0 parity=0 crc=0 status=0 glitch=0\n",
            ),
            hil_telnet.ShellResult(
                "motor state status",
                "Motor Status:\n  Error: NONE (0)\n",
            ),
            hil_telnet.ShellResult(
                "motor fault snapshot status",
                "Fault snapshot:\n  Latched:    NO\n",
            ),
        ]

        report = hil_telnet.evaluate_results(_args("encoder-trace-open-loop"), results, None)

        self.assertEqual(report.verdict, "FAIL")
        trace = next(check for check in report.checks if check.name == "open_loop_trace")
        self.assertEqual(trace.values["ctrl_delta_mdeg"], 0)

    def test_encoder_robust_passes_with_valid_mapping_and_apply(self) -> None:
        results = [
            hil_telnet.ShellResult(
                "motor commission encoder robust 0.150 0.050 1.000 bidirectional",
                """
Encoder mapping result: valid=YES dir=-1 corr=-0.9852 off_mech=0.209 deg off_elec=10.452 deg
Robust encoder mapping combined: valid=YES dir=-1 corr=-0.9849 off_mech=0.210 deg off_elec=10.500 deg
""",
            ),
            hil_telnet.ShellResult(
                "motor commission encoder status",
                """
Encoder Mapping Detect:
  Staged valid:   YES
  Valid:          YES
""",
            ),
            hil_telnet.ShellResult(
                "motor commission encoder apply",
                "Encoder mapping applied: sign=-1 commutation_offset=0.2100 deg mechanical\n",
            ),
            hil_telnet.ShellResult(
                "motor encoder control_status",
                """
Encoder Control Readiness:
  Ready:           YES
  Mapping applied: YES
  Protocol ok:     YES
  Acquisition errors: transport=0 parity=0 crc=0 glitch=0 status=0
""",
            ),
            hil_telnet.ShellResult(
                "motor encoder acquisition",
                "Encoder acquisition:\n  Errors:   transport=0 frame=0 parity=0 crc=0 status=0 glitch=0\n",
            ),
            hil_telnet.ShellResult(
                "motor state status",
                "Motor Status:\n  Error: NONE (0)\n",
            ),
        ]

        report = hil_telnet.evaluate_results(_args("encoder-robust"), results, None)

        self.assertEqual(report.verdict, "PASS")

    def test_encoder_robust_fails_without_apply(self) -> None:
        results = [
            hil_telnet.ShellResult(
                "motor commission encoder robust 0.150 0.050 1.000",
                "Encoder mapping result: valid=YES dir=-1 corr=-0.9852 off_mech=0.209 deg off_elec=10.452 deg\n",
            ),
            hil_telnet.ShellResult(
                "motor commission encoder apply",
                "No valid staged encoder mapping result\n",
            ),
            hil_telnet.ShellResult(
                "motor encoder acquisition",
                "Encoder acquisition:\n  Errors:   transport=0 frame=0 parity=0 crc=0 status=0 glitch=0\n",
            ),
            hil_telnet.ShellResult(
                "motor state status",
                "Motor Status:\n  Error: NONE (0)\n",
            ),
        ]

        report = hil_telnet.evaluate_results(_args("encoder-robust"), results, None)

        self.assertEqual(report.verdict, "FAIL")
        apply_check = next(
            check for check in report.checks
            if check.name == "encoder_robust_mapping_applied"
        )
        self.assertEqual(apply_check.status, "FAIL")

    def test_position_validate_does_not_require_current_validation_output(self) -> None:
        results = [
            hil_telnet.ShellResult(
                "motor commission boot 0.150 0.050 1.000",
                "Boot commissioning complete: sign=-1 commutation_offset=1.0000 deg mechanical outer=PI\n",
            ),
            hil_telnet.ShellResult(
                "motor commission validate velocity 0.500 1000",
                """
Velocity encoder validation: active PI gains, max=0.500 Hz hold=1000 ms DOB=off detent=off
  target=  0.500 Hz ref=  0.500 Hz meas=  0.460 Hz err=  0.040 Hz Iq_ref=0.0300 A Iq=0.0200 A Id=0.0000 A warn=0 err=0
""",
            ),
            hil_telnet.ShellResult(
                "motor commission validate position 5.000 2000",
                "Position encoder validation complete: pos=10.000 deg vel=0.000 Hz Iq=0.0000 A\n",
            ),
            hil_telnet.ShellResult(
                "motor encoder control_status",
                """
Encoder Control Readiness:
  Ready:           YES
  Mapping applied: YES
  Protocol ok:     YES
  Acquisition errors: transport=0 parity=0 crc=0 glitch=0 status=0
""",
            ),
            hil_telnet.ShellResult(
                "motor encoder acquisition",
                "Encoder acquisition:\n  Errors:   transport=0 frame=0 parity=0 crc=0 status=0 glitch=0\n",
            ),
            hil_telnet.ShellResult(
                "motor state status",
                "Motor Status:\n  Error: NONE (0)\n",
            ),
        ]

        report = hil_telnet.evaluate_results(_args("position-validate"), results, None)

        self.assertEqual(report.verdict, "PASS")
        self.assertFalse(any(check.name == "current_validation" for check in report.checks))


if __name__ == "__main__":
    unittest.main()
