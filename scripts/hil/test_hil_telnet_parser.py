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


if __name__ == "__main__":
    unittest.main()
