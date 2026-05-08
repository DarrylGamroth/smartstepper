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
                "motor state transition",
                """
Transition Status:
  Result:       completed
  Reason:       online mode entered
""",
            ),
            hil_telnet.ShellResult(
                "motor state recovery",
                """
Recovery Status:
  Fault latched:     NO
  Gate reset req:    NO
  Gate reset done:   YES
  Encoder rec req:   NO
  Encoder rec done:  YES
  Safe idle ready:   YES
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

    def test_status_fails_with_rejected_transition(self) -> None:
        results = [
            hil_telnet.ShellResult(
                "motor state status",
                "Motor Status:\n  State: IDLE (7)\n  Error: NONE (0)\n",
            ),
            hil_telnet.ShellResult(
                "motor state transition",
                """
Transition Status:
  Result:       rejected
  Reason:       encoder mapping has not been applied
""",
            ),
            hil_telnet.ShellResult(
                "motor fault snapshot status",
                "Fault snapshot:\n  Latched:    NO\n",
            ),
            hil_telnet.ShellResult(
                "motor encoder acquisition",
                "Encoder acquisition:\n  Errors:   transport=0 frame=0 parity=0 crc=0 status=0 glitch=0\n",
            ),
        ]

        report = hil_telnet.evaluate_results(_args("status"), results, None)

        self.assertEqual(report.verdict, "FAIL")
        transition = next(check for check in report.checks
                          if check.name == "transition_not_rejected")
        self.assertEqual(transition.values["result"], "rejected")

    def test_recovery_status_fails_when_gate_reset_required(self) -> None:
        results = [
            hil_telnet.ShellResult(
                "motor state status",
                "Motor Status:\n  State: ERROR (17)\n  Error: OVERCURRENT (2)\n",
            ),
            hil_telnet.ShellResult(
                "motor state recovery",
                """
Recovery Status:
  Fault latched:     YES
  Last error:        OVERCURRENT (2)
  Gate reset req:    YES
  Gate reset done:   NO
  Encoder rec req:   NO
  Encoder rec done:  YES
  Safe idle ready:   NO
""",
            ),
            hil_telnet.ShellResult(
                "motor encoder acquisition",
                "Encoder acquisition:\n  Errors:   transport=0 frame=0 parity=0 crc=0 status=0 glitch=0\n",
            ),
        ]

        report = hil_telnet.evaluate_results(_args("recovery-status"), results, None)

        self.assertEqual(report.verdict, "FAIL")
        recovery = next(check for check in report.checks if check.name == "recovery_ready")
        self.assertTrue(recovery.values["gate_required"])
        self.assertFalse(recovery.values["gate_done"])

    def test_standard_commission_fails_without_completion_text(self) -> None:
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

        report = hil_telnet.evaluate_results(_args("standard-commission"), results, None)

        self.assertEqual(report.verdict, "FAIL")
        self.assertTrue(any(check.name == "standard_commission_complete" for check in report.checks))

    def test_standard_commission_passes_with_ready_mapping_and_completion_text(self) -> None:
        results = [
            hil_telnet.ShellResult(
                "motor commission run confirm apply",
                "Standard baseline commissioning workflow complete\n",
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

        report = hil_telnet.evaluate_results(_args("standard-commission"), results, None)

        self.assertEqual(report.verdict, "PASS")

    def test_velocity_validate_commands_apply_bandwidth_pi_by_default(self) -> None:
        args = hil_telnet.parse_args([
            "velocity-validate",
            "--no-log",
            "--velocity-pi-bandwidth-hz",
            "10",
            "--velocity-pi-zeta",
            "1.0",
            "--velocity-pi-iq-limit",
            "0.25",
        ])

        commands = [command.command for command in hil_telnet.scenario_velocity_validate_commands(args)]

        self.assertIn("motor velocity pi bandwidth 10.000 1.000 0.250", commands)
        self.assertIn("motor velocity pi status", commands)

    def test_velocity_validate_commands_allow_explicit_pi_override(self) -> None:
        args = hil_telnet.parse_args([
            "velocity-validate",
            "--no-log",
            "--velocity-pi-kp",
            "0.08",
            "--velocity-pi-ki",
            "0.16",
            "--velocity-pi-iq-limit",
            "0.25",
        ])

        commands = [command.command for command in hil_telnet.scenario_velocity_validate_commands(args)]

        self.assertIn("motor velocity pi set 0.080000 0.160000 0.250000", commands)
        self.assertFalse(any(command.startswith("motor velocity pi bandwidth")
                             for command in commands))

    def test_custom_commission_commands_get_long_timeouts(self) -> None:
        args = hil_telnet.parse_args([
            "custom",
            "--no-log",
            "--command",
            "motor commission run confirm apply",
            "--command",
            "motor state status",
            "--command-timeout",
            "3",
        ])

        commands = hil_telnet.scenario_custom(args)

        self.assertGreaterEqual(commands[0].timeout_s, args.standard_commission_timeout_s)
        self.assertEqual(commands[1].timeout_s, 3.0)

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
                "motor commission run confirm apply",
                "Standard baseline commissioning workflow complete\n",
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
                "motor commission run confirm apply",
                "Standard baseline commissioning workflow complete\n",
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

    def test_velocity_validate_fails_when_reference_does_not_follow_target(self) -> None:
        results = [
            hil_telnet.ShellResult(
                "motor commission run confirm apply",
                "Standard baseline commissioning workflow complete\n",
            ),
            hil_telnet.ShellResult(
                "motor commission validate velocity 0.050 1000",
                """
Velocity encoder validation: active PI gains, max=0.050 Hz hold=1000 ms DOB=on detent=on
  target=  0.010 Hz ref=  0.000 Hz meas=  0.004 Hz err=  0.006 Hz Iq_ref=0.0000 A Iq=0.0200 A Id=0.0000 A warn=0 err=0
  target=  0.050 Hz ref=  0.000 Hz meas=  0.004 Hz err=  0.046 Hz Iq_ref=0.0000 A Iq=0.0200 A Id=0.0000 A warn=0 err=0
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
        self.assertGreater(velocity.values["max_ref_err_hz"], 0.005)

    def test_feature_matrix_fails_when_reference_does_not_follow_target(self) -> None:
        results = [
            hil_telnet.ShellResult(
                "motor commission run slow apply",
                "Standard baseline commissioning workflow complete\n",
            ),
            hil_telnet.ShellResult(
                "motor commission detent status",
                "Quality:   raw=PASS fill=PASS apply=PASS conf=0.55\n"
                "Samples:   accepted=191617 rejected=2208638 bins=256/256 raw=215 filled=41\n",
            ),
            hil_telnet.ShellResult(
                "motor commission validate velocity 0.050 1000 active",
                """
Velocity encoder validation: max=0.050 Hz hold=1000 ms outer=MPR DOB=on detent=on
  target=  0.010 Hz ref=  0.000 Hz meas=  0.004 Hz err=  0.006 Hz Iq_ref=0.0000 A Iq=0.0200 A Id=0.0000 A warn=0 err=0
  target=  0.050 Hz ref=  0.000 Hz meas=  0.004 Hz err=  0.046 Hz Iq_ref=0.0000 A Iq=0.0200 A Id=0.0000 A warn=0 err=0
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

        report = hil_telnet.evaluate_results(_args("mpr-dob-detent"), results, None)

        self.assertEqual(report.verdict, "FAIL")
        matrix = next(check for check in report.checks if check.name == "velocity_validation_matrix")
        self.assertEqual(matrix.values["failed"], 1)
        self.assertGreater(matrix.values["max_ref_err_hz"], 0.005)

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
                "motor commission run confirm apply",
                "Standard baseline commissioning workflow complete\n",
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

    def test_velocity_sweep_passes_with_full_rotation_trace(self) -> None:
        args = hil_telnet.parse_args([
            "velocity-sweep",
            "--no-log",
            "--velocity-sweep-commission",
            "none",
            "--velocity-sweep-target-hz",
            "0.5",
            "--velocity-sweep-target-hz",
            "-0.5",
        ])
        results = [
            hil_telnet.ShellResult(
                "motor velocity target 0.500",
                "Velocity target set to 0.50 Hz\n",
            ),
            hil_telnet.ShellResult("motor velocity status", """
Velocity Controller Status:
  Target:     0.50 Hz
  Ref:        0.50 Hz
  Measured:   0.49 Hz
  Error:      0.01 Hz
  Outer loop: PI
  Iq limit:   0.225 A
"""),
            hil_telnet.ShellResult("motor encoder trace summary", """
Encoder raw trace summary:
  Stored:       512 / 512
  Decimation:   80
  Overrun:      0
  Raw delta:    360000 mdeg, avg 500 mHz
  Ctrl delta:   360000 mdeg, avg 500 mHz
  Counts:       clean=512 fresh=512 ctrl_en=512 warn=0 err=0 io=0
  Delta drops:  0
"""),
            hil_telnet.ShellResult(
                "motor velocity target -0.500",
                "Velocity target set to -0.50 Hz\n",
            ),
            hil_telnet.ShellResult("motor velocity status", """
Velocity Controller Status:
  Target:     -0.50 Hz
  Ref:        -0.50 Hz
  Measured:   -0.48 Hz
  Error:      -0.02 Hz
  Outer loop: PI
  Iq limit:   0.225 A
"""),
            hil_telnet.ShellResult("motor encoder trace summary", """
Encoder raw trace summary:
  Stored:       512 / 512
  Decimation:   80
  Overrun:      0
  Raw delta:    -355000 mdeg, avg -493 mHz
  Ctrl delta:   -355000 mdeg, avg -493 mHz
  Counts:       clean=512 fresh=512 ctrl_en=512 warn=0 err=0 io=0
  Delta drops:  0
"""),
            hil_telnet.ShellResult(
                "motor encoder acquisition",
                "Encoder acquisition:\n  Errors:   transport=0 frame=0 parity=0 crc=0 status=0 glitch=0\n",
            ),
            hil_telnet.ShellResult(
                "motor state status",
                "Motor Status:\n  Error: NONE (0)\n",
            ),
        ]

        report = hil_telnet.evaluate_results(args, results, None)

        self.assertEqual(report.verdict, "PASS")
        sweep = next(check for check in report.checks if check.name == "velocity_sweep")
        self.assertEqual(sweep.values["samples"], 2)

    def test_velocity_sweep_fails_wrong_direction(self) -> None:
        args = hil_telnet.parse_args([
            "velocity-sweep",
            "--no-log",
            "--velocity-sweep-commission",
            "none",
            "--velocity-sweep-target-hz",
            "0.5",
        ])
        results = [
            hil_telnet.ShellResult("motor velocity target 0.500", ""),
            hil_telnet.ShellResult("motor velocity status", """
Velocity Controller Status:
  Target:     0.50 Hz
  Ref:        0.50 Hz
  Measured:   -0.50 Hz
  Error:      1.00 Hz
  Outer loop: PI
  Iq limit:   0.225 A
"""),
            hil_telnet.ShellResult("motor encoder trace summary", """
Encoder raw trace summary:
  Stored:       512 / 512
  Raw delta:    -360000 mdeg, avg -500 mHz
  Ctrl delta:   -360000 mdeg, avg -500 mHz
  Counts:       clean=512 fresh=512 ctrl_en=512 warn=0 err=0 io=0
  Delta drops:  0
"""),
            hil_telnet.ShellResult(
                "motor encoder acquisition",
                "Encoder acquisition:\n  Errors:   transport=0 frame=0 parity=0 crc=0 status=0 glitch=0\n",
            ),
            hil_telnet.ShellResult(
                "motor state status",
                "Motor Status:\n  Error: NONE (0)\n",
            ),
        ]

        report = hil_telnet.evaluate_results(args, results, None)

        self.assertEqual(report.verdict, "FAIL")
        point = next(check for check in report.checks if check.name == "velocity_sweep_0")
        self.assertEqual(point.status, "FAIL")

    def test_required_standard_commission_rejects_failed_response(self) -> None:
        cmd = hil_telnet.ShellCommand(
            "motor commission run confirm apply",
            require_success=True,
        )

        reason = hil_telnet._command_success_failure_reason(
            cmd,
            "Encoder boot commissioning failed (err -14)\n",
        )

        self.assertIsNotNone(reason)

    def test_required_standard_commission_accepts_completion_response(self) -> None:
        cmd = hil_telnet.ShellCommand(
            "motor commission run confirm apply",
            require_success=True,
        )

        reason = hil_telnet._command_success_failure_reason(
            cmd,
            "Standard baseline commissioning workflow complete\n",
        )

        self.assertIsNone(reason)

    def test_parse_encoder_fault_reason(self) -> None:
        reason = hil_telnet._parse_encoder_fault_reason(
            "Fault snapshot:\n"
            "  Fault:      ENCODER_FAULT (3)\n"
            "  Enc reason: stale (4)\n"
        )

        self.assertEqual(reason, ("stale", 4))

    def test_status_reports_encoder_fault_reason(self) -> None:
        results = [
            hil_telnet.ShellResult(
                "motor state status",
                "Motor Status:\n"
                "  State: ERROR (17)\n"
                "  Error: ENCODER_FAULT (3)\n"
                "  Enc reason: stale (4)\n",
            ),
            hil_telnet.ShellResult(
                "motor fault snapshot status",
                "Fault snapshot:\n"
                "  Latched:    YES\n"
                "  Fault:      ENCODER_FAULT (3)\n"
                "  Enc reason: stale (4)\n",
            ),
            hil_telnet.ShellResult(
                "motor encoder acquisition",
                "Encoder acquisition:\n"
                "  Errors:   transport=0 frame=0 parity=0 crc=0 status=0 glitch=0\n",
            ),
        ]

        report = hil_telnet.evaluate_results(_args("status"), results, None)

        reason = next(check for check in report.checks
                      if check.name == "encoder_fault_reason")
        self.assertEqual(reason.status, "INFO")
        self.assertEqual(reason.values["reason"], "stale")
        self.assertEqual(reason.values["code"], 4)

    def test_encoder_fault_without_reason_fails(self) -> None:
        results = [
            hil_telnet.ShellResult(
                "motor state status",
                "Motor Status:\n"
                "  State: ERROR (17)\n"
                "  Error: ENCODER_FAULT (3)\n",
            ),
            hil_telnet.ShellResult(
                "motor fault snapshot status",
                "Fault snapshot:\n"
                "  Latched:    YES\n"
                "  Fault:      ENCODER_FAULT (3)\n",
            ),
            hil_telnet.ShellResult(
                "motor encoder acquisition",
                "Encoder acquisition:\n"
                "  Errors:   transport=0 frame=0 parity=0 crc=0 status=0 glitch=0\n",
            ),
        ]

        report = hil_telnet.evaluate_results(_args("status"), results, None)

        reason = next(check for check in report.checks
                      if check.name == "encoder_fault_reason")
        self.assertEqual(reason.status, "FAIL")

    def test_production_electrical_id_commands_run_and_apply(self) -> None:
        args = hil_telnet.parse_args([
            "production-electrical-id",
            "--no-log",
            "--yes-live-motion",
            "--electrical-id-current",
            "0.04",
            "--electrical-id-pulse",
            "0.02",
            "--electrical-id-samples",
            "16",
        ])

        commands = [command.command for command in hil_telnet.scenario_production_electrical_id(args)]

        self.assertIn("motor commission electrical plan", commands)
        self.assertIn("motor commission electrical run 0.040 0.020 16", commands)
        self.assertIn("motor commission electrical apply", commands)
        self.assertIn("motor commission electrical validate 0.040 300 0.010", commands)

    def test_production_electrical_id_required_response_checks(self) -> None:
        run_cmd = hil_telnet.ShellCommand(
            "motor commission electrical run 0.040 0.020 16",
            require_success=True,
        )
        apply_cmd = hil_telnet.ShellCommand(
            "motor commission electrical apply",
            require_success=True,
        )
        validate_cmd = hil_telnet.ShellCommand(
            "motor commission electrical validate 0.040 300 0.010",
            require_success=True,
        )

        self.assertIsNone(hil_telnet._command_success_failure_reason(
            run_cmd,
            "Production inductance staged: Ld=0.003 H Lq=0.003 H\n",
        ))
        self.assertIsNone(hil_telnet._command_success_failure_reason(
            apply_cmd,
            "Production electrical ID applied: Rs=2.2 Ld=0.003 Lq=0.003\n",
        ))
        self.assertIsNotNone(hil_telnet._command_success_failure_reason(
            run_cmd,
            "Production Rs measurement rejected (err -34)\n",
        ))
        self.assertIsNone(hil_telnet._command_success_failure_reason(
            validate_cmd,
            "Production electrical current-step validation: PASS target=0.0400 A\n",
        ))

    def test_mechanical_id_v2_passes_with_valid_confident_fit(self) -> None:
        results = [
            hil_telnet.ShellResult(
                "motor state status",
                "Motor Status:\n  State: IDLE (7)\n  Error: NONE (0)\n",
            ),
            hil_telnet.ShellResult(
                "motor commission auto run confirm",
                "Auto commission complete. Run 'motor commission auto apply' to apply.\n",
            ),
            hil_telnet.ShellResult(
                "motor commission status",
                """
Commission Status:
  Estimates:      psi_f=VALID mech=VALID
  Mech fit:       J=0.00000570 kgm2 B=0.00001000 Nm/(rad/s) Tc=0.02000000 Nm T0=0.00000000 Nm
  Mech quality:   rms=0.001000 Nm R2=0.9000 N=128
  Mech v2:        valid=YES friction=YES inertia=YES detent=none accelN=120
  Mech v2 qual:   friction_rms=0.001000 Nm inertia_rms=0.001500 Nm J/fallback=1.000
  Mech reject:    reason=none err=0 tq_sign=1
  Mech repeat:    runs=3 conf=0.82 Jstd=0.00000010 Bstd=0.00000100 Tcstd=0.00100000
""",
            ),
            hil_telnet.ShellResult(
                "motor encoder acquisition status",
                "Encoder acquisition:\n  Errors:   transport=0 frame=0 parity=0 crc=0 status=0 glitch=0\n",
            ),
        ]

        report = hil_telnet.evaluate_results(_args("mechanical-id-v2"), results, None)

        self.assertEqual(report.verdict, "PASS")

    def test_mechanical_id_v2_accepts_explicit_rejection(self) -> None:
        results = [
            hil_telnet.ShellResult(
                "motor state status",
                "Motor Status:\n  State: IDLE (7)\n  Error: NONE (0)\n",
            ),
            hil_telnet.ShellResult(
                "motor commission auto run confirm",
                "Auto commission complete. Run 'motor commission auto apply' to apply.\n",
            ),
            hil_telnet.ShellResult(
                "motor commission status",
                """
Commission Status:
  Estimates:      psi_f=VALID mech=INVALID
  Mech v2:        valid=NO friction=YES inertia=NO detent=none accelN=120
  Mech v2 qual:   friction_rms=0.001000 Nm inertia_rms=0.100000 Nm J/fallback=20.000
  Mech reject:    reason=implausible err=0 tq_sign=1
  Mech repeat:    runs=1 conf=0.20 Jstd=0.00000000 Bstd=0.00000000 Tcstd=0.00000000
""",
            ),
            hil_telnet.ShellResult(
                "motor encoder acquisition status",
                "Encoder acquisition:\n  Errors:   transport=0 frame=0 parity=0 crc=0 status=0 glitch=0\n",
            ),
        ]

        report = hil_telnet.evaluate_results(_args("mechanical-id-v2"), results, None)

        self.assertEqual(report.verdict, "PASS")


if __name__ == "__main__":
    unittest.main()
