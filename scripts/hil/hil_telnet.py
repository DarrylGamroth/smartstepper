#!/usr/bin/env python3
"""Repeatable HIL shell workflows over the Zephyr telnet shell.

The script intentionally uses only Python's socket module. telnetlib is removed
in newer Python versions, and the Zephyr telnet shell only needs minimal option
refusal plus line-oriented command/response handling.
"""

from __future__ import annotations

import argparse
import datetime as _dt
import json
import os
import re
import socket
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable, Sequence

IAC = 255
DONT = 254
DO = 253
WONT = 252
WILL = 251

ANSI_RE = re.compile(rb"\x1b\[[0-9;?]*[ -/]*[@-~]")
PROMPT_RE = re.compile(r"(?:^|\n|\r)(?::?~\$)\s*$")


@dataclass(frozen=True)
class ShellCommand:
    command: str
    timeout_s: float = 3.0
    settle_s: float = 0.05


@dataclass(frozen=True)
class ShellResult:
    command: str
    response: str


@dataclass
class VerdictCheck:
    name: str
    status: str
    detail: str
    values: dict[str, float | int | str | bool] = field(default_factory=dict)


@dataclass
class ScenarioReport:
    scenario: str
    verdict: str
    log_path: str | None
    checks: list[VerdictCheck]

    def to_json_dict(self) -> dict[str, object]:
        return {
            "scenario": self.scenario,
            "verdict": self.verdict,
            "log_path": self.log_path,
            "checks": [
                {
                    "name": check.name,
                    "status": check.status,
                    "detail": check.detail,
                    "values": check.values,
                }
                for check in self.checks
            ],
        }


class TelnetShell:
    def __init__(self, host: str, port: int, timeout_s: float, log_path: Path | None):
        self.host = host
        self.port = port
        self.timeout_s = timeout_s
        self.log_path = log_path
        self.sock: socket.socket | None = None
        self.log_file = None

    def __enter__(self) -> "TelnetShell":
        self.sock = socket.create_connection((self.host, self.port), timeout=self.timeout_s)
        self.sock.settimeout(0.1)
        if self.log_path is not None:
            self.log_path.parent.mkdir(parents=True, exist_ok=True)
            self.log_file = self.log_path.open("w", encoding="utf-8")
        banner = self.read_until_prompt(timeout_s=self.timeout_s, send_newline=True)
        if banner:
            self._print_and_log("### CONNECT\n" + banner)
        self.drain_pending(quiet_s=0.15, timeout_s=1.0)
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        if self.log_file is not None:
            self.log_file.close()
        if self.sock is not None:
            self.sock.close()

    def _write_raw(self, data: bytes) -> None:
        assert self.sock is not None
        self.sock.sendall(data)

    def _read_raw(self) -> bytes:
        assert self.sock is not None
        try:
            return self.sock.recv(4096)
        except socket.timeout:
            return b""

    def _filter_telnet(self, data: bytes) -> bytes:
        out = bytearray()
        i = 0
        while i < len(data):
            byte = data[i]
            if byte != IAC:
                out.append(byte)
                i += 1
                continue
            if i + 1 >= len(data):
                break
            cmd = data[i + 1]
            if cmd in (DO, DONT, WILL, WONT) and i + 2 < len(data):
                opt = data[i + 2]
                if cmd == DO:
                    self._write_raw(bytes([IAC, WONT, opt]))
                elif cmd == WILL:
                    self._write_raw(bytes([IAC, DONT, opt]))
                i += 3
            else:
                i += 2
        return bytes(out)

    @staticmethod
    def _clean(data: bytes) -> str:
        data = ANSI_RE.sub(b"", data)
        return data.decode("utf-8", errors="replace")

    def _print_and_log(self, text: str) -> None:
        print(text, end="" if text.endswith("\n") else "\n")
        if self.log_file is not None:
            self.log_file.write(text)
            if not text.endswith("\n"):
                self.log_file.write("\n")
            self.log_file.flush()

    def read_until_prompt(self, timeout_s: float, send_newline: bool = False) -> str:
        if send_newline:
            self._write_raw(b"\r\n")
        deadline = time.monotonic() + timeout_s
        buf = bytearray()
        while time.monotonic() < deadline:
            chunk = self._read_raw()
            if chunk:
                buf.extend(self._filter_telnet(chunk))
                clean = self._clean(bytes(buf))
                if PROMPT_RE.search(clean):
                    return clean
            else:
                time.sleep(0.02)
        return self._clean(bytes(buf))

    def drain_pending(self, quiet_s: float, timeout_s: float) -> str:
        deadline = time.monotonic() + timeout_s
        quiet_deadline = time.monotonic() + quiet_s
        buf = bytearray()
        while time.monotonic() < deadline:
            chunk = self._read_raw()
            if chunk:
                buf.extend(self._filter_telnet(chunk))
                quiet_deadline = time.monotonic() + quiet_s
                continue
            if time.monotonic() >= quiet_deadline:
                break
            time.sleep(0.02)
        return self._clean(bytes(buf))

    def run(self, cmd: ShellCommand) -> str:
        pending = self.drain_pending(quiet_s=0.08, timeout_s=0.5)
        if pending.strip():
            self._print_and_log("\n### ASYNC\n" + pending)
        header = f"\n### CMD: {cmd.command}\n"
        self._print_and_log(header)
        self._write_raw((cmd.command + "\r\n").encode("utf-8"))
        if cmd.settle_s > 0.0:
            time.sleep(cmd.settle_s)
        response = self.read_command_response(cmd.command, timeout_s=cmd.timeout_s)
        self._print_and_log(response)
        return response

    def read_command_response(self, command: str, timeout_s: float) -> str:
        deadline = time.monotonic() + timeout_s
        buf = bytearray()
        saw_echo = False
        while time.monotonic() < deadline:
            chunk = self._read_raw()
            if chunk:
                buf.extend(self._filter_telnet(chunk))
                clean = self._clean(bytes(buf))
                if command in clean:
                    saw_echo = True
                if saw_echo and PROMPT_RE.search(clean):
                    return clean
            else:
                time.sleep(0.02)
        return self._clean(bytes(buf))


def timestamp() -> str:
    return _dt.datetime.now().strftime("%Y%m%d_%H%M%S")


def scenario_status(args: argparse.Namespace) -> list[ShellCommand]:
    return [
        ShellCommand("motor state status"),
        ShellCommand("motor state policy"),
        ShellCommand("motor outer status"),
        ShellCommand("motor encoder control_status"),
        ShellCommand("motor encoder acquisition"),
        ShellCommand("motor fault snapshot status"),
        ShellCommand("motor info live"),
    ]


def scenario_boot_commission(args: argparse.Namespace) -> list[ShellCommand]:
    duration_s = max(10.0, (float(args.cycles) / max(float(args.boot_hz), 0.001)) + 12.0)
    return [
        ShellCommand("motor state status"),
        ShellCommand("motor state clear_error", timeout_s=2.0),
        ShellCommand("motor velocity target 0", timeout_s=1.5),
        ShellCommand("motor current iq 0", timeout_s=1.5),
        ShellCommand("motor disarm", timeout_s=1.5),
        ShellCommand("motor state idle", timeout_s=2.0),
        ShellCommand("motor safety timeout 0"),
        ShellCommand("motor encoder acquisition_reset"),
        ShellCommand(
            f"motor commission boot {args.boot_current:.3f} {args.boot_hz:.3f} {args.cycles:.3f}",
            timeout_s=duration_s,
        ),
        ShellCommand("motor encoder control_status"),
        ShellCommand("motor encoder acquisition"),
        ShellCommand("motor state status"),
    ]


def scenario_encoder_validate(args: argparse.Namespace) -> list[ShellCommand]:
    cmds = scenario_boot_commission(args)
    cmds.extend([
        *scenario_current_validate_commands(args),
        *scenario_velocity_validate_commands(args),
    ])
    if args.include_position:
        cmds.extend(scenario_position_validate_commands(args))
    return cmds


def scenario_encoder_robust(args: argparse.Namespace) -> list[ShellCommand]:
    duration_s = max(
        10.0,
        (float(args.cycles) / max(abs(float(args.boot_hz)), 0.001)) *
        (2.0 if args.bidirectional else 1.0) + 15.0,
    )
    suffix = " bidirectional" if args.bidirectional else ""
    return [
        ShellCommand("motor state status"),
        ShellCommand("motor state clear_error", timeout_s=2.0),
        ShellCommand("motor velocity target 0", timeout_s=1.5),
        ShellCommand("motor current iq 0", timeout_s=1.5),
        ShellCommand("motor disarm", timeout_s=1.5),
        ShellCommand("motor state idle", timeout_s=2.0),
        ShellCommand("motor safety timeout 0"),
        ShellCommand("motor encoder acquisition_reset", timeout_s=2.0),
        ShellCommand("motor arm", timeout_s=2.0),
        ShellCommand(
            f"motor commission encoder robust {args.boot_current:.3f} "
            f"{args.boot_hz:.3f} {args.cycles:.3f}{suffix}",
            timeout_s=duration_s,
        ),
        ShellCommand("motor commission encoder status", timeout_s=3.0),
        ShellCommand("motor commission encoder apply", timeout_s=3.0),
        ShellCommand("motor encoder control_status", timeout_s=3.0),
        ShellCommand("motor encoder acquisition", timeout_s=3.0),
        ShellCommand("motor state status", timeout_s=3.0),
    ]


def _optional_velocity_pi_commands(args: argparse.Namespace) -> list[ShellCommand]:
    if args.velocity_pi_kp is None and args.velocity_pi_ki is None:
        return []
    if args.velocity_pi_kp is None or args.velocity_pi_ki is None:
        raise ValueError("--velocity-pi-kp and --velocity-pi-ki must be supplied together")
    return [
        ShellCommand(
            f"motor velocity pi set {args.velocity_pi_kp:.6f} "
            f"{args.velocity_pi_ki:.6f} {args.velocity_pi_iq_limit:.6f}",
            timeout_s=2.0,
        ),
        ShellCommand("motor velocity pi status", timeout_s=2.0),
    ]


def scenario_current_validate_commands(args: argparse.Namespace) -> list[ShellCommand]:
    return [
        ShellCommand(
            f"motor commission validate current {args.current_iq:.3f} {args.current_hold_ms}",
            timeout_s=max(4.0, args.current_hold_ms / 1000.0 * 4.0 + 3.0),
        ),
        ShellCommand("motor current iq 0", timeout_s=1.5),
        ShellCommand("motor encoder acquisition"),
        ShellCommand("motor encoder control_status"),
        ShellCommand("motor state status"),
    ]


def scenario_velocity_validate_commands(args: argparse.Namespace) -> list[ShellCommand]:
    return [
        *_optional_velocity_pi_commands(args),
        ShellCommand(
            f"motor commission validate velocity {args.velocity_hz:.3f} {args.velocity_hold_ms}",
            timeout_s=max(12.0, args.velocity_hold_ms / 1000.0 * 9.0 + 4.0),
        ),
        ShellCommand("motor velocity target 0", timeout_s=1.5),
        ShellCommand("motor current iq 0", timeout_s=1.5),
        ShellCommand("motor encoder acquisition"),
        ShellCommand("motor encoder control_status"),
        ShellCommand("motor state status"),
    ]


def scenario_position_validate_commands(args: argparse.Namespace) -> list[ShellCommand]:
    return [
        ShellCommand(
            f"motor commission validate position {args.position_delta_deg:.3f} "
            f"{args.position_hold_ms}",
            timeout_s=max(12.0, args.position_hold_ms / 1000.0 * 3.0 + 6.0),
        ),
        ShellCommand("motor velocity target 0", timeout_s=1.5),
        ShellCommand("motor current iq 0", timeout_s=1.5),
        ShellCommand("motor encoder acquisition"),
        ShellCommand("motor encoder control_status"),
        ShellCommand("motor state status"),
    ]


def scenario_current_validate(args: argparse.Namespace) -> list[ShellCommand]:
    return [
        *scenario_boot_commission(args),
        *scenario_current_validate_commands(args),
    ]


def scenario_velocity_validate(args: argparse.Namespace) -> list[ShellCommand]:
    return [
        *scenario_boot_commission(args),
        *scenario_velocity_validate_commands(args),
    ]


def scenario_position_validate(args: argparse.Namespace) -> list[ShellCommand]:
    return [
        *scenario_boot_commission(args),
        *scenario_velocity_validate_commands(args),
        *scenario_position_validate_commands(args),
    ]


def scenario_encoder_trace_open_loop(args: argparse.Namespace) -> list[ShellCommand]:
    duration_s = max(1.0, args.trace_ms / 1000.0)
    return [
        ShellCommand("motor state clear_error", timeout_s=2.0),
        ShellCommand("motor safety timeout 0"),
        ShellCommand("motor state mode velocity_generated"),
        ShellCommand("motor state online", timeout_s=3.0),
        ShellCommand("motor arm"),
        ShellCommand("motor current id 0"),
        ShellCommand(f"motor current iq {args.open_loop_iq:.3f}"),
        ShellCommand("motor encoder trace clear"),
        ShellCommand(f"motor encoder trace start {args.trace_decimation}"),
        ShellCommand(f"motor velocity target {args.open_loop_hz:.3f}",
                     timeout_s=2.0, settle_s=duration_s),
        ShellCommand("motor encoder trace stop"),
        ShellCommand("motor velocity target 0"),
        ShellCommand("motor current iq 0"),
        ShellCommand("motor encoder trace summary"),
        ShellCommand(f"motor encoder trace dump {args.trace_dump}"),
        ShellCommand("motor encoder acquisition"),
        ShellCommand("motor disarm"),
        ShellCommand("motor state idle", timeout_s=2.0),
    ]


def scenario_custom(args: argparse.Namespace) -> list[ShellCommand]:
    return [ShellCommand(cmd, timeout_s=args.command_timeout) for cmd in args.command]


SCENARIOS = {
    "status": (scenario_status, False),
    "boot-commission": (scenario_boot_commission, True),
    "current-validate": (scenario_current_validate, True),
    "encoder-robust": (scenario_encoder_robust, True),
    "encoder-validate": (scenario_encoder_validate, True),
    "encoder-trace-open-loop": (scenario_encoder_trace_open_loop, True),
    "position-validate": (scenario_position_validate, True),
    "velocity-validate": (scenario_velocity_validate, True),
    "custom": (scenario_custom, False),
}


def _combined_text(results: Sequence[ShellResult]) -> str:
    return "\n".join(result.response for result in results)


def _last_response(results: Sequence[ShellResult], command_prefix: str) -> str:
    for result in reversed(results):
        if result.command.startswith(command_prefix):
            return result.response
    return ""


def _check(checks: list[VerdictCheck], name: str, ok: bool, detail: str,
           values: dict[str, float | int | str | bool] | None = None) -> None:
    checks.append(VerdictCheck(name, "PASS" if ok else "FAIL", detail, values or {}))


def _info(checks: list[VerdictCheck], name: str, detail: str,
          values: dict[str, float | int | str | bool] | None = None) -> None:
    checks.append(VerdictCheck(name, "INFO", detail, values or {}))


def _parse_motor_error(response: str) -> tuple[str, int] | None:
    match = re.search(r"^\s*Error:\s+([A-Z0-9_]+)\s+\(([-0-9]+)\)", response, re.MULTILINE)
    if match is None:
        return None
    return match.group(1), int(match.group(2))


def _parse_yes_no_field(response: str, label: str) -> bool | None:
    match = re.search(rf"^\s*{re.escape(label)}:\s+(YES|NO)\b", response, re.MULTILINE)
    if match is None:
        return None
    return match.group(1) == "YES"


def _parse_latch_field(response: str, label: str) -> bool | None:
    match = re.search(rf"^\s*{re.escape(label)}:\s+(YES|NO|SET|CLEAR)\b", response, re.MULTILINE)
    if match is None:
        return None
    return match.group(1) in ("YES", "SET")


def _parse_acquisition_errors(response: str) -> dict[str, int] | None:
    match = re.search(
        r"Errors:\s+transport=(\d+)\s+frame=(\d+)\s+parity=(\d+)\s+crc=(\d+)\s+status=(\d+)\s+glitch=(\d+)",
        response,
    )
    if match is not None:
        keys = ("transport", "frame", "parity", "crc", "status", "glitch")
        return {key: int(value) for key, value in zip(keys, match.groups())}

    match = re.search(
        r"Acquisition errors:\s+transport=(\d+)\s+parity=(\d+)\s+crc=(\d+)\s+glitch=(\d+)\s+status=(\d+)",
        response,
    )
    if match is not None:
        transport, parity, crc, glitch, status = (int(value) for value in match.groups())
        return {
            "transport": transport,
            "frame": 0,
            "parity": parity,
            "crc": crc,
            "status": status,
            "glitch": glitch,
        }

    return None


def _max_acquisition_errors(results: Sequence[ShellResult]) -> dict[str, int] | None:
    maxima: dict[str, int] = {}
    found = False
    for result in results:
        parsed = _parse_acquisition_errors(result.response)
        if parsed is None:
            continue
        found = True
        for key, value in parsed.items():
            maxima[key] = max(maxima.get(key, 0), value)
    return maxima if found else None


def _parse_current_validation(response: str) -> dict[str, float | int] | None:
    pos = re.search(
        r"\+Iq:\s+net=\s*([-+0-9.]+)\s+deg\s+abs=\s*([-+0-9.]+)\s+deg\s+samples=(\d+)\s+warn=(\d+)\s+err=(\d+)",
        response,
    )
    neg = re.search(
        r"-Iq:\s+net=\s*([-+0-9.]+)\s+deg\s+abs=\s*([-+0-9.]+)\s+deg\s+samples=(\d+)\s+warn=(\d+)\s+err=(\d+)",
        response,
    )
    if pos is None or neg is None:
        return None
    return {
        "pos_net_deg": float(pos.group(1)),
        "pos_abs_deg": float(pos.group(2)),
        "pos_samples": int(pos.group(3)),
        "pos_warn": int(pos.group(4)),
        "pos_err": int(pos.group(5)),
        "neg_net_deg": float(neg.group(1)),
        "neg_abs_deg": float(neg.group(2)),
        "neg_samples": int(neg.group(3)),
        "neg_warn": int(neg.group(4)),
        "neg_err": int(neg.group(5)),
    }


def _parse_velocity_samples(response: str) -> list[dict[str, float | int]]:
    samples: list[dict[str, float | int]] = []
    for match in re.finditer(
        r"target=\s*([-+0-9.]+)\s+Hz\s+ref=\s*([-+0-9.]+)\s+Hz\s+meas=\s*([-+0-9.]+)\s+Hz\s+err=\s*([-+0-9.]+)\s+Hz\s+(?:Iq_ref=([-+0-9.]+)\s+A\s+)?Iq=([-+0-9.]+)\s+A\s+Id=([-+0-9.]+)\s+A\s+warn=(\d+)\s+err=(\d+)",
        response,
    ):
        samples.append({
            "target_hz": float(match.group(1)),
            "ref_hz": float(match.group(2)),
            "meas_hz": float(match.group(3)),
            "err_hz": float(match.group(4)),
            "iq_ref_a": float(match.group(5)) if match.group(5) is not None else 0.0,
            "iq_a": float(match.group(6)),
            "id_a": float(match.group(7)),
            "warn": int(match.group(8)),
            "err": int(match.group(9)),
        })
    return samples


def _evaluate_current_validation(args: argparse.Namespace, checks: list[VerdictCheck],
                                 results: Sequence[ShellResult]) -> None:
    current_response = _last_response(results, "motor commission validate current")
    current = _parse_current_validation(current_response)
    if current is None:
        checks.append(VerdictCheck("current_validation", "FAIL",
                                   "Current validation summary not parsed"))
        return

    min_motion = args.min_current_motion_deg
    opposite_sign = (
        abs(float(current["pos_net_deg"])) >= min_motion and
        abs(float(current["neg_net_deg"])) >= min_motion and
        float(current["pos_net_deg"]) * float(current["neg_net_deg"]) < 0.0
    )
    clean = (
        int(current["pos_err"]) <= args.max_sample_errors and
        int(current["neg_err"]) <= args.max_sample_errors and
        int(current["pos_warn"]) <= args.max_sample_warnings and
        int(current["neg_warn"]) <= args.max_sample_warnings
    )
    _check(checks, "current_validation", opposite_sign and clean,
           "Current validation motion is clean and opposite sign"
           if opposite_sign and clean else
           "Current validation motion/sign or sample quality failed",
           current)


def _evaluate_velocity_validation(args: argparse.Namespace, checks: list[VerdictCheck],
                                  results: Sequence[ShellResult]) -> None:
    velocity_response = _last_response(results, "motor commission validate velocity")
    velocity_samples = _parse_velocity_samples(velocity_response)
    if not velocity_samples:
        checks.append(VerdictCheck("velocity_validation", "FAIL",
                                   "Velocity validation samples not parsed"))
        return

    nonzero_samples = [
        sample for sample in velocity_samples
        if abs(float(sample["target_hz"])) > 1.0e-6
    ]
    max_abs_err = max(abs(float(sample["err_hz"])) for sample in velocity_samples)
    max_warn = max(int(sample["warn"]) for sample in velocity_samples)
    max_err = max(int(sample["err"]) for sample in velocity_samples)
    tracking_samples = [
        sample for sample in nonzero_samples
        if abs(float(sample["meas_hz"])) >=
        abs(float(sample["target_hz"])) * args.min_velocity_tracking_fraction
    ]
    wrong_sign = [
        sample for sample in tracking_samples
        if float(sample["target_hz"]) * float(sample["meas_hz"]) < 0.0
    ]
    max_meas_abs_hz = max(abs(float(sample["meas_hz"])) for sample in velocity_samples)
    max_target_abs_hz = max(abs(float(sample["target_hz"])) for sample in velocity_samples)
    overshoot_ratio = max_meas_abs_hz / max(max_target_abs_hz, 1.0e-6)
    ok = (
        max_abs_err <= args.max_velocity_error_hz and
        overshoot_ratio <= args.max_velocity_overshoot_ratio and
        max_warn <= args.max_sample_warnings and
        max_err <= args.max_sample_errors and
        not wrong_sign and
        len(tracking_samples) >= max(1, args.min_velocity_tracking_samples)
    )
    _check(checks, "velocity_validation", ok,
           "Velocity validation samples are within thresholds" if ok
           else "Velocity validation samples exceed thresholds",
           {
               "samples": len(velocity_samples),
               "tracking_samples": len(tracking_samples),
               "max_abs_err_hz": max_abs_err,
               "max_meas_abs_hz": max_meas_abs_hz,
               "overshoot_ratio": overshoot_ratio,
               "max_warn": max_warn,
               "max_err": max_err,
               "wrong_sign_samples": len(wrong_sign),
           })


def _evaluate_position_validation(checks: list[VerdictCheck],
                                  results: Sequence[ShellResult]) -> None:
    position_response = _last_response(results, "motor commission validate position")
    complete = "Position encoder validation complete" in position_response
    _check(checks, "position_validation", complete,
           "Position validation completed" if complete else
           "Position validation completion text not found")


def _evaluate_encoder_robust(checks: list[VerdictCheck],
                             results: Sequence[ShellResult]) -> None:
    robust_response = _last_response(results, "motor commission encoder robust")
    status_response = _last_response(results, "motor commission encoder status")
    apply_response = _last_response(results, "motor commission encoder apply")
    response = "\n".join((robust_response, status_response, apply_response))

    mapping_valid = (
        "Robust encoder mapping combined: valid=YES" in response or
        "Encoder mapping result: valid=YES" in response or
        re.search(r"^\s*Valid:\s+YES\b", status_response, re.MULTILINE) is not None
    )
    mapping_applied = "Encoder mapping applied:" in apply_response
    _check(checks, "encoder_robust_mapping_valid", mapping_valid,
           "Robust encoder mapping produced a valid staged result" if mapping_valid
           else "Robust encoder mapping did not produce a valid staged result")
    _check(checks, "encoder_robust_mapping_applied", mapping_applied,
           "Robust encoder mapping applied" if mapping_applied
           else "Robust encoder mapping was not applied")


def evaluate_results(args: argparse.Namespace, results: Sequence[ShellResult],
                     log_path: Path | None) -> ScenarioReport:
    checks: list[VerdictCheck] = []
    text = _combined_text(results)

    fatal_patterns = (
        "HARD FAULT",
        "FATAL ERROR",
        "OVERCURRENT",
        "Validation stopped",
        "stopped by motor fault",
        "stopped by fault",
    )
    fatal_hits = [pattern for pattern in fatal_patterns if pattern in text]
    _check(checks, "no_fatal_text", not fatal_hits,
           "No fatal/fault-stop text found" if not fatal_hits else ", ".join(fatal_hits),
           {"hits": ",".join(fatal_hits)})

    state_response = _last_response(results, "motor state status")
    motor_error = _parse_motor_error(state_response)
    if motor_error is None:
        checks.append(VerdictCheck("motor_error_none", "INCONCLUSIVE",
                                   "No motor state status error field parsed"))
    else:
        error_name, error_code = motor_error
        _check(checks, "motor_error_none", error_name == "NONE" and error_code == 0,
               f"Motor error is {error_name} ({error_code})",
               {"error": error_name, "code": error_code})

    fault_response = _last_response(results, "motor fault snapshot status")
    if fault_response:
        latched = _parse_latch_field(fault_response, "Latched")
        if latched is None:
            checks.append(VerdictCheck("fault_snapshot_clear", "INCONCLUSIVE",
                                       "Fault snapshot latch field not parsed"))
        else:
            _check(checks, "fault_snapshot_clear", not latched,
                   "Fault snapshot latch is clear" if not latched else "Fault snapshot latch is set",
                   {"latched": latched})

    acquisition_errors = _max_acquisition_errors(results)
    if acquisition_errors is None:
        checks.append(VerdictCheck("encoder_acquisition_errors", "INCONCLUSIVE",
                                   "No encoder acquisition counters parsed"))
    else:
        max_transport = args.max_transport_errors
        max_crc = args.max_crc_errors
        max_status = args.max_status_errors
        max_glitch = args.max_glitch_errors
        ok = (
            acquisition_errors.get("transport", 0) <= max_transport and
            acquisition_errors.get("frame", 0) <= max_transport and
            acquisition_errors.get("parity", 0) <= max_crc and
            acquisition_errors.get("crc", 0) <= max_crc and
            acquisition_errors.get("status", 0) <= max_status and
            acquisition_errors.get("glitch", 0) <= max_glitch
        )
        detail = (
            "Encoder acquisition counters within thresholds" if ok
            else "Encoder acquisition counters exceed thresholds"
        )
        _check(checks, "encoder_acquisition_errors", ok, detail, acquisition_errors)

    control_response = _last_response(results, "motor encoder control_status")
    if control_response:
        ready = _parse_yes_no_field(control_response, "Ready")
        mapping = _parse_yes_no_field(control_response, "Mapping applied")
        protocol = _parse_yes_no_field(control_response, "Protocol ok")
        if args.scenario == "boot-commission":
            _check(checks, "encoder_ready", ready is True,
                   "Encoder control ready" if ready else "Encoder control not ready",
                   {"ready": bool(ready)})
            _check(checks, "mapping_applied", mapping is True,
                   "Encoder mapping applied" if mapping else "Encoder mapping not applied",
                   {"mapping_applied": bool(mapping)})
            _check(checks, "encoder_protocol_ok", protocol is True,
                   "Encoder protocol ok" if protocol else "Encoder protocol not ok",
                   {"protocol_ok": bool(protocol)})
        elif args.scenario in (
            "current-validate",
            "encoder-robust",
            "encoder-validate",
            "velocity-validate",
            "position-validate",
        ):
            _info(checks, "encoder_ready", "Encoder readiness parsed after validation",
                  {"ready": bool(ready)})
            _check(checks, "mapping_applied", mapping is True,
                   "Encoder mapping applied" if mapping else "Encoder mapping not applied",
                   {"mapping_applied": bool(mapping)})
            _check(checks, "encoder_protocol_ok", protocol is True,
                   "Encoder protocol ok" if protocol else "Encoder protocol not ok",
                   {"protocol_ok": bool(protocol)})
        else:
            _info(checks, "encoder_ready", "Encoder readiness parsed",
                  {
                      "ready": bool(ready),
                      "mapping_applied": bool(mapping),
                      "protocol_ok": bool(protocol),
                  })

    if args.scenario in (
        "boot-commission",
        "current-validate",
        "encoder-validate",
        "velocity-validate",
        "position-validate",
    ):
        complete = "Boot commissioning complete" in text
        _check(checks, "boot_commission_complete", complete,
               "Boot commissioning completed" if complete else "Boot commissioning completion text not found")

    if args.scenario == "encoder-robust":
        _evaluate_encoder_robust(checks, results)

    if args.scenario in ("current-validate", "encoder-validate", "position-validate"):
        _evaluate_current_validation(args, checks, results)

    if args.scenario in ("velocity-validate", "encoder-validate", "position-validate"):
        _evaluate_velocity_validation(args, checks, results)

    if args.scenario == "position-validate" or (
        args.scenario == "encoder-validate" and args.include_position
    ):
        _evaluate_position_validation(checks, results)

    has_fail = any(check.status == "FAIL" for check in checks)
    has_inconclusive = any(check.status == "INCONCLUSIVE" for check in checks)
    verdict = "FAIL" if has_fail else ("INCONCLUSIVE" if has_inconclusive else "PASS")
    return ScenarioReport(args.scenario, verdict, str(log_path) if log_path is not None else None, checks)


def print_report(report: ScenarioReport) -> None:
    print(f"\n### VERDICT: {report.verdict}")
    for check in report.checks:
        print(f"{check.status:12s} {check.name}: {check.detail}")


def stop_commands() -> list[ShellCommand]:
    return [
        ShellCommand("motor velocity target 0", timeout_s=1.5),
        ShellCommand("motor current iq 0", timeout_s=1.5),
        ShellCommand("motor disarm", timeout_s=1.5),
        ShellCommand("motor state idle", timeout_s=2.0),
        ShellCommand("motor safety timeout 1000", timeout_s=1.5),
        ShellCommand("motor encoder control_status", timeout_s=2.0),
        ShellCommand("motor encoder acquisition", timeout_s=2.0),
        ShellCommand("motor state status", timeout_s=2.0),
        ShellCommand("motor fault snapshot status", timeout_s=2.0),
    ]


def parse_args(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scenario", choices=sorted(SCENARIOS))
    parser.add_argument("--host", default=os.environ.get("CHOPPER_TELNET_HOST", "10.0.0.171"))
    parser.add_argument("--port", type=int, default=int(os.environ.get("CHOPPER_TELNET_PORT", "23")))
    parser.add_argument("--connect-timeout", type=float, default=5.0)
    parser.add_argument("--log-dir", default="hil_logs")
    parser.add_argument("--no-log", action="store_true")
    parser.add_argument("--json-report",
                        help="Write a machine-readable report with PASS/FAIL checks.")
    parser.add_argument("--yes-live-motion", action="store_true",
                        help="Required for scenarios that can energize or move the motor.")
    parser.add_argument("--leave-timeout-disabled", action="store_true")
    parser.add_argument("--max-transport-errors", type=int, default=0)
    parser.add_argument("--max-crc-errors", type=int, default=0,
                        help="Maximum parity/CRC errors tolerated in a scenario.")
    parser.add_argument("--max-status-errors", type=int, default=0)
    parser.add_argument("--max-glitch-errors", type=int, default=0)
    parser.add_argument("--max-sample-warnings", type=int, default=0)
    parser.add_argument("--max-sample-errors", type=int, default=0)
    parser.add_argument("--min-current-motion-deg", type=float, default=0.01)
    parser.add_argument("--max-velocity-error-hz", type=float, default=0.10)
    parser.add_argument("--max-velocity-overshoot-ratio", type=float, default=3.0,
                        help="Maximum |measured velocity| / |target velocity| during validation.")
    parser.add_argument("--min-velocity-tracking-fraction", type=float, default=0.10,
                        help="Minimum measured/target fraction for at least one nonzero sample.")
    parser.add_argument("--min-velocity-tracking-samples", type=int, default=1,
                        help="Minimum number of nonzero velocity samples that must move enough.")

    parser.add_argument("--boot-current", type=float, default=0.15)
    parser.add_argument("--boot-hz", type=float, default=0.05)
    parser.add_argument("--cycles", type=float, default=1.0)
    parser.add_argument("--bidirectional", action="store_true",
                        help="Use forward+reverse generated sweeps for encoder-robust.")
    parser.add_argument("--current-iq", type=float, default=0.03)
    parser.add_argument("--current-hold-ms", type=int, default=160)
    parser.add_argument("--velocity-hz", type=float, default=0.05)
    parser.add_argument("--velocity-hold-ms", type=int, default=1000)
    parser.add_argument("--velocity-pi-kp", type=float,
                        help="Optional velocity PI Kp to set before velocity validation.")
    parser.add_argument("--velocity-pi-ki", type=float,
                        help="Optional velocity PI Ki to set before velocity validation.")
    parser.add_argument("--velocity-pi-iq-limit", type=float, default=0.12,
                        help="Iq limit used with --velocity-pi-kp/--velocity-pi-ki.")
    parser.add_argument("--include-position", action="store_true")
    parser.add_argument("--position-delta-deg", type=float, default=5.0)
    parser.add_argument("--position-hold-ms", type=int, default=2000)

    parser.add_argument("--open-loop-iq", type=float, default=0.12)
    parser.add_argument("--open-loop-hz", type=float, default=0.10)
    parser.add_argument("--trace-ms", type=int, default=1000)
    parser.add_argument("--trace-decimation", type=int, default=1)
    parser.add_argument("--trace-dump", type=int, default=32)

    parser.add_argument("--command", action="append", default=[],
                        help="Command for the custom scenario. Can be repeated.")
    parser.add_argument("--command-timeout", type=float, default=3.0)
    return parser.parse_args(argv)


def main(argv: Sequence[str]) -> int:
    args = parse_args(argv)
    builder, live_motion = SCENARIOS[args.scenario]
    if (args.velocity_pi_kp is None) != (args.velocity_pi_ki is None):
        print("ERROR: --velocity-pi-kp and --velocity-pi-ki must be supplied together",
              file=sys.stderr)
        return 2
    if live_motion and not args.yes_live_motion:
        print(f"ERROR: scenario '{args.scenario}' can energize/move hardware; pass --yes-live-motion",
              file=sys.stderr)
        return 2
    if args.scenario == "custom" and not args.command:
        print("ERROR: custom scenario requires at least one --command", file=sys.stderr)
        return 2

    log_path = None
    if not args.no_log:
        log_path = Path(args.log_dir) / f"{timestamp()}_{args.scenario}.log"

    commands = builder(args)
    exit_code = 0
    results: list[ShellResult] = []
    with TelnetShell(args.host, args.port, args.connect_timeout, log_path) as shell:
        try:
            for cmd in commands:
                response = shell.run(cmd)
                results.append(ShellResult(cmd.command, response))
        except KeyboardInterrupt:
            exit_code = 130
            print("Interrupted; sending stop commands", file=sys.stderr)
        finally:
            if live_motion and not args.leave_timeout_disabled:
                for cmd in stop_commands():
                    try:
                        response = shell.run(cmd)
                        results.append(ShellResult(cmd.command, response))
                    except Exception as exc:  # noqa: BLE001 - best-effort hardware stop path
                        print(f"WARN: stop command failed: {cmd.command}: {exc}", file=sys.stderr)
                        exit_code = exit_code or 1
    if log_path is not None:
        print(f"Log saved: {log_path}")

    report = evaluate_results(args, results, log_path)
    print_report(report)
    if args.json_report:
        json_path = Path(args.json_report)
        json_path.parent.mkdir(parents=True, exist_ok=True)
        json_path.write_text(json.dumps(report.to_json_dict(), indent=2) + "\n",
                             encoding="utf-8")
        print(f"JSON report saved: {json_path}")

    if report.verdict == "FAIL":
        exit_code = exit_code or 1
    elif report.verdict == "INCONCLUSIVE":
        exit_code = exit_code or 3
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
