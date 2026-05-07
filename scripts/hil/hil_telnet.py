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
from math import ceil
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
    note: str | None = None
    require_success: bool = False


@dataclass(frozen=True)
class ShellResult:
    command: str
    response: str


class ScenarioAbort(RuntimeError):
    def __init__(self, command: str, reason: str):
        super().__init__(f"{command}: {reason}")
        self.command = command
        self.reason = reason


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
        if cmd.note:
            self._print_and_log(f"\n### NOTE: {cmd.note}\n")
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
        ShellCommand("motor state clear_error", timeout_s=2.0),
        ShellCommand("motor fault snapshot clear", timeout_s=2.0),
        ShellCommand("motor disarm", timeout_s=1.5),
        ShellCommand("motor state idle", timeout_s=2.0),
        ShellCommand("motor state status", timeout_s=5.0),
        ShellCommand("motor state transition", timeout_s=5.0),
        ShellCommand("motor state recovery", timeout_s=5.0),
        ShellCommand("motor state policy", timeout_s=5.0),
        ShellCommand("motor control status", timeout_s=5.0),
        ShellCommand("motor observer status", timeout_s=5.0),
        ShellCommand("motor encoder status", timeout_s=5.0),
        ShellCommand("motor outer status", timeout_s=5.0),
        ShellCommand("motor encoder control_status", timeout_s=5.0),
        ShellCommand("motor encoder acquisition status", timeout_s=5.0),
        ShellCommand("motor fault recovery", timeout_s=5.0),
        ShellCommand("motor fault snapshot status", timeout_s=5.0),
        ShellCommand("motor info live", timeout_s=5.0),
    ]


def scenario_boot_commission(args: argparse.Namespace) -> list[ShellCommand]:
    duration_s = max(10.0, (float(args.cycles) / max(float(args.boot_hz), 0.001)) + 12.0)
    return [
        ShellCommand("motor state status"),
        ShellCommand("motor state clear_error", timeout_s=2.0),
        ShellCommand("motor fault snapshot clear", timeout_s=2.0),
        ShellCommand("motor velocity target 0", timeout_s=1.5),
        ShellCommand("motor current iq 0", timeout_s=1.5),
        ShellCommand("motor disarm", timeout_s=1.5),
        ShellCommand("motor state idle", timeout_s=2.0),
        ShellCommand("motor safety timeout 0"),
        ShellCommand("motor encoder acquisition reset"),
        ShellCommand(
            f"motor commission boot {args.boot_current:.3f} {args.boot_hz:.3f} {args.cycles:.3f}",
            timeout_s=duration_s,
            note=(
                f"Boot commissioning: offset calibration has no rotation; encoder mapping "
                f"then rotates {args.cycles:.2f} rev at {args.boot_hz:.3f} Hz; "
                "+Iq validation should move briefly."
            ),
            require_success=True,
        ),
        ShellCommand("motor encoder control_status"),
        ShellCommand("motor encoder acquisition status"),
        ShellCommand("motor state status"),
        ShellCommand("motor velocity target 0", timeout_s=1.5),
        ShellCommand("motor current iq 0", timeout_s=1.5),
        ShellCommand("motor disarm", timeout_s=1.5),
        ShellCommand("motor state idle", timeout_s=2.0),
        ShellCommand("motor safety timeout 1000"),
        ShellCommand("motor encoder control_status"),
        ShellCommand("motor encoder acquisition status"),
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
        ShellCommand("motor fault snapshot clear", timeout_s=2.0),
        ShellCommand("motor velocity target 0", timeout_s=1.5),
        ShellCommand("motor current iq 0", timeout_s=1.5),
        ShellCommand("motor disarm", timeout_s=1.5),
        ShellCommand("motor state idle", timeout_s=2.0),
        ShellCommand("motor safety timeout 0"),
        ShellCommand("motor encoder acquisition reset", timeout_s=2.0),
        ShellCommand("motor arm", timeout_s=2.0),
        ShellCommand(
            f"motor commission encoder robust {args.boot_current:.3f} "
            f"{args.boot_hz:.3f} {args.cycles:.3f}{suffix}",
            timeout_s=duration_s,
            note=(
                f"Encoder mapping: generated-angle Id sweep at {args.boot_hz:.3f} Hz; "
                "slow visible rotation is expected."
            ),
            require_success=True,
        ),
        ShellCommand("motor commission encoder status", timeout_s=3.0),
        ShellCommand("motor commission encoder apply", timeout_s=3.0),
        ShellCommand("motor encoder control_status", timeout_s=3.0),
        ShellCommand("motor encoder acquisition status", timeout_s=3.0),
        ShellCommand("motor state status", timeout_s=3.0),
    ]


def _optional_velocity_pi_commands(args: argparse.Namespace) -> list[ShellCommand]:
    if args.velocity_pi_kp is not None or args.velocity_pi_ki is not None:
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

    return [
        ShellCommand(
            f"motor velocity pi bandwidth {args.velocity_pi_bandwidth_hz:.3f} "
            f"{args.velocity_pi_zeta:.3f} {args.velocity_pi_iq_limit:.3f}",
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
        ShellCommand("motor disarm", timeout_s=1.5),
        ShellCommand("motor state idle", timeout_s=2.0),
        ShellCommand("motor encoder acquisition status"),
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
        ShellCommand("motor disarm", timeout_s=1.5),
        ShellCommand("motor state idle", timeout_s=2.0),
        ShellCommand("motor encoder acquisition status"),
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
        ShellCommand("motor disarm", timeout_s=1.5),
        ShellCommand("motor state idle", timeout_s=2.0),
        ShellCommand("motor encoder acquisition status"),
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
        ShellCommand("motor fault snapshot clear", timeout_s=2.0),
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
        ShellCommand("motor encoder acquisition status"),
        ShellCommand("motor disarm"),
        ShellCommand("motor state idle", timeout_s=2.0),
    ]


def production_electrical_id_commands(args: argparse.Namespace,
                                      *,
                                      validate: bool) -> list[ShellCommand]:
    commands = [
        ShellCommand("motor commission electrical clear", timeout_s=2.0),
        ShellCommand("motor commission electrical plan", timeout_s=2.0),
        ShellCommand(
            f"motor commission electrical run {args.electrical_id_current:.3f} "
            f"{args.electrical_id_pulse:.3f} {args.electrical_id_samples}",
            timeout_s=max(20.0, args.electrical_id_samples * 0.08 + 8.0),
            note=(
                "Production electrical ID: bipolar Rs current injection and direct-voltage "
                "Ld/Lq pulses; no sustained rotation is expected."
            ),
            require_success=True,
        ),
        ShellCommand("motor commission electrical status", timeout_s=3.0),
        ShellCommand("motor commission electrical apply", timeout_s=3.0,
                     require_success=True),
    ]
    if validate:
        commands.extend([
            ShellCommand(
                f"motor commission electrical validate {args.electrical_id_current:.3f} "
                f"{args.electrical_id_validate_ms} {args.electrical_id_max_error:.3f}",
                timeout_s=max(8.0, args.electrical_id_validate_ms / 1000.0 + 4.0),
                require_success=True,
            ),
            ShellCommand("motor current gain get id", timeout_s=2.0),
            ShellCommand("motor current gain get iq", timeout_s=2.0),
        ])
    commands.append(ShellCommand("motor info measured", timeout_s=3.0))
    return commands


def _velocity_sweep_targets(args: argparse.Namespace) -> list[float]:
    if args.velocity_sweep_target_hz:
        return [float(target) for target in args.velocity_sweep_target_hz]

    magnitudes = (0.1, 0.3, 0.5, 1.0, 5.0)
    targets: list[float] = []
    for magnitude in magnitudes:
        targets.extend((magnitude, -magnitude))
    return targets


def _velocity_sweep_hold_s(args: argparse.Namespace, target_hz: float) -> float:
    min_hold_s = float(args.velocity_sweep_min_hold_ms) / 1000.0
    if abs(target_hz) <= 1.0e-6:
        return min_hold_s
    rotation_hold_s = float(args.velocity_sweep_rotations) / abs(target_hz)
    return max(min_hold_s, rotation_hold_s)


def _velocity_sweep_trace_decimation(hold_s: float) -> int:
    # Size the 512-sample trace window to cover the full measurement interval.
    decimation = int(ceil(max(hold_s, 0.001) * 20000.0 / 512.0))
    return max(1, min(decimation, 65535))


def _velocity_sweep_regulator_setup(args: argparse.Namespace,
                                    regulator: str) -> list[ShellCommand]:
    if regulator == "pi":
        return [
            ShellCommand("motor outer mode pi", timeout_s=2.0),
            ShellCommand(
                f"motor velocity pi bandwidth {args.velocity_pi_bandwidth_hz:.3f} "
                f"{args.velocity_pi_zeta:.3f} {args.velocity_sweep_iq_limit:.3f}",
                timeout_s=2.0,
            ),
            ShellCommand("motor velocity pi status", timeout_s=2.0),
        ]

    return [
        ShellCommand(
            f"motor velocity pi bandwidth {args.velocity_pi_bandwidth_hz:.3f} "
            f"{args.velocity_pi_zeta:.3f} {args.velocity_sweep_iq_limit:.3f}",
            timeout_s=2.0,
        ),
        ShellCommand(f"motor velocity mpr bandwidth {args.mpr_bandwidth_hz:.3f}",
                     timeout_s=2.0),
        ShellCommand("motor outer mode mpr", timeout_s=2.0),
        ShellCommand("motor velocity mpr status", timeout_s=2.0),
    ]


def scenario_velocity_sweep(args: argparse.Namespace) -> list[ShellCommand]:
    targets = _velocity_sweep_targets(args)
    commission_timeout_s = args.standard_commission_timeout_s
    if args.velocity_sweep_commission == "boot":
        commission_duration_s = max(
            10.0,
            (float(args.cycles) / max(float(args.boot_hz), 0.001)) + 12.0,
        )
        commission_commands = [
            ShellCommand(
                f"motor commission boot {args.boot_current:.3f} "
                f"{args.boot_hz:.3f} {args.cycles:.3f}",
                timeout_s=commission_duration_s,
                note=(
                    f"Boot commissioning before sweep: offset calibration has no rotation; "
                    f"mapping rotates {args.cycles:.2f} rev at {args.boot_hz:.3f} Hz."
                ),
                require_success=True,
            ),
        ]
    elif args.velocity_sweep_commission == "standard":
        commission_commands = []
        if not args.skip_production_electrical:
            commission_commands.extend(production_electrical_id_commands(args, validate=True))
        commission_commands.extend([
            ShellCommand(f"motor commission run {args.commission_profile} apply",
                         timeout_s=commission_timeout_s,
                         note=(
                             "Standard commissioning: R/L excitation does not rotate; "
                             "encoder mapping is a slow generated sweep; mechanical ID "
                             "moves forward/reverse."
                         ),
                         require_success=True),
            # The standard workflow can reject auto-tune repeatability while still
            # leaving valid flux/mechanical estimates staged. Apply those active
            # model values explicitly before deriving PI/MPR bandwidth settings.
            ShellCommand("motor commission apply", timeout_s=3.0),
            ShellCommand("motor commission status", timeout_s=3.0),
            ShellCommand("motor info measured", timeout_s=3.0),
        ])
    else:
        commission_commands = []

    commands = [
        ShellCommand("motor state clear_error", timeout_s=2.0),
        ShellCommand("motor fault snapshot clear", timeout_s=2.0),
        ShellCommand("motor velocity target 0", timeout_s=1.5),
        ShellCommand("motor current iq 0", timeout_s=1.5),
        ShellCommand("motor disarm", timeout_s=1.5),
        ShellCommand("motor state idle", timeout_s=2.0),
        ShellCommand("motor safety timeout 0"),
        ShellCommand("motor encoder acquisition reset", timeout_s=2.0),
        *commission_commands,
        ShellCommand("motor velocity dob enable 0", timeout_s=2.0),
        ShellCommand("motor commission detent clear", timeout_s=2.0),
        *_velocity_sweep_regulator_setup(args, args.velocity_sweep_regulator),
        ShellCommand("motor state mode velocity_encoder", timeout_s=2.0),
        ShellCommand("motor state online", timeout_s=4.0),
        ShellCommand("motor arm", timeout_s=2.0),
        ShellCommand("motor encoder control_status", timeout_s=3.0),
    ]

    for target_hz in targets:
        hold_s = _velocity_sweep_hold_s(args, target_hz)
        settle_s = float(args.velocity_sweep_settle_ms) / 1000.0
        decimation = _velocity_sweep_trace_decimation(hold_s)
        commands.extend([
            ShellCommand(f"motor velocity target {target_hz:.3f}",
                         timeout_s=2.0, settle_s=settle_s),
            ShellCommand("motor encoder trace clear", timeout_s=2.0),
            ShellCommand(f"motor encoder trace start {decimation}", timeout_s=2.0),
            ShellCommand("kernel uptime", timeout_s=2.0, settle_s=hold_s),
            ShellCommand("motor encoder trace stop", timeout_s=2.0),
            ShellCommand("motor velocity status", timeout_s=2.0),
            ShellCommand("motor encoder trace summary", timeout_s=3.0),
            ShellCommand("motor velocity target 0", timeout_s=2.0,
                         settle_s=float(args.velocity_sweep_stop_ms) / 1000.0),
        ])

    commands.extend([
        ShellCommand("motor current iq 0", timeout_s=1.5),
        ShellCommand("motor disarm", timeout_s=1.5),
        ShellCommand("motor state idle", timeout_s=2.0),
        ShellCommand("motor encoder acquisition status", timeout_s=3.0),
        ShellCommand("motor state status", timeout_s=3.0),
    ])
    return commands


def _feature_velocity_validate_commands(args: argparse.Namespace,
                                        label: str,
                                        outer_mode: str,
                                        detent_enable: bool,
                                        dob_enable: bool) -> list[ShellCommand]:
    commands = [
        ShellCommand("motor velocity target 0", timeout_s=1.5),
        ShellCommand("motor current iq 0", timeout_s=1.5),
        ShellCommand("motor state idle", timeout_s=2.0),
        ShellCommand("motor arm", timeout_s=2.0),
        ShellCommand("motor velocity dob enable 0", timeout_s=2.0),
        ShellCommand(f"motor commission detent apply {1 if detent_enable else 0}",
                     timeout_s=3.0),
    ]
    if outer_mode == "mpr":
        commands.extend([
            ShellCommand(f"motor velocity mpr bandwidth {args.mpr_bandwidth_hz:.3f}",
                         timeout_s=2.0),
            ShellCommand("motor outer mode mpr", timeout_s=2.0),
        ])
    else:
        commands.append(ShellCommand("motor outer mode pi", timeout_s=2.0))
    if dob_enable:
        commands.extend([
            ShellCommand("motor velocity dob defaults safe", timeout_s=2.0),
            ShellCommand("motor velocity dob status", timeout_s=2.0),
            ShellCommand("motor velocity dob enable 1", timeout_s=2.0),
            ShellCommand("motor velocity dob status", timeout_s=2.0),
        ])
    commands.extend([
        ShellCommand("motor outer status", timeout_s=2.0),
        ShellCommand(f"motor commission validate velocity {args.velocity_hz:.3f} "
                     f"{args.velocity_hold_ms} active",
                     timeout_s=max(12.0, args.velocity_hold_ms / 1000.0 * 9.0 + 4.0)),
        ShellCommand("motor velocity target 0", timeout_s=1.5),
        ShellCommand("motor current iq 0", timeout_s=1.5),
        ShellCommand("motor velocity dob enable 0", timeout_s=2.0),
        ShellCommand("motor outer status", timeout_s=2.0),
    ])
    return [ShellCommand("kernel uptime", timeout_s=1.5)] + commands


def scenario_mpr_dob_detent(args: argparse.Namespace) -> list[ShellCommand]:
    detent_timeout_s = max(
        20.0,
        (2.0 * float(args.detent_cycles) / max(abs(float(args.detent_hz)), 0.001)) + 20.0,
    )
    commands = [
        ShellCommand("motor state status"),
        ShellCommand("motor state clear_error", timeout_s=2.0),
        ShellCommand("motor fault snapshot clear", timeout_s=2.0),
        ShellCommand("motor velocity target 0", timeout_s=1.5),
        ShellCommand("motor current iq 0", timeout_s=1.5),
        ShellCommand("motor disarm", timeout_s=1.5),
        ShellCommand("motor state idle", timeout_s=2.0),
        ShellCommand("motor safety timeout 0"),
        ShellCommand("motor encoder acquisition reset", timeout_s=2.0),
        *([] if args.skip_production_electrical
          else production_electrical_id_commands(args, validate=True)),
        ShellCommand(f"motor commission run {args.commission_profile} apply",
                     timeout_s=args.standard_commission_timeout_s,
                     note=(
                         "Standard commissioning: R/L excitation does not rotate; "
                         "encoder mapping is a slow generated sweep; mechanical ID "
                         "moves forward/reverse."
                     ),
                     require_success=True),
        ShellCommand("motor commission status", timeout_s=3.0),
        ShellCommand("motor encoder control_status", timeout_s=3.0),
        ShellCommand("motor encoder acquisition status", timeout_s=3.0),
        ShellCommand("motor arm", timeout_s=2.0),
        ShellCommand("motor outer mode pi", timeout_s=2.0),
        ShellCommand("motor velocity dob enable 0", timeout_s=2.0),
        ShellCommand("motor commission detent clear", timeout_s=2.0),
        ShellCommand(
            f"motor commission detent run {args.detent_hz:.3f} "
            f"{args.detent_cycles:.3f} {args.detent_decimation} "
            f"{args.detent_iq_limit:.3f}",
            timeout_s=detent_timeout_s,
        ),
        ShellCommand("motor commission detent status", timeout_s=3.0),
        ShellCommand("motor commission detent dump 0 16", timeout_s=4.0),
        ShellCommand("motor commission detent apply 0", timeout_s=3.0),
        ShellCommand(
            f"motor commission detent validate {args.detent_validate_hz:.3f} "
            f"{args.detent_validate_ms}",
            timeout_s=max(12.0, args.detent_validate_ms / 1000.0 * 5.0 + 8.0),
        ),
    ]

    combo_names = args.feature_combo
    combo_builders = {
        "pi": lambda: _feature_velocity_validate_commands(args, "pi", "pi", False, False),
        "pi_detent": lambda: _feature_velocity_validate_commands(args, "pi_detent", "pi", True, False),
        "pi_dob": lambda: _feature_velocity_validate_commands(args, "pi_dob", "pi", False, True),
        "mpr": lambda: _feature_velocity_validate_commands(args, "mpr", "mpr", False, False),
        "mpr_detent": lambda: _feature_velocity_validate_commands(args, "mpr_detent", "mpr", True, False),
        "mpr_dob": lambda: _feature_velocity_validate_commands(args, "mpr_dob", "mpr", False, True),
        "mpr_dob_detent": lambda: _feature_velocity_validate_commands(
            args, "mpr_dob_detent", "mpr", True, True),
    }
    for name in combo_names:
        commands.extend(combo_builders[name]())
    commands.extend([
        ShellCommand("motor commission detent status", timeout_s=3.0),
        ShellCommand("motor velocity mpr status", timeout_s=2.0),
        ShellCommand("motor velocity dob status", timeout_s=2.0),
        ShellCommand("motor outer status", timeout_s=2.0),
        ShellCommand("motor encoder acquisition status", timeout_s=3.0),
        ShellCommand("motor state status", timeout_s=3.0),
    ])
    return commands


def scenario_mechanical_id_v2(args: argparse.Namespace) -> list[ShellCommand]:
    commands = [
        ShellCommand("motor state status"),
        ShellCommand("motor state clear_error", timeout_s=2.0),
        ShellCommand("motor fault snapshot clear", timeout_s=2.0),
        ShellCommand("motor velocity target 0", timeout_s=1.5),
        ShellCommand("motor current iq 0", timeout_s=1.5),
        ShellCommand("motor current id 0", timeout_s=1.5),
        ShellCommand("motor disarm", timeout_s=1.5),
        ShellCommand("motor state idle", timeout_s=2.0),
        ShellCommand("motor safety timeout 0"),
        ShellCommand("motor encoder acquisition reset", timeout_s=2.0),
        *([] if args.skip_production_electrical
          else production_electrical_id_commands(args, validate=True)),
        ShellCommand(
            f"motor commission run {args.mechanical_id_profile}",
            timeout_s=args.standard_commission_timeout_s,
            note=(
                "Mechanical ID v2: standard commissioning runs encoder mapping, "
                "flux ID, staged friction plateau fit, transient inertia fit, "
                "and confidence gating. Apply is intentionally not requested here."
            ),
            require_success=True,
        ),
        ShellCommand("motor commission status", timeout_s=5.0),
        ShellCommand("motor encoder acquisition status", timeout_s=5.0),
        ShellCommand("motor state status", timeout_s=5.0),
        ShellCommand("motor safety timeout 1000"),
    ]
    return commands


def scenario_custom(args: argparse.Namespace) -> list[ShellCommand]:
    commands: list[ShellCommand] = []
    for cmd in args.command:
        timeout_s = args.command_timeout
        if cmd.strip().startswith("motor commission boot"):
            timeout_s = max(timeout_s, args.boot_commission_timeout_s)
        elif cmd.strip().startswith("motor commission run"):
            timeout_s = max(timeout_s, args.standard_commission_timeout_s)
        commands.append(ShellCommand(cmd, timeout_s=timeout_s))
    return commands


def scenario_recovery_status(args: argparse.Namespace) -> list[ShellCommand]:
    return [
        ShellCommand("motor state recovery", timeout_s=5.0),
        ShellCommand("motor fault recovery", timeout_s=5.0),
        ShellCommand("motor gate status", timeout_s=5.0),
        ShellCommand("motor encoder acquisition status", timeout_s=5.0),
        ShellCommand("motor state status", timeout_s=5.0),
    ]


def scenario_production_electrical_id(args: argparse.Namespace) -> list[ShellCommand]:
    return [
        ShellCommand("motor state clear_error", timeout_s=2.0),
        ShellCommand("motor fault snapshot clear", timeout_s=2.0),
        ShellCommand("motor velocity target 0", timeout_s=1.5),
        ShellCommand("motor current iq 0", timeout_s=1.5),
        ShellCommand("motor current id 0", timeout_s=1.5),
        ShellCommand("motor disarm", timeout_s=1.5),
        ShellCommand("motor state idle", timeout_s=2.0),
        ShellCommand("motor safety timeout 0"),
        *production_electrical_id_commands(args, validate=True),
        ShellCommand("motor state status", timeout_s=3.0),
        ShellCommand("motor safety timeout 1000"),
    ]


SCENARIOS = {
    "status": (scenario_status, False),
    "boot-commission": (scenario_boot_commission, True),
    "current-validate": (scenario_current_validate, True),
    "encoder-robust": (scenario_encoder_robust, True),
    "encoder-validate": (scenario_encoder_validate, True),
    "encoder-trace-open-loop": (scenario_encoder_trace_open_loop, True),
    "mechanical-id-v2": (scenario_mechanical_id_v2, True),
    "mpr-dob-detent": (scenario_mpr_dob_detent, True),
    "position-validate": (scenario_position_validate, True),
    "production-electrical-id": (scenario_production_electrical_id, True),
    "recovery-status": (scenario_recovery_status, False),
    "velocity-sweep": (scenario_velocity_sweep, True),
    "velocity-validate": (scenario_velocity_validate, True),
    # A custom command list can include any live motor command. Treat it as a
    # live scenario so the caller must opt in and the standard stop/status
    # postlude always runs.
    "custom": (scenario_custom, True),
}


def _combined_text(results: Sequence[ShellResult]) -> str:
    return "\n".join(result.response for result in results)


def _command_success_failure_reason(cmd: ShellCommand, response: str) -> str | None:
    if not cmd.require_success:
        return None

    lower = response.lower()
    command = cmd.command
    if command.startswith("motor commission boot"):
        if "boot commissioning complete" in lower:
            return None
        return "boot commissioning did not complete"

    if command.startswith("motor commission run"):
        if ("standard commissioning workflow complete" in lower or
            "auto commission complete" in lower or
            "auto commission complete and applied" in lower):
            return None
        return "standard commissioning did not complete"

    if command.startswith("motor commission encoder robust"):
        if "encoder mapping result: valid=yes" in lower:
            return None
        return "encoder mapping did not produce a valid result"

    if command.startswith("motor commission encoder apply"):
        if "encoder mapping applied:" in lower:
            return None
        return "encoder mapping was not applied"

    if command.startswith("motor commission validate"):
        if "complete" in lower:
            return None
        return "commissioning validation did not complete"

    if command.startswith("motor commission electrical run"):
        if ("production inductance staged" in lower or
            "demodulated inductance staged" in lower):
            return None
        return "production electrical ID did not stage inductance"

    if command.startswith("motor commission electrical apply"):
        if "production electrical id applied" in lower:
            return None
        return "production electrical ID was not applied"

    if command.startswith("motor commission electrical validate"):
        if "validation: pass" in lower:
            return None
        return "production electrical current-step validation failed"

    failure_patterns = (
        " failed",
        "failed ",
        "fault",
        "error state",
        "entering error",
        "cannot arm",
    )
    if any(pattern in lower for pattern in failure_patterns):
        return "command response contains failure/fault text"
    return None


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


def _parse_encoder_fault_reason(response: str) -> tuple[str, int] | None:
    match = re.search(
        r"^\s*Enc reason:\s+([a-zA-Z0-9_]+)\s+\(([-0-9]+)\)",
        response,
        re.MULTILINE,
    )
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


def _parse_field_value(response: str, label: str) -> str | None:
	match = re.search(rf"^\s*{re.escape(label)}:\s+(.+?)\s*$", response, re.MULTILINE)
	if match is None:
		return None
	return match.group(1).strip()


def _parse_mechanical_v2_status(response: str) -> dict[str, float | int | str | bool] | None:
    v2 = re.search(
        r"Mech v2:\s+valid=(YES|NO)\s+friction=(YES|NO)\s+inertia=(YES|NO)\s+"
        r"detent=([a-zA-Z0-9_]+)\s+accelN=(\d+)",
        response,
    )
    qual = re.search(
        r"Mech v2 qual:\s+friction_rms=([-+0-9.eE]+)\s+Nm\s+"
        r"inertia_rms=([-+0-9.eE]+)\s+Nm\s+J/fallback=([-+0-9.eE]+)",
        response,
    )
    repeat = re.search(
        r"Mech repeat:\s+runs=(\d+)\s+conf=([-+0-9.eE]+)",
        response,
    )
    reject = re.search(r"Mech reject:\s+reason=([a-zA-Z0-9_]+)", response)
    if v2 is None:
        return None

    values: dict[str, float | int | str | bool] = {
        "valid": v2.group(1) == "YES",
        "friction_valid": v2.group(2) == "YES",
        "inertia_valid": v2.group(3) == "YES",
        "detent": v2.group(4),
        "accel_count": int(v2.group(5)),
    }
    if qual is not None:
        values.update({
            "friction_rms_nm": float(qual.group(1)),
            "inertia_rms_nm": float(qual.group(2)),
            "j_fallback_ratio": float(qual.group(3)),
        })
    if repeat is not None:
        values.update({
            "runs": int(repeat.group(1)),
            "confidence": float(repeat.group(2)),
        })
    if reject is not None:
        values["reject_reason"] = reject.group(1)
    return values

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


def _parse_velocity_status(response: str) -> dict[str, float | str] | None:
    target = re.search(r"^\s*Target:\s+([-+0-9.]+)\s+Hz", response, re.MULTILINE)
    ref = re.search(r"^\s*Ref:\s+([-+0-9.]+)\s+Hz", response, re.MULTILINE)
    measured = re.search(r"^\s*Measured:\s+([-+0-9.]+)\s+Hz", response, re.MULTILINE)
    error = re.search(r"^\s*Error:\s+([-+0-9.]+)\s+Hz", response, re.MULTILINE)
    iq_limit = re.search(r"^\s*Iq limit:\s+([-+0-9.]+)\s+A", response, re.MULTILINE)
    outer = re.search(r"^\s*Outer loop:\s+([A-Z]+)", response, re.MULTILINE)
    if target is None or ref is None or measured is None or error is None:
        return None
    return {
        "target_hz": float(target.group(1)),
        "ref_hz": float(ref.group(1)),
        "measured_hz": float(measured.group(1)),
        "error_hz": float(error.group(1)),
        "iq_limit_a": float(iq_limit.group(1)) if iq_limit is not None else 0.0,
        "outer_loop": outer.group(1) if outer is not None else "",
    }


def _parse_open_loop_trace_summary(response: str) -> dict[str, float | int] | None:
    stored = re.search(r"Stored:\s+(\d+)\s*/\s*(\d+)", response)
    raw_delta = re.search(r"Raw delta:\s+([-+0-9]+)\s+mdeg,\s+avg\s+([-+0-9]+)\s+mHz",
                          response)
    ctrl_delta = re.search(r"Ctrl delta:\s+([-+0-9]+)\s+mdeg,\s+avg\s+([-+0-9]+)\s+mHz",
                           response)
    counts = re.search(
        r"Counts:\s+clean=(\d+)\s+fresh=(\d+)\s+ctrl_en=(\d+)\s+warn=(\d+)\s+err=(\d+)\s+io=(\d+)",
        response,
    )
    drops = re.search(r"Delta drops:\s+(\d+)", response)
    if stored is None or raw_delta is None or ctrl_delta is None or counts is None:
        return None
    return {
        "stored": int(stored.group(1)),
        "capacity": int(stored.group(2)),
        "raw_delta_mdeg": int(raw_delta.group(1)),
        "raw_avg_mhz": int(raw_delta.group(2)),
        "ctrl_delta_mdeg": int(ctrl_delta.group(1)),
        "ctrl_avg_mhz": int(ctrl_delta.group(2)),
        "clean": int(counts.group(1)),
        "fresh": int(counts.group(2)),
        "ctrl_en": int(counts.group(3)),
        "warn": int(counts.group(4)),
        "err": int(counts.group(5)),
        "io": int(counts.group(6)),
        "delta_drops": int(drops.group(1)) if drops is not None else 0,
    }


def _parse_target_command(command: str) -> float | None:
    match = re.match(r"motor velocity target\s+([-+0-9.]+)$", command)
    if match is None:
        return None
    return float(match.group(1))


def _collect_velocity_sweep_samples(
    results: Sequence[ShellResult],
) -> list[dict[str, float | int | str | bool]]:
    samples: list[dict[str, float | int | str | bool]] = []
    active_target: float | None = None
    active_status: dict[str, float | str] | None = None

    for result in results:
        target = _parse_target_command(result.command)
        if target is not None:
            active_target = target if abs(target) > 1.0e-6 else None
            active_status = None
            continue

        if active_target is None:
            continue

        if result.command == "motor velocity status":
            active_status = _parse_velocity_status(result.response)
            continue

        if result.command == "motor encoder trace summary":
            trace = _parse_open_loop_trace_summary(result.response)
            if trace is None:
                samples.append({
                    "target_hz": active_target,
                    "parsed": False,
                })
                active_target = None
                active_status = None
                continue

            avg_hz = float(trace["ctrl_avg_mhz"]) / 1000.0
            status_measured_hz = (
                float(active_status["measured_hz"])
                if active_status is not None and "measured_hz" in active_status
                else avg_hz
            )
            samples.append({
                "target_hz": active_target,
                "avg_hz": avg_hz,
                "status_measured_hz": status_measured_hz,
                "status_ref_hz": (
                    float(active_status["ref_hz"])
                    if active_status is not None and "ref_hz" in active_status
                    else 0.0
                ),
                "status_error_hz": (
                    float(active_status["error_hz"])
                    if active_status is not None and "error_hz" in active_status
                    else active_target - avg_hz
                ),
                "outer_loop": (
                    str(active_status["outer_loop"])
                    if active_status is not None and "outer_loop" in active_status
                    else ""
                ),
                "iq_limit_a": (
                    float(active_status["iq_limit_a"])
                    if active_status is not None and "iq_limit_a" in active_status
                    else 0.0
                ),
                "ctrl_delta_mdeg": int(trace["ctrl_delta_mdeg"]),
                "raw_delta_mdeg": int(trace["raw_delta_mdeg"]),
                "stored": int(trace["stored"]),
                "clean": int(trace["clean"]),
                "fresh": int(trace["fresh"]),
                "warn": int(trace["warn"]),
                "err": int(trace["err"]),
                "io": int(trace["io"]),
                "delta_drops": int(trace["delta_drops"]),
                "parsed": True,
            })
            active_target = None
            active_status = None

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
    max_ref_err = max(
        abs(float(sample["target_hz"]) - float(sample["ref_hz"]))
        for sample in nonzero_samples
    ) if nonzero_samples else 0.0
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
        max_ref_err <= args.max_velocity_ref_error_hz and
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
               "max_ref_err_hz": max_ref_err,
               "max_meas_abs_hz": max_meas_abs_hz,
               "overshoot_ratio": overshoot_ratio,
               "max_warn": max_warn,
               "max_err": max_err,
               "wrong_sign_samples": len(wrong_sign),
           })


def _evaluate_open_loop_trace(args: argparse.Namespace, checks: list[VerdictCheck],
                              results: Sequence[ShellResult]) -> None:
    trace_response = _last_response(results, "motor encoder trace summary")
    summary = _parse_open_loop_trace_summary(trace_response)
    if summary is None:
        checks.append(VerdictCheck("open_loop_trace", "FAIL",
                                   "Open-loop trace summary not parsed"))
        return

    min_delta_mdeg = int(round(args.min_trace_control_delta_deg * 1000.0))
    enough_motion = abs(int(summary["ctrl_delta_mdeg"])) >= min_delta_mdeg
    clean = (
        int(summary["clean"]) > 0 and
        int(summary["err"]) <= args.max_sample_errors and
        int(summary["warn"]) <= args.max_sample_warnings and
        int(summary["io"]) <= args.max_transport_errors and
        int(summary["delta_drops"]) == 0
    )
    _check(checks, "open_loop_trace", enough_motion and clean,
           "Open-loop trace moved and samples are clean" if enough_motion and clean
           else "Open-loop trace motion or sample quality failed",
           summary)


def _evaluate_all_velocity_validations(args: argparse.Namespace, checks: list[VerdictCheck],
                                       results: Sequence[ShellResult]) -> None:
    validation_results = [
        result for result in results
        if result.command.startswith("motor commission validate velocity")
    ]
    if not validation_results:
        checks.append(VerdictCheck("velocity_validation_matrix", "FAIL",
                                   "No velocity validation commands were run"))
        return

    failed = 0
    parsed = 0
    max_abs_err = 0.0
    max_ref_err = 0.0
    max_warn = 0
    max_err = 0
    for idx, result in enumerate(validation_results):
        samples = _parse_velocity_samples(result.response)
        if not samples:
            failed += 1
            checks.append(VerdictCheck(f"velocity_validation_{idx}", "FAIL",
                                       "Velocity validation samples not parsed"))
            continue
        parsed += 1
        nonzero_samples = [
            sample for sample in samples
            if abs(float(sample["target_hz"])) > 1.0e-6
        ]
        sample_max_err = max(abs(float(sample["err_hz"])) for sample in samples)
        sample_max_ref_err = max(
            abs(float(sample["target_hz"]) - float(sample["ref_hz"]))
            for sample in nonzero_samples
        ) if nonzero_samples else 0.0
        sample_max_warn = max(int(sample["warn"]) for sample in samples)
        sample_max_sample_err = max(int(sample["err"]) for sample in samples)
        max_abs_err = max(max_abs_err, sample_max_err)
        max_ref_err = max(max_ref_err, sample_max_ref_err)
        max_warn = max(max_warn, sample_max_warn)
        max_err = max(max_err, sample_max_sample_err)
        ok = (
            sample_max_err <= args.max_velocity_error_hz and
            sample_max_ref_err <= args.max_velocity_ref_error_hz and
            sample_max_warn <= args.max_sample_warnings and
            sample_max_sample_err <= args.max_sample_errors
        )
        if not ok:
            failed += 1
        _check(checks, f"velocity_validation_{idx}", ok,
               "Velocity validation samples are within thresholds" if ok
               else "Velocity validation samples exceed thresholds",
               {
                   "samples": len(samples),
                   "max_abs_err_hz": sample_max_err,
                   "max_ref_err_hz": sample_max_ref_err,
                   "max_warn": sample_max_warn,
                   "max_err": sample_max_sample_err,
               })

    _check(checks, "velocity_validation_matrix", failed == 0,
           "All feature-combination velocity validations passed" if failed == 0
           else "One or more feature-combination velocity validations failed",
           {
            "validations": len(validation_results),
            "parsed": parsed,
            "failed": failed,
            "max_abs_err_hz": max_abs_err,
            "max_ref_err_hz": max_ref_err,
            "max_warn": max_warn,
            "max_err": max_err,
        })


def _evaluate_velocity_sweep(args: argparse.Namespace, checks: list[VerdictCheck],
                             results: Sequence[ShellResult]) -> None:
    samples = _collect_velocity_sweep_samples(results)
    expected_targets = _velocity_sweep_targets(args)
    if len(samples) != len(expected_targets):
        checks.append(VerdictCheck(
            "velocity_sweep_sample_count",
            "FAIL",
            f"Parsed {len(samples)} sweep samples, expected {len(expected_targets)}",
            {"parsed": len(samples), "expected": len(expected_targets)},
        ))
        return

    failed = 0
    max_abs_err = 0.0
    max_warn = 0
    max_err = 0
    max_io = 0
    worst_target = 0.0
    min_motion_ratio = 999.0
    required_motion_mdeg = int(
        round(max(float(args.velocity_sweep_rotations), 0.0) * 360000.0 *
              float(args.velocity_sweep_min_motion_fraction))
    )

    for idx, sample in enumerate(samples):
        target_hz = float(sample.get("target_hz", 0.0))
        if not bool(sample.get("parsed", False)):
            failed += 1
            checks.append(VerdictCheck(
                f"velocity_sweep_{idx}",
                "FAIL",
                "Sweep trace summary not parsed",
                {"target_hz": target_hz},
            ))
            continue

        avg_hz = float(sample["avg_hz"])
        abs_err = abs(target_hz - avg_hz)
        err_limit = max(
            float(args.velocity_sweep_max_error_hz),
            abs(target_hz) * float(args.velocity_sweep_max_error_ratio),
        )
        clean = (
            int(sample["warn"]) <= args.max_sample_warnings and
            int(sample["err"]) <= args.max_sample_errors and
            int(sample["io"]) <= args.max_transport_errors and
            int(sample["delta_drops"]) == 0
        )
        sign_ok = target_hz * avg_hz > 0.0
        motion_mdeg = abs(int(sample["ctrl_delta_mdeg"]))
        motion_ok = motion_mdeg >= required_motion_mdeg
        ok = abs_err <= err_limit and sign_ok and motion_ok and clean

        if not ok:
            failed += 1
        if abs_err > max_abs_err:
            max_abs_err = abs_err
            worst_target = target_hz
        max_warn = max(max_warn, int(sample["warn"]))
        max_err = max(max_err, int(sample["err"]))
        max_io = max(max_io, int(sample["io"]))
        if required_motion_mdeg > 0:
            min_motion_ratio = min(min_motion_ratio, motion_mdeg / required_motion_mdeg)

        _check(checks, f"velocity_sweep_{idx}", ok,
               "Sweep point tracked within threshold" if ok
               else "Sweep point failed tracking, direction, motion, or trace quality",
               {
                   "target_hz": target_hz,
                   "avg_hz": avg_hz,
                   "abs_err_hz": abs_err,
                   "err_limit_hz": err_limit,
                   "status_measured_hz": float(sample["status_measured_hz"]),
                   "status_ref_hz": float(sample["status_ref_hz"]),
                   "ctrl_delta_mdeg": motion_mdeg,
                   "required_motion_mdeg": required_motion_mdeg,
                   "warn": int(sample["warn"]),
                   "err": int(sample["err"]),
                   "io": int(sample["io"]),
                   "delta_drops": int(sample["delta_drops"]),
               })

    _check(checks, "velocity_sweep", failed == 0,
           "All exact velocity sweep points passed" if failed == 0
           else "One or more exact velocity sweep points failed",
           {
               "samples": len(samples),
               "failed": failed,
               "max_abs_err_hz": max_abs_err,
               "worst_target_hz": worst_target,
               "max_warn": max_warn,
               "max_err": max_err,
               "max_io": max_io,
               "min_motion_ratio": min_motion_ratio if min_motion_ratio != 999.0 else 0.0,
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


def _evaluate_detent_capture(checks: list[VerdictCheck],
                             results: Sequence[ShellResult]) -> None:
    status_response = _last_response(results, "motor commission detent status")
    run_response = _last_response(results, "motor commission detent run")
    response = "\n".join((run_response, status_response))

    raw_valid = re.search(r"raw=PASS", response) is not None
    fill_valid = re.search(r"fill=PASS", response) is not None
    apply_valid = re.search(r"apply=PASS", response) is not None
    bins_match = re.search(r"bins=(\d+)/(\d+)", response)
    raw_match = re.search(r"raw=(\d+)", response)
    populated_bins = int(bins_match.group(1)) if bins_match else 0
    total_bins = int(bins_match.group(2)) if bins_match else 0
    raw_bins = int(raw_match.group(1)) if raw_match else 0

    _check(checks, "detent_capture_apply_valid", apply_valid,
           "Detent map passed apply-quality gate" if apply_valid
           else "Detent map did not pass apply-quality gate",
           {
               "raw_valid": raw_valid,
               "fill_valid": fill_valid,
               "apply_valid": apply_valid,
               "populated_bins": populated_bins,
               "raw_bins": raw_bins,
               "total_bins": total_bins,
           })


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
    state_encoder_fault_reason = _parse_encoder_fault_reason(state_response)
    if motor_error is None:
        checks.append(VerdictCheck("motor_error_none", "INCONCLUSIVE",
                                   "No motor state status error field parsed"))
    else:
        error_name, error_code = motor_error
        _check(checks, "motor_error_none", error_name == "NONE" and error_code == 0,
               f"Motor error is {error_name} ({error_code})",
               {"error": error_name, "code": error_code})

    transition_response = _last_response(results, "motor state transition")
    if transition_response:
        transition_result = _parse_field_value(transition_response, "Result")
        transition_reason = _parse_field_value(transition_response, "Reason")
        transition_ok = transition_result not in ("rejected", "fault", "timeout")
        _check(checks, "transition_not_rejected", transition_ok,
               f"Latest transition result is {transition_result or 'unknown'}",
               {
                   "result": transition_result or "",
                   "reason": transition_reason or "",
               })

    recovery_response = _last_response(results, "motor state recovery")
    if not recovery_response:
        recovery_response = _last_response(results, "motor fault recovery")
    if recovery_response:
        safe_ready = _parse_yes_no_field(recovery_response, "Safe idle ready")
        fault_latched = _parse_yes_no_field(recovery_response, "Fault latched")
        gate_req = _parse_yes_no_field(recovery_response, "Gate reset req")
        gate_done = _parse_yes_no_field(recovery_response, "Gate reset done")
        enc_req = _parse_yes_no_field(recovery_response, "Encoder rec req")
        enc_done = _parse_yes_no_field(recovery_response, "Encoder rec done")
        recovery_ok = (
            fault_latched is False or
            (
                (gate_req is not True or gate_done is True) and
                (enc_req is not True or enc_done is True) and
                safe_ready is True
            )
        )
        _check(checks, "recovery_ready", recovery_ok,
               "Recovery status is ready" if recovery_ok else "Recovery status needs action",
               {
                   "fault_latched": bool(fault_latched),
                   "gate_required": bool(gate_req),
                   "gate_done": bool(gate_done),
                   "encoder_required": bool(enc_req),
                   "encoder_done": bool(enc_done),
                   "safe_idle_ready": bool(safe_ready),
               })

    fault_response = _last_response(results, "motor fault snapshot status")
    fault_encoder_fault_reason = _parse_encoder_fault_reason(fault_response)
    encoder_fault_reason = fault_encoder_fault_reason or state_encoder_fault_reason
    if encoder_fault_reason is not None:
        reason_name, reason_code = encoder_fault_reason
        _info(checks, "encoder_fault_reason",
              f"Encoder fault reason is {reason_name} ({reason_code})",
              {"reason": reason_name, "code": reason_code})
    elif motor_error is not None and motor_error[0] == "ENCODER_FAULT":
        _check(checks, "encoder_fault_reason", False,
               "Motor is in ENCODER_FAULT but no encoder fault reason was parsed")

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

    if args.scenario == "mpr-dob-detent":
        complete = "Standard commissioning workflow complete" in text
        _check(checks, "standard_commission_complete", complete,
               "Standard commissioning completed" if complete
               else "Standard commissioning completion text not found")
        _evaluate_detent_capture(checks, results)
        _evaluate_all_velocity_validations(args, checks, results)

    if args.scenario == "mechanical-id-v2":
        status_response = _last_response(results, "motor commission status")
        mech_v2 = _parse_mechanical_v2_status(status_response)
        if mech_v2 is None:
            checks.append(VerdictCheck("mechanical_v2_status", "INCONCLUSIVE",
                                       "Mechanical v2 status fields were not parsed"))
        else:
            _check(checks, "mechanical_v2_has_accel",
                   int(mech_v2.get("accel_count", 0)) > 0,
                   "Windowed acceleration samples are present",
                   mech_v2)
            valid = bool(mech_v2.get("valid", False))
            confidence = float(mech_v2.get("confidence", 0.0))
            reject_reason = str(mech_v2.get("reject_reason", ""))
            if valid:
                _check(checks, "mechanical_v2_confidence_gate",
                       confidence >= args.min_mech_confidence,
                       "Accepted mechanical model meets confidence gate",
                       mech_v2)
                _check(checks, "mechanical_v2_subfits_valid",
                       bool(mech_v2.get("friction_valid", False)) and
                       bool(mech_v2.get("inertia_valid", False)),
                       "Friction and inertia subfits are valid",
                       mech_v2)
            else:
                _check(checks, "mechanical_v2_rejected_explicitly",
                       reject_reason not in ("", "none"),
                       f"Rejected mechanical model reports reason '{reject_reason}'",
                       mech_v2)

    if args.scenario == "velocity-sweep":
        if args.velocity_sweep_commission == "standard":
            complete = "Standard commissioning workflow complete" in text
            applied = "Commissioning results applied to active runtime parameters" in text
            _check(checks, "standard_commission_complete_or_estimates_applied",
                   complete or applied,
                   "Standard commissioning completed or valid estimates were applied"
                   if complete or applied
                   else "No standard commissioning completion/apply evidence found")
            _check(checks, "commission_estimates_applied", applied,
                   "Commissioning estimates applied to active model" if applied
                   else "Commissioning estimates were not applied to active model")
        elif args.velocity_sweep_commission == "boot":
            complete = "Boot commissioning complete" in text
            _check(checks, "boot_commission_complete", complete,
                   "Boot commissioning completed" if complete
                   else "Boot commissioning completion text not found")
        _evaluate_velocity_sweep(args, checks, results)

    if args.scenario in ("current-validate", "encoder-validate"):
        _evaluate_current_validation(args, checks, results)

    if args.scenario in ("velocity-validate", "encoder-validate", "position-validate"):
        _evaluate_velocity_validation(args, checks, results)

    if args.scenario == "position-validate" or (
        args.scenario == "encoder-validate" and args.include_position
    ):
        _evaluate_position_validation(checks, results)

    if args.scenario == "encoder-trace-open-loop":
        _evaluate_open_loop_trace(args, checks, results)

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
        ShellCommand("motor encoder acquisition status", timeout_s=2.0),
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
    parser.add_argument("--max-velocity-ref-error-hz", type=float, default=0.005,
                        help="Maximum |target-ref| error tolerated after each hold.")
    parser.add_argument("--max-velocity-overshoot-ratio", type=float, default=3.0,
                        help="Maximum |measured velocity| / |target velocity| during validation.")
    parser.add_argument("--min-velocity-tracking-fraction", type=float, default=0.10,
                        help="Minimum measured/target fraction for at least one nonzero sample.")
    parser.add_argument("--min-velocity-tracking-samples", type=int, default=1,
                        help="Minimum number of nonzero velocity samples that must move enough.")

    parser.add_argument("--boot-current", type=float, default=0.15)
    parser.add_argument("--boot-hz", type=float, default=0.10)
    parser.add_argument("--cycles", type=float, default=1.0)
    parser.add_argument("--bidirectional", action="store_true",
                        help="Use forward+reverse generated sweeps for encoder-robust.")
    parser.add_argument("--current-iq", type=float, default=0.03)
    parser.add_argument("--current-hold-ms", type=int, default=160)
    parser.add_argument("--velocity-hz", type=float, default=0.50)
    parser.add_argument("--velocity-hold-ms", type=int, default=1000)
    parser.add_argument("--mpr-bandwidth-hz", type=float, default=1.0,
                        help="Velocity MPR bandwidth used by mpr-dob-detent feature combos.")
    parser.add_argument("--velocity-pi-bandwidth-hz", type=float, default=10.0,
                        help="Model-based velocity PI bandwidth for velocity-sweep.")
    parser.add_argument("--velocity-pi-zeta", type=float, default=1.0,
                        help="Damping ratio for model-based velocity PI tuning.")
    parser.add_argument("--velocity-sweep-regulator", choices=("pi", "mpr"), default="pi",
                        help="Outer regulator under test for velocity-sweep.")
    parser.add_argument("--velocity-sweep-commission",
                        choices=("standard", "boot", "none"), default="standard",
                        help="Commissioning sequence to run before velocity-sweep.")
    parser.add_argument("--velocity-sweep-target-hz", type=float, action="append",
                        help="Exact velocity target for velocity-sweep. Can be repeated.")
    parser.add_argument("--velocity-sweep-rotations", type=float, default=1.0,
                        help="Minimum mechanical rotations measured at each nonzero target.")
    parser.add_argument("--velocity-sweep-min-hold-ms", type=int, default=1000,
                        help="Minimum trace hold time per sweep target.")
    parser.add_argument("--velocity-sweep-settle-ms", type=int, default=500,
                        help="Settling time after target command and before trace capture.")
    parser.add_argument("--velocity-sweep-stop-ms", type=int, default=500,
                        help="Settling time after commanding zero between sweep points.")
    parser.add_argument("--velocity-sweep-iq-limit", type=float, default=0.225,
                        help="Velocity controller Iq authority used by PI and MPR sweeps.")
    parser.add_argument("--velocity-sweep-max-error-hz", type=float, default=0.05,
                        help="Absolute velocity error limit for exact sweep points.")
    parser.add_argument("--velocity-sweep-max-error-ratio", type=float, default=0.15,
                        help="Relative velocity error limit for exact sweep points.")
    parser.add_argument("--velocity-sweep-min-motion-fraction", type=float, default=0.80,
                        help="Required fraction of requested full-rotation trace motion.")
    parser.add_argument("--velocity-pi-kp", type=float,
                        help="Optional velocity PI Kp to set before velocity validation.")
    parser.add_argument("--velocity-pi-ki", type=float,
                        help="Optional velocity PI Ki to set before velocity validation.")
    parser.add_argument("--velocity-pi-iq-limit", type=float, default=0.12,
                        help="Iq limit used with velocity PI set/bandwidth validation.")
    parser.add_argument("--include-position", action="store_true")
    parser.add_argument("--position-delta-deg", type=float, default=5.0)
    parser.add_argument("--position-hold-ms", type=int, default=2000)
    parser.add_argument("--standard-commission-timeout-s", type=float, default=120.0,
                        help="Timeout for 'motor commission run <profile> apply'.")
    parser.add_argument("--boot-commission-timeout-s", type=float, default=30.0,
                        help="Timeout for a custom 'motor commission boot ...' command.")
    parser.add_argument("--commission-profile", choices=("slow", "confirm"), default="confirm",
                        help="Auto-commissioning motion profile used by mpr-dob-detent.")
    parser.add_argument("--mechanical-id-profile", choices=("slow", "confirm"), default="confirm",
                        help="Standard commissioning profile used by mechanical-id-v2.")
    parser.add_argument("--min-mech-confidence", type=float, default=0.50,
                        help="Minimum confidence accepted for a valid mechanical v2 model.")
    parser.add_argument("--detent-hz", type=float, default=0.10,
                        help="Mechanical Hz for detent map capture.")
    parser.add_argument("--detent-cycles", type=float, default=10.0,
                        help="Forward/reverse cycles for detent map capture.")
    parser.add_argument("--detent-decimation", type=int, default=1,
                        help="Detent capture decimation.")
    parser.add_argument("--detent-iq-limit", type=float, default=0.12,
                        help="Velocity Iq limit during detent capture.")
    parser.add_argument("--detent-validate-hz", type=float, default=0.10,
                        help="Mechanical Hz for detent off/on validation.")
    parser.add_argument("--detent-validate-ms", type=int, default=3000,
                        help="Duration per detent validation pass.")
    parser.add_argument("--electrical-id-current", type=float, default=0.30,
                        help="D-axis current for production electrical Rs measurement.")
    parser.add_argument("--electrical-id-pulse", type=float, default=0.50,
                        help="Small direct D/Q voltage pulse for production electrical Ld/Lq measurement.")
    parser.add_argument("--electrical-id-samples", type=int, default=128,
                        help="Sample/repeat count for production electrical ID.")
    parser.add_argument("--electrical-id-validate-ms", type=int, default=300,
                        help="Hold time for production electrical current-step validation.")
    parser.add_argument("--electrical-id-max-error", type=float, default=0.01,
                        help="Average current error limit for production electrical validation.")
    parser.add_argument("--skip-production-electrical", action="store_true",
                        help="Skip production Rs/Ld/Lq ID before standard control-tuning scenarios.")
    parser.add_argument(
        "--feature-combo",
        action="append",
        choices=(
            "pi",
            "pi_detent",
            "pi_dob",
            "mpr",
            "mpr_detent",
            "mpr_dob",
            "mpr_dob_detent",
        ),
        default=None,
        help="Feature combo for mpr-dob-detent. Can be repeated. Default runs all.",
    )

    parser.add_argument("--open-loop-iq", type=float, default=0.12)
    parser.add_argument("--open-loop-hz", type=float, default=0.10)
    parser.add_argument("--trace-ms", type=int, default=1000)
    parser.add_argument("--trace-decimation", type=int, default=1)
    parser.add_argument("--trace-dump", type=int, default=32)
    parser.add_argument("--min-trace-control-delta-deg", type=float, default=0.05,
                        help="Minimum absolute control-angle motion for open-loop trace validation.")

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
    if args.feature_combo is None:
        args.feature_combo = [
            "pi",
            "pi_detent",
            "pi_dob",
            "mpr",
            "mpr_detent",
            "mpr_dob",
            "mpr_dob_detent",
        ]

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
                failure_reason = _command_success_failure_reason(cmd, response)
                if failure_reason is not None:
                    raise ScenarioAbort(cmd.command, failure_reason)
        except ScenarioAbort as exc:
            exit_code = 1
            print(f"Aborting scenario after required command failed: {exc}", file=sys.stderr)
            results.append(ShellResult(
                "hil abort",
                f"HIL scenario aborted after '{exc.command}': {exc.reason}\n",
            ))
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
