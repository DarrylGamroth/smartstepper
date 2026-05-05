#!/usr/bin/env python3
"""Repeatable HIL shell workflows over the Zephyr telnet shell.

The script intentionally uses only Python's socket module. telnetlib is removed
in newer Python versions, and the Zephyr telnet shell only needs minimal option
refusal plus line-oriented command/response handling.
"""

from __future__ import annotations

import argparse
import datetime as _dt
import os
import re
import socket
import sys
import time
from dataclasses import dataclass
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
        ShellCommand(
            f"motor commission validate current {args.current_iq:.3f} {args.current_hold_ms}",
            timeout_s=max(4.0, args.current_hold_ms / 1000.0 * 4.0 + 3.0),
        ),
        ShellCommand("motor encoder acquisition"),
        ShellCommand("motor state status"),
        ShellCommand(
            f"motor commission validate velocity {args.velocity_hz:.3f} {args.velocity_hold_ms}",
            timeout_s=max(12.0, args.velocity_hold_ms / 1000.0 * 9.0 + 4.0),
        ),
        ShellCommand("motor encoder acquisition"),
        ShellCommand("motor state status"),
    ])
    if args.include_position:
        cmds.extend([
            ShellCommand(
                f"motor commission validate position {args.position_delta_deg:.3f} {args.position_hold_ms}",
                timeout_s=max(12.0, args.position_hold_ms / 1000.0 * 3.0 + 6.0),
            ),
            ShellCommand("motor encoder acquisition"),
            ShellCommand("motor state status"),
        ])
    return cmds


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
        ShellCommand(f"motor velocity target {args.open_loop_hz:.3f}", timeout_s=duration_s + 1.0),
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
    "encoder-validate": (scenario_encoder_validate, True),
    "encoder-trace-open-loop": (scenario_encoder_trace_open_loop, True),
    "custom": (scenario_custom, False),
}


def stop_commands() -> list[ShellCommand]:
    return [
        ShellCommand("motor velocity target 0", timeout_s=1.5),
        ShellCommand("motor current iq 0", timeout_s=1.5),
        ShellCommand("motor disarm", timeout_s=1.5),
        ShellCommand("motor state idle", timeout_s=2.0),
        ShellCommand("motor safety timeout 1000", timeout_s=1.5),
    ]


def parse_args(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scenario", choices=sorted(SCENARIOS))
    parser.add_argument("--host", default=os.environ.get("CHOPPER_TELNET_HOST", "10.0.0.171"))
    parser.add_argument("--port", type=int, default=int(os.environ.get("CHOPPER_TELNET_PORT", "23")))
    parser.add_argument("--connect-timeout", type=float, default=5.0)
    parser.add_argument("--log-dir", default="hil_logs")
    parser.add_argument("--no-log", action="store_true")
    parser.add_argument("--yes-live-motion", action="store_true",
                        help="Required for scenarios that can energize or move the motor.")
    parser.add_argument("--leave-timeout-disabled", action="store_true")

    parser.add_argument("--boot-current", type=float, default=0.15)
    parser.add_argument("--boot-hz", type=float, default=0.05)
    parser.add_argument("--cycles", type=float, default=1.0)
    parser.add_argument("--current-iq", type=float, default=0.03)
    parser.add_argument("--current-hold-ms", type=int, default=160)
    parser.add_argument("--velocity-hz", type=float, default=0.05)
    parser.add_argument("--velocity-hold-ms", type=int, default=1000)
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
    with TelnetShell(args.host, args.port, args.connect_timeout, log_path) as shell:
        try:
            for cmd in commands:
                shell.run(cmd)
        except KeyboardInterrupt:
            exit_code = 130
            print("Interrupted; sending stop commands", file=sys.stderr)
        finally:
            if live_motion and not args.leave_timeout_disabled:
                for cmd in stop_commands():
                    try:
                        shell.run(cmd)
                    except Exception as exc:  # noqa: BLE001 - best-effort hardware stop path
                        print(f"WARN: stop command failed: {cmd.command}: {exc}", file=sys.stderr)
                        exit_code = exit_code or 1
    if log_path is not None:
        print(f"Log saved: {log_path}")
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
