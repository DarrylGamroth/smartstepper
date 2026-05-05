# HIL Telnet Scripts

`hil_telnet.py` runs repeatable Zephyr shell workflows over the device telnet
shell. It uses only Python's standard library socket module.

Default target:

```bash
python3 scripts/hil/hil_telnet.py status --host 10.0.0.171
```

Motion-producing scenarios require an explicit safety acknowledgement:

```bash
python3 scripts/hil/hil_telnet.py boot-commission --yes-live-motion
python3 scripts/hil/hil_telnet.py encoder-validate --yes-live-motion
python3 scripts/hil/hil_telnet.py encoder-trace-open-loop --yes-live-motion
```

Useful options:

```bash
python3 scripts/hil/hil_telnet.py encoder-validate \
  --yes-live-motion \
  --boot-current 0.15 \
  --boot-hz 0.05 \
  --cycles 1 \
  --current-iq 0.03 \
  --velocity-hz 0.05
```

Logs are saved under `hil_logs/` by default. Use `--no-log` to disable file
logging or `--log-dir <path>` to choose another location.

Machine-readable evidence:

```bash
python3 scripts/hil/hil_telnet.py status \
  --host 10.0.0.171 \
  --json-report hil_logs/status.json
```

Each scenario prints a final `PASS`, `FAIL`, or `INCONCLUSIVE` verdict and can
write the same checks to JSON with `--json-report`. The script exits nonzero on
`FAIL` or `INCONCLUSIVE`, so it can be used by regression scripts.

Parser-only tests:

```bash
python3 -m unittest scripts/hil/test_hil_telnet_parser.py
```

Thresholds for encoder errors, current motion, and velocity tracking are
configurable with options such as `--max-crc-errors`,
`--min-current-motion-deg`, and `--max-velocity-error-hz`.

Safety behavior:

- Live-motion scenarios refuse to run without `--yes-live-motion`.
- The script sends best-effort stop commands at the end of live-motion scenarios:
  `motor velocity target 0`, `motor current iq 0`, `motor disarm`,
  `motor state idle`, and `motor safety timeout 1000`.
- Use `--leave-timeout-disabled` only for manual debugging sessions where the
  command watchdog must remain disabled after the script exits.
