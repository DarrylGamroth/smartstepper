# HIL Telnet Scripts

`hil_telnet.py` runs repeatable Zephyr shell workflows over the device telnet
shell. It uses only Python's standard library socket module.

Default target:

```bash
python3 scripts/hil/hil_telnet.py status --host 10.0.0.44
python3 scripts/hil/hil_telnet.py recovery-status --host 10.0.0.44
```

Motion-producing scenarios require an explicit safety acknowledgement:

```bash
python3 scripts/hil/hil_telnet.py boot-commission --yes-live-motion
python3 scripts/hil/hil_telnet.py current-validate --yes-live-motion
python3 scripts/hil/hil_telnet.py velocity-validate --yes-live-motion
python3 scripts/hil/hil_telnet.py position-validate --yes-live-motion
python3 scripts/hil/hil_telnet.py encoder-robust --yes-live-motion
python3 scripts/hil/hil_telnet.py encoder-validate --yes-live-motion
python3 scripts/hil/hil_telnet.py encoder-trace-open-loop --yes-live-motion
python3 scripts/hil/hil_telnet.py mpr-dob-detent --yes-live-motion
```

Useful options:

```bash
python3 scripts/hil/hil_telnet.py encoder-validate \
  --yes-live-motion \
  --boot-current 0.15 \
  --boot-hz 0.10 \
  --cycles 1 \
  --current-iq 0.03 \
  --velocity-hz 0.50
```

Use the split scenarios while bringing up encoder control. They run the same
boot mapping gate but isolate the failing layer and still send stop commands at
script exit:

```bash
python3 scripts/hil/hil_telnet.py current-validate --yes-live-motion \
  --boot-current 0.15 --boot-hz 0.10 --cycles 1 \
  --current-iq 0.03 --current-hold-ms 160

python3 scripts/hil/hil_telnet.py velocity-validate --yes-live-motion \
  --boot-current 0.15 --boot-hz 0.10 --cycles 1 \
  --velocity-pi-kp 0.100 --velocity-pi-ki 0.250 --velocity-pi-iq-limit 0.120 \
  --velocity-hz 0.50 --velocity-hold-ms 1000
```

Use the robust encoder-mapping scenario to test only generated-sweep mapping
and apply, without running current/velocity/position validation afterward:

```bash
python3 scripts/hil/hil_telnet.py encoder-robust --yes-live-motion \
  --boot-current 0.15 --boot-hz 0.10 --cycles 1 --bidirectional
```

Use the advanced motion scenario after basic encoder validation is passing. It
runs standard commissioning, captures a forward/reverse detent map, validates
detent off/on ripple, then checks selected PI/MPR/DOB/detent combinations with
the same velocity-validation command:

```bash
python3 scripts/hil/hil_telnet.py mpr-dob-detent \
  --host 10.0.0.44 \
  --yes-live-motion \
  --commission-profile confirm \
  --detent-hz 0.10 \
  --detent-cycles 10 \
  --mpr-bandwidth-hz 1.0 \
  --velocity-hz 0.50 \
  --velocity-hold-ms 1000
```

Run a shorter subset while tuning:

```bash
python3 scripts/hil/hil_telnet.py mpr-dob-detent \
  --host 10.0.0.44 \
  --yes-live-motion \
  --feature-combo pi \
  --feature-combo mpr \
  --feature-combo mpr_detent \
  --mpr-bandwidth-hz 1.0
```

Logs are saved under `hil_logs/` by default. Use `--no-log` to disable file
logging or `--log-dir <path>` to choose another location.

Machine-readable evidence:

```bash
python3 scripts/hil/hil_telnet.py status \
  --host 10.0.0.44 \
  --json-report hil_logs/status.json
```

Each scenario prints a final `PASS`, `FAIL`, or `INCONCLUSIVE` verdict and can
write the same checks to JSON with `--json-report`. The script exits nonzero on
`FAIL` or `INCONCLUSIVE`, so it can be used by regression scripts.

Parser-only tests:

```bash
python3 -m unittest scripts/hil/test_hil_telnet_parser.py
```

Regression gate:

```bash
scripts/hil/run_hil_gate.sh --host 10.0.0.44
```

Useful non-motion diagnostics:

```bash
python3 scripts/hil/hil_telnet.py status --host 10.0.0.44
python3 scripts/hil/hil_telnet.py recovery-status --host 10.0.0.44
```

The status scenario verifies `motor state transition` and recovery readiness so
command echo success does not hide rejected state-machine transitions or
incomplete fault recovery.

The default gate is non-motion status only. The live gate runs the currently
recommended safe baseline:

```bash
scripts/hil/run_hil_gate.sh --host 10.0.0.44 --live
```

Live baseline scenarios:

- `status`
- `boot-commission`
- `encoder-trace-open-loop`
- `current-validate`

Known-unstable encoder closed-loop checks are opt-in so they do not hide
regressions in the generated/open-loop baseline:

```bash
scripts/hil/run_hil_gate.sh --host 10.0.0.44 --live --include-velocity
scripts/hil/run_hil_gate.sh --host 10.0.0.44 --live --include-position
```

If a known current-encoder issue is being investigated, keep the generated
baseline independent:

```bash
scripts/hil/run_hil_gate.sh --host 10.0.0.44 --live --skip-current
```

Each scenario writes a JSON report under the timestamped log directory. The gate
also writes `summary.json` with the selected options and aggregate failure
count. Use `--keep-going` when collecting evidence across known failures.

Thresholds for encoder errors, current motion, and velocity tracking are
configurable with options such as `--max-crc-errors`,
`--min-current-motion-deg`, `--max-velocity-error-hz`,
`--max-velocity-overshoot-ratio`, and
`--min-velocity-tracking-fraction`.

Safety behavior:

- Live-motion scenarios refuse to run without `--yes-live-motion`.
- The script sends best-effort stop commands at the end of live-motion scenarios:
  `motor velocity target 0`, `motor current iq 0`, `motor disarm`,
  `motor state idle`, and `motor safety timeout 1000`, followed by status
  commands used for the final verdict.
- Use `--leave-timeout-disabled` only for manual debugging sessions where the
  command watchdog must remain disabled after the script exits.
