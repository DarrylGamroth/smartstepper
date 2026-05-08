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
python3 scripts/hil/hil_telnet.py standard-commission --yes-live-motion
python3 scripts/hil/hil_telnet.py current-validate --yes-live-motion
python3 scripts/hil/hil_telnet.py velocity-validate --yes-live-motion
python3 scripts/hil/hil_telnet.py position-validate --yes-live-motion
python3 scripts/hil/hil_telnet.py encoder-robust --yes-live-motion
python3 scripts/hil/hil_telnet.py encoder-validate --yes-live-motion
python3 scripts/hil/hil_telnet.py encoder-trace-open-loop --yes-live-motion
python3 scripts/hil/hil_telnet.py mechanical-id-v2 --yes-live-motion
python3 scripts/hil/hil_telnet.py velocity-sweep --yes-live-motion
python3 scripts/hil/hil_telnet.py mpr-dob-detent --yes-live-motion
python3 scripts/hil/hil_telnet.py custom --yes-live-motion --command 'motor state status'
```

Useful options:

```bash
python3 scripts/hil/hil_telnet.py encoder-validate \
  --yes-live-motion \
  --current-iq 0.03 \
  --velocity-hz 0.50
```

Use the split scenarios while bringing up encoder control. They run the
standard baseline commissioning workflow first, isolate the failing layer, and
still send stop commands at script exit:

```bash
python3 scripts/hil/hil_telnet.py current-validate --yes-live-motion \
  --current-iq 0.03 --current-hold-ms 160

python3 scripts/hil/hil_telnet.py velocity-validate --yes-live-motion \
  --velocity-pi-kp 0.100 --velocity-pi-ki 0.250 --velocity-pi-iq-limit 0.120 \
  --velocity-hz 0.50 --velocity-hold-ms 1000
```

Use the robust encoder-mapping scenario to test only generated-sweep mapping
and apply, without running current/velocity/position validation afterward:

```bash
python3 scripts/hil/hil_telnet.py encoder-robust --yes-live-motion \
  --map-current 0.15 --map-hz 0.10 --cycles 1 --bidirectional
```

Use the mechanical identification v2 scenario to run the staged
friction/inertia workflow separately from baseline electrical + encoder
commissioning. The scenario runs baseline commissioning first, then runs
`motor commission auto run <profile>` without applying the staged mechanical
model. The verdict checks that windowed acceleration is present and that any
accepted mechanical model passed the confidence gate:

```bash
python3 scripts/hil/hil_telnet.py mechanical-id-v2 \
  --host 10.0.0.44 \
  --yes-live-motion \
  --mechanical-id-profile confirm
```

Use the advanced motion scenario after basic encoder validation is passing. It
runs standard baseline commissioning, captures a forward/reverse detent map,
validates detent off/on ripple, then checks selected PI/MPR/DOB/detent
combinations with the same velocity-validation command:

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

Use the velocity sweep scenario for PI/MPR baseline tuning. It records an
encoder trace for each target and evaluates direction, minimum motion, trace
quality, and velocity error. With `--velocity-sweep-commission standard`, the
script first runs standard baseline commissioning, which includes current
offsets, RoverL bootstrap, production Rs/Ld/Lq, and encoder mapping.

```bash
python3 scripts/hil/hil_telnet.py velocity-sweep \
  --host 10.0.0.44 \
  --yes-live-motion \
  --velocity-sweep-commission standard \
  --velocity-sweep-regulator pi \
  --velocity-sweep-target-hz 0.1 --velocity-sweep-target-hz -0.1 \
  --velocity-sweep-target-hz 0.3 --velocity-sweep-target-hz -0.3 \
  --velocity-sweep-target-hz 0.5 --velocity-sweep-target-hz -0.5 \
  --velocity-sweep-target-hz 1.0 --velocity-sweep-target-hz -1.0 \
  --velocity-sweep-target-hz 3.0 --velocity-sweep-target-hz -3.0 \
  --velocity-sweep-target-hz 5.0 --velocity-sweep-target-hz -5.0
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

The `mpr-dob-detent` scenario also runs standard baseline commissioning before
advanced feature checks.

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
- `standard-commission`
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
- `custom` is treated as live-motion because arbitrary command lists can
  energize hardware. This also ensures the stop/status postlude runs, so encoder
  acquisition counters are available to the final verdict.
- The script sends best-effort stop commands at the end of live-motion scenarios:
  `motor velocity target 0`, `motor current iq 0`, `motor disarm`,
  `motor state idle`, and `motor safety timeout 1000`, followed by status
  commands used for the final verdict.
- Use `--leave-timeout-disabled` only for manual debugging sessions where the
  command watchdog must remain disabled after the script exits.
