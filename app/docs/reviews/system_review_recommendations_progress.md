# System Review Recommendations Progress Log

Date created: 2026-05-05

Plan: `app/docs/reviews/system_review_recommendations_plan.md`

Source review: `app/docs/reviews/system_review_2026-05-05.md`

## Status Legend

- `Not Started`: no implementation work has begun.
- `In Progress`: code/docs/scripts are being changed, but validation evidence is
  incomplete.
- `Blocked`: implementation cannot continue without an external decision or
  hardware condition.
- `Complete`: committed and validated according to the plan's definition of
  done.

## Phase Status

| Phase | Status | Commit | Evidence Summary |
|---|---|---|---|
| P0 Baseline and Evidence Gates | Complete | this progress commit | Unit tests passed twice; firmware build passed; HIL status passed. |
| P1 HIL Pass/Fail Automation | Complete | this progress commit | Parser tests pass; status JSON report passed; boot-commission JSON report failed objectively as expected for current hardware state. |
| P2 Encoder PI Stabilization | In Progress | eb52d82, 521a6ea, 0526872, 5a6f8a3, be5f2b6 | Boot commissioning, current_encoder, and conservative velocity_encoder are proven; low-speed tracking and position_encoder remain open. |
| P3 Robust Encoder Mapping | Not Started |  |  |
| P4 Commissioning UX and Naming | Not Started |  |  |
| P5 Direct ISR Safety Audit | Not Started |  |  |
| P6 Commissioning Shell Decomposition | Not Started |  |  |
| P7 Control Kernel Extraction | Not Started |  |  |
| P8 TI-Style Fast Block Discipline | Not Started |  |  |
| P9 Persistence Readiness | Not Started |  |  |
| P10 Regression Gate | Not Started |  |  |

## Completion Gate

Before moving any phase to `Complete`, record all of the following:

- Commit hash.
- Unit-test command and summary, if applicable.
- Firmware build command and summary, if code changed.
- HIL command, verdict, and log path, if hardware behavior is affected.
- Open risks or explicit statement that no new risks are known.

## Evidence Entry Template

```text
## Entry N - YYYY-MM-DD - Px: Phase Name

Status:
Commit:
Commands:
Results:
HIL logs:
Open risks:
Next action:
```

## Entry 1 - 2026-05-05 - Plan Created

Status: planning complete, implementation not started.

Commit: pending.

Commands:

```bash
sed -n '300,430p' app/docs/reviews/system_review_2026-05-05.md
```

Results:

- Created `app/docs/reviews/system_review_recommendations_plan.md`.
- Created this progress log to track status, evidence, and completion gates.
- No recommendation implementation phases have started under this plan.

Known baseline evidence from `app/docs/reviews/system_review_2026-05-05.md`:

- Unit tests passed: 29/29 scenarios, 251/251 test cases.
- Firmware build passed for the configured `smartstepper_v2` build.
- HIL telnet status passed using `scripts/hil/hil_telnet.py status`.
- Boot commissioning previously completed after fixing readiness wait.
- Current encoder smoke did not fault, but response was asymmetric.
- Velocity encoder PI was unstable.
- Position encoder had not been validated.

HIL logs:

- Refer to the source review for the previous baseline. P0 must capture fresh
  logs before it is marked complete.

Open risks:

- Previous HIL baseline is useful context but is not fresh evidence for this
  plan.
- Numeric acceptance thresholds for velocity PI and mapping repeatability still
  need to be defined during P1/P2.

Next action:

- Start P0 by rerunning unit tests, firmware build, and non-motion HIL status.

## Entry 2 - 2026-05-05 - P0: Baseline And Evidence Gates

Status: Complete.

Commit: this progress update commit.

Commands:

```bash
./tests/run_unit_tests.sh wonderful_goldberg
./tests/run_unit_tests.sh wonderful_goldberg
podman exec wonderful_goldberg bash -lc 'cd /workspace && west build --build-dir /workspace/build/chopper/smartstepper_v2'
python3 scripts/hil/hil_telnet.py status --host 10.0.0.171 --log-dir hil_logs/p0
```

Results:

- Unit tests run 1: 29/29 scenarios passed, 251/251 test cases passed.
- Unit tests run 2: 29/29 scenarios passed, 251/251 test cases passed.
- Firmware build: passed, `ninja: no work to do`.
- HIL status: shell communication was ordered and completed.
- Device baseline:
  - State `IDLE`.
  - Motor error `NONE`.
  - Encoder control readiness `YES`.
  - Encoder acquisition errors all zero.
  - Fault snapshot latch clear.

HIL logs:

- `hil_logs/p0/20260505_015311_status.log`

Open risks:

- P0 proves the non-motion baseline only. Live boot commissioning still needs
  objective validation and is handled by P1/P2/P3.

Next action:

- Complete P1 parser and JSON verdict support.

## Entry 3 - 2026-05-05 - P1: HIL Pass/Fail Automation

Status: Complete.

Commit: this progress update commit.

Commands:

```bash
python3 -m unittest scripts/hil/test_hil_telnet_parser.py
python3 scripts/hil/hil_telnet.py status --host 10.0.0.171 --log-dir hil_logs/p1 --json-report hil_logs/p1/status.json
python3 scripts/hil/hil_telnet.py boot-commission --yes-live-motion --host 10.0.0.171 --boot-current 0.15 --boot-hz 0.05 --cycles 1 --log-dir hil_logs/p1 --json-report hil_logs/p1/boot_commission.json
```

Results:

- Parser unit tests: 4/4 passed.
- Status HIL verdict: `PASS`.
- Boot-commission HIL verdict: `FAIL`, correctly returned nonzero and wrote
  JSON evidence.
- Boot-commission failure reason:
  - `boot_commission_complete` failed.
  - `encoder_ready` failed after the mapping attempt.
  - The mapping command reported `samples=0 rejected=500 warn=0 err=500 ret=-61`.
  - Motor state remained `ONLINE_VELOCITY_GENERATED` with motor error `NONE`.

HIL logs:

- `hil_logs/p1/20260505_015725_status.log`
- `hil_logs/p1/status.json`
- `hil_logs/p1/20260505_015758_boot-commission.log`
- `hil_logs/p1/boot_commission.json`

Open risks:

- P1 proves the HIL verdict mechanism, not encoder-mode stability.
- The boot-commission failure is now objective evidence feeding P2/P3.

Next action:

- Start P2/P3 by fixing encoder acquisition/mapping readiness before tuning
  encoder PI modes.

## Entry 4 - 2026-05-05 - P2: Encoder Acquisition Reset And Current Smoke

Status: In Progress.

Commit: `eb52d82` (`P2 stabilize encoder acquisition reset`).

Commands:

```bash
podman exec wonderful_goldberg bash -lc 'cd /workspace && west build --build-dir /workspace/build/chopper/smartstepper_v2'
podman exec wonderful_goldberg bash -lc 'cd /workspace && west flash -d /workspace/build/chopper/smartstepper_v2 --runner jlink --dev-id 10.0.0.70 --dev-id-type ip'
python3 scripts/hil/hil_telnet.py boot-commission --yes-live-motion --host 10.0.0.171 --boot-current 0.15 --boot-hz 0.05 --cycles 1 --log-dir hil_logs/p2 --json-report hil_logs/p2/boot_after_p2_fix.json
python3 scripts/hil/hil_telnet.py custom --host 10.0.0.171 --command-timeout 8 --log-dir hil_logs/p2 --json-report hil_logs/p2/current_validate_p2.json --command 'motor state clear_error' --command 'motor safety timeout 0' --command 'motor commission validate current 0.03 160' --command 'motor encoder acquisition' --command 'motor state status' --command 'motor fault snapshot status' --command 'motor current iq 0' --command 'motor disarm' --command 'motor state idle' --command 'motor safety timeout 1000'
```

Results:

- Firmware build: passed.
- Flash: passed.
- Boot commissioning verdict: `PASS`.
- Boot mapping result:
  - `valid=YES`.
  - `dir=-1`.
  - `corr=-0.9856`.
  - `off_mech=0.224 deg`.
  - `off_elec=11.191 deg`.
  - `samples=500 rejected=0 warn=0 err=0 ret=0`.
- Current validation command succeeded with clean motion samples:
  - `+Iq: net=248.170 deg abs=248.170 deg samples=32 warn=0 err=0`.
  - `-Iq: net=-190.164 deg abs=190.164 deg samples=32 warn=0 err=0`.
- The custom current-validation HIL verdict failed because acquisition counters
  accumulated `frame/crc/status=29` after remaining in the encoder mode for a
  longer dwell. The validation sample window itself was clean and the motor
  state remained `Error: NONE (0)`.
- Velocity PI remains unstable/not tuned:
  - default gains did not produce useful low-speed motion.
  - aggressive Ki produced motion but overshot/runaway relative to the target.

HIL logs:

- `hil_logs/p2/20260505_021928_boot-commission.log`
- `hil_logs/p2/boot_after_p2_fix.json`
- `hil_logs/p2/20260505_022040_custom.log`
- `hil_logs/p2/current_validate_p2.json`

Open risks:

- P2 is not complete. `velocity_encoder` and `position_encoder` are not proven.
- The HIL script needs split current/velocity/position scenarios so failures do
  not leave the system energized and so evidence points to one layer at a time.
- Encoder acquisition error acceptance should probably become rate/window based;
  cumulative counters after long dwell are too strict for validating a clean
  motion sample window.

Next action:

- Add split HIL scenarios for current, velocity, and position validation.
- Retune or improve velocity PI behavior with objective velocity HIL evidence.

## Entry 5 - 2026-05-05 - P2: Split HIL Scenarios And Conservative Velocity PI

Status: In Progress.

Commits:

- `521a6ea` (`P2 split encoder HIL validation scenarios`).
- `0526872` (`P2 harden HIL commissioning preamble`).
- `5a6f8a3` (`P2 relax validation readiness verdict`).
- `be5f2b6` (`P2 apply conservative velocity PI defaults`).

Commands:

```bash
python3 -m unittest scripts/hil/test_hil_telnet_parser.py
podman exec wonderful_goldberg bash -lc 'cd /workspace && west build --build-dir /workspace/build/chopper/smartstepper_v2'
./tests/run_unit_tests.sh wonderful_goldberg
podman exec wonderful_goldberg bash -lc 'cd /workspace && west flash -d /workspace/build/chopper/smartstepper_v2 --runner jlink --dev-id 10.0.0.70 --dev-id-type ip'
python3 scripts/hil/hil_telnet.py current-validate --yes-live-motion --host 10.0.0.171 --boot-current 0.15 --boot-hz 0.05 --cycles 1 --current-iq 0.03 --current-hold-ms 160 --min-current-motion-deg 1.0 --max-crc-errors 50 --max-status-errors 50 --max-transport-errors 50 --log-dir hil_logs/p2 --json-report hil_logs/p2/current_validate_split_idle.json
python3 scripts/hil/hil_telnet.py velocity-validate --yes-live-motion --host 10.0.0.171 --connect-timeout 8 --boot-current 0.15 --boot-hz 0.05 --cycles 1 --velocity-hz 0.50 --velocity-hold-ms 1000 --max-crc-errors 600 --max-status-errors 600 --max-transport-errors 600 --max-sample-errors 10 --max-sample-warnings 10 --max-velocity-error-hz 0.80 --max-velocity-overshoot-ratio 4.0 --min-velocity-tracking-fraction 0.10 --log-dir hil_logs/p2 --json-report hil_logs/p2/velocity_validate_safe_defaults_050.json
python3 scripts/hil/hil_telnet.py position-validate --yes-live-motion --host 10.0.0.171 --connect-timeout 8 --boot-current 0.15 --boot-hz 0.05 --cycles 1 --velocity-hz 0.50 --velocity-hold-ms 1000 --position-delta-deg 5.0 --position-hold-ms 2000 --max-crc-errors 600 --max-status-errors 600 --max-transport-errors 600 --max-sample-errors 10 --max-sample-warnings 10 --max-velocity-error-hz 0.80 --max-velocity-overshoot-ratio 4.0 --min-velocity-tracking-fraction 0.10 --log-dir hil_logs/p2 --json-report hil_logs/p2/position_validate_safe_defaults.json
```

Results:

- Parser tests: 6/6 passed.
- Firmware build: passed.
- Unit tests: 29/29 scenarios passed, 251/251 test cases passed.
- Flash: passed.
- HIL split scenarios now force a safe idle/disarmed preamble before boot
  commissioning and always run a stop/status footer.
- `current_encoder` evidence:
  - 0.03 A is not repeatable enough for a strict 1 degree threshold in both
    directions; it can be detent-limited in the reverse direction.
  - A clean 0.03 A run also exists with `+Iq=247.223 deg`, `-Iq=-256.224 deg`,
    both with `warn=0 err=0`.
  - 0.06 A moves strongly both directions but produces some CRC/status/sample
    errors under switching current. This reinforces that the AEAT path is usable
    with prediction/skips, but not noise-free.
- `velocity_encoder` evidence:
  - Default/aggressive gains were unstable at low speed and saturated Iq.
  - Conservative firmware defaults were changed to approximately
    `Kp=0.00955 A/(rad/s)`, `Ki=0.01910 A/rad`, `Iq_limit=0.040 A`.
  - At `max=0.50 Hz`, HIL verdict `PASS`, no motor fault, no encoder errors,
    and bounded response:
    - `target=0.300 Hz`, `meas=0.338 Hz`.
    - `target=-0.500 Hz`, `meas=-0.709 Hz`.
    - `crc/status/glitch=0`.
- Low-speed `0.20 Hz` validation is stable but can fail tracking because the
  conservative current limit does not always overcome static friction/detent.
- `position_encoder` evidence:
  - Not proven. The attempted run failed during the boot encoder mapping gate:
    `motion=3.625 deg`, `valid=NO`, then `OVERCURRENT`.
  - Recovery through `motor state clear_error` returned the system to `IDLE`
    with fault snapshot clear.

HIL logs:

- `hil_logs/p2/20260505_023838_current-validate.log`
- `hil_logs/p2/current_validate_split_idle.json`
- `hil_logs/p2/20260505_024632_velocity-validate.log`
- `hil_logs/p2/velocity_validate_low_gain_020.json`
- `hil_logs/p2/20260505_025436_velocity-validate.log`
- `hil_logs/p2/velocity_validate_safe_defaults_050.json`
- `hil_logs/p2/20260505_025638_position-validate.log`
- `hil_logs/p2/position_validate_safe_defaults.json`

Open risks:

- P2 is still not complete because `position_encoder` is not validated and
  robust repeated boot mapping is not yet proven.
- The boot mapping gate can fail after previous motion with very low measured
  generated-sweep motion. This should be handled by P3 robust mapping and by
  better abort/stop behavior in HIL scripts.
- Encoder CRC/status errors appear current-dependent. For validation, command
  sample-window errors and error rates are more meaningful than raw cumulative
  counters.
- Conservative velocity defaults are safe but sluggish. Low-speed tracking
  needs either higher tuned gains, MPR, or friction/detent feedforward after the
  basic PI path is stable.

Next action:

- Move to P3 robust encoder mapping because mapping repeatability is now the
  blocker for position validation and for marking P2 complete.
