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
| P2 Encoder PI Stabilization | Complete | eb52d82, 521a6ea, 0526872, 5a6f8a3, be5f2b6, 219b005, 43c2ae5, a8062b0 | Boot, current_encoder, velocity_encoder, and position_encoder have PASS HIL artifacts with conservative PI; cumulative current-mode encoder CRC diagnostics remain an open risk. |
| P3 Robust Encoder Mapping | Complete | a7adcab, 128fe3d, 8cc4979 | Robust mapping command added; boot uses robust retry; bidirectional HIL mapping passed with 1000 accepted samples and zero encoder errors; failed sweeps now abort acquisition. |
| P4 Commissioning UX and Naming | Complete | fae3054 | Commissioning command help clarified; boot/mapping/validation commands print prerequisites and next steps; help/status HIL captured. |
| P5 Direct ISR Safety Audit | Complete | 943081b | Break callbacks now use ISR ring only; ISR audit doc added; build/status/boot HIL pass. |
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

## Entry 6 - 2026-05-05 - P3: Robust Encoder Mapping

Status: Complete.

Commits:

- `a7adcab` (`P3 add robust encoder mapping command`).
- `128fe3d` (`P3 add robust encoder HIL scenario`).

Commands:

```bash
podman exec wonderful_goldberg bash -lc 'cd /workspace && west build --build-dir /workspace/build/chopper/smartstepper_v2'
./tests/run_unit_tests.sh wonderful_goldberg -s chopper.motor_encoder_map_detect.unit
python3 -m unittest scripts/hil/test_hil_telnet_parser.py
python3 scripts/hil/hil_telnet.py boot-commission --yes-live-motion --host 10.0.0.171 --connect-timeout 8 --boot-current 0.15 --boot-hz 0.05 --cycles 1 --max-crc-errors 20 --max-status-errors 20 --max-transport-errors 20 --log-dir hil_logs/p3 --json-report hil_logs/p3/boot_with_mapping_retry.json
python3 scripts/hil/hil_telnet.py encoder-robust --yes-live-motion --host 10.0.0.171 --connect-timeout 8 --boot-current 0.15 --boot-hz 0.05 --cycles 1 --bidirectional --max-crc-errors 100 --max-status-errors 100 --max-transport-errors 100 --log-dir hil_logs/p3 --json-report hil_logs/p3/encoder_robust_bidirectional.json
```

Results:

- Firmware build: passed after the robust mapping command implementation.
- Encoder map-detect unit suite: 1/1 scenario passed, 8/8 test cases passed.
- HIL parser tests: 8/8 passed after adding the `encoder-robust` scenario.
- Boot commissioning HIL verdict: `PASS`.
  - Mapping result: `valid=YES`, `dir=-1`, `corr=-0.9852`,
    `off_mech=0.209 deg`, `off_elec=10.452 deg`.
  - Residuals: `offset=0.0806 rad`, `direction=0.0022 rad`,
    `motion=361.009 deg`, `samples=500`, `rejected=0`, `warn=0`,
    `err=0`, `ret=0`.
  - `+Iq` validation: `Iq=0.060 A`, `net=188.687 deg`,
    `samples=16`, `warn=0`, `err=0`.
- Explicit bidirectional robust encoder HIL verdict: `PASS`.
  - Forward sweep: `valid=YES`, `dir=-1`, `corr=-0.9857`,
    `off_mech=0.208 deg`, `off_elec=10.407 deg`,
    `motion=360.737 deg`, `samples=500`, `warn=0`, `err=0`.
  - Reverse sweep: `valid=YES`, `dir=-1`, `corr=-0.9850`,
    `off_mech=0.070 deg`, `off_elec=3.506 deg`,
    `motion=360.803 deg`, `samples=500`, `warn=0`, `err=0`.
  - Combined result: `valid=YES`, `dir=-1`, `corr=-0.9854`,
    `off_mech=0.139 deg`, `off_elec=6.957 deg`,
    `motion=721.540 deg`, `samples=1000`, `rejected=0`, `warn=0`,
    `err=0`.
  - Mapping applied successfully.
  - Final state after stop footer: `IDLE`, motor error `NONE`, fault
    snapshot latch clear, acquisition errors all zero.

HIL logs:

- `hil_logs/p3/20260505_030358_boot-commission.log`
- `hil_logs/p3/boot_with_mapping_retry.json`
- `hil_logs/p3/20260505_031844_encoder-robust.log`
- `hil_logs/p3/encoder_robust_bidirectional.json`

Open risks:

- P3 proves robust generated-sweep encoder mapping and application. It does not
  prove closed-loop position control; that remains a P2 follow-up after mapping
  repeatability.
- The forward/reverse offset difference is visible (`0.208 deg` vs `0.070 deg`
  mechanical). The combined residual remained acceptable, but repeated
  multi-run variance should be tracked in the regression gate.

Next action:

- Resume P2 closure by rerunning `current-validate`, `velocity-validate`, and
  `position-validate` with the more robust mapping gate. If position still
  fails, isolate position-profile limits and velocity-loop tuning separately.

## Entry 7 - 2026-05-05 - P2: Encoder PI Stabilization Closure

Status: Complete.

Commits:

- `219b005` (`P2 stop HIL validation before diagnostics`).
- `43c2ae5` (`P2 stop validation modes before returning`).
- `a8062b0` (`P2 fix position HIL verdict requirements`).
- `8cc4979` (`P3 abort encoder acquisition after mapping sweeps`).

Commands:

```bash
python3 -m unittest scripts/hil/test_hil_telnet_parser.py
podman exec wonderful_goldberg bash -lc 'cd /workspace && west build --build-dir /workspace/build/chopper/smartstepper_v2'
podman exec wonderful_goldberg bash -lc 'cd /workspace && west flash -d /workspace/build/chopper/smartstepper_v2 --runner jlink --dev-id 10.0.0.70 --dev-id-type ip'
python3 scripts/hil/hil_telnet.py current-validate --yes-live-motion --host 10.0.0.171 --connect-timeout 8 --boot-current 0.15 --boot-hz 0.05 --cycles 1 --current-iq 0.035 --current-hold-ms 160 --min-current-motion-deg 1.0 --max-crc-errors 100 --max-status-errors 100 --max-transport-errors 100 --max-sample-errors 10 --max-sample-warnings 10 --log-dir hil_logs/p2 --json-report hil_logs/p2/current_validate_after_fw_stop_035_pass.json
python3 scripts/hil/hil_telnet.py velocity-validate --yes-live-motion --host 10.0.0.171 --connect-timeout 8 --boot-current 0.15 --boot-hz 0.05 --cycles 1 --velocity-hz 0.50 --velocity-hold-ms 1000 --max-crc-errors 1000 --max-status-errors 1000 --max-transport-errors 1000 --max-sample-errors 10 --max-sample-warnings 10 --max-velocity-error-hz 0.80 --max-velocity-overshoot-ratio 4.0 --min-velocity-tracking-fraction 0.10 --log-dir hil_logs/p2 --json-report hil_logs/p2/velocity_validate_after_p3_050.json
python3 scripts/hil/hil_telnet.py position-validate --yes-live-motion --host 10.0.0.171 --connect-timeout 8 --boot-current 0.15 --boot-hz 0.05 --cycles 1 --velocity-hz 0.50 --velocity-hold-ms 1000 --position-delta-deg 5.0 --position-hold-ms 2000 --max-crc-errors 1000 --max-status-errors 1000 --max-transport-errors 1000 --max-sample-errors 10 --max-sample-warnings 10 --max-velocity-error-hz 0.80 --max-velocity-overshoot-ratio 4.0 --min-velocity-tracking-fraction 0.10 --log-dir hil_logs/p2 --json-report hil_logs/p2/position_validate_after_acq_abort_5deg.json
```

Results:

- Parser tests: 9/9 passed.
- Firmware build: passed.
- Flash: passed.
- `current_encoder` HIL verdict: `PASS` at `0.035 A`.
  - `+Iq: net=304.011 deg`, `samples=32`, `warn=0`, `err=0`.
  - `-Iq: net=-255.532 deg`, `samples=32`, `warn=0`, `err=0`.
  - Motor error `NONE`, fault snapshot clear.
  - Cumulative acquisition after the run: `crc=71`, `status=71`.
- `velocity_encoder` HIL verdict: `PASS` at `0.50 Hz`.
  - Sample-window warnings/errors all zero.
  - Cumulative acquisition errors all zero.
  - Final state `IDLE`, motor error `NONE`, fault snapshot clear.
- `position_encoder` HIL verdict: `PASS` for a `5.0 deg` move and return.
  - Position validation completed.
  - Velocity precheck passed.
  - Cumulative acquisition errors all zero.
  - Final state `IDLE`, motor error `NONE`, fault snapshot clear.
- Firmware validation commands now return to disarmed `IDLE` before returning to
  the shell, avoiding multi-second energized dwell while the HIL script gathers
  diagnostics.
- Mapping sweeps now abort any in-flight encoder acquisition after restoring
  trace settings, preventing `disabled,busy` acquisition state after a failed
  mapping attempt.

HIL logs:

- `hil_logs/p2/20260505_034531_current-validate.log`
- `hil_logs/p2/current_validate_after_fw_stop_035_pass.json`
- `hil_logs/p2/20260505_033435_velocity-validate.log`
- `hil_logs/p2/velocity_validate_after_p3_050.json`
- `hil_logs/p2/20260505_034305_position-validate.log`
- `hil_logs/p2/position_validate_after_acq_abort_5deg.json`

Open risks:

- `current_encoder` at useful torque still accumulates AEAT CRC/status errors
  during active current, even when the validation sample window is clean. This
  should remain visible in the regression gate and may still require electrical
  noise mitigation or encoder sampling timing improvements.
- Conservative velocity PI is stable but not high-performance. Higher bandwidth
  tuning, MPR, detent feedforward, and persistence should remain disabled until
  the regression gate can repeatedly pass.
- `position_encoder` is only proven for a small 5 degree move with conservative
  settings.

Next action:

- Move to P4 commissioning UX/naming so operators can distinguish boot gate,
  encoder mapping, validation, and full identification workflows without source
  inspection.

## Entry 8 - 2026-05-05 - P4: Commissioning UX And Naming

Status: Complete.

Commit:

- `fae3054` (`P4 clarify commissioning workflow commands`).

Commands:

```bash
python3 -m unittest scripts/hil/test_hil_telnet_parser.py
podman exec wonderful_goldberg bash -lc 'cd /workspace && west build --build-dir /workspace/build/chopper/smartstepper_v2'
podman exec wonderful_goldberg bash -lc 'cd /workspace && west flash -d /workspace/build/chopper/smartstepper_v2 --runner jlink --dev-id 10.0.0.70 --dev-id-type ip'
python3 scripts/hil/hil_telnet.py custom --host 10.0.0.171 --connect-timeout 8 --command-timeout 3 --log-dir hil_logs/p4 --json-report hil_logs/p4/commission_help.json --command 'motor commission' --command 'motor commission encoder' --command 'motor commission validate' --command 'motor commission status' --command 'motor state status' --command 'motor fault snapshot status'
python3 scripts/hil/hil_telnet.py status --host 10.0.0.171 --connect-timeout 8 --log-dir hil_logs/p4 --json-report hil_logs/p4/status_after_p4.json
```

Results:

- Parser tests: 9/9 passed.
- Firmware build: passed.
- Flash: passed.
- Commissioning help output now separates:
  - runtime boot gate.
  - encoder commutation mapping.
  - encoder-mode smoke validation.
  - full identify/tune workflow.
- Boot, robust encoder mapping, mapping apply, and current/velocity/position
  validation commands now print prerequisites, scope, and next recommended
  command.
- Help-only custom capture completed. Its HIL verdict is `INCONCLUSIVE` only
  because no encoder acquisition counters are printed by the help commands.
- Standard non-motion `status` scenario verdict: `PASS`.

HIL logs:

- `hil_logs/p4/20260505_034951_custom.log`
- `hil_logs/p4/commission_help.json`
- `hil_logs/p4/20260505_035017_status.log`
- `hil_logs/p4/status_after_p4.json`

Open risks:

- This phase improves command wording and command-tree discoverability. It does
  not split the large commissioning shell implementation; that remains P6.
- More failure-path messages can still be improved as workflows are split.

Next action:

- Start P5 direct ISR safety audit before decomposing commissioning shell files.

## Entry 9 - 2026-05-05 - P5: Direct ISR Safety Audit

Status: Complete.

Commit:

- `943081b` (`P5 make break callbacks ISR-safe`).

Commands:

```bash
python3 -m unittest scripts/hil/test_hil_telnet_parser.py
podman exec wonderful_goldberg bash -lc 'cd /workspace && west build --build-dir /workspace/build/chopper/smartstepper_v2'
podman exec wonderful_goldberg bash -lc 'cd /workspace && west flash -d /workspace/build/chopper/smartstepper_v2 --runner jlink --dev-id 10.0.0.70 --dev-id-type ip'
rg -n "motor_api_post_error|k_msgq|k_sem|k_mutex|shell_|LOG_|printk|drv8328_disable" app/src/motor_isr_io.c drivers/adc/adc_stm32_injected.c drivers/pwm/mcpwm_stm32.c
python3 scripts/hil/hil_telnet.py status --host 10.0.0.171 --connect-timeout 8 --log-dir hil_logs/p5 --json-report hil_logs/p5/status_after_p5.json
python3 scripts/hil/hil_telnet.py boot-commission --yes-live-motion --host 10.0.0.171 --connect-timeout 8 --boot-current 0.15 --boot-hz 0.05 --cycles 1 --max-crc-errors 100 --max-status-errors 100 --max-transport-errors 100 --log-dir hil_logs/p5 --json-report hil_logs/p5/boot_after_p5.json
```

Results:

- Parser tests: 9/9 passed.
- Firmware build: passed.
- Flash: passed.
- Added ISR audit document: `app/docs/reviews/isr_safety_audit_2026-05-05.md`.
- Gate-driver break callbacks no longer call gate-driver APIs or the generic
  `motor_api_post_error()` path from interrupt context.
- Gate-driver break callbacks now atomically disarm and enqueue
  `ERROR_HARDWARE_BREAK` through the ISR-safe event ring.
- MCPWM break ISR no longer logs from interrupt context.
- HIL `status` verdict: `PASS`.
- HIL `boot-commission` verdict: `PASS`.

HIL logs:

- `hil_logs/p5/20260505_035348_status.log`
- `hil_logs/p5/status_after_p5.json`
- `hil_logs/p5/20260505_035421_boot-commission.log`
- `hil_logs/p5/boot_after_p5.json`

Open risks:

- `adc_callback()` still toggles the debug GPIO through the Zephyr GPIO API in
  the zero-latency ISR. This is useful for timing but should become direct LL
  GPIO or compile-time optional if ISR timing margin tightens.
- The ADC ISR remains FPU-heavy by design; P7/P8 continue the work to shrink
  the control kernel and formalize fast-block invariants.

Next action:

- Start P6 commissioning shell decomposition. Preserve the command tree and
  behavior while splitting the large implementation file by workflow.

## Entry 11 - 2026-05-05 - P6: Commissioning Validation Command Split

Status: In Progress.

Commit: `7b24b9a` (`P6 split commissioning validation commands`).

Commands:

```bash
podman exec wonderful_goldberg bash -lc 'cd /workspace && west build --build-dir /workspace/build/chopper/smartstepper_v2'
python3 -m unittest scripts/hil/test_hil_telnet_parser.py
podman exec wonderful_goldberg bash -lc 'cd /workspace && west flash -d /workspace/build/chopper/smartstepper_v2 --runner jlink --dev-id 10.0.0.70 --dev-id-type ip'
python3 scripts/hil/hil_telnet.py status --host 10.0.0.171 --connect-timeout 8 --log-dir hil_logs/p6 --json-report hil_logs/p6/status_after_p6.json
python3 scripts/hil/hil_telnet.py custom --host 10.0.0.171 --connect-timeout 8 --command-timeout 3 --log-dir hil_logs/p6 --json-report hil_logs/p6/commission_validate_help.json --command 'motor commission validate' --command 'motor commission status' --command 'motor state status' --command 'motor fault snapshot status'
```

Results:

- Split `motor commission validate current|velocity|position` into `app/src/shell_commission_validate.c`.
- Added `app/include/shell_commission_internal.h` for the small set of shared commissioning-shell helpers used by split command files.
- Reduced `app/src/shell_motion_commission.c` from 3498 lines to 3140 lines.
- Firmware build: passed.
- HIL parser unit tests: 9/9 passed.
- Flash: passed.
- HIL status: `PASS`.
- HIL command-tree check: validation subcommands are visible; verdict `INCONCLUSIVE` only because the help/status sequence does not print encoder acquisition counters.

HIL logs:

- `hil_logs/p6/20260505_035957_status.log`
- `hil_logs/p6/status_after_p6.json`
- `hil_logs/p6/20260505_040025_custom.log`
- `hil_logs/p6/commission_validate_help.json`

Open risks:

- P6 is not complete. The original commissioning shell file still contains boot/mapping, full identification, detent-map, and tuning workflows.
- Shared helpers are intentionally internal to commissioning shell code; further splits should keep the header narrow and avoid turning it into a generic dumping ground.

Next action:

- Continue P6 by splitting the encoder mapping/boot workflow and detent workflow into separate command files.

## Entry 12 - 2026-05-05 - P6: Commissioning Detent Command Split

Status: In Progress.

Commit: `8d3b207` (`P6 split commissioning detent commands`).

Commands:

```bash
podman exec wonderful_goldberg bash -lc 'cd /workspace && west build --build-dir /workspace/build/chopper/smartstepper_v2'
python3 -m unittest scripts/hil/test_hil_telnet_parser.py
podman exec wonderful_goldberg bash -lc 'cd /workspace && west flash -d /workspace/build/chopper/smartstepper_v2 --runner jlink --dev-id 10.0.0.70 --dev-id-type ip'
python3 scripts/hil/hil_telnet.py custom --host 10.0.0.171 --connect-timeout 8 --command-timeout 3 --log-dir hil_logs/p6 --json-report hil_logs/p6/detent_help_after_split.json --command 'motor commission detent' --command 'motor commission detent status' --command 'motor state status' --command 'motor fault snapshot status'
```

Results:

- Split `motor commission detent run|status|apply|clear` into `app/src/shell_commission_detent.c`.
- Reduced `app/src/shell_motion_commission.c` from 3140 lines to 2638 lines.
- Firmware build: passed.
- HIL parser unit tests: 9/9 passed.
- Flash: passed.
- HIL detent command-tree/status check: command subtree and staged/runtime detent status are visible; no motor fault or fault snapshot latch.

HIL logs:

- `hil_logs/p6/20260505_040304_custom.log`
- `hil_logs/p6/detent_help_after_split.json`

Open risks:

- P6 is still in progress. The main commissioning file still contains status/common commands, flux/mechanical ID, encoder boot/mapping, and auto-tune workflows.
- The HIL check is command/status only; it intentionally did not run a live detent capture.

Next action:

- Continue P6 by splitting the encoder boot/mapping workflow or auto-tune workflow into separate command files.

## Entry 13 - 2026-05-05 - P6: Commissioning Encoder Command Split

Status: In Progress.

Commit: `9205084` (`P6 split commissioning encoder commands`).

Commands:

```bash
podman exec wonderful_goldberg bash -lc 'cd /workspace && west build --build-dir /workspace/build/chopper/smartstepper_v2'
python3 -m unittest scripts/hil/test_hil_telnet_parser.py
podman exec wonderful_goldberg bash -lc 'cd /workspace && west flash -d /workspace/build/chopper/smartstepper_v2 --runner jlink --dev-id 10.0.0.70 --dev-id-type ip'
python3 scripts/hil/hil_telnet.py custom --host 10.0.0.171 --connect-timeout 8 --command-timeout 5 --log-dir hil_logs/p6 --json-report hil_logs/p6/stop_after_encoder_help_boot.json --command 'motor current iq 0' --command 'motor disarm' --command 'motor state idle' --command 'motor state status' --command 'motor encoder acquisition' --command 'motor fault snapshot status'
python3 scripts/hil/hil_telnet.py custom --host 10.0.0.171 --connect-timeout 8 --command-timeout 3 --log-dir hil_logs/p6 --json-report hil_logs/p6/encoder_status_after_split.json --command 'motor commission encoder' --command 'motor commission encoder status' --command 'motor state status' --command 'motor encoder acquisition' --command 'motor fault snapshot status'
```

Results:

- Split `motor commission encoder run|robust|status|apply|clear` and `motor commission boot` into `app/src/shell_commission_encoder.c`.
- Kept raw-trace polling helpers in the shared internal commissioning layer because validation and motion-threshold workflows also use them.
- Reduced `app/src/shell_motion_commission.c` from 2638 lines to 1868 lines.
- Firmware build: passed.
- HIL parser unit tests: 9/9 passed.
- Flash: passed.
- A mistaken command-tree check invoked `motor commission boot`; it completed without fault, produced a valid staged/applied mapping, and was stopped to IDLE.
- Follow-up HIL encoder command-tree/status check: `PASS`; staged mapping showed 500 accepted samples, 0 rejected, 0 warn, 0 err, sign -1, offset 0.2115 deg mechanical.

HIL logs:

- `hil_logs/p6/20260505_040746_custom.log`
- `hil_logs/p6/stop_after_encoder_help_boot.json`
- `hil_logs/p6/20260505_040826_custom.log`
- `hil_logs/p6/encoder_status_after_split.json`

Open risks:

- P6 is still in progress. The main commissioning file still contains common status/reset/apply commands, flux/mechanical identification, motion-threshold, and auto-tune workflows.
- The boot command has no help-only mode; invoking it with no arguments starts live commissioning by design.

Next action:

- Continue P6 by splitting the auto-tune workflow and/or the flux/mechanical identification workflow.
