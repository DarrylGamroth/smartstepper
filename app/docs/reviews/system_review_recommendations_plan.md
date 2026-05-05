# System Review Recommendations Implementation Plan

Date: 2026-05-05

Source review: `app/docs/reviews/system_review_2026-05-05.md`

## Goal

Turn the system-review recommendations into a controlled implementation program
with objective proof for each step. A phase is not complete until the code is
implemented, tests/builds pass, HIL evidence is captured where applicable, and
the progress log is updated.

## Execution Rules

1. Work phases in order unless a later phase is explicitly needed to unblock an
   earlier one.
2. Keep commits phase-scoped. Include the phase ID in the commit message.
3. Update `app/docs/reviews/system_review_recommendations_progress.md` after
   every phase or meaningful subphase.
4. Do not mark a phase complete based only on code review. Each phase requires
   explicit evidence listed in that phase.
5. Prefer native_sim unit tests for pure `motor_core` behavior.
6. Prefer telnet HIL scripts for real hardware behavior:

   ```bash
   python3 scripts/hil/hil_telnet.py status --host 10.0.0.171
   python3 scripts/hil/hil_telnet.py encoder-validate --yes-live-motion --host 10.0.0.171
   ```

7. Store HIL logs under `hil_logs/` or attach the chosen log path to the
   progress entry.
8. Do not enable MPR, DOB, or detent feedforward as default bring-up behavior
   until PI encoder modes are stable and proven.

## Definition Of Done

A phase is complete only when all are true:

- Code/docs/scripts for the phase are committed.
- Relevant native_sim tests pass.
- Firmware build passes with the current configured hardware build.
- HIL pass/fail evidence exists for phases that affect hardware behavior.
- The progress log contains:
  - phase ID.
  - commit hash.
  - validation commands.
  - summarized results.
  - open risks.

## Phase Overview

| Phase | Name | Primary Risk Addressed | Hardware Required |
|---|---|---|---|
| P0 | Baseline and Evidence Gates | Unknown current state | Optional |
| P1 | HIL Pass/Fail Automation | Logs without objective verdicts | Optional for parser, yes for validation |
| P2 | Encoder PI Stabilization | Unstable encoder velocity/position modes | Yes |
| P3 | Robust Encoder Mapping | Single-cycle mapping is fragile/misleading | Yes |
| P4 | Commissioning UX and Naming | Operator confusion between boot gate and full ID | Optional |
| P5 | Direct ISR Safety Audit | Direct/zero-latency ISR calling unsafe paths | Optional, HIL recommended |
| P6 | Commissioning Shell Decomposition | `shell_motion_commission.c` is too large | No |
| P7 | Control Kernel Extraction | ISR still coupled to `motor_parameters` | No initially, HIL after |
| P8 | TI-Style Fast Block Discipline | Runtime checks/config work in hot path | No initially, HIL after |
| P9 | Persistence Readiness | Every boot needs runtime commissioning | Yes before enable |
| P10 | Regression Gate | No repeatable release checklist | Yes |

## P0: Baseline And Evidence Gates

### Purpose

Freeze the current known state so future changes can be compared against it.

### Work

- Ensure the full unit-test wrapper runs repeatedly.
- Ensure the firmware build command in `AGENTS.md` is accurate.
- Ensure `scripts/hil/hil_telnet.py status` produces ordered command output.
- Record current status of:
  - build.
  - unit tests.
  - non-motion HIL status.
  - latest known boot commissioning and encoder validation behavior.

### Likely Files

- `tests/run_unit_tests.sh`
- `scripts/hil/hil_telnet.py`
- `AGENTS.md`
- `app/docs/reviews/system_review_recommendations_progress.md`

### Acceptance

- `./tests/run_unit_tests.sh` passes twice in a row.
- `podman exec wonderful_goldberg bash -lc 'cd /workspace && west build --build-dir /workspace/build/chopper/smartstepper_v2'` passes.
- `python3 scripts/hil/hil_telnet.py status --host 10.0.0.171` produces ordered command/response output.

### Evidence

- Unit-test summary: scenario count and test-case count.
- Build output summary.
- HIL status log path.

## P1: HIL Pass/Fail Automation

### Purpose

Convert HIL scripts from log capture to objective validation tools.

### Work

- Add parser support to `scripts/hil/hil_telnet.py` for key command outputs.
- Emit a final verdict for each scenario: `PASS`, `FAIL`, or `INCONCLUSIVE`.
- Add configurable numeric thresholds for:
  - encoder readiness.
  - acquisition transport/frame/CRC/status/glitch errors.
  - fault snapshot latch.
  - boot mapping validity.
  - minimum mapping motion.
  - current smoke positive/negative motion sign and minimum magnitude.
  - velocity measured target error and overshoot.
- Preserve raw logs even when parsed verdict fails.
- Add `--json-report <path>` for machine-readable evidence.
- Add a dry-run/parser unit test path that feeds saved sample output to the
  parser without hardware.

### Likely Files

- `scripts/hil/hil_telnet.py`
- `scripts/hil/README.md`
- `tests/host/` or `scripts/hil/test_hil_telnet_parser.py`

### Acceptance

- Parser unit tests pass on sample outputs.
- `status` scenario can produce JSON report without live motion.
- `boot-commission` scenario returns nonzero exit code on failed mapping or
  encoder readiness failure.
- `encoder-validate` scenario returns nonzero exit code on encoder errors,
  motor faults, or configured velocity error threshold violation.

### Evidence

- Parser test command and output.
- Example `status` JSON report.
- One HIL `boot-commission` JSON report.

## P2: Encoder PI Stabilization

### Purpose

Make `current_encoder`, `velocity_encoder`, and then `position_encoder` stable
with PI control before returning to MPR/DOB/detent feedforward.

### Work

- Use boot mapping as the required gate.
- Keep outer loop mode `PI`.
- Keep DOB disabled.
- Keep detent feedforward disabled.
- Verify current loop operation in `current_encoder` at low current.
- Investigate asymmetric +Iq/-Iq response:
  - direction sign.
  - commutation offset convention.
  - trim command effect.
  - static friction/detent threshold.
  - `Id`/`Iq` axis convention.
- Tune velocity PI using measured low-speed response:
  - start with very low targets.
  - enforce current limit.
  - reset/bleed integrator on zero target and mode entry.
- Only test `position_encoder` after velocity is stable.
- Ensure position commands use bounded profiles and conservative velocity caps.

### Likely Files

- `app/src/shell_motion_commission.c`
- `app/src/motor_states_online.c`
- `modules/motor_core/src/runtime/outer_loop_runtime.c`
- `modules/motor_core/src/control/velocity_regulator.c`
- `modules/motor_core/src/control/position_regulator.c`
- `tests/unit/pi_controller/src/main.c`
- `scripts/hil/hil_telnet.py`

### Acceptance

- `motor commission boot 0.15 0.05 1` passes with no encoder warn/error samples.
- `motor commission validate current 0.03 160` passes:
  - no motor fault.
  - no encoder CRC/status/glitch errors above threshold.
  - +Iq and -Iq show expected opposite tendency or documented detent-limited
    behavior with higher safe current.
- `motor commission validate velocity 0.03 1000` passes:
  - no fault.
  - no runaway.
  - measured velocity has correct sign.
  - target error and overshoot within configured thresholds.
- `position_encoder` small move passes only after velocity acceptance is met.

### Evidence

- HIL JSON/log for boot commissioning.
- HIL JSON/log for current validation.
- HIL JSON/log for velocity validation.
- HIL JSON/log for position validation if attempted.
- Unit tests for any changed regulator behavior.

## P3: Robust Encoder Mapping

### Purpose

Replace the fragile single one-direction mapping expectation with a robust
generated-sweep encoder mapping workflow.

### Work

- Decide whether to extend `motor commission boot` or add:

  ```text
  motor commission encoder robust <current_a> <mech_hz> <cycles> [bidirectional]
  ```

- Add multi-cycle averaging.
- Add optional bidirectional sweep.
- Report:
  - direction sign.
  - mechanical offset.
  - electrical offset.
  - motion span.
  - correlation.
  - residuals.
  - accepted/rejected sample counts.
  - encoder diagnostics.
- Ensure boot gate can call the robust mapper with conservative defaults.
- Ensure mapping output uses the same offset convention documented in code.

### Likely Files

- `app/src/shell_motion_commission.c`
- `modules/motor_core/src/calibration/encoder_map_detect.c`
- `modules/motor_core/include/motor/calibration/encoder_map_detect.h`
- `tests/unit/motor_encoder_map_detect/src/main.c`
- `scripts/hil/hil_telnet.py`

### Acceptance

- Unit tests cover multi-cycle and bidirectional sample sets.
- Boot mapping still works with one-cycle default.
- Robust mapping works with at least two cycles.
- HIL evidence shows stable repeated offsets across at least three runs.

### Evidence

- Unit-test output.
- Three HIL mapping logs with offset variance summary.

## P4: Commissioning UX And Naming

### Purpose

Remove ambiguity between boot gating, encoder mapping, full identification, and
auto tuning.

### Work

- Rename/document command descriptions so:
  - boot commissioning = runtime boot gate.
  - encoder mapping = commutation direction/offset.
  - full commissioning = parameter identification/tuning.
  - validation = HIL smoke/acceptance workflow.
- Make every commissioning command print:
  - what it does.
  - what it does not do.
  - required previous step.
  - next recommended command.
  - failure reason and next diagnostic command.
- Use VESC-style practical outputs: measured values, pass/fail, fault reason,
  and suggested next action.

### Likely Files

- `app/src/shell_motion_commission.c`
- `app/src/shell_commands.c`
- `app/include/shell_commands_commission.h`
- `app/docs/*commission*`
- `scripts/hil/README.md`

### Acceptance

- `motor commission` help tree is understandable without source inspection.
- Failed mapping prints a next action.
- Failed validation prints fault and diagnostic command suggestions.
- Review document examples match actual shell commands.

### Evidence

- Captured shell help output.
- HIL failure or simulated failure output showing actionable next steps.

## P5: Direct ISR Safety Audit

### Purpose

Ensure direct/zero-latency ISR paths do not call kernel-only APIs and do minimal
work.

### Work

- Audit all callbacks configured from timer/ADC/gate-driver interrupts.
- Classify each callback:
  - thread context.
  - normal ISR.
  - direct ISR.
  - zero-latency ISR.
- Replace unsafe calls with:
  - atomic latch.
  - SPSC event ring.
  - ISR-safe enqueue path already used by ADC faults.
- Precompute encoder sampling gate outside `encoder1_callback()` where possible.
- Keep `encoder1_callback()` as request-only:

  ```text
  if (sample_gate) request_sample();
  ```

- Audit gate-driver break callbacks. If they can run in direct/zero-latency
  context, remove `motor_api_post_error()` from that path.

### Likely Files

- `app/src/motor_isr_io.c`
- `app/src/motor_states.c`
- `app/src/motor_control_api.c`
- `app/include/motor_events.h`
- `app/include/config.h`

### Acceptance

- Documented ISR-context table exists.
- No direct/zero-latency callback calls non-ISR-safe kernel APIs.
- Encoder request ISR has no shell/API/state-machine policy work.
- Build passes.
- HIL status and boot-commission still pass.

### Evidence

- Static grep/audit notes in progress log.
- Build output.
- HIL status and boot-commission logs.

## P6: Commissioning Shell Decomposition

### Purpose

Split `shell_motion_commission.c` by workflow without changing runtime behavior.

### Work

Split into files such as:

- `shell_commission_common.c`
- `shell_commission_boot_encoder.c`
- `shell_commission_motion_threshold.c`
- `shell_commission_flux.c`
- `shell_commission_mech.c`
- `shell_commission_detent.c`
- `shell_commission_auto.c`
- `shell_commission_validate.c`

Keep shared helpers private where possible. Move only intentionally shared
helpers into a small private header.

### Likely Files

- `app/src/shell_motion_commission.c`
- new `app/src/shell_commission_*.c`
- new optional `app/include/shell_commission_internal.h`
- `app/CMakeLists.txt`

### Acceptance

- No command behavior changes.
- Command tree remains unchanged.
- Build passes.
- Existing HIL status script passes.
- At least one commissioning help/status command is captured after split.

### Evidence

- Build output.
- `motor commission` help output.
- HIL status log.

## P7: Control Kernel Extraction

### Purpose

Move one more layer from app-specific `motor_control_loop.c` into a reusable
`motor_core` control-kernel step that consumes explicit inputs and produces
explicit outputs.

### Work

- Define a small `motor_control_kernel_input` and `motor_control_kernel_output`
  in `motor_core`.
- Keep Zephyr/device/state-machine data out of the kernel API.
- Move pure dataflow pieces first:
  - policy-derived ref selection.
  - current-ref policy.
  - outer-loop runtime invocation wrapper if feasible.
  - actuator-ref construction.
  - FOC fast block invocation wrapper if feasible.
- Keep ADC conversion, PWM device write, event posting, shell-visible live data,
  and telemetry storage in app.
- Add unit tests for the kernel with generated and encoder feedback inputs.

### Likely Files

- `modules/motor_core/include/motor/runtime/control_kernel.h`
- `modules/motor_core/src/runtime/control_kernel.c`
- `app/src/motor_control_loop.c`
- `app/include/motor_rt_control_context.h`
- new `tests/unit/motor_control_kernel/`

### Acceptance

- Unit tests cover disabled, generated velocity, current encoder, velocity
  encoder, bad feedback hold, and current limit behavior.
- `motor_control_loop.c` line count and responsibilities decrease.
- Firmware build passes.
- HIL status and boot-commission pass.

### Evidence

- Unit-test output for new suite.
- Build output.
- HIL log.
- Progress entry listing what remains app-owned.

## P8: TI-Style Fast Block Discipline

### Purpose

Make ISR-called modules follow a consistent init/validate/fast-step pattern.

### Work

- Audit all functions called by `motor_control_loop_step()`.
- For each module, classify:
  - init/config validation.
  - runtime catastrophic guard.
  - fast step.
- Avoid repeated heavy validation in the ISR where config invariants are already
  enforced.
- Keep null checks only on public non-fast APIs.
- Keep catastrophic finite/range checks where a bad value can cause dangerous
  PWM/current output.
- Document which fast APIs assume prevalidated config.

### Likely Files

- `modules/motor_core/src/control/*.c`
- `modules/motor_core/include/motor/control/*.h`
- `modules/motor_core/src/runtime/*.c`
- `app/src/motor_control_loop.c`
- tests for fast/validated equivalence.

### Acceptance

- Every ISR-called module has documented invariant assumptions.
- Fast and validated paths have equivalence tests where both exist.
- Firmware build passes.
- ISR max-cycle telemetry is no worse than baseline in a comparable HIL status
  or generated-mode run.

### Evidence

- Audit table in progress log or a dedicated doc.
- Unit-test output.
- Build output.
- ISR timing before/after.

## P9: Persistence Readiness

### Purpose

Prepare runtime persistence for commissioned parameters without enabling unsafe
autoload prematurely.

### Work

- Define schema for:
  - current offsets.
  - encoder direction and commutation offset.
  - motor Rs/L/flux/mechanical parameters.
  - PI/MPR/DOB/detent settings.
  - version and CRC.
- Define load policy:
  - disabled by default.
  - explicit shell enable/apply.
  - rollback-safe defaults.
  - reject stale/invalid schema.
- Decide NVMEM/settings backend later; do not require EEPROM implementation in
  this phase unless explicitly requested.
- Add shell visibility for staged/persisted values.

### Likely Files

- `app/docs/settings_persistence_plan.md`
- `modules/motor_core/include/motor/runtime/config_snapshot.h`
- `app/src/shell_commands*.c`
- future storage backend files.

### Acceptance

- Persistence schema doc exists.
- Unit tests validate schema/version/CRC helper if implemented.
- No boot autoload is enabled until HIL stability criteria are met.

### Evidence

- Schema document.
- Unit-test output if code added.

## P10: Regression Gate

### Purpose

Make future changes safe by defining a repeatable pre-merge/pre-HIL checklist.

### Work

- Add a checklist document for:
  - full unit tests.
  - firmware build.
  - non-motion HIL status.
  - boot commissioning.
  - current encoder smoke.
  - velocity encoder PI.
  - position encoder once stable.
- Add script wrappers for common HIL runs.
- Define what logs must be retained and where.
- Optionally add a `make` or shell wrapper that runs non-HIL checks.

### Likely Files

- `app/docs/reviews/system_review_regression_gate.md`
- `scripts/hil/*.sh`
- `scripts/checks/*.sh`
- `AGENTS.md`

### Acceptance

- A new agent/developer can run the documented gate without asking for command
  syntax.
- HIL logs contain enough evidence to reproduce pass/fail decisions.
- Gate distinguishes required checks from optional/stress checks.

### Evidence

- Dry-run of non-HIL checks.
- One full HIL status log.
- One live-motion HIL log when hardware is available.

## Cross-Phase Evidence Matrix

| Evidence | Required For |
|---|---|
| `./tests/run_unit_tests.sh` | P0, P2, P3, P7, P8 |
| firmware `west build` | Every phase with code changes |
| `hil_telnet.py status` | P0, P5, P6, P7, P10 |
| `hil_telnet.py boot-commission` | P2, P3, P5, P7 |
| `hil_telnet.py encoder-validate` | P2, P3, P8, P10 |
| parser JSON report | P1 and later HIL phases |
| ISR timing data | P5, P7, P8 |

## Current Known Open Questions

- What numeric thresholds should define velocity PI acceptance for the hybrid
  stepper at very low speed?
- Should robust encoder mapping be the default `boot` behavior or a separate
  explicit command that `boot` can call?
- Is +Iq/-Iq asymmetry mostly static friction/detent, commutation phase, or
  remaining sign convention?
- Should position validation wait until persistence exists, or remain purely
  runtime after boot mapping?
- Should gate-driver break callbacks be treated as direct/zero-latency in all
  builds?
