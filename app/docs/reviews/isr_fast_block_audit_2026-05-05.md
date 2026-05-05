# ISR Fast Block Audit

Date: 2026-05-05

Scope: functions reached from `adc_callback()` through `motor_control_loop_step()`
that execute in the real-time control path.

## Fast-Path Rule

ISR-called motor-core blocks should follow this split:

- `validate` / `init`: configuration-time checks, finite checks for static
  parameters, coefficient/precompute work, and state initialization.
- `step`: defensive public API for tests, shell/deferred code, and non-ISR use.
- `step_fast`: ISR API. Assumes non-NULL pointers, prevalidated configuration,
  initialized state, and finite runtime inputs unless a runtime value can
  directly produce unsafe PWM/current output.

Runtime guards retained in fast paths should be limited to catastrophic output
safety checks such as positive voltage limits, valid denominators, saturation
bounds, or explicit disabled paths.

## Audit Table

| Block | ISR Call Site | Current Discipline | Required Runtime Guards | P8 Action |
|---|---|---|---|---|
| `motor_encoder_feedback_update` | `motor_control_step_read_encoder()` | ISR-facing encoder update with acquisition status counters. | Transport status and stale/error handling. | Keep as-is; hardware-facing guard path is required. |
| `motor_control_kernel_step_fast` | `motor_control_loop_step()` | Fast kernel for feedback validity and actuator reference construction. | Feedback stale/error/speed sanity guards. | Added in P7; keep finite speed sanity because corrupt encoder speed can energize the wrong command. |
| `motor_outer_loop_runtime_step` | `motor_control_step_reference_stage()` | Runtime orchestrator, still app-context heavy. | Feedback quality, decimation, saturation. | P8 switched PI regulators to `*_step_fast`; future work should prebuild configs on mode/config changes. |
| `motor_position_regulator_step_fast` | `motor_outer_loop_position_step()` | Fast PI position step added in P8. | None beyond caller guarantees. Output clamp remains part of algorithm. | Validated API retains finite/config checks; equivalence tests added. |
| `motor_velocity_regulator_step_fast` | `motor_outer_loop_velocity_pi_step()` | Fast PI velocity step added in P8. | None beyond caller guarantees. Output clamp remains part of algorithm. | Validated API retains finite/config checks; equivalence tests added. |
| `motor_mpr_position_step_fast` | `motor_outer_loop_position_step()` | Fast MPR path with initialized/config-cache guard. | Denominator and limit guards. | Keep; horizon precompute is already done in `init`. |
| `motor_mpr_velocity_step_fast` | `motor_outer_loop_velocity_mpr_step()` | Fast MPR path, but model struct is rebuilt per step. | Denominator and clamp guards. | Keep for now; future P8/P10 work can move model/config refresh to mode/config transition hooks. |
| `motor_dob_step_fast` | `motor_outer_loop_velocity_dob_step()` | Fast DOB path, but config/model fallback is rebuilt per step. | Disabled path, initialized state, torque/current denominator guards. | Keep for now; DOB is lower priority and disabled unless configured. |
| `motor_detent_map_step_fast` | `motor_outer_loop_detent_ff_step()` | Fast table lookup path. | Initialized/config enabled guards. | Keep; map initialization should eventually move to apply/config time. |
| `motor_dq_decoupling_feedforward_step_fast_values` | `motor_control_step_foc_stage()` | Value fast path. | Voltage magnitude, decoupling limit/headroom. | Keep; DQ decoupling is disabled/low priority and safety-clamped. |
| `motor_current_loop_step_fast_values` | `motor_control_step_foc_stage()` | Value fast path. | Positive voltage magnitude and vector clamp. | Keep; clamping is algorithmic and safety-critical. |
| `motor_transforms_inv_park` | `motor_control_step_foc_stage()` | Static-inline transform with pointer checks. | None needed if caller-owned pointers are valid. | Future cleanup can add a no-pointer-return value variant, but current cost is small. |
| `motor_pwm_synthesis_step_fast_values` | `motor_control_step_foc_stage()` | Value fast path. | Duty clamps and optional braking clamp. | Keep; output clamping is required for PWM safety. |
| telemetry/fault capture | publish/finalize stages | Bounded ISR writes. | Ring bounds and enable/decimation checks. | Keep opt-in/decimated; future telemetry work can move larger dumps to deferred context. |

## P8 Implemented Changes

- Added `motor_position_regulator_step_fast()`.
- Added `motor_velocity_regulator_step_fast()`.
- Changed `motor_outer_loop_runtime_step()` PI paths to call the fast regulator
  APIs after the local config/state setup.
- Kept `motor_position_regulator_step()` and `motor_velocity_regulator_step()`
  as validated public APIs for non-ISR callers and tests.
- Added unit tests proving the validated and fast PI regulator paths produce
  identical output/state for valid inputs.

## Remaining Fast-Path Work

- Move recurring outer-loop config/model construction out of the step path when
  mode/config transition hooks are available.
- Consider value-return transform variants if disassembly/timing shows the
  current pointer-return transform wrappers matter.
- Move detent/MPR/DOB initialization from opportunistic step-time checks to
  explicit mode/config apply points.
- Keep `CONFIG_MOTOR_ISR_SANITY_CHECKS` disabled for performance builds and use
  it only for debug/fault-isolation builds.
