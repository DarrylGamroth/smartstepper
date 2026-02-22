# `motor_core` Library Review and Improvement Plan

Date: 2026-02-22
Scope: `modules/motor_core` and corresponding unit tests under `tests/unit/*`

## Summary

`motor_core` is a reusable control/math library for the chopper firmware. It currently provides:

- Control primitives: PI, trajectory ramping, first/second-order filters, PWM mapping.
- Estimation/observer blocks: angle observer, position convert, RLS/online Rs estimators, thermal model, PRBS.
- Motion modules: quintic profile generation and sequence helpers.
- Outer-loop regulators: velocity/position MPR and velocity DOB.
- Commissioning support: flux/mechanical identification and auto-tune output generation.
- Autonomy utility: keepalive policy helper.

Architecture is generally good (single-purpose modules, clear API boundaries). Core behavior is broadly validated by unit tests.

## Validation Snapshot

Unit tests executed via:

```bash
./tests/run_unit_tests.sh
```

Result:

- 17/17 suites passed
- 132/132 test cases passed

## Findings (Code Review)

### High

1. Non-finite encoder sample can poison `motor_position_convert` state.
   - Risk: `NaN` can enter wrapped/unwrapped position and then propagate into speed/acceleration consumers.
   - Files:
     - `modules/motor_core/src/motor_position_convert.c:145`
     - `modules/motor_core/src/motor_position_convert.c:151`
     - `modules/motor_core/src/motor_position_convert.c:179`

### Medium

2. `rls_motor_est` has no explicit `lambda` guard.
   - Risk: invalid runtime config (e.g. zero/negative/non-finite `lambda`) can trigger unstable covariance math.
   - Files:
     - `modules/motor_core/src/rls_motor_est.c:29`
     - `modules/motor_core/src/rls_motor_est.c:101`
     - `modules/motor_core/src/rls_motor_est.c:133`

3. `motor_foc_voltage_pwm_step` input hardening is incomplete.
   - Risk: invalid modulation/braking scalars can produce out-of-range duty commands.
   - Files:
     - `modules/motor_core/src/motor_foc_voltage_pwm.c:43`
     - `modules/motor_core/src/motor_foc_voltage_pwm.c:97`
     - `modules/motor_core/src/motor_foc_voltage_pwm.c:101`

4. One-cycle completion semantic mismatch in `motor_position_move_resolve`.
   - Risk: `active` is sampled before stepping; post-step outputs may not reflect true completion state in same call.
   - Files:
     - `modules/motor_core/src/motor_motion_modules.c:122`
     - `modules/motor_core/src/motor_motion_modules.c:124`
     - `modules/motor_core/src/motor_motion_modules.c:135`

### Low

5. `angle_gen_run` only wraps once.
   - Risk: if angle delta exceeds `2*pi` in one update, output can leave expected range.
   - File:
     - `modules/motor_core/include/angle_gen.h:99`

6. `thermal_model_init` has no `update_freq > 0` validation.
   - Risk: divide-by-zero at init.
   - File:
     - `modules/motor_core/src/thermal_model.c:19`

7. Mixed unit conventions (`rad/s` vs `deg/s`) in estimator APIs.
   - Risk: integration mistakes and subtle tuning errors.
   - Files:
     - `modules/motor_core/include/rs_online.h:70`
     - `modules/motor_core/src/rs_online.c:107`

## Module Inventory

- `angle_observer.[ch]`: alpha-beta mechanical/electrical angle tracking with delay compensation.
- `motion_profile.[ch]`: quintic segment planning/evaluation and constraint checks.
- `motor_motion_modules.[ch]`: velocity ramp helper and position sequence helpers.
- `motor_mpr.[ch]`: velocity and position MPR regulators.
- `motor_dob.[ch]`: velocity disturbance observer + iq feedforward.
- `motor_position_convert.[ch]`: unwrap, velocity/accel derivation, quality flags and stale/glitch detection.
- `motor_foc_voltage_pwm.[ch]`: current-loop PI, decoupling feedforward, inverse Park, PWM duty generation.
- `motor_commission_id.[ch]`: flux and mechanical parameter identification.
- `motor_commission_tune.[ch]`: tuning defaults from fit quality.
- `rls_motor_est.[ch]`: recursive least-squares parameter estimator.
- `rs_online.[ch]`: gradient-based online Rs estimator with probe projection.
- `thermal_model.[ch]`: first-order winding thermal model.
- `prbs.[ch]`: maximal-length PRBS generator.
- `motor_autonomy.[ch]`: keepalive decision helper.
- Inline utility modules:
  - `pi.h`, `traj.h`, `filter_fo.h`, `filter_so.h`, `pwmgen.h`, `angle_wrap.h`, `angle_gen.h`, `math_constants.h`

## Improvement Plan

### Phase 1: Safety/Hardening (highest priority)

1. Add finite-value validation to runtime update APIs.
   - Target modules:
     - `motor_position_convert`, `rls_motor_est`, `thermal_model`, `motor_foc_voltage_pwm`.
   - Required tests:
     - New negative tests for `NaN/Inf` inputs and invalid scalar configs.
   - Acceptance:
     - Invalid inputs return error/no-op without state corruption.

2. Enforce strict FOC input constraints and duty clamp backstop.
   - Validate `max_modulation_index` and braking fields.
   - Clamp generated duties to `[0,1]` before output.
   - Acceptance:
     - Deterministic and bounded duty outputs under malformed input.

3. Fix `motor_position_move_resolve` completion semantics.
   - Recompute `active` after stepping.
   - Ensure velocity feedforward is zeroed exactly on completion boundary.
   - Acceptance:
     - Boundary behavior is deterministic and covered by unit tests.

### Phase 2: API Consistency and Numerical Robustness

4. Standardize units for estimator APIs.
   - Preferred: expose radians-based API at module boundaries.
   - Keep any required degree conversion internal.
   - Acceptance:
     - Public docs and call sites are unit-consistent (`rad`, `rad/s`, `rad/s^2`).

5. Add explicit parameter validation helpers for estimator/tuner configs.
   - Add guards for `lambda`, bandwidths, sample rates, min/max ordering.
   - Acceptance:
     - Every config entry point has a validation path and tests.

6. Improve computational efficiency for fixed-config runtime loops.
   - Precompute invariant coefficients where config is static.
   - Keep behavior bitwise-close where possible.
   - Acceptance:
     - No behavior regression in unit tests; documented perf impact (if measured).

### Phase 3: Test Coverage Expansion

7. Add fault-oriented tests focused on hardening gaps.
   - `motor_position_convert`: non-finite measurement handling, glitch + stale interactions.
   - `motor_foc_voltage_pwm`: invalid modulation/braking parameter tests, duty bounds.
   - `rls_motor_est`: invalid `lambda` and finite-guard coverage.
   - `thermal_model`: invalid init parameters.
   - Status: implemented and expanded across `tests/unit/motor_position_convert`,
     `tests/unit/motor_foc_voltage_pwm`, `tests/unit/rls_motor_est`,
     and `tests/unit/thermal_model`.

8. Add integration-style unit tests for module contracts.
   - `motion_profile` + `motor_motion_modules` completion boundary behavior.
   - `position_convert` + `mpr`/`dob` compatibility under stale/fresh transitions.
   - Status: implemented in `tests/unit/motor_core_contracts` (`chopper.motor_core_contracts.unit`).

### Execution Order

1. Phase 1.1-1.3 (safety-critical and low-risk fixes)
2. Phase 2.4-2.5 (API consistency and validation)
3. Phase 3.7 (tests to lock behavior)
4. Phase 2.6 and Phase 3.8 (optimization and contract integration tests)

## Deliverables Checklist

- [x] Hardened runtime validation and bounded outputs in core modules.
- [x] Deterministic profile completion semantics.
- [x] Unit consistency cleanup (`rad/s` API standardization).
- [x] Expanded unit tests for all hardened paths.
- [ ] Updated module docs/comments reflecting final contracts.
