# motor_*.c Migration Plan (`app/src` -> `modules/motor_core`)

Historical note:
- Early execution phases below refer to `modules/motor_core/src/runtime/motor_core_step.c` as an intermediate integration point.
- That top-level runtime entrypoint was removed in commit `328a82b`.
- Current fast-loop orchestration lives in `app/src/motor_control_loop.c`.

## Objective

Move reusable control/runtime logic out of `app/src/motor_*.c` into `modules/motor_core` while keeping Zephyr/board/state-machine glue in app.

Primary goals:
- Reduce app-layer control complexity.
- Make `motor_core` composable and ISR-friendly.
- Keep hard real-time behavior unchanged.

Non-goals:
- Preserve backwards internal file structure in app.
- Change shell UX or control semantics during this migration.

## Boundary Rules

Move to `motor_core` when code is:
- Pure math/control/runtime logic.
- Reusable across boards/encoders.
- Independent of Zephyr driver APIs (`k_*`, `sensor_*`, `gpio_*`, `smf_*`, device handles).

Keep in app when code is:
- ISR callback wiring and hardware driver calls.
- SMF state transitions/policy ownership.
- Shell/API event plumbing and board init.

Migration policy:
- Do not retain app compatibility wrappers after each phase cutover.
- Move callsites directly to `motor_core` APIs and delete legacy app facades in the same phase.

## Target File Ownership

Move candidates:
1. `app/src/motor_control_outer_loops.c`
- New home: `modules/motor_core/src/runtime/outer_loop_runtime.c` (or equivalent runtime split).
- Keep only thin call adapter in app.

2. `app/src/motor_encoder_feedback.c`
- New home: `modules/motor_core/src/observers/encoder_feedback_runtime.c` and/or telemetry helper in core.
- Keep app-side data-source hookup only.

3. `app/src/motor_commission.c` (runtime/estimation internals)
- New home: `modules/motor_core/src/runtime/commission_runtime.c` plus existing `estimation/*` modules.
- Keep app-side command/state entry points minimal.

4. `app/src/motor_rls_runtime.c`
- New home: `modules/motor_core/src/estimation/rls_runtime.c`.
- Keep app-side feature/mode enable policy adapter only if needed.

Keep in app:
- `app/src/motor_isr_io.c`
- `app/src/motor_encoder_pipeline.c`
- `app/src/motor_states*.c`
- `app/src/motor_control_api.c`
- `app/src/motor_hardware.c`

## Phases

## Phase 0: Baseline and Guardrails

Tasks:
1. Record baseline behavior and binary size/runtime counters.
2. Build both overlays:
- `configs/motor_mt6835_2a.overlay`
- `configs/motor_aeat9955_067a.overlay`
3. Run unit tests and save baseline pass list.

Acceptance:
- Baseline build and tests pass before refactor starts.

### Phase 0 Execution Report (2026-03-03)

Status:
- Complete for build and unit-test baseline capture.
- Runtime counter snapshot captured via serial shell.
- Remote flash still blocked from this host (cannot reach J-Link IP `10.0.0.70`).

Commands executed:
1. Incremental baseline build:
```bash
podman exec priceless_wiles bash -lc 'cmake --build /workspace/build/chopper/smartstepper_v2 -j4'
```
Result:
- Passed (`ninja: no work to do.`)

2. Clean build (MT6835 overlay):
```bash
podman exec priceless_wiles bash -lc 'set -euo pipefail; cd /workspace; west build -p always -b smartstepper_v2/stm32h743xx /workspace/chopper/app -d /workspace/build/chopper/baseline_mt6835 -S serial-shell -S serial-console -- -DDTC_OVERLAY_FILE="boards/smartstepper_v2.overlay;configs/motor_mt6835_2a.overlay"'
```
Result:
- Passed
- Region summary: `FLASH 232308 B`, `RAM 126368 B`, `DTCM 5888 B`

3. Clean build (AEAT-9955 overlay):
```bash
podman exec priceless_wiles bash -lc 'set -euo pipefail; cd /workspace; west build -p always -b smartstepper_v2/stm32h743xx /workspace/chopper/app -d /workspace/build/chopper/baseline_aeat9955 -S serial-shell -S serial-console -- -DDTC_OVERLAY_FILE="boards/smartstepper_v2.overlay;configs/motor_aeat9955_067a.overlay"'
```
Result:
- Passed
- Region summary: `FLASH 236708 B`, `RAM 127136 B`, `DTCM 5888 B`

4. ELF size capture:
```bash
podman exec priceless_wiles bash -lc '/opt/toolchains/zephyr-sdk-0.17.4/arm-zephyr-eabi/bin/arm-zephyr-eabi-size /workspace/build/chopper/baseline_mt6835/zephyr/zephyr.elf && /opt/toolchains/zephyr-sdk-0.17.4/arm-zephyr-eabi/bin/arm-zephyr-eabi-size /workspace/build/chopper/baseline_aeat9955/zephyr/zephyr.elf'
```
Result:
- `baseline_mt6835`: `text=229400`, `data=2906`, `bss=144424`, `dec=376730`
- `baseline_aeat9955`: `text=233552`, `data=3154`, `bss=145064`, `dec=381770`

5. Unit tests:
```bash
./tests/run_unit_tests.sh
```
Result:
- `22/23` test configurations passed
- One baseline error in `chopper.motor_foc_voltage_pwm.unit` (CMake source path points to missing `modules/motor_core/src/control/transforms.c`)
- Twister summary: `22 passed, 1 errored` (95.65%)

6. Hardware runtime counter attempt:
```bash
podman exec priceless_wiles bash -lc 'cd /workspace && west flash -d /workspace/build/chopper/baseline_mt6835 --runner jlink --dev-id 10.0.0.70 --dev-id-type ip'
```
Result:
- Blocked: J-Link connection failed from current environment.
- Host probe check: no ICMP response from `10.0.0.70`.

7. Runtime counter capture over serial shell:
```bash
DEV=/dev/serial/by-id/usb-FTDI_TTL232R-3V3_FTE3B04Y-if00-port0
motor state status
motor safety status
motor info live
motor encoder pipeline stats
```
Result snapshot:
- `State=IDLE`, `Error=NONE`, `Armed=NO`
- `Timeout count=0`, `Timeout latch=CLEAR`
- `Max telemetry runtime baseline`: `Encoder faults=0`, `warn=0`, `err=0`
- Encoder RTIO pipeline counters: `request ok=0`, `collect empty=822651`, `transport/frame/parity/status errors=0`

## Phase 1: Move Outer-Loop Runtime

Tasks:
1. Create core runtime API for outer-loop step inputs/outputs/state update.
2. Move logic from `motor_control_outer_loops_step()` into core.
3. Remove app wrapper once core callsites are migrated.
4. Preserve decimation semantics and MPR/PI/DOB fallback behavior.

Acceptance:
- No behavior delta in `velocity_closed` and `position` modes.
- Existing unit tests pass; add targeted tests for scheduler and fallback gates.

### Phase 1 Execution Report (2026-03-03)

Status:
- Complete.

Implemented:
1. Added new core runtime API/header:
- `modules/motor_core/include/motor/runtime/outer_loop_runtime.h`

2. Moved full outer-loop runtime logic into motor_core:
- `modules/motor_core/src/runtime/outer_loop_runtime.c`

3. Removed app compatibility wrapper and legacy header:
- Deleted `app/src/motor_control_outer_loops.c`
- Deleted `app/include/motor_control_outer_loops.h`
- Removed `src/motor_control_outer_loops.c` from `app/CMakeLists.txt`

4. Updated fast control path to use core runtime API directly:
- `modules/motor_core/src/runtime/motor_core_step.c`
  - Replaced state-pointer input with explicit booleans:
    - `position_active`
    - `velocity_active`

5. Added new source into core RT library build:
- `modules/motor_core/src/CMakeLists.txt`

Validation:
1. Firmware build:
```bash
podman exec priceless_wiles bash -lc 'cmake --build /workspace/build/chopper/smartstepper_v2 -j4'
```
Result:
- Passed

2. Targeted unit suites:
```bash
./tests/run_unit_tests.sh priceless_wiles -s chopper.motor_outer_loop_sched.unit -s chopper.motor_mpr.unit -s chopper.motor_dob.unit
```
Result:
- Passed (`3/3` configs)

Notes:
- The known baseline unrelated unit-test issue remains unchanged:
  - `chopper.motor_foc_voltage_pwm.unit` references missing `modules/motor_core/src/control/transforms.c`.

## Phase 2: Move Encoder Feedback Runtime

Tasks:
1. Move observer input selection, quality propagation, and stale accounting to core.
2. Keep RTIO collect/request path in app (`motor_encoder_pipeline.c`, ISR callback).
3. Expose a compact core API:
- `encoder_runtime_update(...)`
- `encoder_capture_pack(...)`
4. Keep raw trace/capture buffer ownership where telemetry architecture expects it.

Acceptance:
- Encoder quality flags and counters match pre-refactor behavior.
- Capture compare output fields unchanged.

### Phase 2 Execution Report (2026-03-04)

Status:
- Complete.

Implemented:
1. Moved encoder feedback runtime API/header into motor_core:
- Added `modules/motor_core/include/motor/observers/encoder_feedback.h`

2. Moved encoder feedback runtime implementation into motor_core observers:
- Added `modules/motor_core/src/observers/encoder_feedback.c`

3. Cut over runtime callsites directly (no app wrapper):
- `modules/motor_core/src/runtime/motor_core_step.c`
  - include updated to `motor/observers/encoder_feedback.h`

4. Removed app-side encoder feedback facade completely:
- Deleted `app/src/motor_encoder_feedback.c`
- Deleted `app/include/motor_encoder_feedback.h`
- Removed `src/motor_encoder_feedback.c` from `app/CMakeLists.txt`

5. Registered new core source in build:
- `modules/motor_core/src/CMakeLists.txt`

Validation:
1. Firmware build:
```bash
podman exec priceless_wiles bash -lc 'cmake --build /workspace/build/chopper/smartstepper_v2 -j4'
```
Result:
- Passed

2. Targeted unit suites:
```bash
./tests/run_unit_tests.sh priceless_wiles -s chopper.motor_encoder_feedback_core.unit -s chopper.angle_observer.unit -s chopper.control_ref_path.unit
```
Result:
- Passed (`3/3` configs)

Notes:
- Phase executed with direct cutover policy (no compatibility wrapper retained).
- Existing unrelated baseline issue remains unchanged:
  - `chopper.motor_foc_voltage_pwm.unit` CMake still references missing `modules/motor_core/src/control/transforms.c`.

## Phase 3: Move Commission Runtime

Tasks:
1. Move sample gating/decimation/derivative filtering/finalization pipeline to core runtime.
2. Keep app-side orchestration (shell + state transitions) minimal.
3. Keep estimation math in `estimation/*` modules, no Zephyr dependencies.
4. Define explicit runtime struct for commission state in core.

Acceptance:
- `commission start/status/apply` shell workflow remains functional.
- Fit quality and mapping validity metrics remain consistent.

### Phase 3 Execution Report (2026-03-04)

Status:
- Complete.

Implemented:
1. Moved commissioning API/header into motor_core:
- Added `modules/motor_core/include/motor_commission.h`

2. Moved commissioning runtime implementation into motor_core:
- Added `modules/motor_core/src/runtime/commission_runtime.c`

3. Removed app-side commissioning implementation/header:
- Deleted `app/src/motor_commission.c`
- Deleted `app/include/motor_commission.h`
- Removed `src/motor_commission.c` from `app/CMakeLists.txt`

4. Added commissioning runtime source to core RT build:
- `modules/motor_core/src/CMakeLists.txt`

5. Removed app state-machine symbol dependency from commissioning runtime:
- Replaced `smf_state*` mode checks with explicit observation flags:
  - `mode_velocity_closed`
  - `mode_torque`
- Updated observation producer in:
  - `modules/motor_core/src/runtime/motor_core_step.c`

Validation:
1. Firmware build:
```bash
podman exec priceless_wiles bash -lc 'cmake --build /workspace/build/chopper/smartstepper_v2 -j4'
```
Result:
- Passed

2. Targeted unit suites:
```bash
./tests/run_unit_tests.sh priceless_wiles -s chopper.motor_commission_tune.unit -s chopper.motor_commission_estimators.unit -s chopper.runtime.unit
```
Result:
- Passed (`3/3` configs)

Notes:
- Phase executed with direct cutover policy (no compatibility wrapper retained).
- Existing unrelated baseline issue remains unchanged:
  - `chopper.motor_foc_voltage_pwm.unit` CMake still references missing `modules/motor_core/src/control/transforms.c`.

## Phase 4: Move RLS Runtime

Tasks:
1. Move RLS gating and staged d/q update logic to core estimation runtime.
2. Keep external policy bits (feature enable, armed/mode) passed as inputs.
3. Keep thermal update decimation path colocated with RLS runtime.

Acceptance:
- RLS convergence counters and estimated parameters remain consistent with baseline.
- No extra ISR-time branching introduced beyond current guardrails.

### Phase 4 Execution Report (2026-03-04)

Status:
- Complete.

Implemented:
1. Moved RLS runtime API/header into motor_core estimation:
- Added `modules/motor_core/include/motor/estimation/rls_runtime.h`

2. Moved RLS runtime implementation into motor_core estimation:
- Added `modules/motor_core/src/estimation/rls_runtime.c`

3. Updated runtime callsite to core header:
- `modules/motor_core/src/runtime/motor_core_step.c`
  - include changed to `motor/estimation/rls_runtime.h`

4. Removed app-side RLS runtime facade:
- Deleted `app/src/motor_rls_runtime.c`
- Deleted `app/include/motor_rls_runtime.h`
- Removed `src/motor_rls_runtime.c` from `app/CMakeLists.txt`

5. Added RLS runtime source into core estimation library build:
- `modules/motor_core/src/CMakeLists.txt`

Validation:
1. Firmware build:
```bash
podman exec priceless_wiles bash -lc 'cmake --build /workspace/build/chopper/smartstepper_v2 -j4'
```
Result:
- Passed

2. Targeted unit suites:
```bash
./tests/run_unit_tests.sh priceless_wiles -s chopper.rls_motor_est.unit -s chopper.rs_online.unit -s chopper.runtime.unit
```
Result:
- Passed (`3/3` configs)

Notes:
- Phase executed with direct cutover policy (no compatibility wrapper retained).
- Existing unrelated baseline issue remains unchanged:
  - `chopper.motor_foc_voltage_pwm.unit` CMake still references missing `modules/motor_core/src/control/transforms.c`.

## Phase 5: App Cleanup and Final Cutover

Tasks:
1. Reduce app files to orchestration/adapters only.
2. Remove duplicate helpers and stale includes.
3. Consolidate headers under `include/motor/...` with clear module boundaries.
4. Update documentation and plan execution log.

Acceptance:
- `app/src` no longer contains reusable control algorithms duplicated with core.
- Ownership boundaries are clear and enforced by includes.

### Phase 5 Execution Report (2026-03-04)

Status:
- Complete.

Implemented:
1. Finalized direct-cutover cleanup by removing remaining app-side runtime algorithm module:
- Deleted `app/src/motor_current_ref_policy.c`
- Deleted `app/include/motor_current_ref_policy.h`
- Removed `src/motor_current_ref_policy.c` from `app/CMakeLists.txt`

2. Moved current-ref policy runtime module into motor_core runtime:
- Added `modules/motor_core/include/motor/runtime/current_ref_policy_runtime.h`
- Added `modules/motor_core/src/runtime/current_ref_policy_runtime.c`
- Added source to `modules/motor_core/src/CMakeLists.txt`
- Updated callsite include in `modules/motor_core/src/runtime/motor_core_step.c`

3. Consolidated commissioning header into `include/motor/runtime/` taxonomy:
- Moved header to `modules/motor_core/include/motor/runtime/commission_runtime.h`
- Updated includes in:
  - `app/include/config.h`
  - `app/src/motor_states.c`
  - `app/src/shell_motion_commission.c`
  - `modules/motor_core/src/runtime/commission_runtime.c`
  - `modules/motor_core/src/runtime/motor_core_step.c`

4. Verified app-side ownership boundaries:
- `app/src` now contains orchestration/glue modules only (`state machine`, `ISR I/O`, `hardware`, `shell`, `API`, `telemetry`, `encoder pipeline`).
- Migrated reusable runtime algorithms remain only in `modules/motor_core/src`.

Validation:
1. Firmware build:
```bash
podman exec priceless_wiles bash -lc 'cmake --build /workspace/build/chopper/smartstepper_v2 -j4'
```
Result:
- Passed

2. Focused migration coverage suites:
```bash
./tests/run_unit_tests.sh priceless_wiles -s chopper.runtime.unit -s chopper.motor_outer_loop_sched.unit -s chopper.motor_encoder_feedback_core.unit -s chopper.motor_commission_estimators.unit -s chopper.rls_motor_est.unit -s chopper.motor_current_ref_policy_core.unit
```
Result:
- Passed (`6/6` configs)

3. Ownership scan:
```bash
rg -n "outer_loop_runtime|encoder_feedback|commission_runtime|rls_runtime|current_ref_policy_runtime" app/src app/include
```
Result:
- Only runtime-header includes in app (`commission_runtime.h`), no app-side algorithm implementations remain.

Notes:
- Existing unrelated baseline issue remains unchanged:
  - `chopper.motor_foc_voltage_pwm.unit` CMake still references missing `modules/motor_core/src/control/transforms.c`.

## API and Dependency Constraints

1. `motor_core` must not include Zephyr kernel/driver headers in control/estimation/observer modules.
2. Runtime step APIs must be C-struct based and allocation-free.
3. No dynamic memory in fast paths.
4. Keep null checks and catastrophic guardrails where required; avoid redundant per-leaf checks in ISR path when ingress validation exists.

## Validation Matrix

Build:
1. Incremental build:
```bash
podman exec priceless_wiles bash -lc 'cmake --build /workspace/build/chopper/smartstepper_v2 -j4'
```
2. Clean build (MT6835):
```bash
podman exec priceless_wiles bash -lc '\
  west build -p always \
    -b smartstepper_v2/stm32h743xx \
    /workspace/chopper/app \
    -d /workspace/build/chopper/smartstepper_v2 \
    -S serial-shell -S serial-console -- \
    -DDTC_OVERLAY_FILE="boards/smartstepper_v2.overlay;configs/motor_mt6835_2a.overlay"'
```
3. Clean build (AEAT-9955):
```bash
podman exec priceless_wiles bash -lc '\
  west build -p always \
    -b smartstepper_v2/stm32h743xx \
    /workspace/chopper/app \
    -d /workspace/build/chopper/smartstepper_v2 \
    -S serial-shell -S serial-console -- \
    -DDTC_OVERLAY_FILE="boards/smartstepper_v2.overlay;configs/motor_aeat9955_067a.overlay"'
```

Tests:
1. Unit tests:
```bash
./tests/run_unit_tests.sh
```
2. Focused suites (as needed): motion profile, RLS, PI, angle observer, MPR, DOB.

Hardware smoke (recommended per major phase):
1. Flash target.
2. Verify `offline -> arm -> velocity_open`.
3. Verify encoder capture/compare pipeline.
4. Verify `torque` and `velocity_closed` transition stability.

## Risks and Mitigations

1. Risk: Behavioral drift during extraction.
- Mitigation: Migrate callsites directly to `motor_core` APIs and validate immediately per phase.

2. Risk: ISR latency regressions.
- Mitigation: Track `max_isr_cycles`, `overrun_count`, and keep ingress validation strategy.

3. Risk: Hidden app-core coupling through `struct motor_parameters`.
- Mitigation: Introduce explicit per-module input/output structs during migration.

## Deliverables

1. New/refined `motor_core` runtime modules for outer loop, encoder runtime, commission runtime, RLS runtime.
2. Thinned app adapters in corresponding `app/src/motor_*.c` files.
3. Updated tests for moved modules.
4. Updated docs and execution log evidence.
