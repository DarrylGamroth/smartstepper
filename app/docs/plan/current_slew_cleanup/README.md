# Current Slew Cleanup Plan

Goal: make Id/Iq current slew limiting a single runtime current-reference stage owned by `motor_parameters` and advanced only by the ADC ISR.

## Scope

- Keep `traj_Id` and `traj_Iq` storage in `struct motor_parameters`.
- Treat calibration, shell, and regulators as target/rate writers only.
- Ensure the ADC ISR is the only code that calls `traj_run()` for current references.
- Preserve special calibration ramp rates for ROVERL, RS_EST, and ALIGN by allowing those states to configure `traj_Id` target/max_delta before the ISR runs.
- Route normal online Id/Iq references through the same current slew stage.

## Non-Goals

- Do not change current PI gains or controller tuning.
- Do not change encoder policy or commissioning algorithms beyond current slew ownership.
- Do not introduce dynamic allocation or kernel work into the ISR.

## Tasks

1. Document current slew ownership and target-writer rule.
2. Add `traj_Iq` runtime state beside `traj_Id`.
3. Move current trajectory advancement into one ISR current-slew stage.
4. Remove calibration-specific `traj_run()` duplication.
5. Ensure stop/disarm paths zero both current trajectory targets.
6. Build with the MT6835 overlay.

## Progress

- [x] T1 ownership plan documented
- [x] T2 `traj_Iq` runtime state added
- [x] T3 single ISR current-slew stage
- [x] T4 remove calibration-specific `traj_run()` duplication
- [x] T5 zero both trajectory targets on stop/disarm paths
- [x] T6 build validation
- [x] T7 shared current-slew helper added in `motor_core`
- [x] T8 `current-command-ramp-ms` devicetree setting added
- [x] T9 current slew shell status added

## Evidence

- Firmware build passed:
  - `podman exec wonderful_goldberg bash -lc 'cd /workspace && west build -p auto -b smartstepper_v2/stm32h743xx /workspace/chopper/app -d /workspace/build/chopper/smartstepper_v2 -S serial-shell -S serial-console -- -DDTC_OVERLAY_FILE="boards/smartstepper_v2.overlay;configs/motor_mt6835_2a.overlay"'`
- Focused unit tests passed:
  - `./tests/run_unit_tests.sh wonderful_goldberg -s chopper.motor_rl_ident.unit`
  - 13/13 test cases passed.
  - `./tests/run_unit_tests.sh wonderful_goldberg -s chopper.traj.unit`
  - 10/10 test cases passed.
- HIL current-slew smoke:
  - Built and flashed MT6835 firmware with J-Link.
  - `motor current iq 0.15` configured `Iq traj target/int/delta = 0.1500 / 0.0000 / 0.000075 A`, matching a 0.15 A command step over 100 ms at 20 kHz.
  - `motor commission run confirm apply` still failed during `+Iq` validation with `ENCODER_FAULT`, reason `velocity_spike`; transport counters remained clean (`transport=0 parity=0 crc=0 glitch=0 status=0`). This indicates the current-step slew cleanup is functioning, but the encoder/control validation still sees a real motion/observer spike during current_encoder validation.
