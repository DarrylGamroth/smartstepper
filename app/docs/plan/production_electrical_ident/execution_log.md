# Execution Log

## 2026-05-06

- Created plan package for production electrical identification.
- Existing Rs/R-over-L measurement is explicitly retained as fallback while a VESC-inspired production path is added alongside it.

## 2026-05-06 Implementation Pass

- Implemented `motor/estimation/electrical_id.h` and `estimation/electrical_id.c` in `motor_core`.
- Added scalar, allocation-free production electrical-ID math:
  - Rs through-origin voltage/current fit with residual/confidence gates.
  - Ld/Lq through-origin `v - R*i = L*di/dt` fit with residual/confidence gates.
  - Combined `Rs`, `Ld`, `Lq`, `Lavg`, `Lq-Ld`, axis mismatch, flags, and confidence result.
  - Current PI recommendation from measured `Rs`, `Ld`, `Lq`, bandwidth, and control sample time.
- Added unit test suite `tests/unit/motor_electrical_id` covering:
  - Rs fit, low-current rejection, out-of-range rejection.
  - Inductance fit, low-slew rejection.
  - Ld/Lq combination, saliency reporting, mismatch rejection.
  - Current PI recommendation and invalid-input rejection.
- Added shell command subtree:
  - `motor commission electrical plan`
  - `motor commission electrical measure rs [current_a] [samples] [settle_ms]`
  - `motor commission electrical measure inductance [pulse_v] [samples] [pulse_ms]`
  - `motor commission electrical run [rs_current_a] [l_pulse_v] [samples]`
  - `motor commission electrical status`
  - `motor commission electrical apply`
  - `motor commission electrical clear`
- Added HIL telnet scenario `production-electrical-id` and parser tests for command construction and required-response checks.
- Validation evidence:
  - `./tests/run_unit_tests.sh wonderful_goldberg -s chopper.motor_electrical_id.unit`: PASS, 8/8 cases.
  - `python3 scripts/hil/test_hil_telnet_parser.py`: PASS, 27 tests.
  - `podman exec wonderful_goldberg bash -lc 'cd /workspace && west build -p auto -b smartstepper_v2/stm32h743xx /workspace/chopper/app -d /workspace/build/chopper/smartstepper_v2 -S serial-shell -S serial-console -- -DDTC_OVERLAY_FILE="boards/smartstepper_v2.overlay;configs/motor_mt6835_2a.overlay"'`: PASS.

Notes:

- The production electrical-ID path is staged and explicit; it does not persist values.
- The old Rs/R-over-L path remains available as fallback/bootstrap.
- The shell-level inductance command uses commanded small current pulses and live voltage/current snapshots. The `motor_core` estimator API is suitable for a future ISR-rate pulse sampler if the shell-rate path is too noisy for final production use.
- Focused electrical-estimation regression: `podman exec wonderful_goldberg bash -lc 'cd /workspace/chopper && west twister -T tests/unit -p native_sim --inline-logs -v -s chopper.motor_electrical_id.unit -s chopper.motor_rl_ident.unit -s chopper.rs_online.unit'`: PASS, 25/25 cases.
- Added `motor commission electrical validate [current_a] [hold_ms] [max_error_a]` for explicit current-loop step validation after applying staged production electrical ID.
- Re-ran final validation after adding current-step validation command:
  - `./tests/run_unit_tests.sh wonderful_goldberg -s chopper.motor_electrical_id.unit`: PASS, 8/8 cases.
  - `python3 scripts/hil/test_hil_telnet_parser.py`: PASS, 27 tests.
  - MT6835 west build: PASS.
  - `git diff --check` for production electrical-ID files: PASS.
- Finished app integration by adding `struct motor_electrical_id_capture_ctx` to `motor_parameters` and updating it from `motor_control_loop_step()` at ISR rate after current-loop voltage/current outputs are available.
- Changed shell electrical-ID collection to arm/stop/read the ISR capture context instead of sampling live values in the shell thread.
- Added automatic current-offset calibration gating before generated-angle electrical-ID capture if boot calibration has not completed.
- Added integration fallback policy: if production inductance pulse capture is rejected, `motor commission electrical run` stages fallback L from the existing measured/nominal inductance and still recommends current PI gains from production Rs + fallback L. Direct `measure inductance` still reports rejection.
- HIL validation on MT6835 target via telnet `10.0.0.44` after flash:
  - Command: `python3 scripts/hil/hil_telnet.py production-electrical-id --host 10.0.0.44 --yes-live-motion --electrical-id-current 0.100 --electrical-id-pulse 0.030 --electrical-id-samples 32 --electrical-id-validate-ms 200 --electrical-id-max-error 0.030 --max-crc-errors 10 --max-status-errors 100 --max-glitch-errors 10 --json-report hil_logs/production_electrical_id_report.json`
  - PASS.
  - Rs result: `2.245083 ohm`, residual ratio `0.1398`, confidence `0.441`.
  - Inductance pulse result: rejected; fallback L staged at `0.002903 H` for both axes.
  - Applied current PI: `Kp=2.736013`, `Ki=0.038668` for Id/Iq.
  - Current-step validation: PASS at `0.100 A`, average error `0.00239 A`, max error `0.01198 A`, `4000` ISR samples.
  - Final state: IDLE, `ERROR_NONE`, no encoder transport/parity/CRC/status/glitch errors.

- Improved production electrical ID after HIL review:
  - Rs capture is now bipolar and waits for the full current ramp before sampling, improving confidence from the prior low-confidence transient capture.
  - Inductance capture now bypasses current PI during explicit electrical-ID only, applies bounded direct D/Q voltage pulses through the existing FOC PWM synthesis path, and estimates scalar L from integrated voltage residual per pulse instead of noisy per-ISR `di/dt`.
  - Q-axis pulse is treated as diagnostic only; if it is rejected due torque/motion contamination, the measured D-axis pulse is mirrored as scalar L for Q.
  - HIL result on MT6835 target: `Rs=2.392606 ohm`, `Ld=Lq=0.006349539 H`, confidence `0.895`, residual `0.0369`, current validation PASS at `0.100 A` with avg error `0.00237 A`.
  - Evidence log: `hil_logs/production_electrical_id_d_axis_scalar_l.log`.

- Added a D-axis inductance sweep command for measurement qualification before applying L:
  - `motor commission electrical sweep [samples]` runs several bounded D-axis voltage pulses across voltage and duration.
  - The command reports each point, compares the sweep average with the active R/L-implied L, and only stages scalar `Ld=Lq` if accepted points are stable within the configured spread threshold.
  - This is intended to resolve disagreement between one-shot pulse L and the older R/L-derived L before changing production defaults.

- HIL sweep after qualified high-SNR staging:
  - Command sequence: `motor commission electrical measure rs 0.100 64 250`, then `motor commission electrical sweep 64`.
  - Rs staged: `2.382659 ohm`, confidence `0.712`, residual `0.0719`.
  - D-axis sweep accepted 12 diagnostic points; all-point average `5.736 mH`, spread `0.564`.
  - Low-voltage `0.150 V` points were lower and are treated as diagnostic/low-SNR.
  - Qualified subset threshold was `0.246 V`; 9 qualified points staged scalar `Ld=Lq=6.275735 mH`, confidence `0.768`, residual `0.0812`, spread `0.139`.
  - Active R/L with the staged Rs implies `3.070067 mH`, so the direct pulse and R/L methods still disagree by about `2x`; both values are now visible in status for comparison.
  - Current-loop validation after applying the qualified pulse L passed at `0.100 A`: avg error `0.00236 A`, max error `0.01092 A`.
  - Evidence log: `hil_logs/production_electrical_id_l_sweep_qualified.log`.
