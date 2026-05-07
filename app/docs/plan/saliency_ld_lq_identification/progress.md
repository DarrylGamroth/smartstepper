# Saliency / Ld-Lq Identification Progress

## 2026-05-06

Plan package created.

Current state:

- Cycle-counted D-axis demod sweep is implemented and HIL-proven on AEAT.
- Direct D/Q demod passes but Q-axis result needs repeatability validation.
- This plan proposes a motor_core saliency estimator plus configurable sweep producer before any six-vector optimization.

Progress:

- Phase 1 complete: added `motor_core` saliency estimator (`motor/estimation/saliency_id`) using a second-harmonic inverse-inductance least-squares fit.
- Phase 1 validation complete: `./tests/run_unit_tests.sh wonderful_goldberg -s chopper.motor_saliency_id.unit` passes 7/7 cases.
- Electrical ID regression validation complete: `./tests/run_unit_tests.sh wonderful_goldberg -s chopper.motor_electrical_id.unit` passes 13/13 cases.
- Phase 2 implementation complete: added `motor commission electrical saliency_sweep [pulse_v] [vectors] [pairs] [revs] [half_cycles]`.
- Phase 2 firmware validation complete: AEAT overlay west build passes without `debug.conf`.

Pending:

- Flash target and run HIL saliency sweep after staging Rs.
- Compare saliency sweep `Ld/Lq` against existing demod and pulse methods on AEAT and MT6835 hardware.

HIL attempt:

- Flashed AEAT build successfully with `configs/motor_aeat9955_067a.overlay`.
- Shell command registration verified on target: `motor commission electrical plan` lists `saliency_sweep`.
- Electrical HIL sequence could not execute because target initialization failed before commissioning:
  - `encoder1 device/transport is not ready`
  - state remained `ERROR`, requested `HW_INIT`
  - `motor commission electrical measure rs` returned `Failed to prepare idle zero-current state (err -14)`
- This blocks saliency HIL validation and is separate from the saliency estimator implementation.

MT6835 HIL attempt:

- Built and flashed `configs/motor_mt6835_2a.overlay` successfully.
- Serial port used: `/dev/serial/by-id/usb-FTDI_TTL232R-3V3_FTHC021S-if00-port0`.
- Initial state was `IDLE`, encoder direction sign `-1`, no fault.
- `motor commission electrical measure rs` completed after current offset calibration:
  - `Rs=2.225446 ohm`
  - `samples=256`
  - `confidence=0.906`
  - `residual=0.0235`
- `motor commission electrical saliency_sweep 0.45 32 4 2 40` executed but rejected:
  - `Ld=0.001728071 H`
  - `Lq=0.005300771 H`
  - `saliency ratio=1.017`
  - `residual=0.104`
  - `confidence=0.482`
  - `samples=256`, `rejected=0`
- Repeat saliency sweeps also rejected:
  - `0.35 V`: `Ld=0.001914471 H`, `Lq=0.005240073 H`, ratio `0.930`, residual `0.075`, confidence `0.627`
  - `0.50 V`: `Ld=0.001900401 H`, `Lq=0.005342009 H`, ratio `0.950`, residual `0.064`, confidence `0.681`
- Cross-check `motor commission electrical measure demod 0.50 128 40` passed:
  - `Ld=0.003929376 H`
  - `Lq=0.002266022 H`
  - `Lavg=0.003097699 H`
  - `D spread=0.023 confidence=0.883`
  - `Q spread=0.049 confidence=0.753`
  - resulting PI valid: Id `Kp=3.703350 Ki=0.028318`, Iq `Kp=2.135675 Ki=0.049105`

Interpretation:

- The saliency command path executes on MT6835 hardware.
- The current Rs path is healthy on this setup.
- The vector saliency estimator is not yet production-valid: it gives repeatable but implausibly high saliency compared with the direct D/Q demod cross-check.
- Most likely issue is the excitation model/sweep convention, not shell/build/encoder transport.

Saliency sweep improvement pass:

- Added generated-frame observer reseed at each vector so the direct-voltage inverse Park angle is the commanded vector, not a slowly converged observer output.
- Added zero-voltage vector-settle time. Default is derived from the electrical time constant (`5*L/R`) and can be overridden with the new optional `[settle_ticks]` command argument.
- Added optional first- and fourth-harmonic nuisance terms to the saliency estimator:
  - `1θ` captures phase/current-sensor asymmetry or fixture bias.
  - `4θ` captures tooth/slotting ripple common in hybrid steppers.
  - `Ld/Lq` remain derived only from the `2θ` term.
- Added unit tests proving nuisance harmonics do not bias the extracted second-harmonic `Ld/Lq` on synthetic data.
- Added reject diagnostics to report `inv_amp1`, `inv_amp2`, and `inv_amp4`.

Validation:

- `./tests/run_unit_tests.sh wonderful_goldberg -s chopper.motor_saliency_id.unit` passes 9/9 cases.
- MT6835 west build passes.

MT6835 HIL after improvements:

- Rs remained healthy: `Rs=2.249714 ohm`, confidence `0.910`, residual `0.0225`.
- `saliency_sweep 0.50 16 4 2 40 200` still rejected:
  - `Ld=0.002367181 H`, `Lq=0.005258683 H`, ratio `0.758`, residual `0.225`, confidence `0.000`.
- `saliency_sweep 0.50 32 4 1 40 200` still rejected:
  - `Ld=0.002353034 H`, `Lq=0.005271825 H`, ratio `0.766`, residual `0.228`, confidence `0.000`.

Interpretation:

- The saliency sweep is improved structurally, but it is still not production-acceptable.
- Rejection is now due to residual, not sample loss or command failure.
- The direct D/Q demod path remains the trusted inductance method for now.
- Next saliency-specific task should capture per-vector inverse-L samples so the waveform can be inspected directly before further model changes.

Vector-bin saliency acceptance pass:

- Changed saliency finalization to support vector-bin Fourier extraction:
  - Ld/Lq are extracted from the averaged second harmonic across commanded electrical vectors.
  - First/fourth harmonic diagnostics remain visible but no longer cause rejection by shape residual.
  - Acceptance now qualifies repeatability of each vector mean instead of rejecting useful hybrid-stepper saliency because of extra spatial harmonics.
- Added unit coverage for bin finalization with an unmodelled third harmonic, verifying the second-harmonic Ld/Lq estimate remains correct.

Validation:

- `./tests/run_unit_tests.sh wonderful_goldberg -s chopper.motor_saliency_id.unit` passes 10/10 cases.
- MT6835 west build passes with `boards/smartstepper_v2.overlay;configs/motor_mt6835_2a.overlay`.
- Flashed target with J-Link at `10.0.0.70` and ran HIL over telnet at `10.0.0.44`.

MT6835 HIL pass:

- `motor commission electrical measure rs`:
  - `Rs=2.259084 ohm`
  - `samples=256`
  - `confidence=0.914`
  - `residual=0.0216`
- Cross-check `motor commission electrical measure demod 0.50 128 40`:
  - `Ld=0.002690219 H`
  - `Lq=0.003431246 H`
  - `Lavg=0.003060733 H`
  - D spread `0.028`, Q spread `0.026`
- `motor commission electrical saliency_sweep 0.50 16 4 2 40 200` now stages successfully:
  - `Ld=0.002101421 H`
  - `Lq=0.005071129 H`
  - `Lavg=0.003586275 H`
  - `ratio=0.828`
  - `phase=0.719 rad`
  - `residual=0.132`
  - `confidence=0.339`
  - `samples=128`, `rejected=0`
- `motor commission electrical saliency_sweep 0.50 32 4 1 40 200` also stages successfully:
  - `Ld=0.002119287 H`
  - `Lq=0.005212575 H`
  - `Lavg=0.003665931 H`
  - `ratio=0.844`
  - `phase=0.737 rad`
  - `residual=0.128`
  - `confidence=0.362`
  - `samples=128`, `rejected=0`
- Final `motor commission electrical status` reported a valid staged result and valid PI recommendation:
  - Id PI: `Kp=1.997381`, `Ki=0.053298`
  - Iq PI: `Kp=4.912736`, `Ki=0.021670`
- `motor state status` ended in `IDLE`, `Error: NONE`.

Interpretation:

- Saliency sweep now passes on MT6835 HIL.
- The saliency result reports a stronger axis split than direct D/Q demod; treat it as useful Ld/Lq saliency information, while the D/Q demod average remains the scalar inductance cross-check.
- No EEPROM/persistence is involved; values are staged only.

Production-source policy update:

- Changed `motor commission electrical saliency_sweep` to diagnostic-only behavior:
  - It measures and reports saliency Ld/Lq, harmonic amplitudes, residual, confidence, and rejected samples.
  - It no longer clears or replaces the staged production inductance/PI result.
  - `motor commission electrical status` now reports saliency as `not-staged` diagnostic unless explicitly applied.
- Added explicit `motor commission electrical saliency_apply` for manual staging of the last valid saliency diagnostic result.
  - This is intentionally opt-in because MT6835 demod has produced a more stable scalar inductance cross-check than saliency.
- Changed `motor commission electrical run` to use the demodulated inductance path as the production default after Rs instead of the older integral pulse path.

Validation:

- `./tests/run_unit_tests.sh wonderful_goldberg -s chopper.motor_saliency_id.unit` passes 10/10 cases.
- MT6835 west build passes.
- Flashed MT6835 target and ran telnet HIL workflow:
  - `motor commission electrical measure rs` staged `Rs=2.259402 ohm`, confidence `0.910`, residual `0.0226`.
  - `motor commission electrical measure demod 0.50 128 40` staged production demod L:
    - `Ld=0.004628600 H`, `Lq=0.001880785 H`, `Lavg=0.003254692 H`.
    - `L source: demod`.
  - `motor commission electrical saliency_sweep 0.50 32 4 1 40 200` ran afterward and did not overwrite the production source.
  - Final status kept `L source: demod` and printed `Saliency diagnostic: not-staged ...`.
  - Final `motor state status`: `State: IDLE`, `Error: NONE`.

Interpretation:

- Demod is now the production inductance source for current-loop PI tuning.
- Saliency remains available for diagnostics and future refinement, but it will not be applied accidentally.
