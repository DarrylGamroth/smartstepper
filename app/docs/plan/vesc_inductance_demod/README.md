# VESC-Style Inductance Demodulation Plan

## Purpose

Improve the experimental VESC-style inductance demodulation path without replacing the production electrical-ID fallback. The current AEAT run showed useful signal, but excessive spread:

- Bidirectional `Rs`: valid and plausible because it includes cabling/system resistance.
- Integral pulse / R-over-L implied `L`: plausible, around the datasheet inductance.
- Demodulated inverse-L: rejected because spread is too high, not because signal is too small.

The goal is to make demodulation a useful cross-check and possible future production method.

## Design Rules

- Keep bidirectional Rs as the production Rs path.
- Keep integral pulse and R/L-implied inductance as production fallbacks.
- Make demod excitation synchronous with the control ISR/PWM cadence.
- Do not rely on millisecond sleep granularity for demod half-cycles.
- Reject/stage based on repeatability and agreement, not just whether raw samples exist.
- Avoid persistence until HIL repeatability is proven.

## Implementation Phases

### Phase 1: Cycle-Counted Excitation

- Add devicetree default `commission-electrical-demod-half-cycles`.
- Interpret `motor commission electrical measure demod` third argument as control-loop half-cycles.
- Compute excitation frequency as `CONTROL_LOOP_FREQUENCY_HZ / (2 * half_cycles)`.
- Wait for control-loop tick deltas instead of `k_msleep()` pulse widths.

### Phase 2: Paired Half-Cycle Estimator

- Add `motor_electrical_id_demod_add_pair()` in `motor_core`.
- Pair positive and negative half-cycles before computing one inverse-L sample:

```text
inv_L = (di_pos - di_neg) / (flux_pos - flux_neg)
```

- Discard the first positive/negative pair after enabling excitation.
- Unit-test offset cancellation and existing rejection behavior.

### Phase 3: Frequency Sweep Command

- Add `motor commission electrical demod_sweep [pulse_v] [samples]`.
- Sweep D-axis half-cycle counts `{100, 67, 50, 40, 25, 20, 10}`.
- At 20 kHz this covers approximately `{100, 149, 200, 250, 400, 500, 1000}` Hz.
- Print pass/fail, L, spread, confidence, samples, and reject counters for each point.
- Stage the best scalar D-axis result only if at least one point passes.

### Phase 4: HIL Decision Gate

Run on AEAT known motor:

```text
motor commission electrical clear
motor commission electrical plan
motor commission electrical measure rs
motor commission electrical measure inductance
motor commission electrical demod_sweep
motor commission electrical status
```

Accept demod as useful only if:

- At least one frequency passes the configured spread/confidence gates.
- The selected L is close to pulse/R-over-L implied L.
- Repeat runs select similar frequencies and L values.

If no frequency passes, keep demod as diagnostic-only and continue using pulse/R-over-L inductance for tuning.

## Status

- Phase 1: implemented.
- Phase 2: implemented.
- Phase 3: implemented.
- Phase 4: initial AEAT HIL complete.

## AEAT HIL Evidence

Test log files:

- `/tmp/aeat_demod_sweep_test.log`
- `/tmp/aeat_measure_demod_test.log`

AEAT known motor, expected datasheet values approximately `Rs=5.3 ohm`, `L=3.4 mH`.
Measured system resistance includes harness/controller path.

Results after cycle-counted paired demod:

```text
Rs = 6.432213 ohm, confidence=0.831, residual=0.0424

D-axis demod sweep, pulse=0.450 V, samples=128:
  100.0 Hz: L=0.003467489 H, spread=0.023, confidence=0.886
  149.3 Hz: L=0.003434349 H, spread=0.024, confidence=0.879
  200.0 Hz: L=0.003400506 H, spread=0.024, confidence=0.880
  250.0 Hz: L=0.003399392 H, spread=0.024, confidence=0.881
  400.0 Hz: L=0.003378306 H, spread=0.027, confidence=0.864
  500.0 Hz: L=0.003370573 H, spread=0.033, confidence=0.834
  1000.0 Hz: L=0.003451075 H, spread=0.061, confidence=0.693
```

The D-axis sweep is now stable and agrees with the datasheet inductance. The
single D/Q command at the default 250 Hz also passes, but Q measured lower on
this run:

```text
Ld=0.003318363 H, Lq=0.002702025 H, Lavg=0.003010194 H
D spread=0.029, Q spread=0.057
```

Decision:

- Use the D-axis demod sweep as a diagnostic cross-check.
- Do not replace the production scalar inductance path with D/Q demod until
  the Q-axis behavior is understood on both AEAT and MT6835 hardware.
- Keep integral pulse and R/L-implied inductance available as fallbacks.
