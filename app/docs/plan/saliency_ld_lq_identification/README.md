# Saliency / Ld-Lq Identification Plan Package

## Purpose

Add a robust commissioning-time saliency estimator for hybrid stepper motors. The estimator should identify scalar inductance, `Ld`, `Lq`, saliency ratio, and saliency phase from high-frequency inverse-inductance samples.

The design must separate:

- `motor_core` estimator math: consumes samples and computes results.
- App/commissioning excitation producer: generates test vectors and collects paired pulse samples.

This lets us start with an easy-to-validate electrical-angle sweep and later replace or supplement it with a faster VESC-style six-vector producer.

## Motivation

Recent AEAT HIL results showed that cycle-counted paired demodulation can measure D-axis inductance close to the `3.4 mH` datasheet value. However, direct D/Q demod reported lower `Lq` on one run. For hybrid steppers, true saliency is plausible, but Q-axis locked measurements can be contaminated by torque-producing motion.

A saliency sweep measures inverse inductance versus electrical angle and extracts the second harmonic:

```text
invL(theta) = offset + a*cos(2theta) + b*sin(2theta)
amplitude = sqrt(a^2 + b^2)
Ld = scale / (offset + amplitude)
Lq = scale / (offset - amplitude)
```

This is the same core information used by VESC HFI/bin0/bin2 methods, but implemented in a commissioning-friendly form first.

## Non-Goals

- Do not remove existing pulse inductance or R/L fallback yet.
- Do not require runtime HFI for normal control.
- Do not persist results yet.
- Do not make shell-visible data the primary architecture goal; shell output is only validation evidence.
- Do not make the estimator depend on Zephyr, shell, or motor app state.

## Architecture

```text
motor_core/estimation/saliency_id
  reset(config)
  add(theta_elec_rad, inv_l_sample)
  finalize(result)

app commissioning producer v1: electrical-angle sweep
  vector_count configurable
  pairs_per_vector configurable
  revolutions configurable
  half_cycles configurable
  pulse_v configurable
  feeds estimator

app commissioning producer v2: VESC-style six-vector/HFI producer
  fixed or configurable vector pattern
  feeds same estimator
```

## Key Concepts

### Estimator Input

Each sample is:

```text
theta_elec_rad
inv_l = (di_pos - di_neg) / (flux_pos - flux_neg)
```

Where paired positive/negative half-cycles are used to cancel current offset and slow drift.

### Configurable Sweep Producer

Parameters:

```text
vector_count      = electrical angle points per electrical revolution
pairs_per_vector  = +V/-V paired samples per point
revolutions       = repeated electrical revolutions
half_cycles       = injection half-period in control-loop ticks
pulse_v           = bounded injection voltage
```

Recommended first defaults:

```text
vector_count = 32
pairs_per_vector = 4
revolutions = 2
half_cycles = 40   # 250 Hz at 20 kHz control loop
pulse_v = motor overlay default demod pulse
```

Faster diagnostic defaults:

```text
vector_count = 16
pairs_per_vector = 2
revolutions = 1
```

Higher-confidence defaults:

```text
vector_count = 32 or 64
pairs_per_vector = 8
revolutions = 2-4
```

## Result Fields

`motor_core` result should include:

```text
l_avg_h
ld_h
lq_h
lq_minus_ld_h
saliency_ratio
phase_rad
inv_l_offset
inv_l_cos2
inv_l_sin2
inv_l_amplitude
sample_count
rejected_count
confidence
valid
flags
```

Confidence should initially use:

- enough samples
- positive denominator checks
- bounded saliency ratio
- residual RMS of fitted model
- repeatability across revolutions if supplied by producer

## Plan Phases

### Phase 0: Requirements and Existing-Path Guardrails

Tasks:

- Keep existing electrical-ID commands working.
- Keep bidirectional Rs as production resistance path.
- Keep pulse/R-over-L inductance as fallback.
- Document command naming and expected use.

Validation:

- Existing `motor_electrical_id` unit tests pass.
- AEAT build still passes.

### Phase 1: motor_core Saliency Estimator

Tasks:

- Add `modules/motor_core/include/motor/estimation/saliency_id.h`.
- Add `modules/motor_core/src/estimation/saliency_id.c`.
- Implement reset/add/finalize.
- Use direct synchronous sums; do not use FFT.
- Add unit tests for:
  - isotropic motor: `Ld == Lq`
  - salient motor with known `Ld/Lq`
  - phase offset recovery
  - excessive residual rejection
  - invalid denominator rejection
  - insufficient sample rejection

Validation:

```bash
./tests/run_unit_tests.sh wonderful_goldberg -s chopper.motor_saliency_id.unit
```

### Phase 2: Sweep Producer Command

Tasks:

- Add app-side commissioning command:

```text
motor commission electrical saliency_sweep [pulse_v] [vectors] [pairs] [revs] [half_cycles]
```

- Use generated electrical-angle hold at each vector.
- At each vector, collect paired +V/-V demod samples.
- Feed `theta_elec_rad` and `invL` to `motor_core` saliency estimator.
- Stage `Ld/Lq` only if result passes validation.
- Print concise evidence: `Ld`, `Lq`, `Lavg`, saliency ratio, phase, residual, confidence.

Validation:

- AEAT build passes.
- Command rejects invalid args without changing staged results.
- Command can run without encoder mapping because generated angle is used.

### Phase 3: HIL Sweep Validation

Tasks:

Run on AEAT known motor:

```text
motor gate reset
motor state clear_error
motor disarm
motor state idle
motor safety timeout 0
motor commission electrical clear
motor commission electrical measure rs
motor commission electrical demod_sweep
motor commission electrical saliency_sweep 0.45 32 4 2 40
motor commission electrical status
```

Acceptance:

- No motor fault.
- `Rs` remains plausible.
- `Lavg` agrees with D-axis demod sweep and datasheet within reasonable tolerance.
- `Ld/Lq` are repeatable across at least two runs.
- Saliency ratio is physically plausible and bounded.

### Phase 4: MT6835 Cross-Check

Tasks:

- Build/flash MT6835 overlay.
- Run same saliency sweep.
- Compare against current scalar inductance methods.

Acceptance:

- Results repeat across runs.
- No uncontrolled motion during sweep.
- If `Ld/Lq` are unstable, keep scalar L only.

### Phase 5: Six-Vector Producer Prototype

Tasks:

- Add a second producer that uses a fixed six-vector pattern.
- Feed the same `motor_core` estimator.
- Compare six-vector result against 32-point sweep.

Acceptance:

- Six-vector agrees with sweep within configured tolerance.
- If it does not, keep sweep as production commissioning path and six-vector as experimental.

### Phase 6: Runtime Use Policy

Tasks:

Define how results are consumed:

- If saliency result valid and repeatable:
  - use `Ld` for D current PI tuning
  - use `Lq` for Q current PI tuning
  - use `Ld/Lq` in decoupling when decoupling is enabled
- If saliency result invalid:
  - use scalar `L` for both axes
  - leave decoupling disabled or scalar-only

Validation:

- Unit-test tuning calculations.
- HIL current-loop validation passes with selected parameters.

## Execution Order

1. Phase 1 estimator and unit tests.
2. Phase 2 sweep command.
3. Phase 3 AEAT HIL.
4. Phase 4 MT6835 HIL.
5. Phase 5 six-vector producer only after sweep is proven.
6. Phase 6 runtime use policy.

## Open Decisions

- Whether `Ld` should be defined as minimum or maximum inductance for this hybrid stepper convention.
- Whether the saliency phase should also be used to validate encoder electrical offset.
- Whether saturation characterization needs multiple bias currents.
- Whether six-vector should be implemented as exactly six vectors or a configurable repeated vector table.
