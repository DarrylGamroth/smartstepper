# Commissioned Parameter Use Plan

## Goal

Use the identified motor parameters safely after `motor commission auto run`.

The commissioning flow produces two classes of data:

- Identified motor model values: `psi_f`, `Kt`, `J`, `B`, `Tc`.
- Conservative controller defaults: velocity PI, position PI, MPR defaults, and DOB limits.

The devicetree motor parameters remain boot/fallback values until persistence is added.

## Plan

1. Keep devicetree `flux_linkage_uvphz` and `inertia-mgcm2` as boot/fallback defaults.
2. Apply staged commissioning results explicitly through `motor commission auto apply`.
3. Keep DOB disabled after auto tuning until velocity and position loops are validated.
4. Keep current decoupling disabled during initial validation.
5. Add a shell validation command that:
   - requires calibration, arm, and a staged auto-tune result,
   - applies the staged model and controller defaults,
   - forces PI outer-loop mode and DOB disabled,
   - enters `velocity_encoder`,
   - runs a conservative positive/negative velocity sweep,
   - prints target, reference, measured velocity, error, and measured current.
6. Only after this validation passes should DOB, MPR, or decoupling be tested.

## Operator Workflow

```text
motor state calibrate
motor safety timeout 0
motor arm
motor commission auto run
motor commission auto validate
motor safety timeout 1000
```

Optional validation arguments:

```text
motor commission auto validate <max_hz> <hold_ms>
```

Example:

```text
motor commission auto validate 5 2000
```

## Acceptance

- No motor fault during validation.
- Encoder warning/error counts do not grow significantly.
- Measured velocity has the expected sign for each command.
- Current remains within the staged velocity current limit.
- Motion sounds smooth enough to proceed to longer velocity-loop testing.

