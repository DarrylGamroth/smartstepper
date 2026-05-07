# Motor Shell Command Tree

The shell tree is organized by orthogonal concerns:

- `state`: lifecycle and operating-mode transitions.
- `current`: direct current references and inner current-loop tuning.
- `outer`: outer-loop regulator selection.
- `velocity`: velocity target, decimation, and velocity-regulator tuning.
- `position`: position target, decimation, and position-regulator tuning.
- `profile`: motion-profile and sequence generation.
- `encoder`: encoder diagnostics and telemetry.
- `commission`: commissioning workflows and staged parameter application.
- `safety`, `gate`, `fault`: protection, driver recovery, and diagnostics.
- `params`: raw parameter access for bring-up and debugging.

## Outer Loop

Select which outer-loop regulator is active:

```text
motor outer status
motor outer mode <pi|mpr>
```

This selection is intentionally separate from tuning. Tuning a PI or MPR command does not
implicitly switch the active regulator.

## Velocity

Velocity commands are grouped by function:

```text
motor velocity target <hz>
motor velocity decimation <ticks>
motor velocity status

motor velocity pi status
motor velocity pi set <kp> <ki> <iq_limit>
motor velocity pi defaults <safe|nominal>
motor velocity pi bandwidth <hz> [zeta] [iq_limit]

motor velocity mpr status
motor velocity mpr set <q_speed> <r_delta_iq> <horizon> <max_delta_iq> [disturbance_ki]
motor velocity mpr bandwidth <hz>

motor velocity dob status
motor velocity dob defaults <safe|nominal>
motor velocity dob enable <0|1>
motor velocity dob gain <nm_per_rad_s>
motor velocity dob torque_limit <nm>
motor velocity dob iq_limit <a>
```

## Position

Position commands mirror velocity where possible:

```text
motor position target <deg>
motor position decimation <ticks>
motor position status

motor position pi status
motor position pi set <kp> <ki>
motor position pi defaults <safe|nominal>
motor position pi bandwidth <hz> [zeta]

motor position mpr status
motor position mpr set <q_position> <q_velocity_ff> <r_delta_velocity> <horizon> [max_delta_velocity]
motor position mpr bandwidth <hz>
```

## Naming Rules

- `target` commands set motion references.
- `decimation` commands set loop update rates.
- `pi` commands tune only the PI regulator.
- `mpr` commands tune only the model-predictive regulator.
- `outer mode` selects the active outer-loop regulator.
- `status` commands report live state without mutating control state.

Avoid adding commands such as `velocity gains` because they hide which regulator is being
tuned. If a new regulator is added, give it a dedicated subtree.

## State And Recovery

State commands distinguish staging from execution:

```text
motor state mode <current_encoder|velocity_generated|position_generated|velocity_encoder|position_encoder>
motor state online
motor state status
motor state transition
motor state recovery
```

`motor state mode ...` stages the requested online submode. It does not mean the
motor is running in that mode until `motor state online` completes and
`motor state transition` reports `completed`.

Fault recovery is explicit:

```text
motor fault recovery
motor gate status
motor gate reset
motor encoder acquisition
motor encoder recover
motor state clear_error
```

`motor state clear_error` is only the state-machine clear request. Hardware
recovery operations are separate so gate-driver nSLEEP pulses and encoder
acquisition resets are visible and deterministic.

## Commissioning

Commissioning commands stage measurements first. `apply` commands explicitly
promote valid staged results into active runtime parameters; no commissioning
command persists values yet.

```text
motor commission boot <current_a> <mech_hz> <cycles>
motor commission run <slow|confirm> [apply]
motor commission status
motor commission apply

motor commission electrical plan
motor commission electrical measure rs [current_a] [samples] [settle_ms]
motor commission electrical measure inductance [pulse_v] [samples] [pulse_ms]
motor commission electrical sweep [samples]
motor commission electrical run [rs_current_a] [l_pulse_v] [samples]
motor commission electrical status
motor commission electrical apply
motor commission electrical validate [current_a] [hold_ms] [max_error_a]
motor commission electrical clear
```

`motor commission electrical` is the production-oriented electrical ID path. It
is intentionally separate from the older Rs/R-over-L bootstrap estimator so both
paths can be compared before replacing the fallback.
