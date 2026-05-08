# Commissioning Parameter Inventory

Date: 2026-05-07

This inventory classifies the devicetree properties that currently affect motor
commissioning. The classification is the source of truth for the overlay split
and later binding cleanup.

## Node Semantics

| Node | Current meaning | Long-term meaning |
| --- | --- | --- |
| `/motor_parameters` | Motor model and hard motor limits | Motor identity, hard safety limits, and conservative fallback model only. |
| `/user_parameters` | Historical name for control/profile/procedure defaults | Keep temporarily; later rename to a clearer control/profile node if worth the churn. |
| `/fault_detection` | Safety thresholds | Hardware/profile safety thresholds derived from board/motor limits where possible. |

## Board Hardware Ownership

These belong in board overlays or board-specific DTS because they describe PCB
facts, not a motor profile:

| Category | Examples | Owner |
| --- | --- | --- |
| ADC topology | current/voltage sense channels, gains, buffer indices | board overlay/DTS |
| PWM/timer topology | MCPWM nodes, trigger timers, gate-driver pins | board overlay/DTS |
| Communication topology | UART, Ethernet, J-Link-independent debug wiring | board overlay/DTS |
| SPI pinmux/bus enable | controller pins and base bus enable | board overlay/DTS |

## Encoder Profile Ownership

These belong in encoder overlays such as
`app/configs/encoder_mt6835_rtspi.overlay` and
`app/configs/encoder_aeat9955_rtspi.overlay`:

| Property/category | Owner | Notes |
| --- | --- | --- |
| `encoder1` alias | encoder overlay | Selects the control encoder. |
| RT SPI compatible/frequency/frame length | encoder overlay | Transport/protocol choice for the selected encoder. |
| SPI mode/CS mode/FIFO | encoder overlay | Protocol/transport details. |
| encoder compatible/protocol props | encoder overlay | e.g. AEAT SPI4 mode, MT6835 CAL pin. |
| `pipeline-delay-samples` | encoder overlay | Encoder/protocol timing fact. |
| `encoder-direction-sign` | motor or encoder assembly profile | It is tied to encoder/motor mechanical assembly convention, not the bare board. |

## Motor Safe-ID Ownership

These belong in `motor_id_safe_*.overlay` and full motor profile overlays:

| Property | Classification | Notes |
| --- | --- | --- |
| `pole-pairs` | motor identity | Hard identity required before angle conversion. |
| `max-current-ma` | hard safety limit | Physical/profile maximum current. Runtime limits must clamp below this. |
| `max-speed-hz` | hard/profile safety limit | Conservative speed limit for the motor/bus/profile. |
| `resistance-mohms` | fallback model | Safe boot/default only; production bidirectional Rs supersedes it in RAM. |
| `inductance-d-uh` | fallback model | Safe boot/default only; demod `Ld` supersedes it in RAM. |
| `inductance-q-uh` | fallback model | Safe boot/default only; demod `Lq` supersedes it in RAM. |
| `flux_linkage_uvphz` | optional fallback model | Omit when no reliable datasheet/profile value exists; flux ID supersedes it in RAM when accepted. |
| `inertia-mgcm2` | fallback/advisory model | Use only after confidence gates; not required for baseline. |
| `align-current-ma` | motor/current-class bootstrap current | May stay motor-specific until generated-sweep mapping fully replaces boot align. |
| `align-duration-ms` | motor/current-class bootstrap timing | Motor/encoder mapping setup behavior. |
| `brake-current-ma` | safety/current-class limit | Motor/driver safety related. |
| `commission-electrical-rs-current-ma` | motor/current-class commissioning safety | Candidate for derivation from `max-current-ma`, but keep explicit for now. |
| `commission-electrical-current-limit-ma` | motor/current-class commissioning safety | Candidate for derivation from `max-current-ma`, but keep explicit for now. |
| `commission-electrical-max-pulse-mv` | motor/driver commissioning safety override | Keep explicit where voltage pulse amplitude must be constrained per profile. |
| `fault_detection.overcurrent-threshold-ma` | safety threshold | Candidate for derivation from `max-current-ma` after code supports it. |

## Shared Commissioning Recipe Ownership

These are not motor identity. They should move to a shared commissioning overlay
such as `app/configs/commissioning_default.overlay`, with motor overlays allowed
to override only when HIL requires it:

| Property | Classification | Notes |
| --- | --- | --- |
| `commission-electrical-samples` | generic recipe | Accepted sample count. |
| `commission-electrical-min-samples` | generic recipe | Binding default currently enough unless overridden. |
| `commission-electrical-max-samples` | generic recipe | Binding default currently enough unless overridden. |
| `commission-electrical-settle-ms` | generic recipe | Rs/demod settling default. |
| `commission-electrical-current-ramp-ms` | generic recipe | Current slew timing default. |
| `commission-electrical-demod-pulse-ms` | generic recipe | Demod pulse timing. |
| `commission-electrical-demod-half-cycles` | generic recipe | Demod excitation cadence. |
| `commission-electrical-min-pulse-mv` | generic recipe/safety floor | Generic lower bound. |
| `commission-electrical-min-pulse-ms` | generic recipe | Remove if scalar pulse path is retired and unused. |
| `commission-electrical-demod-scale-mpu` | generic recipe/calibration factor | Keep with demod algorithm defaults. |
| `commission-electrical-demod-max-spread-mpu` | generic recipe/quality gate | Keep with demod algorithm defaults. |
| `commission-electrical-sweep-*` | diagnostic recipe | Move to experimental or remove if sweep is not baseline. |
| `commission-auto-*` | generic one-command recipe | Move to shared commissioning overlay unless a motor profile needs an override. |
| `commission-validate-current-default-iq-ma` | profile smoke-test current | Usually motor/current-class-specific; can remain in motor overlay if friction differs. |
| `commission-standard-min-auto-iq-ma` | bootstrap authority floor | Candidate for derivation from `max-current-ma`; keep explicit until tuned. |

## RoverL and Legacy RS_EST

| Property | Classification | Disposition |
| --- | --- | --- |
| `roverl-est-current-ma` | RoverL bootstrap current | Keep. RoverL is the first electrical estimator. |
| `roverl-est-freq-hz` | RoverL bootstrap recipe | Keep. Can move to shared overlay if same across motors. |
| `roverl-est-settling-ms` | RoverL bootstrap recipe | Keep. Can move to shared overlay if same across motors. |
| `roverl-est-duration-ms` | RoverL bootstrap recipe | Keep. Can move to shared overlay if same across motors. |
| `rs-est-current-ma` | legacy DC `RS_EST` | Deprecate; remove after production bidirectional Rs replaces the normal path. |
| `rs-est-rampup-ms` | legacy DC `RS_EST` | Deprecate with the old state. |
| `rs-est-duration-ms` | legacy DC `RS_EST` | Deprecate with the old state. |

## Experimental / Diagnostic Defaults

These should not clutter normal safe-ID overlays unless the feature is enabled
and part of the target workflow:

| Property/category | Disposition |
| --- | --- |
| `rls-*` | Move to an experimental RLS overlay or rely on binding defaults. Runtime Rs tracking can be revisited later. |
| `thermal-*` | Move to an experimental thermal overlay or rely on binding defaults until thermal protection is active. |
| scalar/pulse inductance properties | Remove from normal overlays once scalar pulse path is removed from the user-facing baseline. |
| saliency sweep defaults | Keep as diagnostic until saliency becomes a production method. |

## Calculable Candidates

Do not change behavior until HIL confirms the formulas, but these are good
future reductions in overlay clutter:

| Value | Candidate derivation |
| --- | --- |
| `commission-electrical-rs-current-ma` | 5-15% of `max-current-ma`, bounded by a minimum SNR current. |
| `commission-electrical-current-limit-ma` | 10-20% of `max-current-ma`, capped by user/safety limit. |
| `overcurrent-threshold-ma` | `max-current-ma` times a profile margin. |
| `commission-standard-min-auto-iq-ma` | fraction of `max-current-ma`, optionally raised by motion-threshold test. |
| velocity PI/MPR bandwidth defaults | derived from accepted `J`, `Kt`, current limit, and desired damping. |
| BEMF velocity limit | derived from accepted flux, Rs, current limit, modulation limit, and bus voltage. |

## Normal Electrical Commissioning Sequence

The baseline sequence is intentionally two-stage:

1. Use conservative devicetree fallback model.
2. Run current offsets.
3. Run RoverL to obtain provisional `Rs` and average `L`.
4. Update current PI with RoverL provisional values.
5. Run production bidirectional Rs.
6. Run production demodulated `Ld/Lq`.
7. Update current PI with production `Rs/Ld/Lq`.
8. Continue to generated-sweep encoder mapping and flux ID.
