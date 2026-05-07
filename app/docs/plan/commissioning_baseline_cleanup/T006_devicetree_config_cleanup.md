# T006 - Devicetree and Runtime Configuration Cleanup

## Problem

Motor profile overlays now mix four different concepts:

- board/peripheral hardware topology,
- immutable motor identity and conservative boot fallback model values,
- product/profile procedure defaults used by commissioning,
- per-unit commissioned/tuned values that should eventually be persisted.

That makes overlays hard to audit and makes it unclear whether a value is a
physical motor limit, a safe fallback, a commissioning recipe knob, or a stale
measured result.

## Naming Clarification

The current node names are historically useful but not ideal:

- `/motor_parameters` currently means **motor model and hard motor limits**.
- `/user_parameters` currently means **control/profile/procedure defaults**,
  not end-user mutable settings.

Do not add more measured or tuned values to `/user_parameters` just because they
are adjustable. Runtime-adjusted values belong in RAM first and later in the
Settings/ZMS persistent record.

## Ownership Policy

### Keep In Board Overlay

`app/boards/smartstepper_v2.overlay` should contain only board wiring and board
electrical measurement topology:

| Category | Examples | Reason |
| --- | --- | --- |
| ADC/current/voltage sense hardware | shunt values, gains, divider values, ADC channels, buffer indices | Board PCB facts. |
| PWM/timer/peripheral wiring | timer topology, MCPWM nodes, GPIOs, UART shell, trigger inputs | Board PCB/peripheral facts. |
| Encoder bus placement | which SPI controller/pins are enabled | Board wiring fact. |
| Generic observer node only if board-wide | `angle_observer` node existence | Application hardware/control topology. |

Board overlays should not contain selected motor electrical values unless the
board is permanently assembled with that motor and no motor profile overlay is
used.

### Keep In Motor Profile Overlay

`app/configs/motor_*.overlay` should contain the selected motor/encoder profile:

| Node | Keep | Examples |
| --- | --- | --- |
| `/aliases` | selected control encoder alias | `encoder1 = &mt6835`, `encoder1 = &aeat9955` |
| encoder child node | encoder identity/protocol details | `compatible`, `spi4-mode`, `pipeline-delay-samples`, encoder GPIOs |
| RT SPI parent override | selected encoder bus runtime mode | `compatible = "rubus,stm32-rt-spi"`, `spi-clock-frequency`, `max-frame-len`, `cs-mode`, `spi-cpol/cpha`, `fifo-enable` |
| `/motor_parameters` | immutable identity and conservative boot fallback model | pole pairs, rated max current, safe max speed, nominal/fallback Rs/Ld/Lq/flux/J |
| `/user_parameters` | profile-level procedure/control defaults | PWM/control frequencies, current loop bandwidth default, current ramp, commissioning recipe defaults, validation currents, velocity profile limits |
| `/fault_detection` | hardware/profile safety thresholds | overcurrent threshold, encoder fault threshold |

### Do Not Keep In Devicetree Long-Term

These values are per-unit results or runtime tuning and should move to RAM +
future Settings/ZMS persistence:

| Value | Future Owner |
| --- | --- |
| current ADC offsets | commissioned baseline record |
| generated-sweep encoder electrical offset / commutation map | commissioned baseline record |
| measured Rs/Ld/Lq | commissioned baseline record, with DT as fallback |
| measured flux linkage / Kt | commissioned baseline record, with DT as fallback |
| measured inertia/friction | advisory/mechanical record only after confidence gates |
| velocity/position PI gains | controller record or bandwidth-derived runtime config |
| MPR/DOB gains and enable states | controller record only after HIL validation |
| detent table | later separate map record, not V1 baseline record |

## Current Placement Review

### Values Correctly In `/motor_parameters`

These belong in `/motor_parameters`, with comments saying they are fallback or
hard-limit values:

- `pole-pairs`
- `max-current-ma`
- `max-speed-hz`
- `resistance-mohms`
- `inductance-d-uh`
- `inductance-q-uh`
- `flux_linkage_uvphz`
- `inertia-mgcm2`

Required cleanup:

- Remove comments such as “Updated from on-hardware commissioning” from fallback
  DT values. DT should not imply the value is the currently commissioned value.
- For MT6835, decide whether `inductance-d-uh` and `inductance-q-uh` should be
  updated to the latest trusted demodulated fallback values or left as nominal
  fallback values. Either is acceptable, but the comment must say
  `fallback/nominal`, not `commissioned active`.
- Keep `max-current-ma` as the physical/profile current limit. Runtime shell
  currents must clamp below it.

### Values Correctly In `/user_parameters`

These are profile/control/procedure defaults and can stay in
`/user_parameters`:

- `nominal_voltage-mv`
- `pwm-frequency-hz`
- `control-loop-frequency-hz`
- `current-loop-bandwidth-hz`
- `current-command-ramp-ms`
- `offset-pole-hz`
- `max-modulation-index-mpu`
- `encoder-direction-sign`
- `velocity-max-hz`
- `velocity-max-accel-hz-per-s`
- `velocity-initial-hz`
- `commission-electrical-rs-current-ma`
- `commission-electrical-current-limit-ma`
- `commission-electrical-samples`
- `commission-electrical-settle-ms`
- `commission-electrical-current-ramp-ms`
- `commission-electrical-demod-pulse-mv`
- `commission-electrical-demod-pulse-ms`
- `commission-electrical-demod-half-cycles`
- `commission-standard-min-auto-iq-ma`
- `commission-auto-velocity-bandwidth-mhz`
- `commission-validate-current-default-iq-ma`
- `commission-auto-*` procedure knobs

Required cleanup:

- Group these in the overlay with comments:
  - loop/control defaults,
  - safety/command limits,
  - legacy/bootstrap measurement defaults,
  - production electrical ID defaults,
  - one-command commissioning defaults.
- Rename later, if we are willing to change bindings, from
  `rubus,user-parameters` to something clearer such as
  `rubus,motor-control-profile`. That is not required for this cleanup task.

### Values To Remove Or Deprecate From `/user_parameters`

These are tied to paths we are removing or demoting:

| Property | Action |
| --- | --- |
| `rs-est-current-ma` | Deprecate after legacy `RS_EST` removal. Keep temporarily only if a fallback state-machine path still compiles against it. |
| `rs-est-rampup-ms` | Same as above. |
| `rs-est-duration-ms` | Same as above. |
| `commission-electrical-l-pulse-mv` | Remove from profile overlays once scalar/pulse inductance shell path is removed and demod no longer falls back to it. |
| `commission-electrical-l-pulse-ms` | Remove with scalar/pulse inductance path. |
| `commission-electrical-min-pulse-ms` | Remove if only used by scalar pulse validation; otherwise rename to demod-specific only if still needed. |
| `commission-electrical-sweep-*` | Remove with scalar/pulse sweep shell path. |
| `rls-*` and `thermal-*` | Keep only if the corresponding feature is enabled/used. Otherwise move to an experimental overlay or leave binding defaults only. |
| `roverl-est-*` | Keep as bootstrap/fallback until the state-machine R/L path is explicitly removed or retained as a fallback. Mark clearly as fallback. |

## Settings Shell Answer

Zephyr provides a generic Settings shell when `CONFIG_SETTINGS_SHELL=y` is
enabled. In the local Zephyr tree it registers:

- `settings list [subtree]`
- `settings read [string|hex] <name>`
- `settings write [string|hex] <name> <value>`
- `settings delete <name>`

This shell can be used to inspect and manually write raw settings keys. It is
not sufficient as the primary motor parameter interface because:

- values are raw bytes or strings, not typed motor quantities with units,
- it does not know which settings are safe to apply while armed or online,
- it does not validate schema/version/CRC/flags for a packed motor record,
- it cannot reset affected fast-loop state after changing gains/model values,
- it cannot preview/apply selected groups in a controlled way.

Use the generic Settings shell only for debug/bring-up. The product interface
should remain dedicated motor commands, as planned in T007:

- `motor settings status`
- `motor settings preview`
- `motor settings save [baseline|model|controllers|all]`
- `motor settings load [baseline|model|controllers|all]`
- `motor settings clear`
- `motor settings autoload status`

## Work

1. Audit `app/boards/smartstepper_v2.overlay`.
   - Confirm it only contains board topology and measurement hardware.
   - Move any selected motor values from board overlay into motor profile
     overlays.

2. Audit `app/configs/motor_mt6835_2a.overlay` and
   `app/configs/motor_aeat9955_067a.overlay`.
   - Add section comments for encoder selection, control defaults, safety
     limits, fallback motor model, and commissioning defaults.
   - Remove stale wording that implies DT values are active commissioned values.
   - Make fallback model comments explicit.

3. Audit `dts/bindings/rubus,user-parameters.yaml`.
   - Mark legacy RS_EST properties as deprecated/fallback once T002 removes the
     normal path.
   - Remove or deprecate scalar/pulse inductance properties after T004 removes
     the shell path.
   - Keep demod D/Q electrical ID defaults.

4. Audit `dts/bindings/rubus,motor-parameters.yaml`.
   - Document every model property as nominal/fallback unless it is a hard
     identity or safety limit.
   - Clarify that commissioned values override these in RAM and later
     Settings/ZMS.

5. Define a future parameter-source API boundary.
   - Boot source: DT fallback values.
   - Runtime source: active RAM values.
   - Persistent source: Settings/ZMS packed record after T007.
   - Shell/API must report both fallback and active values where useful.

6. Update documentation.
   - Add a short table to the commissioning inventory showing DT vs RAM vs
     Settings ownership.
   - Record any properties scheduled for removal in the execution log.

## Constraints

- Do not move hardware topology or sensor scaling into Settings.
- Do not persist or auto-load values yet in this task.
- Do not remove DT fallback values; firmware must still boot safely with erased
  settings.
- Do not let stale DT “commissioned” comments imply that HIL commissioning has
  already run on the unit.
- Do not use the generic `settings write` shell as the normal motor tuning
  interface.

## Validation

- Resolved `zephyr.dts` contains expected board hardware nodes, selected encoder
  alias, safe motor fallback values, and no duplicate selected motor profiles.
- MT6835 build passes.
- AEAT-9955 build passes.
- `motor commission electrical plan` and docs describe demod D/Q as production
  electrical ID and scalar pulse as removed/deprecated.
- `motor settings ...` remains the planned typed/safe interface; generic
  `settings ...` shell is documented as debug-only.
- Baseline commissioning does not depend on stale tuned DT values.
