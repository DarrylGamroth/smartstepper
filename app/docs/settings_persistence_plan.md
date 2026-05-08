# Motor Settings Persistence Plan

Date: 2026-05-07

This document defines the persistence contract for commissioned motor data. Boot
autoload remains disabled until repeated HIL reset/reboot gates pass; operators
must explicitly run `motor settings load ...`.

## Current Policy

- Runtime commissioning remains explicit.
- Boot does not autoload persisted motor parameters.
- Devicetree motor parameters remain safe fallback defaults.
- Devicetree identity and safety limits should be conservative values that are
  safe enough for identification/commissioning to run on a fresh controller.
  Persisted settings may narrow runtime motion limits after commissioning, but
  they must not be required for a safe initial boot.
- ADC current offsets are **not persisted**. They are fast, board/runtime analog
  calibrations and must be measured on every boot.
- Persisted settings are opt-in, shell-visible, typed, versioned, and guarded
  against armed/online mutation.

## Storage Backend

Use Zephyr Settings over the ZMS backend on the internal flash
`settings_storage` partition. NVS is a fallback only if ZMS is unavailable.
Direct NVMEM/EEPROM cell access remains deferred for future fixed manufacturing
fields or service counters.

The generic Zephyr `settings` shell is a debug tool only. The product interface
is the typed `motor settings ...` command tree because it enforces grouped
preview/apply semantics and state guards.

## Key Layout

Settings are stored as individual typed binary values, not as one opaque blob.
The root namespace is `motor/`.

Metadata:

```text
motor/meta/schema_version
motor/meta/generation
motor/meta/valid_groups
```

Encoder mapping group:

```text
motor/encoder/direction_sign
motor/encoder/commutation_offset_mech_rad
motor/encoder/trim_elec_rad
motor/encoder/mapping_correlation
motor/encoder/mapping_residual_rad
```

Motor identity group:

```text
motor/identity/pole_pairs
```

Identity values describe the motor/hardware assembly. In the current firmware
`pole_pairs` is still consumed from devicetree in several ISR/control paths, so
loading this group validates that the persisted identity matches the active
firmware image rather than partially applying an inconsistent identity.

Electrical model group:

```text
motor/model/electrical/rs_ohm
motor/model/electrical/ld_h
motor/model/electrical/lq_h
motor/model/electrical/flux_linkage_wb
```

`Kt` is not persisted. It is derived on load/apply as
`Kt = 1.5 * pole_pairs * psi_f` and cached in RAM.

Controller group:

```text
motor/controllers/outer_loop_mode
motor/controllers/velocity_bandwidth_hz
motor/controllers/position_bandwidth_hz
motor/controllers/damping_ratio
motor/controllers/velocity_iq_limit_a
motor/controllers/velocity_dob_enabled
motor/controllers/velocity_dob_gain_scale
```

The controller group stores tuning intent, not raw derived coefficients. On
load the firmware recomputes velocity PI, position PI, velocity MPR, position
MPR, and DOB limits from the stored bandwidth/current-limit settings and the
active motor model. This avoids stale MPR/DOB values after model, decimation,
sample-time, or current-limit changes.

Detent metadata group:

```text
motor/detent/enabled
motor/detent/bins
motor/detent/phase_advance_bins
motor/detent/gain
motor/detent/iq_ff_limit_a
motor/detent/table_crc32
```

The detent table itself is not persisted in this version. Loading detent metadata
will only enable detent feedforward if the volatile table CRC matches the stored
CRC.

Runtime limits group:

```text
motor/limits/nominal_voltage_v
motor/limits/max_current_a
motor/limits/brake_current_a
motor/limits/max_velocity_hz
motor/limits/max_accel_hz_s
motor/limits/command_timeout_ms
```

`max_velocity_hz`, `max_accel_hz_s`, and `command_timeout_ms` are runtime-owned
and are applied on load. `nominal_voltage_v`, `max_current_a`, and
`brake_current_a` are persisted for visibility and future runtime-limit
refactoring; today they are validated against the devicetree-backed firmware
image because those values are still used directly by several hot paths.

## Shell Interface

```text
motor settings status
motor settings preview
motor settings save [model electrical|model encoder|identity|limits|controllers|detent|all]
motor settings load [model electrical|model encoder|identity|limits|controllers|detent|all]
motor settings clear [model electrical|model encoder|identity|limits|controllers|detent|all]
motor settings autoload status
```

`model electrical` stores only the active electrical model (`Rs/Ld/Lq/psi_f`).
`model encoder` stores the encoder mapping group. Mechanical ID values are not
part of baseline persistence yet. ADC current offsets are intentionally excluded
and must be measured on every boot.

## Guards

Save/load/clear require:

- motor initialized,
- disarmed,
- not in an ONLINE control state,
- not currently running calibration/commissioning.

Preview/status are read-only and allowed at any time.

## Apply Policy

Load validates selected values before applying. After apply:

- encoder mapping reload updates the angle observer offset and resets feedback
  trust state,
- identity reload verifies the persisted motor identity matches this firmware
  image,
- electrical model reload updates active Rs/Ld/Lq/flux, derives cached Kt, and
  updates electrical model source markers,
- limits reload applies runtime motion profile velocity/acceleration limits and
  command timeout, and validates compile-time electrical safety limits,
- controller reload recomputes PI/MPR/DOB coefficients from stored tuning
  intent, then resets PI integrators, velocity/position regulator state, MPR
  state, and DOB state,
- detent metadata reload resets detent runtime state and only enables the map if
  its stored CRC matches the current volatile table.

## Commissioning Save Policy

Commissioning never saves to non-volatile storage implicitly. The operator must
run an explicit `motor settings save ...` command after the relevant
commissioning stage has passed validation and has been applied to the active
runtime configuration.

Recommended save points:

- After production electrical ID passes and `motor commission electrical apply`
  has updated the active current model: `motor settings save model electrical`.
- After standard commissioning passes encoder mapping and applies it:
  `motor settings save model electrical` and `motor settings save model encoder`.
- After velocity/position controller tuning has been validated:
  `motor settings save controllers`.
- After runtime motion limits are intentionally changed and validated:
  `motor settings save limits`.
- After a detent table is captured and off/on validation shows improvement or no
  regression: `motor settings save detent`.

Avoid `motor settings save all` during bring-up unless every group is known good.
Do not save ADC current offsets; they are intentionally excluded and must be
measured every boot.

## HIL Gate Before Autoload

Autoload remains disabled until these pass repeatedly:

- status HIL pass after flash/reset,
- boot commissioning pass,
- explicit settings save/reset/preview/load flow,
- current_encoder validation pass,
- velocity_encoder validation pass with no motor faults,
- position_encoder validation pass once stable,
- no encoder hard errors above accepted threshold.
