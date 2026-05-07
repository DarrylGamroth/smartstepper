# Motor Settings Persistence Plan

Date: 2026-05-07

This document defines the persistence contract for commissioned motor data. Boot
autoload remains disabled until repeated HIL reset/reboot gates pass; operators
must explicitly run `motor settings load ...`.

## Current Policy

- Runtime commissioning remains explicit.
- Boot does not autoload persisted motor parameters.
- Devicetree motor parameters remain safe fallback defaults.
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

Encoder mapping group (`baseline` alias):

```text
motor/encoder/direction_sign
motor/encoder/commutation_offset_mech_rad
motor/encoder/trim_elec_rad
motor/encoder/mapping_correlation
motor/encoder/mapping_residual_rad
```

Motor model group:

```text
motor/model/rs_ohm
motor/model/ld_h
motor/model/lq_h
motor/model/flux_linkage_wb
motor/model/kt_nm_per_a
motor/model/inertia_kgm2
motor/model/viscous_friction_nm_per_rad_s
motor/model/coulomb_friction_nm
```

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

## Shell Interface

```text
motor settings status
motor settings preview
motor settings save [baseline|encoder|model|controllers|detent|all]
motor settings load [baseline|encoder|model|controllers|detent|all]
motor settings clear [baseline|encoder|model|controllers|detent|all]
motor settings autoload status
```

`baseline` is an alias for the encoder mapping group. It intentionally does not
include ADC current offsets.

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
- model reload updates active Rs/Ld/Lq/flux/Kt/J/B/Tc and model source markers,
- controller reload recomputes PI/MPR/DOB coefficients from stored tuning
  intent, then resets PI integrators, velocity/position regulator state, MPR
  state, and DOB state,
- detent metadata reload resets detent runtime state and only enables the map if
  its stored CRC matches the current volatile table.

## HIL Gate Before Autoload

Autoload remains disabled until these pass repeatedly:

- status HIL pass after flash/reset,
- boot commissioning pass,
- explicit settings save/reset/preview/load flow,
- current_encoder validation pass,
- velocity_encoder validation pass with no motor faults,
- position_encoder validation pass once stable,
- no encoder hard errors above accepted threshold.
