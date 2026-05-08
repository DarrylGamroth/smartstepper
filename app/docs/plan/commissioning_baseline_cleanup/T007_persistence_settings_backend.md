# T007 - Guarded Persistence With Settings Backend

## Decision

Use Zephyr Settings as the first persistence API, backed by **ZMS** on an
internal-MCU-flash storage partition where practical. Store individual typed Settings keys under the `motor/` namespace with group metadata. Do not store one opaque blob for this phase.

NVS is an acceptable fallback only if ZMS is unavailable on the target. Direct
NVMEM/EEPROM cell access is deferred until there is a concrete use case. It may
be useful later for fixed manufacturing data, a tiny last-known-good recovery
record, or service counters, but it is not part of the first
commissioned-settings backend.

External QSPI NOR is not the preferred location for critical commissioned motor
settings. Use QSPI for firmware-update staging, filesystem data, logs, high-rate
trace dumps, and large future maps. Critical motor settings should survive QSPI
filesystem failures and should not depend on external-flash initialization.

## Rationale

- Settings gives standard key/value load/save and shell/debug integration.
- ZMS is the preferred modern Zephyr settings backend for this project.
- NVS is a fallback backend only.
- Individual typed keys make Settings shell inspection useful while group metadata preserves schema, generation, and apply semantics.
- Direct NVMEM cells are better for fixed manufacturing data or compact custom
  two-slot records, but they are not as convenient as a settings API for staged
  commissioned values.

## Work

- Define typed settings namespaces under `motor/meta`, `motor/encoder`, `motor/identity`,
  `motor/model`, `motor/limits`, `motor/controllers`, and `motor/detent`.
- Add build configuration for:
  - `CONFIG_SETTINGS=y`
  - `CONFIG_ZMS=y`
  - `CONFIG_SETTINGS_ZMS=y`
  - suitable ZMS sector count/partition settings.
- Add a devicetree storage partition suitable for ZMS if one does not already
  exist. Prefer internal MCU flash; coordinate with the field-update bootloader
  partition plan before final sizing. The current `smartstepper_v2` flash
  partition map is provisional and under-allocates the STM32H743 internal flash;
  fix that layout before finalizing the settings partition.
- Store individual typed keys under `motor/encoder`, `motor/identity`, `motor/model`,
  `motor/limits`, `motor/controllers`, and `motor/detent`, plus
  `motor/meta/schema_version`, `motor/meta/generation`, and `motor/meta/valid_groups`.
- Treat devicetree identity and safety limits as conservative fallback values
  that are safe enough for identification/commissioning to run before any
  persisted settings are applied.
- Store motor identity separately from measured model data:
  - `motor/identity/pole_pairs`
- Store runtime/safety limits separately from controller tuning intent:
  - `motor/limits/nominal_voltage_v`
  - `motor/limits/max_current_a`
  - `motor/limits/brake_current_a`
  - `motor/limits/max_velocity_hz`
  - `motor/limits/max_accel_hz_s`
  - `motor/limits/command_timeout_ms`
- Apply runtime-owned limits (`max_velocity_hz`, `max_accel_hz_s`, and
  `command_timeout_ms`) on load. Validate but do not partially apply
  compile-time-coupled identity/electrical safety values until those hot paths
  are fully runtime-configured.
- Store controller tuning intent, not raw generated coefficients:
  - `motor/controllers/outer_loop_mode`
  - `motor/controllers/velocity_bandwidth_hz`
  - `motor/controllers/position_bandwidth_hz`
  - `motor/controllers/damping_ratio`
  - `motor/controllers/velocity_iq_limit_a`
  - `motor/controllers/velocity_dob_enabled`
  - `motor/controllers/velocity_dob_gain_scale`
- Recompute velocity PI, position PI, velocity MPR, position MPR, and DOB
  runtime coefficients on load from the stored tuning intent and active motor
  model.
- Do not persist ADC current offsets; run the quick offset calibration every boot.
- Add shell commands:
  - `motor settings status`
  - `motor settings preview`
  - `motor settings save [model electrical|model encoder|identity|limits|controllers|detent|all]`
  - `motor settings load [model electrical|model encoder|identity|limits|controllers|detent|all]`
  - `motor settings clear [model electrical|model encoder|identity|limits|controllers|detent|all]`
  - `motor settings autoload status`
- Keep autoload disabled in this plan.
- Refuse save/load while armed or online unless command is read-only preview.
- On load, validate schema/group presence/value ranges and print all values before apply.
- Apply only explicitly selected groups, recompute derived controller
  coefficients, and reset affected fast-loop state.
- Enable `CONFIG_SETTINGS_SHELL` only as a debug/bring-up aid if useful. The
  generic `settings` shell can list/read/write/delete raw keys, but it must not
  be treated as the product motor-parameter interface because it bypasses typed
  validation, safe armed/online guards, grouped preview/apply semantics, and
  controller-state reset.

## Constraints

- No automatic boot apply until HIL reset/reboot gates pass.
- Never persist advisory/invalid mechanical ID or unvalidated advanced features.
- Do not persist detent table in V1; store only metadata/table CRC.
- Must work safely when settings are absent or corrupted: fall back to DT.

## Validation

- Unit tests for persistence schema helpers remain in place; app-level Settings path is validated by firmware build and HIL shell flow.
- Native settings backend smoke test if practical.
- HIL manual flow:
  1. baseline commission,
  2. save baseline (encoder mapping only; current offsets remain volatile),
  3. reset target,
  4. preview saved values,
  5. explicitly load baseline,
  6. verify current/encoder status,
  7. clear settings and verify DT fallback.
