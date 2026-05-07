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

- Define typed settings namespaces under `motor/meta`, `motor/encoder`, `motor/model`, `motor/controllers`, and `motor/detent`.
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
- Store individual typed keys under `motor/encoder`, `motor/model`, `motor/controllers`, and `motor/detent`, plus `motor/meta/schema_version`, `motor/meta/generation`, and `motor/meta/valid_groups`.
- Do not persist ADC current offsets; run the quick offset calibration every boot.
- Add shell commands:
  - `motor settings status`
  - `motor settings preview`
  - `motor settings save [baseline|encoder|model|controllers|detent|all]`
  - `motor settings load [baseline|encoder|model|controllers|detent|all]`
  - `motor settings clear [baseline|encoder|model|controllers|detent|all]`
  - `motor settings autoload status`
- Keep autoload disabled in this plan.
- Refuse save/load while armed or online unless command is read-only preview.
- On load, validate schema/group presence/value ranges and print all values before apply.
- Apply only explicitly selected groups and reset affected fast-loop state.
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
