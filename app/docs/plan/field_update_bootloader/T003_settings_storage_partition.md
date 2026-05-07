# T003 - Settings Storage Partition

## Decision

Store critical motor commissioned settings in internal MCU flash using Zephyr
Settings over ZMS, not in QSPI as the primary backend.

## Rationale

- Settings are small and safety-critical.
- Settings should be available before filesystem/QSPI dependencies matter.
- A QSPI fault should not erase or hide the motor's commissioned identity.
- Firmware update staging can erase/rewrite QSPI without risking commissioned
  motor settings.

## EEPROM Role

The existing 2 KiB EEPROM is deferred. Do not use it in the first persistence or
bootloader implementation. Revisit it only when there is a concrete need, such
as:

- board serial/manufacturing facts,
- last-known-good compact baseline copy,
- service counter or small recovery marker.

## Work

- Add or resize an internal-flash partition for settings.
- Point `zephyr,settings-partition` at the chosen internal-flash partition.
- Remove or ignore the current EEPROM `settings_storage` role when the internal
  flash settings partition is introduced.
- Configure:
  - `CONFIG_SETTINGS=y`
  - `CONFIG_ZMS=y`
  - `CONFIG_SETTINGS_ZMS=y`
  - `CONFIG_SETTINGS_SHELL=y` only for debug builds if useful.
- Keep generic Settings shell debug-only; product access is through
  `motor settings ...` commands.
- Ensure settings writes are disabled while armed/online.

## Validation

- Save/load smoke test survives reset.
- Firmware update does not erase settings.
- Corrupt/missing settings falls back to DT defaults.
