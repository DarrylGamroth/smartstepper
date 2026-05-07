# Field Update Bootloader Plan Package

## Goal

Add a production-safe bootloader and Ethernet field-update path while keeping
commissioned motor settings independent from firmware image updates.

## Decisions

- Use MCUboot as the bootloader.
- Use signed images before any field-update workflow is considered production.
- Development flashing remains J-Link.
- Field update path is MCUboot + MCUmgr SMP over UDP/IPv4 Ethernet on port
  `1337`. TCP remains enabled for telnet shell/debugging, not for MCUmgr.
- Treat the current `smartstepper_v2.dts` fixed-partition map as provisional and
  probably wrong. The STM32H743 has more internal flash than the current map
  allocates, so the first task is to size the full internal flash and redesign
  partitions around real image sizes.
- Use internal STM32 flash for critical commissioned/settings data unless a later
  flash-layout audit proves this is not practical.
- Use external QSPI NOR for large update staging, filesystem data, logs, and HIL
  artifacts.
- Keep settings and firmware-update state in separate partitions/namespaces.
- Do not enable automatic application of persisted commissioned values until the
  existing reset/reboot HIL gates pass repeatedly.

## Sysbuild Model

Zephyr sysbuild is the multi-image build flow used for this bootloader design.
For this project, `west build --sysbuild` builds two firmware images in one
build directory:

- MCUboot, a small standalone bootloader image that starts first after reset.
- The application image, linked for slot0 and signed so MCUboot can validate it.

MCUboot is not compiled into the application. It lives in the `mcuboot`
partition, validates the signed application in `image-0`, handles pending
updates in `image-1`, then jumps to the selected app.

Development can still use J-Link. For normal motor-control work, use the
app-only build because it is faster. For bootloader, partition, signing,
rollback, settings-retention, or field-update validation, use the sysbuild
directory and flash it with the J-Link runner.

Expected sysbuild artifacts:

- `mcuboot/zephyr/zephyr.bin` - bootloader image.
- `app/zephyr/zephyr.signed.bin` - update payload for MCUmgr upload.
- `app/zephyr/zephyr.signed.hex` - signed app in Intel HEX form.

Runtime update interfaces:

- Built-in Zephyr `mcuboot` shell command from `CONFIG_MCUBOOT_SHELL`.
- MCUmgr image and OS groups over UDP/IPv4 from `CONFIG_MCUMGR_TRANSPORT_UDP`.
- App-side boot log/optional delayed confirmation helper in `app_update`.
- Auto-confirm is disabled by default; test images roll back if not confirmed.

## Why Settings Should Prefer Internal Flash

Critical motor settings are small and required early. They should not depend on
external QSPI initialization, pinmux, QSPI driver behavior, or filesystem mount
success. Internal flash also keeps commissioning values coupled to the MCU that
is actually controlling the motor, which simplifies failure analysis and board
replacement behavior.

QSPI is still valuable, but it should be treated as capacity storage: firmware
update staging, filesystem, logs, trace captures, large detent maps, and future
bulk data.

The current board also has a 2 KB I2C EEPROM. Leave it unused for now. It can be
introduced later when there is a concrete need, such as manufacturing identity,
a compact last-known-good marker, or service counters.

## Task Order

Execute in this order before returning to motor-control feature work:

1. Bootloader/partition/MCUboot.
2. Settings/ZMS persistence.
3. Motor commissioning/control cleanup.

Detailed field-update task order:

1. `T001_flash_layout_audit.md` - document full STM32H743 internal flash
   capacity, current under-allocation, QSPI, EEPROM, and image size constraints.
2. `T002_mcuboot_partition_design.md` - choose a boot/update partition layout.
3. `T003_settings_storage_partition.md` - add a dedicated internal-flash
   Settings/ZMS partition for motor settings.
4. `T004_sysbuild_mcuboot.md` - enable MCUboot through sysbuild and signed app
   images.
5. `T005_ethernet_update_transport.md` - select and enable Ethernet DFU
   transport.
6. `T006_image_confirmation_rollback.md` - add app-side health confirmation and
   rollback policy.
7. `T007_security_key_version_policy.md` - define signing keys and downgrade
   policy.
8. `T008_hil_update_validation.md` - validate update, rollback, settings
   retention, and recovery.

## Scope Boundaries

- This package does not implement motor settings persistence itself; it defines
  the storage layout needed by the persistence plan.
- This package does not define a cloud service. Initial Ethernet update can be
  local network tooling.
- This package must not change real-time motor-control behavior.
