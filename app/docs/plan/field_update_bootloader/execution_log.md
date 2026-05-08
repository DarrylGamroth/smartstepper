# Field Update Bootloader Execution Log

- 2026-05-07: Plan package created. Initial storage decision: critical motor
  settings should prefer internal MCU flash with Settings/ZMS; QSPI should be
  used for update staging/filesystem/logs unless the MCUboot partition audit
  forces a different layout.
- 2026-05-07: Updated plan to treat the current `smartstepper_v2` internal
  flash partition map as provisional/incorrect because it does not allocate the
  full STM32H743 flash. EEPROM use is deferred until a specific need exists.
- 2026-05-07: Recorded execution order: bootloader/partition/MCUboot first,
  Settings/ZMS persistence second, motor commissioning/control cleanup third.
  Field updates will use MCUboot + MCUmgr SMP over UDP/IPv4 Ethernet on port
  `1337`; development flashing remains J-Link.
- 2026-05-07: T001 initial evidence recorded. Board metadata exposes 2048 KiB
  flash, current DTS partitions cover only 640 KiB, and current app ROM load is
  about 490 KiB, so the existing 256 KiB image slots are invalid for MCUboot.
- 2026-05-07: T002/T003 base layout implemented in
  `boards/rubus/smartstepper_v2/smartstepper_v2.dts`: 128 KiB MCUboot,
  832 KiB slot0, 832 KiB slot1, and 256 KiB internal-flash
  `settings_storage`. EEPROM child fixed partitions were removed from the base
  DTS because EEPROM is deferred and MCUboot's flash map cannot reference an
  EEPROM device when the bootloader image does not build the I2C/EEPROM stack.
  The settings partition is two STM32H743 erase sectors because ZMS cannot
  mount with a single-sector partition.
- 2026-05-07: T004 sysbuild files added. `app/sysbuild.conf` enables MCUboot
  with ECDSA-P256 signing and swap-using-offset. `app/sysbuild/mcuboot.conf`
  disables `CONFIG_CODE_DATA_RELOCATION` for the bootloader image because the
  board enables relocation for the application ISR stack but MCUboot has no app
  relocation rules.
- 2026-05-07: MT6835 MCUboot sysbuild passed with:
  `podman exec wonderful_goldberg bash -lc 'cd /workspace && west build
  --sysbuild -p always -b smartstepper_v2/stm32h743xx /workspace/chopper/app
  -d /workspace/build/chopper/smartstepper_v2_mcuboot -S serial-shell
  -S serial-console -- -DDTC_OVERLAY_FILE="boards/smartstepper_v2.overlay;configs/motor_mt6835_2a.overlay"'`.
  Evidence at the time: MCUboot image uses 35,592 B of 128 KiB FLASH; signed application
  uses 503,356 B of the then-896 KiB slot region. Artifacts include
  `app/zephyr/zephyr.signed.bin`, `app/zephyr/zephyr.signed.hex`, and
  `mcuboot/zephyr/zephyr.bin`.
- 2026-05-07: T003 storage backend Kconfig enabled in `app/prj.conf`:
  `CONFIG_SETTINGS=y`, `CONFIG_FLASH_MAP=y`, `CONFIG_ZMS=y`,
  `CONFIG_SETTINGS_ZMS=y`, and `CONFIG_SETTINGS_SHELL=y`. Initial attempt
  without `CONFIG_FLASH_MAP=y` fell back to `CONFIG_SETTINGS_NONE`; adding
  `FLASH_MAP` selected the ZMS backend correctly. Evidence from
  `/workspace/build/chopper/smartstepper_v2_mcuboot/app/zephyr/.config`:
  `CONFIG_BOOTLOADER_MCUBOOT=y`, `CONFIG_FLASH_MAP=y`,
  `CONFIG_SETTINGS_ZMS=y`, and `# CONFIG_SETTINGS_NONE is not set`.
- 2026-05-07: MT6835 MCUboot sysbuild revalidated after enabling Settings/ZMS.
  Evidence at the time: app image uses 504,948 B of the then-896 KiB slot
  region and emits signed artifacts successfully.
- 2026-05-07: T005/T006 runtime update support implemented. `west.yml` now
  includes the Zephyr `zcbor` module required by MCUmgr CBOR support. The
  application enables `CONFIG_MCUMGR`, image and OS management groups,
  MCUmgr UDP/IPv4 transport on port `1337`, image manager, stream flash, and
  Zephyr's built-in `CONFIG_MCUBOOT_SHELL`. A custom project `motor update`
  command was intentionally removed because the built-in `mcuboot` shell command
  is the correct local operator interface.
- 2026-05-07: App-side update helper added. `app_update_init()` logs active
  slot, confirmation state, and pending swap type at boot. Optional delayed
  auto-confirm is available behind `CONFIG_APP_UPDATE_AUTO_CONFIRM_AFTER_BOOT`
  but remains disabled by default so test images roll back unless explicitly
  confirmed.
- 2026-05-07: MT6835 sysbuild revalidated with MCUmgr enabled:
  `CONFIG_BOOTLOADER_MCUBOOT=y`, `CONFIG_MCUBOOT_SHELL=y`,
  `CONFIG_MCUMGR=y`, `CONFIG_MCUMGR_GRP_IMG=y`,
  `CONFIG_MCUMGR_GRP_OS=y`, `CONFIG_MCUMGR_TRANSPORT_UDP=y`,
  `CONFIG_SETTINGS_ZMS=y`, `CONFIG_ZCBOR=y`, and
  `# CONFIG_APP_UPDATE_AUTO_CONFIRM_AFTER_BOOT is not set`. Artifacts:
  `app/zephyr/zephyr.signed.bin` is 520 KiB, MCUboot `zephyr.bin` is 36 KiB.
  `imgtool dumpinfo` confirms the signed image contains SHA256, KEYHASH, and
  ECDSA signature TLVs.
- 2026-05-07: Remaining external validation item: run the HIL
  upload/test/reset/confirm/rollback flow over Ethernet with the host-installed
  `/home/dgamroth/.cargo/bin/mcumgrctl`. Device-side firmware support and signed
  image generation are in place.
- 2026-05-07: AEAT-9955 sysbuild profile also passed:
  `podman exec wonderful_goldberg bash -lc 'cd /workspace && west build
  --sysbuild -p always -b smartstepper_v2/stm32h743xx /workspace/chopper/app
  -d /workspace/build/chopper/smartstepper_v2_mcuboot_aeat -S serial-shell
  -S serial-console -- -DDTC_OVERLAY_FILE="boards/smartstepper_v2.overlay;configs/motor_aeat9955_067a.overlay"'`.
  Evidence: app image uses 543,100 B of the 832 KiB slot region; MCUboot uses
  35,592 B of the 128 KiB boot partition.
- 2026-05-07: Removed the app-level
  `CONFIG_MCUMGR_GRP_IMG_TOO_LARGE_SYSBUILD=y` choice selection after it caused
  a warning in non-sysbuild app builds. Final app-only build and MT6835 sysbuild
  both pass without that warning. The slot still has sufficient headroom; a
  later MCUmgr HIL upload test should verify host/device rejection behavior for
  deliberately oversized images if that guard is required.
- 2026-05-07: HIL MCUmgr update flow verified on MT6835 target at `10.0.0.44`.
  Flashed the MCUboot sysbuild through J-Link at `10.0.0.70`, verified
  `mcumgrctl --udp 10.0.0.44:1337 os mcumgr-parameters` reports
  `buf_size=2048` and `buf_count=4`, uploaded signed test image version
  `1.0.0.1` with hash
  `71b659098dc75abf887e82c1e7e26e794f7afe5d81516321ad1a362a6347709c`,
  marked it pending, reset into it, confirmed it, then reset again. Final
  `image get-state` showed slot0 active and confirmed on version `1.0.0.1`.
  Upload emitted a host warning that device-side checksum verification was not
  requested; MCUboot image hash/state verification still passed. Production
  tooling should pass an upload checksum when supported by the client.
- 2026-05-07: Rollback-negative HIL test passed. Starting from confirmed
  version `1.0.0.1`, staged signed version `1.0.0`, marked it pending/test,
  reset into it, intentionally did not confirm, then reset again. Final
  `image get-state` showed version `1.0.0.1` active and confirmed, with
  version `1.0.0` inactive/unconfirmed in slot1.
- 2026-05-07: Enabled device-side upload hash checking with
  `CONFIG_IMG_ENABLE_IMAGE_CHECK=y` and updated the operator guide to pass
  `mcumgrctl image upload --checksum "$(sha256sum image.bin)"`.
- 2026-05-07: Added `tf-psa-crypto` to the Zephyr module import whitelist
  because `CONFIG_IMG_ENABLE_IMAGE_CHECK=y` selects
  `CONFIG_FLASH_AREA_CHECK_INTEGRITY=y`, which uses PSA SHA-256 in Zephyr 4.4.
  Ran `GIT_CONFIG_GLOBAL=/dev/null west update` to fetch the missing module.
  Rebuilt MT6835 sysbuild successfully; app `.config` contains
  `CONFIG_IMG_ENABLE_IMAGE_CHECK=y` and `CONFIG_FLASH_AREA_CHECK_INTEGRITY=y`.
- 2026-05-07: Flashed checksum-enabled sysbuild and reran checksum upload HIL
  against `10.0.0.44`. Command used
  `mcumgrctl --udp 10.0.0.44:1337 image upload --checksum <sha256>
  zephyr.signed.bin`. The upload completed without the earlier
  "Device did not perform image checksum verification" warning, and slot1
  reported uploaded version `1.0.0.1` with hash
  `71b659098dc75abf887e82c1e7e26e794f7afe5d81516321ad1a362a6347709c`.
