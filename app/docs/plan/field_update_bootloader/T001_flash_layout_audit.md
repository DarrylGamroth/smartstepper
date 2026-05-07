# T001 - Flash Layout Audit

## Problem

The current DTS already defines internal flash MCUboot/image slots, a QSPI
filesystem partition, and an EEPROM settings partition. The internal flash
partition map is provisional and likely wrong: the STM32H743 has more flash than
the current fixed partitions allocate. Before enabling MCUboot or Settings/ZMS,
we need evidence that the partition sizes use the actual device capacity and
match real image/storage needs.

## Current Known Layout

From `boards/rubus/smartstepper_v2/smartstepper_v2.dts`:

| Storage | Current Use | Current Partitioning |
| --- | --- | --- |
| STM32 internal flash | boot/image area | `mcuboot` 128 KiB, `image-0` 256 KiB, `image-1` 256 KiB |
| QSPI NOR W25Q128 | bulk external storage | `filesystem` 16 MiB |
| I2C EEPROM M24C16 | currently defined but deferred | `settings_storage` 1 KiB, `user_data` 1 KiB |

The current internal-flash partition map accounts for only 640 KiB. That is not
a statement of total STM32H743 capacity. The audit must confirm the exact flash
size exposed by the selected `smartstepper_v2/stm32h743xx` target and then
redesign the partition map accordingly.

## Initial Evidence

- `boards/rubus/smartstepper_v2/smartstepper_v2.yaml` declares `flash: 2048`.
- The selected DTS includes `stm32h743Xi.dtsi`, which is the 2 MiB STM32H743
  class used by this board.
- Existing internal fixed partitions cover only:
  - `0x00000000..0x00020000`: 128 KiB MCUboot,
  - `0x00020000..0x00060000`: 256 KiB slot0,
  - `0x00060000..0x000A0000`: 256 KiB slot1.
- Existing fixed partitions therefore cover only 640 KiB of the 2048 KiB device
  flash and leave the upper internal flash unpartitioned.
- Current non-MCUboot MT6835 build evidence from
  `../build/chopper/smartstepper_v2/zephyr/zephyr.stat`:
  - ROM load segment size around `0x7A900` bytes, about `490 KiB`.
  - This already exceeds the current 256 KiB image slots.

## Initial Corrected Layout Candidate

Use the full 2 MiB internal flash with 128 KiB alignment:

| Partition | Offset | Size | Purpose |
| --- | ---: | ---: | --- |
| `mcuboot` | `0x00000000` | 128 KiB | Bootloader |
| `image-0` | `0x00020000` | 896 KiB | Primary app slot |
| `image-1` | `0x00100000` | 896 KiB | Secondary app slot |
| `settings_storage` | `0x001E0000` | 128 KiB | Settings/ZMS |

This layout gives each app slot enough room for the current ~490 KiB image plus
substantial growth, keeps settings in internal flash, and still uses all 2 MiB
of internal flash.

## Work

- Build MT6835 and AEAT profiles and record final image sizes.
- Confirm the exact MCU flash size from board metadata, generated DTS, linker
  memory regions, and final `zephyr.map`.
- Compare the full available internal flash against the current fixed-partition
  coverage and record unused/unallocated regions.
- Inspect resolved `zephyr.dts` for selected partitions.
- Confirm whether `zephyr,code-partition = &slot0_partition` is active in the
  current non-MCUboot build and whether partition overflow is currently being
  enforced.
- Record internal flash page/sector erase constraints for STM32H743.
- Record QSPI erase block size and driver support for image management.
- Decide whether the current 256 KiB image slots are viable. They likely are not
  if current app images remain around the previously observed ~500 KiB range.
- Propose a corrected internal-flash layout that uses the full STM32H743 flash
  capacity while leaving room for settings and required MCUboot swap/scratch
  semantics.
- Leave EEPROM out of the first implementation unless the audit discovers a
  concrete need for it.

## Validation

- Audit document lists image size, partition sizes, and pass/fail for current
  partition viability.
- Audit document explicitly states total internal flash, partitioned flash,
  unallocated flash, and recommended corrected layout.
- No bootloader work starts until slot sizing is resolved.
