# T002 - MCUboot Partition Design

## Decision To Make

Choose whether update images live entirely in internal flash or whether the
secondary slot/staging area uses QSPI.

## Recommended Direction

First redesign the internal flash map using the full STM32H743 flash capacity.
The current map under-allocates internal flash and should not drive the final
architecture.

Prefer internal flash for bootloader, primary image, secondary image, and
critical settings if the corrected layout can comfortably hold the firmware.
Use QSPI for update staging/secondary image only if the corrected internal flash
layout still cannot hold two appropriately sized application slots plus
settings.

## Candidate Layouts

### Selected Initial Layout - Internal Dual Slot

Use this as the first implementation target:

- `mcuboot`: 128 KiB at `0x00000000`
- `image-0`: 832 KiB at `0x00020000`
- `image-1`: 832 KiB at `0x000F0000`
- `settings_storage`: 256 KiB at `0x001C0000`

Rationale:

- It uses the full 2 MiB STM32H743 internal flash instead of the current
  under-allocated 640 KiB map.
- It keeps the boot path independent of QSPI.
- It leaves enough headroom for the current ~560 KiB application image.
- It keeps critical motor settings in internal flash.
- It gives Settings/ZMS two STM32H743 internal-flash erase sectors. ZMS mount
  requires at least two sectors; the previous 128 KiB settings partition was
  only one sector and could not register a settings save backend.
- It avoids needing QSPI support inside MCUboot for the first field-update
  bring-up.

### Option A - Internal Dual Slot

- Internal flash: MCUboot + slot0 + slot1 + settings.
- QSPI: filesystem/logs only.

Pros:

- Simpler boot path.
- Does not depend on QSPI during swap.
- Better for safety-critical control firmware.

Cons:

- May not fit if app image grows beyond half of usable internal flash.

### Option B - Internal Primary, QSPI Secondary

- Internal flash: MCUboot + slot0 + settings/scratch.
- QSPI: slot1/download + filesystem/logs.

Pros:

- Uses abundant QSPI capacity.
- Leaves more internal flash for the main image and settings.

Cons:

- Bootloader must reliably read external QSPI.
- External-flash driver and pin setup become part of the boot chain.
- Needs extra HIL coverage for QSPI failure modes.

### Option C - Download To QSPI, Copy To Internal Secondary

- Internal flash: MCUboot + slot0 + slot1 + settings.
- QSPI: temporary network download/cache.

Pros:

- Bootloader swap remains internal-flash based.
- Network download can tolerate larger temporary files.

Cons:

- Still requires enough internal flash for dual slots.
- Requires application-side copy/verify path.

## Work

- Select layout after T001 image-size audit.
- Update DTS fixed partitions.
- Use the actual STM32H743 flash capacity, not the current provisional 640 KiB
  fixed-partition coverage.
- Keep a dedicated `settings_partition` in internal flash if possible.
- Keep QSPI `filesystem` or split into `dfu_cache`, `trace_storage`, and
  `filesystem` if needed.
- Define scratch partition only if selected MCUboot swap mode requires it.
- Keep EEPROM out of the partition strategy for now; revisit only when a
  specific manufacturing/recovery use case exists.

## Validation

- Both MT6835 and AEAT builds link against the selected layout.
- MCUboot can see required partitions.
- Settings partition remains independent of image erase/write operations.
