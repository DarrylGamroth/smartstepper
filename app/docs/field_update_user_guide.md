# Field Update User Guide

## Scope

This project uses MCUboot as a standalone bootloader and MCUmgr SMP over
UDP/IPv4 Ethernet for field updates. Development flashing still uses the J-Link
runner.

The current implementation provides:

- internal flash partitions for MCUboot, slot0, slot1, and Settings/ZMS,
- sysbuild configuration for MCUboot plus a signed application image,
- MCUmgr image/OS management groups over UDP port `1337`,
- Zephyr's built-in `mcuboot` shell command,
- app-side boot logging for image confirmation and pending swap state.

Auto-confirm is disabled by default. A test image must be explicitly confirmed
after boot or MCUboot will roll back on the next reset.

## Build A Signed Development Image

MT6835 profile:

```bash
podman exec wonderful_goldberg bash -lc '\
  cd /workspace && \
  west build --sysbuild -p always \
    -b smartstepper_v2/stm32h743xx \
    /workspace/chopper/app \
    -d /workspace/build/chopper/smartstepper_v2_mcuboot \
    -S serial-shell -S serial-console -- \
    -DDTC_OVERLAY_FILE="boards/smartstepper_v2.overlay;configs/motor_mt6835_2a.overlay"'
```

AEAT-9955 profile:

```bash
podman exec wonderful_goldberg bash -lc '\
  cd /workspace && \
  west build --sysbuild -p always \
    -b smartstepper_v2/stm32h743xx \
    /workspace/chopper/app \
    -d /workspace/build/chopper/smartstepper_v2_mcuboot_aeat \
    -S serial-shell -S serial-console -- \
    -DDTC_OVERLAY_FILE="boards/smartstepper_v2.overlay;configs/motor_aeat9955_067a.overlay"'
```

Expected artifacts:

```text
<build>/mcuboot/zephyr/zephyr.bin
<build>/app/zephyr/zephyr.signed.bin
<build>/app/zephyr/zephyr.signed.hex
```

The normal field-update payload is `app/zephyr/zephyr.signed.bin`.

## Flash During Development

Flash the sysbuild directory so the bootloader and app are programmed together:

```bash
podman exec wonderful_goldberg bash -lc '\
  west flash -d /workspace/build/chopper/smartstepper_v2_mcuboot \
    --runner jlink --dev-id 10.0.0.70 --dev-id-type ip'
```

Use the AEAT build directory if that profile was built:

```bash
podman exec wonderful_goldberg bash -lc '\
  west flash -d /workspace/build/chopper/smartstepper_v2_mcuboot_aeat \
    --runner jlink --dev-id 10.0.0.70 --dev-id-type ip'
```

## Local Shell Commands

The firmware enables Zephyr's built-in `mcuboot` shell command. Useful commands:

```text
mcuboot
mcuboot confirm
mcuboot request_upgrade
mcuboot request_upgrade permanent
mcuboot erase <area_id>
```

Use `mcuboot confirm` only after the new image has booted and basic health checks
pass.

## MCUmgr Ethernet Update Flow

The device-side transport is MCUmgr SMP over UDP/IPv4 on port `1337`. The host
has `mcumgrctl` installed at `/home/dgamroth/.cargo/bin/mcumgrctl`. Use that
host tool; it is not installed inside the build container.

Get the board IP address from boot logs, telnet, or the Zephyr network shell:

```text
net iface
```

Then upload and test a new image. If using a container build artifact, copy or
reference the corresponding host-visible build path.

```bash
IP=<board-ip>
IMG=/home/dgamroth/workspaces/zephyr-workspace/build/chopper/smartstepper_v2_mcuboot/app/zephyr/zephyr.signed.bin
CHECKSUM=$(sha256sum "${IMG}" | awk '{print $1}')
HASH=$(mcumgrctl firmware get-image-info --json mcuboot "${IMG}" | jq -r '.hash')

mcumgrctl --udp "${IP}:1337" os mcumgr-parameters
mcumgrctl --udp "${IP}:1337" image get-state
mcumgrctl --udp "${IP}:1337" image upload --checksum "${CHECKSUM}" "${IMG}"
mcumgrctl --udp "${IP}:1337" image get-state
mcumgrctl --udp "${IP}:1337" image set-state --hash "${HASH}"
mcumgrctl --udp "${IP}:1337" os system-reset
```

After the new image boots and health checks pass, confirm it:

```text
mcuboot confirm
```

or, from the host:

```bash
mcumgrctl --udp "${IP}:1337" image set-state --confirm
```

If the image is not confirmed, MCUboot rolls back on the next reset.

The upload path may print:

```text
Device did not perform image checksum verification
```

That warning means the running firmware does not have device-side image checking
enabled, or the host did not send a checksum. This project enables
`CONFIG_IMG_ENABLE_IMAGE_CHECK=y`; use `--checksum "$(sha256sum image.bin)"` in
the host upload command so the device verifies the bytes written to flash.

## Inspect A Signed Image

```bash
podman exec wonderful_goldberg bash -lc '\
  /opt/python/venv/bin/imgtool dumpinfo \
    /workspace/build/chopper/smartstepper_v2_mcuboot/app/zephyr/zephyr.signed.bin'
```

The signed image should contain SHA256, KEYHASH, and ECDSA signature TLVs.

## Production Key Policy

The current development build uses MCUboot's default ECDSA-P256 key. That is
acceptable for bring-up only. Do not ship it.

Generate a production key outside the repository:

```bash
podman exec wonderful_goldberg bash -lc '\
  mkdir -p /workspace/secure-keys && \
  /opt/python/venv/bin/imgtool keygen \
    -k /workspace/secure-keys/chopper-prod-ec-p256.pem \
    -t ecdsa-p256'
```

Build with that key:

```bash
podman exec wonderful_goldberg bash -lc '\
  cd /workspace && \
  west build --sysbuild -p always \
    -b smartstepper_v2/stm32h743xx \
    /workspace/chopper/app \
    -d /workspace/build/chopper/smartstepper_v2_mcuboot_prodkey \
    -S serial-shell -S serial-console -- \
    -DDTC_OVERLAY_FILE="boards/smartstepper_v2.overlay;configs/motor_mt6835_2a.overlay" \
    -DSB_CONFIG_BOOT_SIGNATURE_KEY_FILE="/workspace/secure-keys/chopper-prod-ec-p256.pem"'
```

Once a bootloader built with a production key is flashed, future application
updates must be signed with the matching private key.

## ZCBOR Module Requirement

MCUmgr requires Zephyr's `zcbor` module. Checksum-verified image upload uses
Zephyr's flash-area SHA-256 integrity path, which requires `tf-psa-crypto` in
Zephyr 4.4. Both modules are listed in `west.yml`.

If a workspace is missing it:

```bash
west update
```

If a container global Git config rewrites `https://github.com` to SSH and breaks
the fetch, override the global config for the update command:

```bash
GIT_CONFIG_GLOBAL=/dev/null west update
```

## Current Validation

Validated on 2026-05-07:

- MT6835 sysbuild completes.
- `CONFIG_BOOTLOADER_MCUBOOT=y`.
- `CONFIG_MCUBOOT_SHELL=y`.
- `CONFIG_MCUMGR=y`.
- `CONFIG_MCUMGR_GRP_IMG=y`.
- `CONFIG_MCUMGR_GRP_OS=y`.
- `CONFIG_MCUMGR_TRANSPORT_UDP=y`.
- `CONFIG_SETTINGS_ZMS=y`.
- `CONFIG_ZCBOR=y`.
- `CONFIG_IMG_ENABLE_IMAGE_CHECK=y`.
- Signed app image is generated and `imgtool dumpinfo` reports ECDSA signature
  TLVs.
- End-to-end MCUmgr upload/test/reset/confirm over Ethernet using the
  host-installed `mcumgrctl` on the MT6835 target at `10.0.0.44`.

HIL evidence from 2026-05-07:

- Flashed sysbuild image through J-Link at `10.0.0.70`.
- `mcumgrctl --udp 10.0.0.44:1337 os mcumgr-parameters` returned
  `buf_size=2048` and `buf_count=4`.
- Uploaded signed test image `1.0.0.1` with hash
  `71b659098dc75abf887e82c1e7e26e794f7afe5d81516321ad1a362a6347709c`.
- Marked slot1 pending, reset, booted test image as active/unconfirmed, then
  confirmed it with `mcumgrctl image set-state --confirm`.
- Reset again; final state remained active and confirmed on version `1.0.0.1`.
- Rollback-negative path passed: staged version `1.0.0`, reset into it without
  confirming, reset again, and MCUboot rolled back to confirmed version
  `1.0.0.1`.
- Device-side upload checksum verification passed after enabling
  `CONFIG_IMG_ENABLE_IMAGE_CHECK=y`: flashing the checksum-enabled firmware and
  uploading with `mcumgrctl image upload --checksum ...` completed without the
  previous checksum warning.
