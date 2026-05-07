# T004 - Sysbuild MCUboot Integration

## Work

- Add sysbuild configuration for MCUboot.
- Enable `CONFIG_BOOTLOADER_MCUBOOT=y` in the application build path.
- Ensure generated images are signed.
- Add development signing key only for local/HIL builds.
- Document production key handling separately; do not commit production private
  keys.
- Update AGENTS/build docs with MCUboot build and flash commands.

## Validation

- Clean sysbuild succeeds for MT6835 and AEAT profiles.
- Signed image boots.
- Unsigned/tampered image is rejected in a controlled test if practical.
