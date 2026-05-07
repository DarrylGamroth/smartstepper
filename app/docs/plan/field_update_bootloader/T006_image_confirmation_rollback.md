# T006 - Image Confirmation And Rollback

## Policy

A new image should not be confirmed immediately at boot. Confirm only after the
application proves enough health for this motor controller.

## Candidate Health Gate

- booted without hard fault,
- network and shell are reachable,
- settings load/preview succeeds or cleanly falls back to DT,
- motor remains disarmed and safe,
- non-motion status HIL passes,
- optional current-offset calibration passes if enabled for the gate.

## Work

- Use Zephyr's built-in `mcuboot` shell command for local status and
  confirmation.
- Add app-side boot logging for active slot, confirmation state, and pending
  swap type.
- Keep automatic confirmation disabled by default. Optional delayed
  auto-confirm can be enabled with `CONFIG_APP_UPDATE_AUTO_CONFIRM_AFTER_BOOT`
  after HIL gates are mature.
- Add rollback test where confirmation is intentionally withheld.

## Validation

- Confirmed image remains after reset.
- Unconfirmed image rolls back after reset.
- Settings survive both confirm and rollback flows.
