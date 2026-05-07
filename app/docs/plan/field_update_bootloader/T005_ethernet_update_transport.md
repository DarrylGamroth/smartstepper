# T005 - Ethernet Update Transport

## Recommended Initial Transport

Use MCUmgr SMP over UDP/IPv4 Ethernet for the first local field-update path.
Zephyr provides the UDP MCUmgr transport for this target; TCP remains enabled
for telnet shell/debugging and is not the update transport.

Development flashing remains J-Link. Ethernet update is for validating and
eventually supporting field-update behavior, not for replacing J-Link during
normal firmware development.

## Work

- Decide SMP transport based on current network footprint and tool support.
- Enable MCUmgr image and OS groups.
- Enable UDP/IPv4 MCUmgr transport on port `1337`.
- Keep telnet shell for diagnostics, not as the update transport.
- Add network status prerequisites to the HIL update script.
- Add host-side update command examples.

## Validation

- Device obtains IPv4 address.
- Host can query image list over Ethernet.
- Host can upload a signed test image.
- Device can mark image pending/test.
