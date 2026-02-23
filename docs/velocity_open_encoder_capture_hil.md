# Velocity-Open Encoder Capture HIL

This procedure validates encoder sampling while the motor is driven in
`velocity_open` mode.

## Preconditions

- Firmware flashed to `smartstepper_v2`.
- Serial shell connected (`/dev/serial/by-id/usb-FTDI_TTL232R-3V3_FTE3B04Y-if00-port0`, `115200`).
- Motor can spin safely.

## Shell Sequence

Run these commands in order:

```text
motor disarm
motor state idle
motor state offline
motor safety timeout 0
motor arm
motor state mode velocity_open
motor current id 0
motor current iq 0.15
motor velocity target 0.80
motor encoder capture clear
motor encoder capture start 1
```

Let it run for about 1-2 seconds, then:

```text
motor encoder capture stop
motor encoder capture status
motor encoder capture dump 256
```

Shutdown sequence:

```text
motor velocity target 0
motor current iq 0
motor disarm
motor state idle
```

## What To Check

- `fresh`: should be high (close to sample count).
- `err`: should be `0` for a healthy stream.
- `warn`: counts samples where the AEAT frame warning bit (`0x80`) was set.
  This is a sensor warning flag, not an RTIO transport fault by itself.
- Sign sanity:
  - If commanded velocity is `+0.80 Hz` but measured encoder velocity is negative,
    encoder/mechanical direction mapping is inverted.
  - Use `motor encoder direction -1` (or `motor encoder direction 1`) to set
    control-loop direction mapping without modifying encoder EEPROM.
  - Closed-loop modes need consistent sign convention.

## Notes

- `warn` and `err` in capture rows come from encoder frame status bits in the
  returned position frame.
- AEAT magnet diagnostics (MHI/MLO) are separate status fields; use sensor/shell
  status commands to inspect those alarm bits directly.
