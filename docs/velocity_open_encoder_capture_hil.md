# Velocity-Open Encoder Capture/Compare HIL

This procedure validates encoder sampling while the motor is driven in
`velocity_open` mode.

## Preconditions

- Firmware flashed to `smartstepper_v2`.
- Serial shell connected (`/dev/serial/by-id/usb-FTDI_TTL232R-3V3_FTE3B04Y-if00-port0`, `115200`).
- Motor can spin safely.

## Shell Sequence

Run these commands in order (note `encoder direction` must be set while not in
ONLINE):

```text
motor state clear_error
motor disarm
motor state idle
motor encoder direction -1
motor safety timeout 0
motor state mode velocity_open
motor state offline
motor arm
motor current id 0
motor current iq 0.15
motor velocity target 5
motor encoder capture clear
motor encoder capture start 1
```

Let it run for about 1-2 seconds, then:

```text
motor encoder capture stop
motor encoder capture status
motor encoder capture dump 256
motor encoder capture compare 220
```

Shutdown sequence:

```text
motor velocity target 0
motor current iq 0
motor disarm
motor state idle
```

## What To Check (Capture)

- `fresh`: should be high (close to sample count).
- `err`: should be `0` for a healthy stream.
- `warn`: counts samples where the AEAT frame warning bit (`0x80`) was set.
  This is a sensor warning flag, not an RTIO transport fault by itself.
- Sign sanity:
  - If commanded velocity is positive but measured encoder velocity is negative,
    encoder/mechanical direction mapping is inverted.
  - Use `motor encoder direction -1` (or `motor encoder direction 1`) to set
    control-loop direction mapping without modifying encoder EEPROM.
  - Closed-loop modes need consistent sign convention.

## What To Check (Compare)

`motor encoder capture compare` prints:

```text
idx loop src fresh warn err cmp enc_m_deg gen_m_deg d_m_deg enc_e_deg gen_e_deg d_e_deg
```

- `cmp=1` means a fresh, warning-free, error-free encoder sample was available.
- `d_m_deg` is mechanical encoder minus generated mechanical angle.
- `d_e_deg` is electrical encoder minus generated electrical angle.

For a healthy open-loop tracking comparison:
- `d_m_deg` should be nearly constant over time (it can be non-zero).
- Drift in `d_m_deg` indicates sign or scaling mismatch.

## Baseline (Current Hardware)

From HIL on this setup:
- With `encoder direction = +1`, `d_m_deg` drifted significantly.
- With `encoder direction = -1`, `d_m_deg` became very stable:
  - mean about `-89.76 deg`
  - standard deviation about `0.078 deg`
  - max drift from first valid sample about `0.19 deg`

Interpretation:
- Encoder stream is coherent in `velocity_open`.
- Direction sign should be `-1` for this motor wiring.
- Non-zero constant `d_m_deg` is expected because generator and absolute encoder
  use different mechanical zero references.

## Alignment Notes

- Dual-polarity ALIGN can produce near-zero polarity separation on this hardware.
- Firmware now falls back to `+Id`-only offset instead of faulting:
  commit `5721d88` (`motor: harden align fallback and add encoder-vs-generator compare capture`).
- This removed intermittent `HARDWARE_BREAK` failures during `motor state offline`.

## Notes

- `warn` and `err` in capture rows come from encoder frame status bits in the
  returned position frame.
- AEAT magnet diagnostics (MHI/MLO) are separate status fields; use sensor/shell
  status commands to inspect those alarm bits directly.
