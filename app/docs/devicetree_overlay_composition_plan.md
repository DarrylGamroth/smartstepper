# Devicetree Overlay Composition

Date: 2026-05-07

## Goal

Keep devicetree defaults safe and composable for motor identification without
requiring a fully commissioned motor profile.

## Design

Use three overlay layers:

1. Board overlay
   - Hardware that is fixed by the PCB.
   - Examples: ADC sense topology, PWM timers, GPIOs, shell UART.
   - File: `app/boards/smartstepper_v2.overlay`.

2. Encoder overlay
   - Encoder transport and encoder device selection only.
   - No motor electrical model or current limits.
   - Files:
     - `app/configs/encoder_mt6835_rtspi.overlay`
     - `app/configs/encoder_aeat9955_rtspi.overlay`

3. Motor identification overlay
   - Conservative motor/control defaults sufficient to run current offsets,
     PI current control, bidirectional Rs, R/L bootstrap, and demodulated Ld/Lq.
   - Values here are safe boot/identification fallbacks, not final commissioned
     values.
   - Files:
     - `app/configs/motor_id_safe_2a.overlay`
     - `app/configs/motor_id_safe_067a.overlay`

The older `motor_*` overlays remain usable as convenience full profiles, but new
bring-up should prefer explicit composition.

## Build Examples

MT6835 encoder with safe 1.5-2.0 A class motor-ID defaults:

```bash
podman exec wonderful_goldberg bash -lc '\
  cd /workspace && \
  west build -p always \
    -b smartstepper_v2/stm32h743xx \
    /workspace/chopper/app \
    -d /workspace/build/chopper/smartstepper_v2_mt6835_id \
    -S serial-shell -S serial-console -- \
    -DDTC_OVERLAY_FILE="boards/smartstepper_v2.overlay;configs/encoder_mt6835_rtspi.overlay;configs/motor_id_safe_2a.overlay"'
```

AEAT-9955 encoder with safe 0.67 A class motor-ID defaults:

```bash
podman exec wonderful_goldberg bash -lc '\
  cd /workspace && \
  west build -p always \
    -b smartstepper_v2/stm32h743xx \
    /workspace/chopper/app \
    -d /workspace/build/chopper/smartstepper_v2_aeat_id \
    -S serial-shell -S serial-console -- \
    -DDTC_OVERLAY_FILE="boards/smartstepper_v2.overlay;configs/encoder_aeat9955_rtspi.overlay;configs/motor_id_safe_067a.overlay"'
```

## Policy

- Devicetree values should be conservative enough for identification to run on
  a fresh controller.
- Commissioned values should be loaded from Settings/ZMS after they are trusted.
- ADC current offsets remain boot measurements and are not persisted.
- Encoder EEPROM or permanent encoder configuration should not be required for
  initial bring-up.
- `motor settings load ...` must reject identity/limit values that are
  incompatible with the active firmware image until those hot paths are fully
  runtime-configured.

## Identification Baseline

The safe motor-ID overlays provide enough for:

1. Offset measurement.
2. PI current control at conservative bandwidth.
3. Bidirectional Rs measurement.
4. R/L bootstrap or fallback.
5. Demodulated D/Q inductance refinement.
6. Flux identification.
7. Mechanical identification after encoder mapping is valid.

## Non-Goals

- Persisting final commissioned values in devicetree.
- Moving board topology into motor profiles.
- Supporting the legacy Zephyr sensor shell path for control encoders.
- Making pole pairs/current/voltage fully dynamic in this phase.
