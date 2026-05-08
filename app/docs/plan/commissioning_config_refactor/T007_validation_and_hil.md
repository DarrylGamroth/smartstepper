# T007 - Validation and HIL

## Goal

Prove the cleanup does not regress safe commissioning or encoder control bring-up.

## Build Validation

Run both composed profiles:

```bash
podman exec wonderful_goldberg bash -lc 'cd /workspace && west build -p always -b smartstepper_v2/stm32h743xx /workspace/chopper/app -d /workspace/build/chopper/smartstepper_v2_mt6835_id -S serial-shell -S serial-console -- -DDTC_OVERLAY_FILE="boards/smartstepper_v2.overlay;configs/encoder_mt6835_rtspi.overlay;configs/commissioning_default.overlay;configs/motor_id_safe_2a.overlay"'
```

```bash
podman exec wonderful_goldberg bash -lc 'cd /workspace && west build -p always -b smartstepper_v2/stm32h743xx /workspace/chopper/app -d /workspace/build/chopper/smartstepper_v2_aeat_id -S serial-shell -S serial-console -- -DDTC_OVERLAY_FILE="boards/smartstepper_v2.overlay;configs/encoder_aeat9955_rtspi.overlay;configs/commissioning_default.overlay;configs/motor_id_safe_067a.overlay"'
```

## Unit Validation

- Run all unit tests:

```bash
./tests/run_unit_tests.sh wonderful_goldberg
```

- Add focused tests for:
  - current PI gain calculation from RoverL average L,
  - current PI gain calculation from production `Ld/Lq`,
  - model source precedence,
  - rejected production electrical ID retaining RoverL provisional model.

## HIL Validation

For MT6835 first, then AEAT-9955 where encoder reliability permits:

1. Flash composed safe-ID build.
2. Clear settings or verify settings autoload disabled.
3. Run baseline commissioning.
4. Confirm state/status reports:
   - current offsets complete,
   - RoverL provisional model applied,
   - production Rs accepted,
   - demod `Ld/Lq` accepted,
   - current PI updated from production model,
   - encoder mapping complete,
   - flux ID accepted or explicitly degraded.
5. Run generated velocity smoke test.
6. Run encoder trace sanity check.
7. Run low-speed `velocity_encoder` smoke test only after encoder mapping is
   baseline-ready.

## Evidence

Record for each run:

- build command and result,
- firmware commit,
- overlay stack,
- commissioning status output,
- electrical ID values and confidence,
- encoder counters,
- final state/error status.
