# ISR Measurement Workflow

This document records the repeatable checks for the motor-control ISR path after the TI-style latency cleanup.

## Build

Use the normal west build from the container:

```bash
podman exec wonderful_goldberg bash -lc 'cd /workspace && west build --build-dir /workspace/build/chopper/smartstepper_v2'
```

For a minimal ISR feature build, use a temporary config file and separate build directory:

```bash
podman exec wonderful_goldberg bash -lc '
cat >/tmp/isr_min.conf <<CONF
CONFIG_MOTOR_OUTER_LOOP_MPR=n
CONFIG_MOTOR_VELOCITY_DOB=n
CONFIG_MOTOR_ISR_COMMISSION_CAPTURE=n
CONFIG_MOTOR_ISR_ENCODER_CAPTURE=n
CONFIG_MOTOR_ISR_ENCODER_RAW_TRACE=n
CONFIG_MOTOR_ISR_FAULT_SNAPSHOT=n
CONFIG_RLS_PARAMETER_ESTIMATION=n
CONF
cd /workspace && west build -p always \
  -b smartstepper_v2/stm32h743xx \
  /workspace/chopper/app \
  -d /workspace/build/chopper/smartstepper_v2_isr_min \
  -S serial-shell -S serial-console -- \
  -DDTC_OVERLAY_FILE="boards/smartstepper_v2.overlay;configs/motor_aeat9955_067a.overlay" \
  -DEXTRA_CONF_FILE=/tmp/isr_min.conf'
```

## Symbol And Stack-Frame Check

Run this after a firmware build:

```bash
podman exec wonderful_goldberg bash -lc '
NM=/opt/toolchains/zephyr-sdk-1.0.0/gnu/arm-zephyr-eabi/bin/arm-zephyr-eabi-nm
OD=/opt/toolchains/zephyr-sdk-1.0.0/gnu/arm-zephyr-eabi/bin/arm-zephyr-eabi-objdump
cd /workspace/build/chopper/smartstepper_v2
$NM -S --size-sort zephyr/zephyr.elf | grep -E "motor_control_loop_step|motor_control_step_|motor_outer_loop_.*step|motor_foc_voltage_pwm_step_fast"
for sym in \
  motor_control_loop_step \
  motor_control_step_reference_stage \
  motor_control_step_foc_stage \
  motor_control_step_read_encoder \
  motor_outer_loop_runtime_step \
  motor_outer_loop_position_step \
  motor_outer_loop_velocity_step; do
  echo "--- $sym"
  $OD -d --demangle zephyr/zephyr.elf | awk "/^[0-9a-f]+ <${sym}[^>]*>:/,/^$/" | sed -n "1,12p"
done'
```

Read stack-frame evidence from instructions like `sub sp, #108`. Include saved registers/FPU separately when estimating total nested stack.

## Boundary Regression Check

The `chopper.runtime.unit` test scans `app/src/motor_control_loop.c` for forbidden ISR dependencies such as kernel queue/work/semaphore APIs, logging, printing, and allocation calls. Run it with:

```bash
./tests/run_unit_tests.sh wonderful_goldberg -s chopper.runtime.unit
```

## HIL Cycle Check

When hardware is stable, use the serial shell to collect ISR timing counters in each important mode.

Open the serial shell:

```bash
tio -b 115200 /dev/serial/by-id/usb-FTDI_TTL232R-3V3_FTE3B04Y-if00-port0
```

Velocity-open baseline:

```text
motor state clear_error
motor disarm
motor state idle
motor safety timeout 0
motor state offline
motor arm
motor state mode velocity_open
motor current id 0
motor current iq 0.15
motor velocity target 5
motor info live
motor state status
motor current iq 0
motor velocity target 0
motor disarm
motor state idle
motor safety timeout 1000
```

Torque/closed-loop checks should only be run when encoder quality is known good:

```text
motor state clear_error
motor disarm
motor state idle
motor safety timeout 0
motor state offline
motor arm
motor state mode torque
motor current id 0
motor current iq 0.05
motor info live
motor state status
motor current iq 0
motor disarm
motor state idle
motor safety timeout 1000
```

Record `max_isr_cycles`, `total_isr_cycles`, loop count, encoder warning/error counters, and any fault snapshot data in `app/docs/plan/execution_log.md`.

## Current Firmware Evidence

Current default build evidence after the TI-style cleanup:

| Symbol | Code size | Local frame observed |
| --- | ---: | ---: |
| `motor_control_loop_step` | `0x5c2` / 1474 B | 108 B |
| `motor_control_step_reference_stage` | `0x48e` / 1166 B | no explicit local `sub sp`; saved registers only |
| `motor_control_step_foc_stage.isra.0` | `0x370` / 880 B | 76 B |
| `motor_control_step_measure_stage` | `0x25c` / 604 B | measured through symbol/disassembly as separate stage |
| `motor_control_step_read_encoder.constprop.0` | `0x15e` / 350 B | 20 B |
| `motor_outer_loop_runtime_step` | `0x60` / 96 B | saved-register scratch only |
| `motor_outer_loop_position_step` | `0x178` / 376 B | 40 B |
| `motor_outer_loop_velocity_step` | `0x15c` / 348 B | 12 B |

The top-level frame is larger than the immediate Phase 1 incremental result because the clean build performs constprop/isra optimization and keeps some scalar spill space in the parent. The large dataflow structs remain in persistent `motor_rt_control_ctx`, not on the ISR stack.
