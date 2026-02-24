# Target Architecture

## Module Layout

1. `modules/motor_core/include/motor/math/*`
2. `modules/motor_core/include/motor/filters/*`
3. `modules/motor_core/include/motor/observers/*`
4. `modules/motor_core/include/motor/motion/*`
5. `modules/motor_core/include/motor/control/*`
6. `modules/motor_core/include/motor/estimation/*`
7. `modules/motor_core/include/motor/runtime/*`

1. `modules/motor_core/src/math/*`
2. `modules/motor_core/src/filters/*`
3. `modules/motor_core/src/observers/*`
4. `modules/motor_core/src/motion/*`
5. `modules/motor_core/src/control/*`
6. `modules/motor_core/src/estimation/*`
7. `modules/motor_core/src/runtime/*`

## Ownership Map

1. `math`: stateless helpers, header-only or pure C.
2. `filters`: signal filtering primitives and wrappers.
3. `observers`: encoder source, angle tracking, position conversion.
4. `motion`: angle generation, trajectories, motion profile planning.
5. `control`: position/velocity regulation, command arbitration, interlocks, current loop, decoupling, pwm synthesis.
6. `estimation`: online/system estimators and identification helpers.
7. `runtime`: coherent config snapshots, fast-state containers, process pipeline entry, keepalive policy.

## ISR Stage Contract

1. `Collect`
- inputs: ADC raw sample frame, encoder pipeline completion, runtime cfg snapshot.
- outputs: `motor_collect_frame`.

2. `Process`
- inputs: `motor_collect_frame`, `motor_rt_cfg_snapshot`, `motor_rt_fast_state`.
- outputs: `motor_process_frame` containing actuator command, protection flags, compact state outputs.

3. `Apply`
- inputs: `motor_process_frame.actuator_cmd`, protection action flags.
- outputs: hardware side effects only.

4. `Telemetry`
- inputs: collect + process frames, timing counters.
- outputs: live telemetry mirror, optional ISR diag capture, optional SPSC enqueue.

## Core API Target

```c
int motor_core_step_fast(const struct motor_rt_cfg_snapshot *cfg,
                         struct motor_rt_fast_state *fast,
                         const struct motor_collect_frame *in,
                         struct motor_process_frame *out);
```

Rules:

1. Allowed side effects: updates to `fast` state and explicit output structs.
2. Disallowed side effects: queue operations, kernel calls, blocking I/O, logs.
