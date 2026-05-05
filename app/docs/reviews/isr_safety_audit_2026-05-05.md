# ISR Safety Audit

Date: 2026-05-05

Scope: application motor ISR entry points and local low-latency PWM/ADC callback plumbing.

## Context Classes

| Entry point | Source | Context | Allowed behavior | Current status |
|---|---|---|---|---|
| `adc_callback()` | `drivers/adc/adc_stm32_injected.c` JEOS ISR | zero-latency direct ISR | fixed-cost control loop, atomics, preallocated rings, no kernel queue/shell/log calls | acceptable; uses explicit FPU save/restore and ISR-safe event ring |
| `encoder1_callback()` | `drivers/pwm/mcpwm_stm32.c` compare direct ISR | direct ISR | request one encoder sample only, no kernel queue/shell/log calls | acceptable; request-only plus simple sampling gate |
| `gate_driver_*_break_callback()` | `drivers/pwm/mcpwm_stm32.c` break IRQ callback | normal IRQ today, treated as ISR-only | latch/disarm and enqueue ISR event only | fixed in P5 |
| `state_timer_expiry()` | Zephyr `k_timer` expiry | interrupt context | enqueue ISR event only | acceptable |

## Changes Made

- `gate_driver_a_break_callback()` and `gate_driver_b_break_callback()` no longer call `drv8328_disable_all_channels()` from interrupt context.
- Gate-driver break callbacks no longer call the general `motor_api_post_error()` path.
- Gate-driver break callbacks now:
  - clear `control_armed` atomically when `motor_parameters` is available.
  - enqueue `MOTOR_EVENT_ERROR/ERROR_HARDWARE_BREAK` through `motor_api_enqueue_event_from_isr()`.
- `mcpwm_stm32_brk_isr()` no longer logs from the break interrupt path.

## Notes

- The hardware timer break input already disables PWM output. Thread-context ERROR handling still performs the full gate-driver software shutdown.
- `adc_callback()` still toggles the debug GPIO using the Zephyr GPIO API. This is intentionally kept as debug instrumentation for now, but it is a candidate for direct LL GPIO access or compile-time disable if ISR timing becomes tight.
- `motor_api_post_event()` and `motor_api_post_error()` now route ISR callers to the ISR ring, but direct/zero-latency callbacks should still prefer `motor_api_enqueue_event_from_isr()` directly so they do not depend on `k_is_in_isr()`.

## Static Checks

Commands used:

```bash
rg -n "motor_api_post_error|k_msgq|k_sem|k_mutex|shell_|LOG_|printk|drv8328_disable" app/src/motor_isr_io.c drivers/adc/adc_stm32_injected.c drivers/pwm/mcpwm_stm32.c
rg -n "ISR_DIRECT_DECLARE|IRQ_DIRECT_CONNECT|IRQ_CONNECT|adc_callback|encoder1_callback|break_callback" app/src drivers -S
```

Findings after P5 changes:

- No `k_msgq`, shell, or `motor_api_post_error()` calls remain in `app/src/motor_isr_io.c` ISR callbacks.
- No break-ISR logging remains in `drivers/pwm/mcpwm_stm32.c`.
- ADC direct ISR still has explicit FPU context preservation.

## Residual Risks

- The MCPWM break IRQ is currently connected with `IRQ_CONNECT`, not `IRQ_DIRECT_CONNECT`, but the callback is intentionally kept ISR-only in case this changes later.
- `adc_callback()` remains large and FPU-heavy by design. P7/P8 cover further control-kernel extraction and fast-block discipline.
- Encoder request paths are ISR-safe by construction but still depend on the selected encoder backend being safe for ISR request/collect semantics.
