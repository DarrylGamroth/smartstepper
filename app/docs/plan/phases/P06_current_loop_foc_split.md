# Phase P06 - Current Loop and FOC Split

## Objective

Split current-loop chain into current control, decoupling, transforms, and PWM synthesis modules.

## Prerequisites

1. P05 complete.

## Touch Files

1. `modules/motor_core/include/motor/control/*`
2. `modules/motor_core/src/control/*`
3. `app/src/motor_control_loop.c`
4. current-loop and FOC unit tests

## Do Not Touch

1. State machine transition logic.

## Tasks

1. Separate decoupling enable/validation from PI current control.
2. Isolate transform steps and PWM synthesis contracts.
3. Preserve saturation and fault behavior.

## Acceptance

1. FOC/current-loop tests pass.
2. No ISR-time regression.
3. Build + HIL smoke pass.

