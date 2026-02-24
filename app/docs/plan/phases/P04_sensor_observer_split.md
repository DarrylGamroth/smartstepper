# Phase P04 - Sensor and Observer Split

## Objective

Decompose sensor/observer path into explicit modules with narrow contracts.

## Prerequisites

1. P03 complete.

## Touch Files

1. `modules/motor_core/include/motor/observers/*`
2. `modules/motor_core/src/observers/*`
3. `app/src/motor_encoder_feedback.c`
4. `app/src/motor_control_loop.c`
5. observer-related unit tests

## Do Not Touch

1. Motion profile and regulator logic.

## Tasks

1. Split encoder source classification from angle tracking and position conversion.
2. Separate control feedback payload from capture/debug payload.
3. Keep observer handoff and delay behavior unchanged.

## Acceptance

1. Existing observer/encoder unit tests pass.
2. HIL encoder behavior unchanged.
3. Build pass for both targets.

