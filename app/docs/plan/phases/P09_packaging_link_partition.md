# Phase P09 - Packaging and Link Partition

## Objective

Split `motor_core` into link-time sub-libraries by runtime criticality.

## Prerequisites

1. P08 complete.

## Touch Files

1. `modules/motor_core/CMakeLists.txt`
2. new sub-library CMake files if introduced
3. docs/logs for size/jitter results

## Do Not Touch

1. Functional algorithm behavior.

## Tasks

1. Partition into `motor_core_rt`, `motor_core_motion`, `motor_core_estimation`, `motor_core_commission`.
2. Ensure ISR path links only required runtime objects.
3. Capture link-map and size delta.

## Acceptance

1. Size reduction or neutral footprint with cleaner partition.
2. No control jitter regression.
3. Full build/test pass.

## Outcome (2026-02-25)

1. Link partition implemented:
  - `motor_core_rt`
  - `motor_core_motion`
  - `motor_core_estimation`
  - `motor_core_commission`
2. Link map evidence (`zephyr.map`):
  - `motor_core/src/libmotor_core_rt.a(...)`
  - `motor_core/src/libmotor_core_motion.a(...)`
  - `motor_core/src/libmotor_core_commission.a(...)`
  - `LOAD motor_core/src/libmotor_core_estimation.a`
3. Size comparison:
  - `smartstepper_v2_mt6835` pre-P09: `FLASH=231704`, `RAM=109984`
  - `smartstepper_v2_mt6835` post-P09: `FLASH=232128`, `RAM=110112`
  - delta: `+424B FLASH`, `+128B RAM`
  - `smartstepper_v2` pre/post-P09 remained `FLASH=236560`, `RAM=111008`
4. Jitter note:
  - No dedicated HIL jitter capture was run in P09.
  - P09 made no control algorithm changes; only link partitioning and archive layout.
