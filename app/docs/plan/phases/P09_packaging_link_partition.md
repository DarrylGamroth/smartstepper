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

