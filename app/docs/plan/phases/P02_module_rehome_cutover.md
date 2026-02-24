# Phase P02 - Module Rehome Cutover

## Objective

Move module files to TI-style directory taxonomy with direct include-path cutover.

## Prerequisites

1. P01 complete.

## Touch Files

1. `modules/motor_core/include/motor/**`
2. `modules/motor_core/src/**`
3. call sites across `app/` and `tests/`
4. `modules/motor_core/CMakeLists.txt`
5. migration docs

## Do Not Touch

1. Algorithm behavior and constants.

## Tasks

1. Rehome headers/sources by family (`math`, `filters`, `observers`, `motion`, `control`, `protection`, `runtime`, `telemetry`).
2. Update all includes directly to new paths.
3. Record migration table and completion status.

## Acceptance

1. Zero legacy include references.
2. Both firmware targets build.
3. Full unit suite passes.

## Include Migration Table

| Family | Old include root | New include root | Source dir | Status |
| --- | --- | --- | --- | --- |
| `math` | `modules/motor_core/include/*.h` | `modules/motor_core/include/motor/math/*.h` | `modules/motor_core/src/math/` | complete |
| `filters` | `modules/motor_core/include/*.h` | `modules/motor_core/include/motor/filters/*.h` | header-only + existing users | complete |
| `observers` | `modules/motor_core/include/*.h` | `modules/motor_core/include/motor/observers/*.h` | `modules/motor_core/src/observers/` | complete |
| `motion` | `modules/motor_core/include/*.h` | `modules/motor_core/include/motor/motion/*.h` | `modules/motor_core/src/motion/` | complete |
| `control` | `modules/motor_core/include/*.h` | `modules/motor_core/include/motor/control/*.h` | `modules/motor_core/src/control/` | complete |
| `estimation` | `modules/motor_core/include/*.h` | `modules/motor_core/include/motor/estimation/*.h` | `modules/motor_core/src/estimation/` | complete |
| `runtime` | `modules/motor_core/include/*.h` | `modules/motor_core/include/motor/runtime/*.h` | `modules/motor_core/src/runtime/` | complete |

Notes:

1. `app/src` and `tests/unit` call sites were directly cut over to `#include "motor/.../*.h"` paths.
2. No compatibility wrapper headers were added.
3. Estimator modules (`rs_online`, `rls_motor_est`, `thermal_model`, `motor_commission_id`) were reclassified from protection/telemetry/runtime into `estimation`.
