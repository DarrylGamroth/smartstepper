# T006 - Settings/ZMS Long-Term Configuration

## Goal

Define the longer-term persistence model without making persistence unsafe during
bring-up.

## Decision

Use Zephyr Settings with ZMS backend for typed motor settings. Do not use raw
Settings shell writes as the product interface.

## Persistent Groups

Recommended groups:

| Group | Contents | Notes |
| --- | --- | --- |
| identity | pole pairs and immutable identity overrides if ever needed | Normally DT-owned; save rarely. |
| limits | user/product limits below hard DT limits | max velocity, max accel, command timeout, optional user current cap. |
| encoder | generated-sweep offset/map and direction convention | Required for encoder modes after boot persistence is enabled. |
| model | accepted commissioned Rs/Ld/Lq/flux/J/fallback confidence | Active model used for PI and BEMF limits. |
| controllers | bandwidth requests, not raw derived gains where possible | Raw PI/MPR/DOB gains should be recalculated from model. |
| detent | later detent table/map record | Exclude from V1 baseline persistence. |

## Work

1. Keep settings load/save explicit:
   - `motor settings preview`,
   - `motor settings load`,
   - `motor settings save`,
   - `motor settings clear`.
2. Keep autoload disabled until HIL validates rollback behavior.
3. Store accepted baseline model values after commissioning only when commanded.
4. Store controller bandwidth/config intent rather than derived MPR/DOB gains
   when the gains can be recomputed from model values.
5. Never store fast current offsets as baseline persistence; run offset
   calibration on every boot.
6. Add schema version, CRC, flags, and model source metadata.
7. Enforce apply guards:
   - cannot mutate while armed/online,
   - cannot apply invalid/incomplete model,
   - cannot exceed DT hard limits.

## Constraints

- Settings are per-unit values; devicetree remains the erased-settings fallback.
- Bad saved values must be clearable from shell and recoverable by J-Link.
- Do not require EEPROM for V1; use MCU flash/ZMS partition as planned.

## Validation

- Unit tests cover schema/version/CRC/load failure paths.
- HIL test proves erased settings boots from DT fallback.
- HIL test proves saved baseline can be loaded explicitly and applied safely.
- Later rollback/autoload tests pass before enabling boot autoload.
