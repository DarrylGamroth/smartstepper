# T001 - Requirements And Safety Envelope

## Outputs

- `rs_ohm`
- `l_avg_h`
- `ld_h`
- `lq_h`
- `lq_minus_ld_h`
- measurement current/voltage/duty actually used
- sample counts
- repeatability metrics
- fault/saturation counters
- current PI recommendation
- validation status

## Safety Limits

- Current limit from motor profile/devicetree.
- Pulse duty/voltage limit from Vbus and motor current limit.
- Overcurrent/hardware-break abort.
- Maximum measurement duration.
- Explicit command only; no automatic boot execution.

## Acceptance Criteria

- Finite positive `Rs`, `Lavg`, `Ld`, and `Lq`.
- Values within configurable bounds relative to devicetree nominal values.
- Repeatability error below threshold.
- No saturation/fault during accepted windows.
- Current step validation passes before promotion.
