# T006 - Current PI Recommendation

## Scope

Generate PI recommendations from measured parameters:

- D-axis: `Kp_d = Ld * wc`, `Ki_d = (Rs/Ld) * Ts`
- Q-axis: `Kp_q = Lq * wc`, `Ki_q = (Rs/Lq) * Ts`

## Guardrails

- Clamp bandwidth to configured safe limits.
- Reject if measured values are invalid or confidence is too low.
- Keep existing devicetree PI as fallback.
