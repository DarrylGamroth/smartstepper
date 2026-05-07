# T005 - Ld/Lq Derivation And Repeatability

## Scope

Derive:

- `Lavg = (Ld + Lq) / 2`
- `Lq-Ld`
- `Ld = Lavg - (Lq-Ld)/2`
- `Lq = Lavg + (Lq-Ld)/2`

## Acceptance

- `Ld > 0`, `Lq > 0`.
- Repeatability across passes within threshold.
- Saliency estimate can be rejected independently while keeping scalar `Lavg`.
- Report confidence separately for scalar inductance and saliency.
