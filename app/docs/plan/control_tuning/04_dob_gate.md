# 04 DOB Gate

## Goal

Use DOB only as a residual disturbance observer after PI/MPR behavior is stable.

## Role

DOB estimates external/model-mismatch torque from velocity tracking residuals and injects a bounded `Iq` feedforward correction. It should clean up residual load/friction effects, not compensate for bad encoder data or unstable regulator tuning.

## Rules

- DOB remains disabled by default.
- Enable only after encoder mapping, measured model, and velocity baseline pass.
- Use `motor velocity dob defaults safe` before enabling.
- Disable DOB on large reference discontinuities or bad feedback.

## HIL Command

Use the feature-combo scenario with PI+DOB first, then MPR+DOB:

```bash
python3 scripts/hil/hil_telnet.py mpr-dob-detent --host 10.0.0.44 --yes-live-motion \
  --feature-combo pi_dob \
  --feature-combo mpr_dob \
  --velocity-hz 0.5 \
  --velocity-hold-ms 3000 \
  --mpr-bandwidth-hz 0.5
```

Acceptance:

- DOB status reports ready before enable.
- DOB feedforward remains bounded.
- Velocity validation passes or is no worse than the same regulator without DOB.
