# 05 Detent Feedforward Gate

## Goal

Treat detent feedforward as a commissioned map that reduces predictable position-periodic ripple, then let DOB handle residual disturbances.

## Order

1. Commission baseline model.
2. Validate PI or MPR without detent.
3. Capture detent map over multiple forward/reverse cycles.
4. Validate detent off/on effect.
5. Only then test detent + DOB.

## HIL Command

```bash
python3 scripts/hil/hil_telnet.py mpr-dob-detent --host 10.0.0.44 --yes-live-motion \
  --feature-combo pi \
  --feature-combo pi_detent \
  --feature-combo mpr \
  --feature-combo mpr_detent \
  --detent-hz 0.20 \
  --detent-cycles 10 \
  --detent-iq-limit 0.12 \
  --velocity-hz 0.5 \
  --velocity-hold-ms 3000 \
  --mpr-bandwidth-hz 0.5
```

Acceptance:

- detent map passes apply-quality gate,
- no new encoder acquisition errors,
- detent-enabled velocity validation is not worse than baseline.
