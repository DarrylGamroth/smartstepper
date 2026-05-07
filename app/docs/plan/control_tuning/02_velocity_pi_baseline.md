# 02 Velocity PI Baseline

## Goal

Establish the reference velocity-controller performance using model-derived PI gains and measured current-loop parameters.

## Workflow

1. Run production electrical ID.
2. Run standard commissioning so `Kt/J/B/Tc` are measured.
3. Set outer mode to PI.
4. Run velocity sweep points over the useful bus-limited range.

Recommended sweep targets for this motor:

```text
+/-0.1, +/-0.3, +/-0.5, +/-1, +/-3, +/-5 Hz
```

Use enough hold time for at least one mechanical revolution where practical.

## HIL Command

```bash
python3 scripts/hil/hil_telnet.py velocity-sweep --host 10.0.0.44 --yes-live-motion \
  --velocity-sweep-commission standard \
  --velocity-sweep-regulator pi \
  --velocity-sweep-rotations 1.0 \
  --velocity-sweep-iq-limit 0.225 \
  --velocity-sweep-target-hz 0.1 --velocity-sweep-target-hz -0.1 \
  --velocity-sweep-target-hz 0.3 --velocity-sweep-target-hz -0.3 \
  --velocity-sweep-target-hz 0.5 --velocity-sweep-target-hz -0.5 \
  --velocity-sweep-target-hz 1.0 --velocity-sweep-target-hz -1.0 \
  --velocity-sweep-target-hz 3.0 --velocity-sweep-target-hz -3.0 \
  --velocity-sweep-target-hz 5.0 --velocity-sweep-target-hz -5.0
```

Acceptance:

- all points pass sign, motion, and trace-quality checks,
- encoder acquisition errors stay within thresholds,
- no current or gate-driver faults.
