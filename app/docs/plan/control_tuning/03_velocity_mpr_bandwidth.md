# 03 Velocity MPR Bandwidth

## Goal

Validate MPR through the user-facing bandwidth interface instead of raw presets.

## Implementation Expectations

- Preferred shell command remains:

```text
motor velocity mpr bandwidth <hz>
```

- Raw MPR `set` remains available for engineering diagnosis only.
- MPR should use the commissioned mechanical model: `Kt`, `J`, `B`, `Tc`, and current limit.
- MPR should not consume `Ld/Lq` directly; those belong to the inner current loop and decoupling path.

## HIL Command

Run the same sweep as PI with `--velocity-sweep-regulator mpr` and a conservative bandwidth first:

```bash
python3 scripts/hil/hil_telnet.py velocity-sweep --host 10.0.0.44 --yes-live-motion \
  --velocity-sweep-commission standard \
  --velocity-sweep-regulator mpr \
  --mpr-bandwidth-hz 0.5 \
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

- MPR must be no less safe than PI.
- Tracking can initially be worse than PI, but must not fault or reverse unexpectedly.
- Increase bandwidth only after the conservative sweep passes.
