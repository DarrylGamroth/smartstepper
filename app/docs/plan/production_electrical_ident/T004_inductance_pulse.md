# T004 - Pulse/HFI-Inspired Inductance Measurement

## Approach

Add an explicit pulse-response inductance measurement path inspired by VESC:

1. Stop normal outer-loop motion.
2. Apply short bounded voltage/current perturbations in controlled generated-frame directions.
3. Sample current response at ISR rate.
4. Estimate inductance from `di/dt = (v - R i) / L` over accepted windows.
5. Repeat over multiple electrical angles or d/q axes.

## Notes

The first implementation can use deterministic generated-frame d/q perturbations rather than full VESC six-vector HFI. If that is not repeatable, add a true HFI-style harmonic extraction phase later.
