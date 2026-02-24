# Phase P10 - Optional RAM Placement and Micro-Optimization

## Objective

Apply optional placement and micro-optimizations after architecture stabilizes.

## Prerequisites

1. P09 complete.

## Touch Files

1. section-placement config/macros
2. hot function attributes
3. stage timing counter implementations

## Do Not Touch

1. Module boundaries and contracts.

## Tasks

1. Add optional section placement controls.
2. Add per-stage min/max/avg timing metrics.
3. Tune inline/layout only where measured benefit exists.

## Acceptance

1. Deterministic jitter budget maintained or improved.
2. No functional regressions.
3. Builds/tests/HIL smoke pass.

