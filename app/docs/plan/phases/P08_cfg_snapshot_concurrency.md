# Phase P08 - Config Snapshot Concurrency Hardening

## Objective

Use coherent snapshot publication/consumption between state thread and ISR.

## Prerequisites

1. P07 complete.

## Touch Files

1. runtime snapshot module files
2. state publication sites
3. ISR consumption sites
4. concurrency-focused unit tests

## Do Not Touch

1. Algorithm math internals.

## Tasks

1. Replace split reads of state/features with coherent snapshot.
2. Include trigger source/settings in same snapshot.
3. Add tests for mixed-epoch prevention.

## Acceptance

1. No mixed epoch behavior in tests/stress.
2. Command/state transitions remain deterministic.
3. Build + unit tests pass.

