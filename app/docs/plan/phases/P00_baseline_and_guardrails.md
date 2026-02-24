# Phase P00 - Baseline and Guardrails

## Objective

Establish measurable baselines and guardrails before structural refactoring.

## Prerequisites

1. `00_scope.md` and `01_architecture_target.md` accepted.

## Touch Files

1. `app/docs/plan/execution_log.md`
2. `app/docs/plan/validation.md`
3. test files needed for baseline assertions

## Do Not Touch

1. ISR control logic implementation files.

## Tasks

1. Add baseline metric collection procedure for ISR cycles and stack watermark.
2. Add compile-time `sizeof` guard checks for hot structs.
3. Add/verify regression tests for encoder quality gating, interlocks, and fault posting order.

## Guardrail Checklist

Before entering P01, confirm all checks below are explicitly covered:

1. Compile-time size guards:
  - `BUILD_ASSERT`/`STATIC_ASSERT` for ISR hot structs in config/runtime headers.
  - guards for fixed-size buffers used in ISR path.
2. Regression suites:
  - encoder quality and frame-gating regressions.
  - interlock ordering regressions.
  - fault posting/latch ordering regressions.
3. Baseline evidence:
  - ISR max/avg cycle baseline captured from `motor info stats`.
  - stack watermark baseline captured via `kernel thread stacks` (or logged unavailable fallback).
4. Documentation traceability:
  - baseline procedure is present in `app/docs/plan/validation.md`.
  - baseline template is present in `app/docs/plan/execution_log.md`.

## Acceptance

1. Baseline metrics captured and logged.
2. Guard checks compile.
3. Full unit suite passes.
4. Firmware builds pass for both targets.
