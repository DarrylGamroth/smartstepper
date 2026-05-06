# Archived Documentation

Archived files are retained for context and traceability, but they are not the
current implementation contract.

## Directories

- `aeat_9955_history/`
  - AEAT-9955-specific encoder, alignment, and HIL stabilization history.
  - Do not apply these results to the MT6835 baseline without fresh evidence.
- `completed_refactors/`
  - Completed migration/refactor plans and execution logs.
  - Useful for understanding why code was moved, not for current task scope.
- `historical_hil/`
  - Old manual HIL logs and test reports.
  - Prefer current scripted HIL JSON/log output for new decisions.
- `superseded_plans/`
  - Plans replaced by current docs in `app/docs/` or by the structured plan
    pack in `app/docs/plan/`.

## Rule

If an archived document contradicts a current document, the current document
wins. If the contradiction affects hardware safety or control behavior, run a
fresh HIL test and record the evidence in a current document.
