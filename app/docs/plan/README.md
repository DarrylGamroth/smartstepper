# Motor Core Refactor Plan Pack

This directory is the execution pack for LLM-driven implementation of the
`motor_core` decomposition.

## Structure

1. `00_scope.md`: goals, non-goals, constraints, glossary.
2. `01_architecture_target.md`: target module map and ISR contract.
3. `phases/`: one file per phase with strict boundaries and acceptance gates.
4. `tasks/`: atomic YAML task cards for implementation.
5. `validation.md`: required build/test/HIL commands.
6. `execution_log.md`: append-only progress and evidence log.

## Execution Rules

1. Execute tasks in order by `id` unless explicitly unblocked/reordered.
2. Do not touch files outside each task `touch_files` list.
3. If blocked, record in `execution_log.md` and stop that task.
4. Run listed validation for every task before marking done.
5. Commit after each completed task or small batch with task IDs in message.

## Task States

1. `pending`
2. `in_progress`
3. `done`
4. `blocked`

## LLM Invocation

For future LLM runs, treat this directory as the source of truth.

Read first:

1. `app/docs/plan/README.md`
2. `app/docs/plan/00_scope.md`
3. `app/docs/plan/01_architecture_target.md`
4. `app/docs/plan/tasks/index.md`
5. `app/docs/plan/validation.md`

Then execute tasks in `app/docs/plan/tasks/index.md` order.
For each `Txxxx.yaml` task card:

1. Follow `touch_files`, `do_not_touch`, and `constraints`.
2. Run listed `validation` commands.
3. Update `app/docs/plan/execution_log.md`.
4. Commit with task ID(s) in message.
5. Do not expand scope beyond current task unless needed to pass validation.

Seed commit for this plan pack: `f22b4b4`.
