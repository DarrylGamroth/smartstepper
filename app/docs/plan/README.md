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

