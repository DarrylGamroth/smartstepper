# Motor State Machine Refactor Plan Pack

This plan pack is the execution contract for cleaning up the motor state machine,
mode transitions, setup workflows, and fault recovery.

The goal is not to add new control features. The goal is to make setup, running,
error handling, notification, and recovery deterministic enough that encoder
modes, commissioning commands, HIL scripts, and shell commands all observe the
same transition model.

## Read Order

1. `00_scope.md`
2. `01_current_findings.md`
3. `02_target_architecture.md`
4. `validation.md`
5. `tasks/index.md`

## Execution Rules

1. Execute task cards in `tasks/index.md` order unless a task is explicitly blocked.
2. Do not expand scope beyond the active task.
3. For each task, obey `touch_files`, `do_not_touch`, and `constraints`.
4. Run the listed validation before marking a task complete.
5. Append evidence to `execution_log.md` after each completed or blocked task.
6. Commit after each completed task or small task batch with task IDs in the message.
7. Do not preserve confusing shell/API behavior just for backwards compatibility.
   Compatibility is allowed only when it does not weaken the new transition contract.

## Design Principles

1. Lifecycle state, setup workflow, operating mode, and fault state are separate concepts.
2. Shell/API commands request transitions; the state machine reports whether they actually completed.
3. ISR feature flags are derived from a single state/mode contract, not scattered incidental side effects.
4. Encoder-control readiness is checked once through a shared guard path.
5. Fault recovery is explicit: latch, diagnose, recover hardware, recover encoder, then safe idle.
6. HIL scripts validate observed state, not just command echo text.
