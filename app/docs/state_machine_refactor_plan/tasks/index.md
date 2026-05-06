# Task Order

Execute in this order unless blocked.

## Phase P00 - Baseline And Transition Map

1. SMF0001 - Record current transition matrix and HIL baseline
2. SMF0002 - Add transition status observability baseline

## Phase P01 - Shared Guards And Vocabulary

3. SMF0101 - Consolidate requested-mode resolver
4. SMF0102 - Introduce operating-mode descriptor table
5. SMF0103 - Split staged mode from run request in shell/API wording

## Phase P02 - Deterministic Transition Results

6. SMF0201 - Add transition result state
7. SMF0202 - Update shell/HIL to verify transition completion
8. SMF0203 - Make commissioning workflows reset requested mode explicitly

## Phase P03 - Setup Workflow Cleanup

9. SMF0301 - Separate boot setup from commissioning identification flow
10. SMF0302 - Convert encoder mapping to explicit setup workflow result
11. SMF0303 - Gate encoder modes on setup result set

## Phase P04 - Runtime Mode Entry Cleanup

12. SMF0401 - Derive ISR feature flags from mode descriptors
13. SMF0402 - Normalize entry/exit reset policies
14. SMF0403 - Add mode transition tests and generated-mode regression HIL

## Phase P05 - Fault And Recovery Hierarchy

15. SMF0501 - Add explicit recovery status and actions
16. SMF0502 - Split fault clear from gate/encoder recovery
17. SMF0503 - Add fault/recovery HIL regression

## Phase P06 - Documentation And Final Gate

18. SMF0601 - Update operator workflow docs
19. SMF0602 - Run full build/unit/HIL gate and archive evidence
