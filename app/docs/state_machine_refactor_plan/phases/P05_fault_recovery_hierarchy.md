# P05 - Fault And Recovery Hierarchy

Purpose: turn clear-error into explicit recovery semantics.

Done when:

1. Fault status and recovery status are visible.
2. Gate-driver recovery and encoder recovery are explicit actions.
3. `clear_error` cannot mask unrecovered hardware/acquisition conditions.
