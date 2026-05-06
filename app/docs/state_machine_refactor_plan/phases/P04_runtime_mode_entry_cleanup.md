# P04 - Runtime Mode Entry Cleanup

Purpose: make ISR feature flags and mode entry/exit reset policies table-driven and testable.

Done when:

1. Mode descriptor table derives ISR feature flags.
2. Entry/exit reset policies are centralized.
3. Generated mode and encoder current mode HIL smoke still pass.
