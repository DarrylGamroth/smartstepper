# Current Findings

## Confirmed Behavior

1. Generated-sweep encoder mapping runs during `motor commission run confirm apply` before encoder-control validation.
2. `motor state mode <mode>` stages `requested_online_mode` when the system is not online; it does not enter `ONLINE`.
3. `motor state online` is required after staging a mode when the current state is `IDLE`.
4. Generated velocity mode can produce phase current and motion after gate-driver reset.
5. Encoder current mode can produce phase current and motion in at least some runs, but behavior is not yet deterministic enough for detent/velocity workflows.
6. Detent capture depends on successful `ONLINE_VELOCITY_ENCODER` entry and stable velocity motion.

## Design Smells

1. `requested_online_mode` persists across workflows and can influence later commands unexpectedly.
2. There are duplicate requested-mode resolver implementations with different guard behavior.
3. Lifecycle states, setup states, and operating modes share one enum and one shell vocabulary.
4. Some commands report that a request was posted, not that the state transition completed.
5. State-machine entry/exit handlers directly mutate ISR feature flags in scattered places.
6. Fault clear goes mostly straight to `IDLE`; recovery of gate driver, encoder protocol, acquisition counters, and stale mode requests is not expressed as a state flow.
7. Calibration and commissioning share a hierarchy even though boot setup and explicit identification have different acceptance and recovery semantics.
8. HIL scripts can pass through command echo text while missing delayed transition rejection/fallback.

## Immediate Risks

1. A commissioning command can leave the system in a staged mode that surprises the next command.
2. A command may time out waiting for a mode without exposing which guard failed.
3. Encoder modes can be attempted without a clean, explicit setup workflow.
4. Fault recovery can clear software state while a hardware or acquisition subsystem still needs recovery.
5. Refactors to ISR feature flags can silently regress generated mode or encoder mode.
