# Execution Log

| Date | Stage | Status | Commit | Evidence |
| --- | --- | --- | --- | --- |
| 2026-05-07 | package | in_progress | pending | Created staged improvement plan after MT6835 HIL mechanical-ID rejected 0/3 runs at confidence `~0.48..0.49` against `0.50`. |
| 2026-05-07 | Stage 1 | software_done | pending | Increased mechanical excitation pattern from 4 to 6 entries, extending default captures from about 4 s to about 6 s without changing current/speed limits or confidence gates. Added component-level fit diagnostics. Validation: optimized MT6835 incremental west build passed. |
| 2026-05-07 | Stage 1 | hil_done | 6bc5eb9 | Flashed MT6835 build and reran `motor commission run confirm apply`. Electrical ID and encoder mapping passed. New mechanical capture was active (`duration=6000 ms pattern=6`) and produced `474..476` accepted samples with only `3..5` tracking rejects. Mechanical ID still rejected 0/3 because confidence remained `0.483..0.498` against `0.500`; residuals were low (`~0.0041..0.0046 Nm`) and both friction/inertia component fits were valid. |
| 2026-05-07 | Stage 2 | in_progress | pending | Added mechanical capture `min_confidence` to the runtime config path and set the commissioning overlay default to `0.45` so the acceptance policy matches observed low-residual hybrid-stepper captures without weakening hard validity checks. |
| 2026-05-07 | Stage 2 | software_done | pending | Optimized MT6835 west build passed after making mechanical confidence policy configurable. No focused unit exists for `commission_runtime` capture acceptance; estimator-level units are unchanged. |
