# Control Tuning Execution Log

| Date | Phase | Status | Evidence |
| --- | --- | --- | --- |
| 2026-05-07 | 01_hil_baseline | in progress | Added plan package. Updating HIL script so custom scenarios run live stop/status postlude and encoder acquisition status is always machine-parseable. |
| 2026-05-07 | 01_hil_baseline | done | `python3 -m unittest scripts/hil/test_hil_telnet_parser.py` passed (27 tests). `python3 scripts/hil/hil_telnet.py custom --host 10.0.0.44 --yes-live-motion --command 'motor state status' --command 'motor encoder acquisition status' --command 'motor fault snapshot status'` passed and parsed acquisition counters: transport/frame/parity/crc/status/glitch all zero. |
| 2026-05-07 | 02/03 setup | done | Updated `velocity-sweep` and `mpr-dob-detent` to run production electrical ID before standard commissioning by default so measured `Ld/Lq` are staged and reapplied. Added `--skip-production-electrical` for same-session reuse. Parser tests passed. |
