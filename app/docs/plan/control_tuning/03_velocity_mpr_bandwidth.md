# 03 Velocity MPR Bandwidth

## Goal

Validate MPR through the user-facing bandwidth interface instead of raw presets.

## Implementation Expectations

- Preferred shell command remains:

```text
motor velocity mpr bandwidth <hz>
```

- Raw MPR `set` remains available for engineering diagnosis only.
- MPR should use the commissioned mechanical model: `Kt`, `J`, `B`, `Tc`, and current limit.
- MPR should not consume `Ld/Lq` directly; those belong to the inner current loop and decoupling path.
- The velocity-loop update period must be mechanically meaningful. A 20 kHz
  outer-loop update with an 8-sample MPR horizon only predicts `0.4 ms`, which
  is too short for this regulator. Current MT6835 HIL evidence uses
  `motor velocity decimation 20` for a 1 kHz velocity update and 8 ms horizon.
- The bandwidth helper must enable the internal MPR disturbance estimator when
  the active motor model is valid. Without it, the model underestimates the
  breakaway/static disturbance of the hybrid stepper and commands only a few
  milliamps at low speed.
- A zero velocity target with near-zero measured speed must clear MPR dynamic
  bias and output zero Iq, matching the PI zero-target behavior.

## HIL Command

Run the same sweep as PI with `--velocity-sweep-regulator mpr` and a conservative bandwidth first:

```bash
python3 scripts/hil/hil_telnet.py velocity-sweep --host 10.0.0.44 --yes-live-motion \
  --velocity-sweep-commission standard \
  --velocity-sweep-regulator mpr \
  --mpr-bandwidth-hz 0.5 \
  --velocity-sweep-rotations 1.0 \
  --velocity-sweep-iq-limit 0.225 \
  --velocity-sweep-target-hz 0.1 --velocity-sweep-target-hz -0.1 \
  --velocity-sweep-target-hz 0.3 --velocity-sweep-target-hz -0.3 \
  --velocity-sweep-target-hz 0.5 --velocity-sweep-target-hz -0.5 \
  --velocity-sweep-target-hz 1.0 --velocity-sweep-target-hz -1.0 \
  --velocity-sweep-target-hz 3.0 --velocity-sweep-target-hz -3.0 \
  --velocity-sweep-target-hz 5.0 --velocity-sweep-target-hz -5.0
```

Acceptance:

- MPR must be no less safe than PI.
- Tracking can initially be worse than PI, but must not fault or reverse unexpectedly.
- Increase bandwidth only after the conservative sweep passes.

## HIL Update 2026-05-08

Findings:

- With velocity-loop decimation `1`, MPR `horizon=8` predicts only `0.4 ms` and
  under-commands low-speed motion.
- With decimation `20`, `motor velocity mpr bandwidth 5` still under-commanded
  until the internal disturbance estimator was enabled.
- A manual test with `dist_ki=0.000500` tracked `0.5 Hz` at about `0.05..0.07 A`,
  which matches the expected static/detent breakaway behavior.

Implemented fixes:

- Bandwidth-derived MPR now sets a bounded disturbance estimator gain from
  `Kt * Iq_limit`.
- MPR zero-speed hold now resets `iq_cmd_a` and disturbance state to zero.
- MPR bandwidth status/save estimation now inverts the scaled model formula,
  so a 5 Hz command reports about `5.000 Hz` instead of hundreds of Hz.

Evidence:

- Unit: `./tests/run_unit_tests.sh wonderful_goldberg -s chopper.motor_mpr.unit`
  passed `21/21`.
- Build: `west build --build-dir /workspace/build/chopper/smartstepper_v2_mt6835_id`
  passed.
- HIL: `hil_logs/20260508_133349_mpr_bandwidth_distki_hil.log`
  showed `motor velocity mpr bandwidth 5` deriving `dist_ki=0.000495`,
  correct `Bandwidth est: 5.000 Hz`, `0.5 Hz` tracking with clean encoder
  acquisition counters, and zero-target reset clearing `Iq cmd` to `0.00000 A`.
- HIL: `hil_logs/20260508_133436_mpr_bandwidth_bidirectional_hil.log`
  passed `+0.5 Hz` and `-0.5 Hz`, then faulted at `+1.0 Hz` with
  `ENCODER_FAULT/velocity_spike` while transport/parity/CRC/glitch counters
  stayed zero.

Current status:

- MPR is improved and usable for low-speed experimental testing at `+/-0.5 Hz`
  with `velocity decimation 20`.
- MPR is still not a replacement for the validated PI baseline because the
  `+1.0 Hz` step can trigger a velocity-spike fault. Next work should tune the
  MPR acceleration/reference transition or observer spike threshold before
  expanding the MPR speed envelope.
