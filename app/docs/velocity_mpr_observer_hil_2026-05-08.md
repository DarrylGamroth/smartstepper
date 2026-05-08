# Velocity/MPR/Observer HIL Notes - 2026-05-08

Hardware: `smartstepper_v2` with `configs/motor_mt6835_2a.overlay`

Purpose: determine whether current velocity/MPR problems are dominated by encoder/observer quality or by regulator tuning.

## Fixes Applied Before Testing

- `motor commission boot [current_a] [velocity_hz] [cycles] [online|idle]` accepted only three optional shell arguments even though the handler supports four. This prevented the `online` completion argument from running. Fixed shell registration.
- Encoder-mode transitions from generated online modes were rejected because realtime encoder acquisition was active. Generated modes use encoder telemetry, so active acquisition is expected. Fixed transition guard to allow active acquisition from any online submode while preserving mapping, injection, and hard-error checks.

Commit:

```text
24d9528 T-control fix encoder mode entry guards
```

## Generated-Mode Encoder/Observer Precheck

Workflow:

```text
motor state calibrate
motor state mode velocity_generated
motor state online
motor arm
motor current id 0
motor current iq 0.100
motor encoder trace start 20
motor velocity target 0.500
motor encoder trace stop
motor encoder trace summary
```

Result:

```text
Raw delta:    -91749 mdeg, avg -499 mHz
Ctrl delta:    91749 mdeg, avg  499 mHz
Counts:        clean=512 fresh=512 ctrl_en=0 warn=0 err=0 io=0
Encoder errors: transport=0 frame=0 parity=0 crc=0 status=0 glitch=0
```

Interpretation:

- Encoder transport is clean.
- Direction convention is consistent: raw encoder sign is opposite control angle, and control angle is corrected by `encoder-direction-sign = -1`.
- This validates generated-mode encoder observation, not encoder-fed observer operation.

## Boot Mapping

Workflow:

```text
motor commission boot 0.100 0.500 1 online
```

Result after command registration fix:

```text
Encoder mapping result: valid=YES dir=-1 corr=-0.9454 off_mech=1.446 deg off_elec=72.288 deg
residuals: offset=0.4907 rad direction=0.0055 rad motion=364.296 deg samples=400 rejected=0 warn=0 err=0
Encoder mapping applied: sign=-1 commutation_offset=1.4458 deg mechanical
```

Interpretation:

- Runtime boot mapping works and is clean.
- Mapping residual is nonzero but accepted; later work should define a tighter production threshold once persistence and repeated-run statistics are available.

## Velocity Encoder PI Baseline

Setup:

```text
motor outer mode pi
motor velocity pi bandwidth <bw_hz> 1.000 0.225
motor state mode velocity_encoder
motor state online
motor arm
motor velocity target 0.500
```

Results:

| PI bandwidth | Kp A/(rad/s) | Ki A/rad | status measured | trace avg | result |
|---:|---:|---:|---:|---:|---|
| 1 Hz | 0.071620 | 0.001067 | 0.29 Hz | 0.268 Hz | too slow |
| 2 Hz | 0.071620 | 0.004266 | 0.50 Hz | 0.397 Hz | best of tested points |
| 4 Hz | 0.071620 | 0.017065 | 0.43 Hz | 0.467 Hz | acceptable but not clearly better |

All PI points had clean encoder trace:

```text
clean=512 fresh=512 ctrl_en=512 warn=0 err=0 io=0
```

Interpretation:

- Observer/encoder quality is not the current limiting factor on MT6835.
- The 1 Hz PI configuration is too weak because the integrator builds current too slowly.
- The 2 Hz PI bandwidth is a better baseline for this motor than 1 Hz.
- `Kp` is clamped at the low-speed authority floor, so increasing bandwidth mainly increases `Ki`.

## Velocity Encoder MPR Baseline

Setup:

```text
motor velocity pi bandwidth 2.000 1.000 0.225
motor velocity mpr bandwidth <bw_hz>
motor outer mode mpr
motor state mode velocity_encoder
motor state online
motor arm
motor velocity target 0.500
```

Results:

| MPR bandwidth command | q_speed | r_delta_iq | dIq A/sample | status measured | trace avg | result |
|---:|---:|---:|---:|---:|---:|---|
| 0.5 Hz | 0.003000 | 83.333336 | 0.000675 | 0.37 Hz | 0.383 Hz | stable, low |
| 1.0 Hz | 0.003000 | 83.333336 | 0.000675 | 0.42 Hz | 0.390 Hz | stable, low |
| 2.0 Hz | 0.003000 | 83.333336 | 0.000675 | 0.26 Hz | 0.397 Hz | not improved |

All MPR points reported `model=active clamped`, and all used identical effective MPR parameters.

Interpretation:

- The MPR bandwidth command is currently not useful over this range because `q_speed` clamps to `MOTOR_MPR_VELOCITY_BW_Q_MIN`.
- MPR is stable but under-commanding torque compared with the PI baseline.
- Next work should tune the MPR bandwidth-to-parameter mapping, especially `q_speed`, `r_delta_iq`, and `max_delta_iq_a`, using the 2 Hz PI case as the baseline.

## Current Conclusion

For MT6835 hardware:

1. Encoder transport is clean.
2. Angle observer is producing sane encoder-fed position/velocity during `velocity_encoder`.
3. The immediate limitation is regulator tuning, not feedback quality.
4. Use velocity PI at about `2 Hz`, `zeta=1.0`, `Iq limit=0.225 A` as the current baseline.
5. Fix MPR bandwidth mapping before evaluating DOB or detent feedforward as mechanical-ID aids.

## Next Recommended Work

1. Update the default commissioned velocity PI bandwidth from `1 Hz`-class behavior to the verified `2 Hz` baseline for this motor profile.
2. Rework `motor_mpr_velocity_config_from_bandwidth()` so `motor velocity mpr bandwidth <hz>` produces distinct, useful parameter sets over `0.5..4 Hz`.
3. Add an automated PI-vs-MPR HIL sweep report that records status velocity and trace-average velocity separately.
4. Only after MPR tracks the PI baseline, re-test DOB and electrical ripple/detent feedforward.
