# HIL Motion-Control Test Log - 2026-05-01

Target:

- Board: `smartstepper_v2`
- Encoder: AEAT-9955
- Shell: telnet at `10.0.0.171:23`
- Motor control mode under test: generated-angle motion with FOC current actuation
- Encoder use: telemetry only, not commutation feedback

## Logging Setup

Telnet shell was used for command/control. UART motor-state logs were disabled to avoid serial shell congestion:

```text
log backend shell_uart_backend disable motor_states
log backend shell_uart_backend disable motor_api
```

Logger health stayed clean:

```text
log mem
```

Observed:

- Capacity: `4092 bytes`
- Current use: `0 bytes`
- Maximum use: `992 bytes`

## Baseline Health

Commands:

```text
motor state status
motor encoder alarm
sensor get aeat9955@0
```

Observed:

- State: `IDLE`
- Error: `NONE`
- Armed: `NO`
- Calibration complete: `YES`
- Encoder direction sign: `-1`
- AEAT alarm raw status: `0xC0`
- MHI: `CLEAR`
- MLO: `CLEAR`

## Profile-Open Single Move

Commands:

```text
motor state clear_error
motor disarm
motor state idle
motor safety timeout 0
motor state mode profile_open
motor state online
motor arm
motor current id 0
motor current iq 0.15
sensor get aeat9955@0
motor profile move 135 0 2000
motor profile status
motor info live
sensor get aeat9955@0
motor state status
```

Observed:

- Profile planned: `45.00 deg -> 135.00 deg in 2000.0 ms`
- Profile completed.
- Policy remained generated-angle:
  - Motion source: `profile`
  - Feedback source: `generated_model`
  - Angle source: `generated`
  - Encoder read: `DISABLED`
  - Encoder required: `NO`
  - Current loop: `ENABLED`
- No faults.
- Iq tracked: `Iq reference = 0.150 A`, `Iq measured ~= 0.152 A`.

This first before/after sensor check aliased and did not prove physical movement by itself.

## Velocity-Open Encoder Telemetry Check

Commands:

```text
motor state mode velocity_open
motor state online
motor current iq 0.15
motor velocity target 1.3
sensor get aeat9955@0
sensor get aeat9955@0
sensor get aeat9955@0
sensor get aeat9955@0
motor velocity target 0
motor current iq 0
```

Observed sensor values changed while the motor was commanded in velocity-open:

- `-0.120483`
- `-0.120483`
- `0.983184`
- `-0.052848`
- `-0.748268`

Conclusion:

- The AEAT path can be used as coarse motion telemetry.
- Single before/after samples can alias during whole-number rotations, so use repeated samples or non-integer timing.

## Profile-Open Move With During-Move Encoder Samples

Commands:

```text
motor state mode profile_open
motor state online
motor info live
sensor get aeat9955@0
motor current iq 0.15
motor profile move 306 0 1000
sensor get aeat9955@0
sensor get aeat9955@0
sensor get aeat9955@0
motor profile status
motor info live
sensor get aeat9955@0
```

Observed:

- Profile planned: `246.26 deg -> 306.00 deg in 1000.0 ms`
- Profile completed.
- Generated reference ended at `306.00 deg`.
- Sensor samples changed from approximately `-0.927932` to `0.7522`.
- No faults.
- Iq tracked: `Iq reference = 0.150 A`, `Iq measured ~= 0.143 A`.

Conclusion:

- Profile-open generated-angle motion can physically move the motor.
- Encoder can be used as measurement-only telemetry for beginning/end or during-move checks.

## Timer-Driven Profile Sequence

Commands:

```text
sensor get aeat9955@0
motor profile seq clear
motor profile seq add 0
motor profile seq add 45
motor profile seq add 90
motor profile seq config 1000 500 0 0
motor profile seq list
motor profile seq start
motor profile seq status
motor profile status
motor info live
sensor get aeat9955@0
```

Observed:

- Sequence points accepted: `0`, `45`, `90` deg.
- Config accepted: period `1000 ms`, move `500 ms`, end velocity `0 Hz`, loop disabled.
- Sequence completed.
- Dropped ticks: `0`.
- Final profile segment: `45.00 -> 90.00 deg`.
- Live generated mechanical angle: `450.0 deg`, equivalent wrapped position `90 deg`.
- Sensor changed after the sequence, with one observed final value around `-0.047912`.
- No faults.

## Final State

Commands:

```text
motor encoder alarm
motor info stats
log mem
motor profile seq stop
motor current iq 0
motor disarm
motor state idle
motor safety timeout 1000
motor state status
```

Observed:

- Final state: `IDLE`
- Error: `NONE`
- Armed: `NO`
- Timeout restored to `1000 ms`
- Timeout latch: `CLEAR`
- Timeout count: `0`
- Encoder alarm raw status: `0xC0`
- MHI/MLO: clear
- Encoder fault/warn/error counters: `0`
- ISR max cycles: `8684`
- ISR avg cycles reported after test: `12`

## Conclusions

1. Telnet shell is suitable for HIL command/control and avoids the serial logging congestion seen earlier.
2. Generated-angle motion control works independently of encoder feedback in both `velocity_open` and `profile_open`.
3. The current loop tracks the commanded `Iq` during generated-angle motion.
4. The AEAT-9955 can be used as coarse measurement-only telemetry for motion validation.
5. Single encoder before/after reads can alias; repeated samples or deterministic non-integer timing are required for reliable movement evidence.
6. Timer-driven profile sequence execution completes without dropped ticks in this test.
7. The tested motion-control layer is usable before revisiting closed-loop actuation/commutation.

## Recommended Next HIL Step

Add a repeatable shell workflow or small host-side script that:

1. opens one persistent telnet session,
2. runs a profile-open move,
3. takes multiple AEAT samples before, during, and after,
4. computes coarse displacement while accounting for wrap,
5. records state/fault/log-memory counters.

## Encoder Recorder Workflow

Use the ISR recorder instead of repeated `sensor get` when high-resolution
encoder telemetry is needed. In generated-angle modes, `trace start` forces
encoder sampling and records raw encoder samples without making the encoder a
control dependency:

```text
motor encoder trace clear
motor encoder trace start 100
motor profile move <target_deg> 0 <duration_ms>
motor encoder trace stop
motor encoder trace summary
motor encoder trace dump 0 32
motor encoder trace dump 32 32
```

Use `capture start` only when comparing the control/reference path is required:

```text
motor encoder capture clear
motor encoder capture start 100
motor profile move <target_deg> 0 <duration_ms>
motor encoder capture stop
motor encoder capture summary
motor encoder capture compare 96 gen
```

Raw trace is the preferred first diagnostic for encoder transport quality and
beginning/end position, especially when generated-angle motion is being tested.

At a 20 kHz control loop, 512 samples cover:

```text
duration_s = 512 * decimation / 20000
```

Examples:

- `decimation=40`: about `1.024 s`
- `decimation=100`: about `2.56 s`
- `decimation=200`: about `5.12 s`

Use `summary` first. It prints first/last angle, wrap-aware delta, estimated
average velocity, min/max, warning/error/io counts, and overrun count. Use
chunked dumps only when raw rows are needed. Dump commands intentionally cap
each response to 32 rows to avoid congesting the telnet/shell backend.

## Encoder Recorder Regression Check

After adding chunked dumps and summary commands, the recorder path was checked
again over telnet on AEAT-9955 hardware.

Idle raw trace:

```text
motor encoder trace clear
motor encoder trace start 100
motor encoder trace stop
motor encoder trace summary
motor encoder trace dump 0 8
```

Result:

- Stored 243 raw samples with no overrun.
- 242 clean samples after ignoring the initial non-fresh drained sample.
- Raw angle stayed fixed at about `88.266 deg`.
- Clean raw delta was `0.000 deg`.
- Warning/error/io counts were all zero.
- Telnet shell remained responsive.

Profile-open raw trace with zero current:

```text
motor state clear_error
motor safety timeout 0
motor state mode profile_open
motor state online
motor arm
motor current iq 0
motor encoder trace clear
motor encoder trace start 100
motor profile move 270 0 1000
motor encoder trace stop
motor encoder trace summary
```

Result:

- Stored 385 raw samples with no overrun.
- Raw encoder angle stayed fixed because current was zero.
- Telnet shell remained responsive.

Profile-open capture plus raw trace with zero current:

```text
motor encoder capture clear
motor encoder trace clear
motor encoder capture start 100
motor encoder trace start 100
motor profile move 260 0 1000
motor encoder capture stop
motor encoder trace stop
motor encoder trace summary
motor encoder capture summary
```

Result:

- Raw trace stored 426 samples with no overrun.
- Capture stored 427 samples with no overrun.
- Generated profile moved `270.000 -> 260.000 deg`.
- Raw encoder stayed fixed, as expected with zero current.
- `motor encoder trace dump 64` correctly rejected the large response.
- Telnet shell remained responsive.

Conclusion:

- The old shell stop was not reproduced with raw trace alone or capture+trace
  while current was zero.
- The remaining suspect is the combination of nonzero current/actuation with
  profile execution, fault handling, or encoder RTIO pressure under switching
  noise.
- For FOC encoder debugging, start with raw trace only. Add capture/compare only
  after raw transport quality is verified.

Nonzero-current velocity-open raw trace:

```text
motor state clear_error
motor safety timeout 0
motor state mode velocity_open
motor state online
motor arm
motor current id 0
motor current iq 0.12
motor velocity target 0
motor encoder trace clear
motor encoder trace start 200
motor velocity target 2
motor velocity target 0
motor encoder trace stop
motor encoder trace summary
motor current iq 0
motor disarm
motor state idle
motor safety timeout 1000
```

Result:

- Raw trace stored 183 samples with no overrun.
- 182 samples were clean; warning/error/io counts were all zero.
- Raw encoder delta was `-874.064 deg`, average `-1.3414 Hz`.
- Control-sign corrected delta was `+874.064 deg`, average `+1.3414 Hz`.
- Shell stayed responsive and no motor state error was reported.

Conclusion:

- Raw trace telemetry is usable during nonzero-current generated-angle motion.
- Use decimation based on capture duration. At 20 kHz, `decimation=200` covers
  about `5.12 s` for a full 512-sample buffer.
