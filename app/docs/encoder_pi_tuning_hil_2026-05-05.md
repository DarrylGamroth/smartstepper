# Encoder PI Tuning HIL Evidence

Date: 2026-05-05
Hardware profile: `configs/motor_mt6835_2a.overlay`
Interface: Telnet shell at `10.0.0.44`

## Preconditions

- Firmware built and flashed with MT6835 profile.
- Standard slow commissioning had completed successfully and staged auto-tuned defaults.
- Staged auto tune applied with:

```text
motor commission auto apply
motor outer mode pi
```

Commissioned/model-based gains after apply:

```text
Velocity PI:
  Kp:       0.027595 A/(rad/s)
  Ki:       1.099625 A/rad
  Iq limit: 0.225000 A

Position PI:
  Kp:       25.132742 (rad/s)/rad
  Ki:       157.913681 (rad/s^2)/rad
```

## Current Encoder Smoke Test

`0.035 A` did not overcome detent/friction. `0.08 A` produced clean encoder-commutated motion.

```text
motor state mode current_encoder
motor state online
motor encoder capture start 20
motor current iq 0.08
motor current iq 0
motor encoder capture summary
```

Result:

```text
Angle delta:  585.074 deg, avg 3.1804 Hz
Counts:       clean=512 fresh=512 ctrl_en=512 warn=0 err=0 compare=512
Status bits:  or=0x00 and=0x00 first=0x00 last=0x00
State: ONLINE_CURRENT_ENCODER
Error: NONE
```

Conclusion: encoder commutation and current loop are usable. A practical current smoke threshold on this motor should be above `0.035 A`; `0.08 A` worked.

## Velocity PI Tests

### 3 Hz Bandwidth

```text
motor velocity pi bandwidth 3 1.0
motor state mode velocity_encoder
motor velocity target 0.5
motor encoder capture summary
```

Result:

```text
Angle delta:  99.202 deg, avg 0.5393 Hz
Counts:       clean=512 fresh=512 ctrl_en=512 warn=0 err=0 compare=512
State: ONLINE_VELOCITY_ENCODER
Error: NONE
```

At `1.0 Hz` target:

```text
Angle delta:  182.960 deg, avg 0.9946 Hz
Counts:       clean=512 fresh=512 ctrl_en=512 warn=0 err=0 compare=512
Error: NONE
```

### 5 Hz Bandwidth

```text
motor velocity pi bandwidth 5 1.0
motor velocity target 1.0
```

Result:

```text
Angle delta:  183.640 deg, avg 0.9983 Hz
Counts:       clean=512 fresh=512 ctrl_en=512 warn=0 err=0 compare=512
Error: NONE
```

At `2.0 Hz` target:

```text
Angle delta:  368.336 deg, avg 2.0023 Hz
Counts:       clean=512 fresh=512 ctrl_en=512 warn=0 err=0 compare=512
Error: NONE
```

### 8 Hz Bandwidth

At `2.0 Hz` target:

```text
Angle delta:  367.825 deg, avg 1.9995 Hz
Counts:       clean=512 fresh=512 ctrl_en=512 warn=0 err=0 compare=512
Error: NONE
```

### 10 Hz Bandwidth

At `2.0 Hz` target:

```text
Angle delta:  368.113 deg, avg 2.0011 Hz
Counts:       clean=512 fresh=512 ctrl_en=512 warn=0 err=0 compare=512
Error: NONE
```

Conclusion: the commissioned 10 Hz velocity PI tuning is acceptable for initial use on this motor.

## Position PI Tests

Position loop was tested after velocity PI was stable. The position command uses bounded profile planning.

### 1 Hz Bandwidth

```text
motor position pi bandwidth 1.0 1.0
motor commission validate position 5 1000
```

Result:

```text
Position encoder validation complete: pos=111.960 deg vel=0.000 Hz Iq=0.0000 A
Error: NONE
```

At `10 deg`:

```text
Position encoder validation complete: pos=111.887 deg vel=-0.000 Hz Iq=-0.0008 A
Error: NONE
```

### 2 Hz Bandwidth

```text
motor position pi bandwidth 2.0 1.0
motor commission validate position 10 1000
```

Result:

```text
Position encoder validation complete: pos=111.981 deg vel=0.000 Hz Iq=-0.0024 A
Error: NONE
```

At `20 deg`:

```text
Position encoder validation complete: pos=111.974 deg vel=0.003 Hz Iq=0.0207 A
Error: NONE
```

Conclusion: the commissioned 2 Hz position PI tuning is acceptable for initial use on this motor.

## Current Recommended PI Tuning

```text
motor commission run slow apply
motor outer mode pi
motor velocity pi bandwidth 10 1.0
motor position pi bandwidth 2.0 1.0
```

Equivalent active gains observed on hardware:

```text
Velocity PI:
  Kp:       0.027595 A/(rad/s)
  Ki:       1.099625 A/rad
  Iq limit: 0.225000 A

Position PI:
  Kp:       25.132742 (rad/s)/rad
  Ki:       157.913681 (rad/s^2)/rad
```

## Notes / Follow-Up

- `motor commission validate current 0.035 160` is too weak for this motor; use at least `0.08 A` for current smoke validation or update the default.
- `motor commission validate current` failed once when starting from a generated mode because encoder acquisition was temporarily active/busy. Manual entry into `current_encoder` from `IDLE` worked. This command path should be hardened later.
- `motor state status` still printed `Online mode: ONLINE_CURRENT_ENCODER` while the active state was `ONLINE_VELOCITY_ENCODER` during velocity tests. The control policy was correct, but the displayed online-mode field appears stale and should be fixed separately.
