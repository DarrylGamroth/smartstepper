# 00 Scope

## In Scope

- MT6835 encoder target using the fast `encoder_rt` path.
- Standard baseline commissioning: current offsets, RoverL bootstrap,
  production electrical ID, and encoder mapping.
- Velocity encoder control using PI and MPR.
- DOB as bounded feedforward residual correction.
- Detent feedforward as an optional commissioned table.
- HIL scripts that provide repeatable command sequences and machine-readable verdicts.

## Out Of Scope

- Non-volatile persistence.
- AEAT-9955 tuning conclusions.
- Nonzero `Id`, MTPA, or reluctance-torque optimization.
- Position-loop optimization beyond verifying it is not regressed by velocity-loop changes.
- Replacing PI with MPR as the default before HIL evidence supports it.

## Current Baseline

Latest MT6835 baseline commissioning with demodulated `Ld/Lq` applied:

- `Rs ~= 2.32 ohm`
- `Ld ~= 4.2 mH`
- `Lq ~= 2.0 mH`
- encoder acquisition errors: zero transport/frame/parity/CRC/status/glitch errors in the last full run

Flux, `Kt`, mechanical `J/B/Tc`, DOB, and detent feedforward are advanced
commissioning steps. They are not prerequisites for baseline velocity PI/MPR
bring-up.

## Required Safety Posture

- Begin from `IDLE`, disarmed, zero current target.
- Disable DOB and detent before PI/MPR baseline tests.
- Restore `motor safety timeout 1000` at the end of live HIL.
- Treat any `OVERCURRENT`, hard fault, watchdog reset, or encoder acquisition error as a blocker.
