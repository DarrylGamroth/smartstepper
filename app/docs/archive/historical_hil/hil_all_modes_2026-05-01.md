# HIL All-Mode Test - 2026-05-01

## Setup

- Hardware profile: `configs/motor_aeat9955_067a.overlay`
- Build directory: `/workspace/build/chopper/smartstepper_v2_hil_shell`
- Build/flash command:

```bash
podman exec wonderful_goldberg bash -lc 'cd /workspace && west build -b smartstepper_v2/stm32h743xx /workspace/chopper/app -d /workspace/build/chopper/smartstepper_v2_hil_shell -S serial-shell -S serial-console -- -DDTC_OVERLAY_FILE="boards/smartstepper_v2.overlay;configs/motor_aeat9955_067a.overlay" -DEXTRA_CONF_FILE="hil_shell.conf;logging.conf" && west flash -d /workspace/build/chopper/smartstepper_v2_hil_shell --runner jlink --dev-id 10.0.0.70 --dev-id-type ip'
```

- Network shell: `10.0.0.171:23`
- Serial shell: `/dev/serial/by-id/usb-FTDI_TTL232R-3V3_FTE3B04Y-if00-port0` at `115200`
- Initial network-shell transcript: `/tmp/chopper_hil_all_modes_20260501_133444.log`

## Result Matrix

| Mode | Result | Evidence |
| --- | --- | --- |
| `velocity_generated` | PASS | Entered `ONLINE_VELOCITY_GENERATED`, policy valid, encoder control disabled, generated angle source, current loop enabled. Ran `Iq=0.120 A`, `velocity target 3 Hz`, no motor error. |
| `position_generated` | PARTIAL PASS | Entered `ONLINE_POSITION_GENERATED`, policy valid, encoder control disabled, generated position source. A `45 deg / 800 ms` profile completed with `Quintic: COMPLETE`, no motor error. The network shell/target later stopped during `motor info live`, requiring J-Link reset. |
| `current_encoder` | FAIL | Entered `ONLINE_CURRENT_ENCODER`, then shell stopped responding immediately after `motor arm` / `motor current iq 0.03`. Required J-Link reset. No clean `ERROR_ENCODER_FAULT` was reported before loss of shell. |
| `velocity_encoder` | FAIL | With requested mode `velocity_encoder`, prepare path stalled after offset-measure timeout and before reaching ONLINE. Shell stopped responding and network ping failed. Required J-Link reset. |
| `position_encoder` | FAIL | With requested mode `position_encoder`, prepare path stalled during `OFFSET_MEAS` before completion. Shell stopped responding and network ping failed. Required J-Link reset. |

## Notable Observations

1. AEAT alarm byte before motion was clean:

```text
Raw status: 0xC0
MHI: CLEAR
MLO: CLEAR
```

2. Generated-angle policies look correct:

```text
Feedback source:  generated_reference
Angle source:     generated
Encoder control:  DISABLED
Encoder required: NO
Policy valid:     YES
```

3. Dual-polarity alignment repeatedly reported invalid small deltas:

```text
ALIGN dual-polarity solve: |delta|=0.06..0.63 deg expected=3.60 deg valid=no
ALIGN dual-polarity mismatch/invalid ... using +Id fallback offset
```

4. Encoder-required modes did not degrade into a clean `ERROR_ENCODER_FAULT` path. They stalled hard enough that the serial shell stopped responding and the network interface stopped replying to ping.

## Follow-Up

1. Treat generated-angle modes as HIL-smoke passing for the current taxonomy changes.
2. Investigate encoder-mode hard stalls before attempting closed-loop tuning.
3. Add a minimal encoder-mode watchdog/fault path that can trip without requiring shell/network responsiveness.
4. Re-run `current_encoder` with extra logs around encoder RTIO request/collect, `motor_control_step_read_encoder()`, and state transitions.
5. Consider a J-Link halt/backtrace capture immediately after the stall to determine whether this is a hard fault, live lock, or interrupt starvation.

## J-Link Fault Capture

After reproducing a `current_encoder` stall, the target was halted via J-Link without reset.

J-Link register log:

```text
/tmp/jlink_current_encoder_stall_20260501_135009.log
```

Observed state:

```text
IPSR = 003 (HardFault)
PC   = 0x08016E62 -> arm_m_must_switch()
LR   = 0x08016B0D -> arm_m_exc_exit()
```

A second halt/dump showed exception/NMI state with execution interrupted around the encoder
feedback path:

```text
/tmp/jlink_stack_current_encoder_stall_20260501_135029.log
/tmp/jlink_encoder_ctx_20260501_135116.log
PC = 0x080358D6 -> motor_encoder_feedback_update()
                  modules/motor_core/src/observers/encoder_feedback.c:134
```

The mapped instruction at `encoder_feedback.c:134` is the stale-count halfword load:

```c
} else if (*ctx->position_stale_count < UINT16_MAX) {
```

The `position_stale_count` pointer in the encoder feedback context pointed at valid SRAM in
`motor_params.live`, so the captured PC is more likely the interrupted encoder-feedback path
than proof that this exact load is invalid. The important finding is that the encoder-mode
failure is a CPU exception/hard fault, not a clean motor overcurrent fault path.

Current injection evidence:

```text
ALIGN traj plan: Id start=0.0000A target=0.0670A max_delta=0.000042A/tick steps=1600
```

This confirms alignment current is ramped through `traj_Id`; the observed encoder-mode failure
does not look like an intentional alignment current step.
