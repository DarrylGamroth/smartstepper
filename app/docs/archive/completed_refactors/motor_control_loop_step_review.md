# `motor_control_loop_step` Review

Date: 2026-02-24  
File: `app/src/motor_control_loop.c`

## Findings

1. High: `state_for_isr` and `feature_flags` are sampled independently and can be incoherent for one ISR cycle.
   Evidence:
   `app/src/motor_control_loop.c:258` reads `params->state_for_isr`; `app/src/motor_control_loop.c:259` reads `feature_flags`.
   Publication is also split: `app/src/motor_states.c:944` writes `state_for_isr`, then `app/src/motor_states.c:945` writes `feature_flags`.
   Impact:
   A mixed old/new pair can transiently execute the wrong gating combination (state checks from one epoch with feature bits from another), especially during mode transitions.
   Recommendation:
   Publish/consume these as one coherent snapshot (single packed atomic, seqlock-style versioning, or IRQ-lock around paired read/write).

2. Medium: ADC channel count is ignored, but the code dereferences fixed indices.
   Evidence:
   `app/src/motor_control_loop.c:253` marks `count` unused, while `values[...]` is read at `app/src/motor_control_loop.c:595`, `app/src/motor_control_loop.c:596`, and `app/src/motor_control_loop.c:597`.
   Impact:
   If ADC callback configuration drifts (or misreports `count`), this can read beyond valid samples.
   Recommendation:
   Validate `count` against required indices and fail safe (`motor_post_error_with_snapshot(..., ERROR_HARDWARE_BREAK)` or equivalent) when insufficient.

3. Medium: Braking decision uses previous-cycle `Iq_ref` instead of the current-cycle command.
   Evidence:
   `app/src/motor_control_loop.c:1011` passes `params->Iq_ref_A` into `foc_inputs.braking_iq_ref_a`, but current-cycle `Iq_ref_A` is only committed at `app/src/motor_control_loop.c:1060`.
   Impact:
   Dynamic braking sign detection can be delayed by one cycle at torque-direction transitions.
   Recommendation:
   Pass local `Iq_ref_A` into `foc_inputs.braking_iq_ref_a`.

4. Low: Encoder capture metadata can mislabel the control input source.
   Evidence:
   `app/src/motor_control_loop.c:516` sets `capture_input_source` to encoder whenever `encoder_sample_available` is true, even if active control source is generated/propagated.
   Impact:
   Capture logs can be misleading during debugging and post-fault analysis.
   Recommendation:
   Store actual control source (`encoder_input_source`) and keep encoder availability/freshness as separate fields (already present).

## Open Questions (Resolved)

1. Braking lag intent: Not intentional.  
   Action: fixed to use current-cycle `Iq_ref_A` in `motor_control_loop_step`.
2. ADC sample count contract: ADC callback always supplies all required channels.  
   Action: documented assumption; no defensive channel-count guard added in this pass.

## Review Scope

Static code review only. No functional changes were applied in this review.
