##  Software Checklist
- [ ] Multi-channel PWM
    - [ ] Master/slave
    - [ ] Chaining
    - [ ] Triggers
- [ ] ADC
    - [ ] Injected channels
    - [ ] Trigger
    - [ ] Direct zero-latency interrupt
    - *NOTE* - For inline sensing the sensed current is direction dependent
- [ ] Space-Vector Modulation
    - [ ] Input V_d, V_q, output 4-channel PWM
    - [ ] Braking mode?
        - Only modulate low-side switches while controlling braking current
- [ ] CMSIS
    - Park
    - Sin/Cos
    - PID
- [ ] PI controller
    - CMSIS PID controller is a series controller without saturation or anti-windup
```c
__STATIC_FORCEINLINE q31_t arm_pi_q31(
    arm_pid_instance_q31 * S,
    q31_t in)
{
    q63_t acc;
    q31_t out;

    // acc = A0 * x[n]
    acc = (q63_t) S->A0 * in;

    // acc += A1 * x[n-1]
    acc += (q63_t) S->A1 * S->state[0];

    // convert output to 1.31 format to add y[n-1]
    out = (q31_t) (acc >> 31U);

    // out += y[n-1]
    out += S->state[1];

    // Update state: only previous input and output
    S->state[0] = in;
    S->state[1] = out;

    return out;
}
```
- [ ] Trajectory/ramp
- [ ] First-order filter
    - Filters for ADC values
    - Add later if necessary
- [ ] User parameter struct
- [ ] MTPA
- [ ] Angle generator
- [ ] Velocity observer
- [ ] Vibration compensation
    - Could run the motor at steady state and record the values for feed-forward table
    - LMS filter may be useful

## Testing Phases
### Phase 1
- 50% PWM output, no current generation
- Check ADC values
### Phase 2
- Use angle generator
