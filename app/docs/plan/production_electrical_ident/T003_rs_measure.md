# T003 - Production Rs Measurement

## Approach

Add a VESC-style locked-rotor Rs measurement alongside the existing Rs estimator:

1. Enter a safe generated-frame locked state.
2. Ramp current slowly to the measurement current.
3. Wait for settling.
4. Average measured current and commanded/applied voltage over a long window.
5. Compute `Rs = V/I`.
6. Repeat at least twice and check repeatability.

## Fallback

If production Rs measurement fails, retain the existing Rs/R-over-L estimator result and mark source as fallback.
