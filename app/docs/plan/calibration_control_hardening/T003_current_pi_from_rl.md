# T003 - Current PI From Electrical ID

## Problem

R/L and Rs measurements are stored but current-loop PI gains remain based on devicetree defaults unless separately configured.

## Implementation

- Add a helper to compute/apply D/Q current PI gains from measured Rs and R/L at the configured current-loop bandwidth.
- Apply gains after ROVERL/RS commissioning succeeds.
- Reset current PI integrators after applying gains.
- Log the applied gains.

## Done When

- Successful electrical identification updates current-loop PI gains automatically.
