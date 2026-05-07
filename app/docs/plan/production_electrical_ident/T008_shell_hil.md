# T008 - Shell And HIL

## Shell Commands

Proposed tree:

- `motor commission electrical plan`
- `motor commission electrical measure rs`
- `motor commission electrical measure inductance`
- `motor commission electrical run`
- `motor commission electrical status`
- `motor commission electrical apply`
- `motor commission electrical clear`

## HIL

Add a Python HIL scenario that:

1. Runs boot encoder mapping if required.
2. Runs production electrical ID.
3. Prints old Rs/R-over-L result and new production result.
4. Runs current step validation.
5. Does not persist values.
