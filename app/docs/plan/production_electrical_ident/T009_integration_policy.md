# T009 - Integration Policy

## Promotion Rules

- Existing Rs/R-over-L remains fallback/bootstrap.
- Production electrical ID becomes preferred only after repeatability and current-step validation pass.
- Applying production values sets source flags to measured.
- Failed production ID does not overwrite active safe values.

## Replacement Criteria

Consider replacing the existing implementation only after:

- Production Rs/L measurement passes repeatedly on MT6835 and AEAT hardware.
- Current-loop validation passes at representative current levels.
- HIL scripts demonstrate repeatability across power cycles.
