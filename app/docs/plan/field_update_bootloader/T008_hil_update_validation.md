# T008 - HIL Update Validation

## Work

Create repeatable HIL scripts for:

1. baseline boot/status before update,
2. settings save/load/reset retention,
3. Ethernet upload of signed update,
4. boot into pending image,
5. health gate and image confirmation,
6. rollback when confirmation is withheld,
7. corrupt settings fallback,
8. QSPI unavailable/failure-mode test if QSPI is part of the update path.

## Validation

- HIL JSON reports are saved under `hil_logs/field_update/`.
- Plan execution log records image versions, hashes, partition layout, and result.
- No motor motion is required for initial update validation.
