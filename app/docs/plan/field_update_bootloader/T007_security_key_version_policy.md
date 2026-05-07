# T007 - Security Key And Version Policy

## Work

- Define development vs production signing keys.
- Ensure production private keys are never committed.
- Keep production keys outside the repository, for example under
  `/workspace/secure-keys`.
- Build production images by overriding
  `SB_CONFIG_BOOT_SIGNATURE_KEY_FILE=<private-key.pem>`.
- Define firmware version source and monotonic version policy.
- Decide whether downgrade prevention is required for first field release.
- Add release checklist for image signing and artifact retention.

## Validation

- Build fails or warns if production build uses development key.
- Release artifact includes signed binary, manifest/version, and hash.
