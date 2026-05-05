# Test Commands

## One-command unit test run (podman)

```sh
./tests/run_unit_tests.sh
```

Optional:

```sh
./tests/run_unit_tests.sh <container_name>
```

## Zephyr unit tests (inside podman)

```sh
podman exec wonderful_goldberg bash -lc 'cd /workspace/chopper && west twister -T tests/unit -p native_sim --inline-logs -v'
```
