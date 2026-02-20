# Test Commands

## Zephyr unit tests (inside podman)

```sh
podman exec priceless_wiles bash -lc 'cd /workspace/chopper && west twister -T tests/unit -p native_sim --inline-logs -v'
```

## Host-only fallback test (no Zephyr required)

```sh
./tests/host/motor_autonomy/run.sh
```
