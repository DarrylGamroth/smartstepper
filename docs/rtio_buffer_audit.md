# RTIO Buffer Usage Audit

_Date: 2025-10-05_

## Scope

Review RTIO-related buffer ownership and lifecycle across the following components prior to any fixes:

- `app/src/main.c`
- `drivers/sensor/brcm_aeat-9955/brcm_aeat-9955.c`
- `drivers/spi/spi_ll_stm32.c`

## Summary

| Area | Status | Notes |
| --- | --- | --- |
| Application CQE handling | ❌ Leak identified | Error path releases the CQE but never returns the mempool buffer, so each failed read permanently consumes one block. |
| AEAT-9955 sensor driver | ⚠️ Needs defensive cleanup | Early error exits after `rtio_sqe_rx_buf()` do not free the reserved buffer; these code paths will leak if triggered. |
| STM32 SPI RTIO backend | ✅ No leak observed | Completes via `spi_rtio_complete()` and does not own higher-level mempool buffers; behaves as expected. |

Overall, the first leaked buffer originates in the application’s CQE error path, explaining the exhaustion after 16–17 samples. Driver-side cleanup gaps should still be addressed to avoid leaks when lower-level errors occur.

## Detailed Findings

### `app/src/main.c`

- `mcpwm_callback()` submits a new async read on every timer ISR without checking whether a prior read is still pending. Because the main loop blocks on `rtio_cqe_consume_block()`, this is acceptable for now but could become an issue if additional asynchronous work is added.
- In the processing loop the code checks `cqe->result` **before** fetching the mempool buffer. When the result is non-zero the logic releases the CQE and `continue`s. The buffer obtained via `rtio_sqe_rx_buf()` in the driver is therefore never reclaimed, causing an immediate leak on the first failure.
- For successful completions the buffer is decoded and released correctly: `sensor_decode()` → `rtio_release_buffer()`. Ensuring the error path mirrors this (fetch buffer, release, then continue) will stop the observed exhaustion.

### `drivers/sensor/brcm_aeat-9955/brcm_aeat-9955.c`

- `aeat9955_submit_one_shot()` reserves a buffer with `rtio_sqe_rx_buf()` before performing setup. Several early error returns (`sensor_clock_get_cycles()` failure, inability to acquire SQEs) report the error via `rtio_iodev_sqe_err()` but **do not** release the buffer they just reserved. In normal operation these branches are rare, yet if hit they will leak one block per occurrence. Add explicit `rtio_release_buffer()` (matching the length passed to `rtio_sqe_rx_buf()`) before returning.
- Once the SPI submission succeeds, ownership flows as intended: the buffer is filled by the SPI driver, `aeat9955_complete_result()` consolidates subordinate CQEs, and the top-level CQE references the same buffer for the application to reclaim.
- No guard exists to prevent multiple outstanding reads per device; consider adding a simple in-flight flag if the driver will be reused in contexts where backpressure is required.

### `drivers/spi/spi_ll_stm32.c`

- The STM32 SPI RTIO integration uses the `spi_rtio_*` helpers. The backend never allocates from, nor releases to, the sensor mempool directly; the TX/RX buffers are provided by higher layers and remain owned by them.
- Completion funnels through `spi_stm32_iodev_complete()` → `spi_rtio_complete()`, which enqueues the CQE for the requesting iodev. No leaks were observed in this layer.
- DMA teardown logic ensures caches are invalidated when needed, but this does not affect mempool ownership.

## Recommendations (Pre-Fix)

1. Update the application loop to always obtain and release the mempool buffer, even on error results.
2. Harden the AEAT-9955 driver by releasing the reserved buffer on any early error return prior to `rtio_submit()`.
3. (Optional) Add an in-flight submission guard either in the app or the driver to avoid piling requests if the ISR rate increases.

These changes should be validated with an RTIO stress test that artificially injects errors to confirm buffers return to the pool.
