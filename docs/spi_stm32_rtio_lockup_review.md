# SPI STM32 RTIO Lockups – Code Review (Zephyr `spi_stm32.c`)

## Scope and assumptions

- **You are using**: `zephyr/drivers/spi/spi_stm32.c` (your active driver).
- **Reference for comparison**: `zephyr/drivers/spi/spi_stm32_upstream.c` (not compiled in your build).
- **Mode of interest**: `CONFIG_SPI_RTIO` enabled.
- **Target family**: STM32H7 (FIFO/EOT paths are relevant).

This review focuses on failure modes that can plausibly **lock up the whole system** (hard faults, infinite loops/IRQ storms, or deadlocked driver state).

## Executive summary

### Highest-likelihood root cause (confirmed correctness bug)

**RTIO path in `spi_stm32.c` used stack-local `struct spi_buf` descriptors and stored pointers to them into `spi_context`.**

That is a classic **use-after-return (UAF)**:

- `spi_stm32_iodev_msg_start()` created:
  - `const struct spi_buf current_tx = {...}`
  - `const struct spi_buf current_rx = {...}`
- Then stored `data->ctx.current_tx = &current_tx;` and `data->ctx.current_rx = &current_rx;`.
- As soon as the function returns, those pointers are dangling.
- Later, the ISR / transfer engine dereferences `data->ctx.current_tx/current_rx`.

Depending on stack reuse, this can manifest as:
- random lengths (runaway transfers)
- unexpected `NULL`/garbage pointers
- out-of-bounds reads/writes
- hard faults
- stuck transfer state where completion never happens

✅ **Fix applied** (see “Fixes applied” below).

### Secondary risk areas (still worth checking)

These are not proven bugs from inspection alone, but are credible lockup mechanisms:

1. **RTIO completion cleanup ordering**
   - RTIO completion is handled late in `spi_stm32_complete()`.
   - If interrupts/peripheral aren’t fully cleaned up before RTIO schedules the next txn, you can get an IRQ storm.
   - In your current `spi_stm32.c`, RTIO completion happens **after** disabling interrupts and the SPI disable/IRQ-disable workaround, which is the safer ordering.

2. **FIFO shifting strategy differences**
   - Your `spi_stm32.c` drains RX FIFO and fills TX FIFO in loops.
   - The reference file had a “single-step” FIFO shift in `spi_stm32_shift_fifo()`.
   - Not draining RX FIFO fully can cause RX overrun on H7; your version’s full-drain/full-fill approach is generally safer for progress.

3. **Busy-wait loops in non-FIFO path**
   - In `spi_stm32_shift_m()` (non-FIFO), there are `while (!ll_func_tx_is_not_full()) {}` and `while (!ll_func_rx_is_not_empty()) {}` loops.
   - If the peripheral is misconfigured or clocks stop, this can spin forever.
   - In RTIO+FIFO cases on H7, you’re usually not on this path, but it remains a potential “whole system lockup” mechanism if activated.

## What changed vs reference (high-level)

Using `diff -u spi_stm32_upstream.c spi_stm32.c`:

- `spi_stm32_shift_fifo()`:
  - Your version **drains RX fully** and **fills TX fully** (loops).
  - Reference version did a single read/write step.

- `spi_stm32_msg_start()` interrupt enabling:
  - Your version has a FIFO pre-fill and conditional enabling of TX/RX interrupts if data remains, plus EOT.
  - Reference version enabled TX/RX interrupts more unconditionally.

- `spi_stm32_complete()` ordering differences around RTIO:
  - Your version performs normal cleanup first, then calls `spi_stm32_iodev_complete()`.
  - Reference version had an early RTIO return in some variants (riskier if it skips cleanup).

## RTIO path walkthrough (what to sanity-check)

### Submit/start

- `spi_stm32_iodev_submit()`
  - Calls `spi_rtio_submit()`.
  - If it returns “we should start now”, it configures via `spi_stm32_iodev_prepare_start()` then starts via `spi_stm32_iodev_start()`.

- `spi_stm32_iodev_start()`
  - Switches on RTIO op (RX/TX/TINY_TX/TXRX) and calls `spi_stm32_iodev_msg_start()`.

### Message start (`spi_stm32_iodev_msg_start()`)

Responsibilities:
- Populate the `spi_context` fields (`tx_buf`, `rx_buf`, lengths/counts, and `current_tx/current_rx`).
- For H7 FIFO master, set transfer size.
- Call `spi_stm32_msg_start()` which enables SPI + IRQs and asserts CS.

The UAF bug lived here.

### Completion

- ISR calls `spi_stm32_complete()` on terminal conditions.
- For RTIO, `spi_stm32_complete()` calls `spi_stm32_iodev_complete()`.
- `spi_stm32_iodev_complete()` either:
  - advances within the same transaction (`RTIO_SQE_TRANSACTION`), or
  - ends transaction, deasserts CS, completes SQEs, and potentially starts the next txn.

## Fixes applied

### 1) Fix RTIO `spi_buf` descriptor lifetime (UAF)

**Files changed:**
- `zephyr/drivers/spi/spi_stm32.h`
- `zephyr/drivers/spi/spi_stm32.c`

**What:**
- Added persistent fields in `struct spi_stm32_data`:
  - `rtio_current_tx`
  - `rtio_current_rx`
- Updated `spi_stm32_iodev_msg_start()` to store descriptors there and point `data->ctx.current_tx/current_rx` at those persistent objects.

### 2) Fix RTIO `CONFIG_SPI_SLAVE` typo

In both your driver and the reference variant, the RTIO block used:

- `ctx->recv_frames = 0;`

…but there is no local `ctx` in that scope.

**Fix:**
- changed to `data->ctx.recv_frames = 0;`

(If `CONFIG_SPI_SLAVE` is not enabled, this is compiled out; but it’s still a correctness bug.)

## Remaining recommendations (no code changes yet)

These are the next most valuable steps if lockups persist even after the UAF fix.

### A) Add bounded timeouts on busy-wait loops

In non-FIFO master shifting, these loops can spin forever:

- wait for TX space
- wait for RX data

Recommendation:
- add a bounded loop counter or time-based timeout and convert to `-EIO` / `-ETIMEDOUT`.

This turns “whole system locked” into an error completion.

### B) Add RTIO-specific instrumentation hooks

Add optional debug counters (under a Kconfig) for:
- number of starts
- number of completes
- last `SR`, `IER`, transfer direction, transfer size
- number of spurious IRQs while IER=0

This helps distinguish:
- stuck peripheral (EOT never comes)
- IRQ storm (ISR keeps firing)
- driver stuck waiting for a condition

### C) Validate CS hold behavior with your sensor protocol

You mentioned previously that some sensor protocols require CS to remain asserted across pipelined frames.

- In Zephyr SPI, this is typically `SPI_HOLD_ON_CS` and/or using a single transceive.
- In STM32H7, the driver has an IRQ-disable workaround when SPI remains enabled.

If you combine RTIO transactions with `SPI_HOLD_ON_CS`, confirm:
- `spi_stm32_complete()` does not deassert CS
- the next txn starts without reinitializing in a way that breaks the hold

## Risk ranking

1. **Confirmed UAF in RTIO (`current_tx/current_rx`)** → can hardfault/lock up.
2. **Infinite busy-waits in non-FIFO master** → can hard lock.
3. **FIFO/RX overrun behavior if RX not drained** → can lead to persistent error IRQs.
4. **RTIO completion ordering and flag clearing** → can cause IRQ storms.

## Notes

- Your FIFO changes (full drain/fill + prefill) look directionally correct for progress.
- The RTIO UAF bug is sufficient by itself to explain intermittent system lockups.

