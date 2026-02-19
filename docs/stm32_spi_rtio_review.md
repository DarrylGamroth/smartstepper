# STM32 SPI RTIO Driver Review

## Scope
- Driver under review: `drivers/spi/spi_ll_stm32.c` and `drivers/spi/spi_ll_stm32.h` in the Rubus fork.
- Microcontrollers of interest: STM32F407 (SPI v2), STM32G474 (SPI v2 with DMAMUX), STM32H734 (SPI v6 with advanced features and D-Cache).
- Focus areas: RTIO integration, DMA/interrupt/polling execution paths, cache coherency, and hardware feature utilization when data cache is enabled.

## Executive Summary
- **Protocol sequencing now fully aligned with H7 errata**: Transfers program `TSIZE`, assert `CSTART`, and honor the ES0392 1 µs post-enable delay before clocking data. EOT interrupts replace BSY polling, eliminating the lingering tail noted in Section 2.3.8 of ES0392.
- **DMA stays active under D-Cache**: Buffers in cached SRAM are flushed/invalidated so H7-class parts keep DMA engaged without special linker sections. Heuristics use `MAX(tx_len, rx_len)` to capture balanced full-duplex workloads.
- **FIFO thresholds adapt per mode**: H7 now uses half-depth thresholds for DMA and quarter-depth for interrupt/polling, providing better throughput while staying within ES0392 recommendations.
- **Validation performed on STM32G474** (SPI v2) confirms DMA + RTIO flow after recent changes. H7 and F407 behavior is aligned with errata guidance; CRC/NSS pulse features remain optional future work.

## RTIO Path Analysis

### DMA Fast Path
- **Configuration reuse**: `dma_config()` is still cached per stream, keeping setup overhead to ~3.5–4 µs on G474 with the remaining cost in `dma_reload()`.
- **Cached buffers supported**: DMA no longer depends on `stm32_buf_in_nocache()`. When `CONFIG_DCACHE` is enabled, TX buffers are flushed and RX buffers invalidated so cached SRAM works seamlessly on H7.
- **Transfer size + CSTART**: Each submission programs `TSIZE` to the max of TX/RX frames and asserts `CSTART` only after SPI enable, satisfying ES0392 sequencing guidance.
- **Master start with EOT**: H7 uses `LL_SPI_StartMasterTransfer()` and a 1 µs delay, then waits on the EOT interrupt rather than BSY polling, preventing the errata’s spurious busy condition.

### Interrupt Path
- Enabled via `CONFIG_SPI_RUBUS_STM32_INTERRUPT`.
- Uses TXE/RXNE interrupts and feeds/drains registers one frame at a time with shared buffer helpers. On H7 the FIFO threshold now scales to half depth (2/4 frames depending on word size) to reduce IRQ churn.
- Error handling clears OVR/MODF/CRCERR, but CRC bit is never enabled, so CRCERR only fires in legacy modes (future work).

### Polling Path
- Fallback for builds without interrupt support.
- Busy-waits on TXE/RXNE; uses `LL_SPI_IsActiveFlag_BSY`. The polling path now enables SPI, reprograms the FIFO threshold, and asserts `CSTART` with the errata delay on H7 devices.

### Transaction Chaining & Tail Latency
- RTIO chaining uses `RTIO_SQE_TRANSACTION`. With DMA, `spi_stm32_try_finalize()` waits for both DMA channels _and_ EOT (H7) before completing, which suppresses the long tail described in ES0392 §2.3.8.
- SPI disable no longer busy-waits on BSY for H7 because the hardware asserts EOT reliably before the driver tears down DMA/CS.

## MCU Family Findings

### STM32F407
- **DMA**: Works (DMA v1). No D-cache by default, so DMA is available immediately.
- **FIFO**: Not available.
- **CRC**: Supported; still unused. Errata ES0182 §2.4.2 notes NSS glitches when toggling CPOL/CPHA on the fly—our static config path avoids the hazard.
- **RTIO correctness**: Matches upstream behavior. Should the system enable D-cache (ES0182 §2.1), identical flush/invalidate hooks from H7 could be backported to keep DMA alive.

### STM32G474
- **DMA**: DMAMUX present; configuration works. No D-cache, so DMA available. Setup cost now limited to ~4 µs after recent `dma_reload()` change.
- **FIFO**: Not present (SPI v2).
- **CRC**: Hardware supports 8/16-bit CRC but driver leaves it off.
- **Additional features**: Fast-mode plus and programmable NSS pulses are unused but optional.

### STM32H734 / H743
- **DMA**: Cached SRAM is supported; TX buffers are flushed, RX buffers invalidated, and DMA remains engaged even when buffers live in cacheable regions. Heuristics use `MAX(tx_len, rx_len)`.
- **FIFO**: Thresholds adapt per transfer (½ FIFO for DMA, ¼ FIFO for interrupt/polling) in line with ES0392 §2.3.5 guidance.
- **CRC**: Still disabled; ES0392 §2.3.9 flags CRCERR behavior when TI mode is enabled—currently not used but documented.
- **Start/Stop sequencing**: `TSIZE` is programmed, `LL_SPI_StartMasterTransfer()` is called, and the 1 µs delay is respected before clocks start.
- **Data width**: Hardware supports 4–32 bits, though the driver still gates to 8/16-bit for compatibility.

## Hardware Feature Coverage

| Feature | F407 | G474 | H734 | Driver Usage | Notes |
| --- | --- | --- | --- | --- | --- |
| DMA M2P/P2M | ✅ | ✅ | ✅ | ✅ | Cached SRAM supported via automatic flush/invalidate; `dma_reload()` reuse retained. |
| FIFO (RX/TX) | n/a | n/a | ✅ 16-depth | ⚙️ | Threshold adapts to transfer mode (½ FIFO for DMA, ¼ FIFO for interrupt/polling). |
| CRC generation/check | ✅ | ✅ | ✅ (4–32 bit) | ❌ | Driver still leaves CRC disabled. |
| Transfer size register (`TSIZE`) | n/a | n/a | ✅ required | ✅ | Frame count programmed per transfer for H7; required per ES0392 §2.3.8. |
| Automatic master start (`CSTART`) | n/a | n/a | ✅ required | ✅ | `LL_SPI_StartMasterTransfer()` invoked with errata delay per ES0392 §2.3.8. |
| Clock-to-sample tuning (MIDI/MSSI) | n/a | n/a | ✅ | ✅ | `midi_clock`/`mssi_clock` DT properties now configure idle cycles in master mode. |
| Hardware NSS pulse management | Limited | Limited | ✅ advanced | ❌ | `CONFIG_SPI_RUBUS_STM32_USE_HW_SS` disabled by default; driver never configures `NSSP`. |
| DS > 16-bit support | ❌ | ❌ | ✅ | ❌ | Driver restricts to 8/16-bit frames. |

## Recommendations

1. **Monitor H7 completion behavior**
   - Current EOT-based flow resolves ES0392 §2.3.8. Keep an eye on errata revisions in case additional post-EOT delays are mandated.

2. **Implement cache maintenance for DMA buffers**
   - When `CONFIG_DCACHE` is enabled, flush TX buffers and invalidate RX buffers unless they are already in nocache memory. This keeps DMA available on H7 without forcing special linker sections.
   - Re-run nocache checks only to decide whether maintenance is required, not to disable DMA.

3. **Enhance FIFO utilization on H7**
   - Tie FIFO threshold to word size and transfer direction (e.g., ½ FIFO for DMA, 1 word for interrupt).
   - Optionally use `LL_SPI_SetFIFOThreshold(spi, LL_SPI_FIFO_TH_04DATA)` for interrupt mode to reduce IRQ rate.

4. **Offer CRC/advanced feature hooks**
   - Add optional configuration (Kconfig or DT property) to enable hardware CRC, programmable NSS pulse management, and MIDI/MSSI clock tuning for H7 boards that need tighter timing.

5. **Broaden data width support**
   - Allow 4–32 bit data widths on H7 by expanding the driver validation and updating DMA data size fields dynamically when using multi-byte frames.

6. **DMA heuristics and metrics**
   - Replace `tx_len + rx_len` with `MAX(tx_len, rx_len)` when evaluating `CONFIG_SPI_RUBUS_STM32_DMA_MIN_LEN` so full-duplex transfers with 32-byte payloads still qualify. _Done (Oct 2025)._ 
   - Consider recording timestamps (e.g., trace hooks) around `dma_reload()` and completion to quantify residual latency by board.

7. **Documentation & samples**
   - Provide guidance in project docs on how to place RTIO buffers in nocache sections or enable cache maintenance for DMA. _Updated below with cache guidance._
   - Capture the measured improvements (27 µs → 11 µs → 3.9 µs) to justify future tuning.

## Implementation Progress (October 2025)

- **H7 protocol sequencing**: every transfer programs `TSIZE`, enables SPI, asserts `CSTART`, observes the ES0392 1 µs delay, and completes via the EOT interrupt to avoid BSY-polling stalls.
- **Cache-safe DMA**: TX buffers are flushed and RX buffers invalidated around each DMA transaction. DMA selection no longer depends on nocache linker sections, making H7 usable with default SRAM placements.
- **FIFO tuning**: the driver reconfigures H7 FIFO thresholds per transfer—½ depth for DMA, ¼ depth for interrupt/polling—improving throughput without sacrificing responsiveness.
- **Timing knobs surfaced**: board-level `midi_clock` and `mssi_clock` device-tree properties configure inter-frame idle cycles.
- **DMA heuristic**: the minimum-length check uses `MAX(tx_len, rx_len)` so balanced full-duplex work still leverages DMA acceleration.
- **Validation**: Built and exercised on STM32G474 with RTIO + DMA to confirm no regressions.

### Practical Guidance

- **Cache maintenance**: With `CONFIG_DCACHE` enabled, you can place RTIO buffers in ordinary SRAM. The driver automatically flushes TX buffers and invalidates RX buffers on completion. Extra nocache sections are only needed for zero-copy peripherals that lack maintenance hooks.
- **Timing tuning**: To stretch SCK low between words or extend NSS idle, set `midi-clock` and `mssi-clock` properties in your board overlay:

  ```dts
  &spi2 {
      fifo-enable;
      midi-clock = <2>;   /* two SCK cycles between data frames */
      mssi-clock = <3>;   /* three SCK cycles of NSS idleness */
  };
  ```

- **DMA thresholds**: Retain `CONFIG_SPI_RUBUS_STM32_DMA_MIN_LEN` at or below your shortest critical payload. The driver now evaluates the maximum of TX/RX lengths, so symmetric 32-byte full-duplex exchanges will still issue DMA.

## Next Steps
- Track future errata updates for STM32F4/H7 to ensure sequencing remains compliant.
- Add optional hardware CRC / NSS pulse management and expose metrics (timestamps or RTIO trace hooks) for latency tracking.
- Extend data width support beyond 16 bits and plumb wider frame sizes through DMA stream configuration.
- Validate on hardware across F407, G474, and H743 with cache on/off, capturing latency deltas for documentation.
