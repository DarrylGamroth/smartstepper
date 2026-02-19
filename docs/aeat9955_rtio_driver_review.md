# AEAT-9955 RTIO Driver Review

_Date: 2025-10-01_

## 1. Protocol recap

The AEAT-9955 offers several SPI framing options. In SPI4-16a (16-bit + parity) each transfer word has:

- Bit 15: even parity over bits 14:0
- Bits 14:8: command (read/write/program/opcode)
- Bits 7:0: register address or payload byte, depending on opcode stage

A position read is a three-word exchange:

1. Command frame (read opcode + register + parity)
2. Dummy frame (sensor prepares data)
3. Data frame (error + parity + 18-bit payload)

The device returns status in bits [15:14] of the data frame and delivers the 18-bit absolute position left-aligned in bits [13:0] across the final two bytes. The application note clarifies that the sensor does not tolerate back-to-back CS assertions shorter than 350 ns, and the minimum re-trigger latency is chiefly dominated by the SPI clock period plus an internal conversion delay (up to 450 ns when tracking fast magnets).

## 2. Current driver snapshot

File: `drivers/sensor/brcm_aeat-9955/brcm_aeat-9955.c`

- Uses Zephyr synchronous `spi_transceive_dt()`/`spi_write_dt()` for register access and an RTIO-based fast path (`aeat9955_submit_one_shot()`).
- `SPI_WORD_SET(8)` is selected, so each protocol word is emitted as two 8-bit writes. Parity is appended manually via `aeat9955_parity()`.
- RTIO path obtains two SQEs (one for the SPI transaction, one callback) and submits a 5-byte transceive (command + reg + three dummy bytes). The response is stored in a local encoding buffer alongside a timestamp from `sensor_clock_get_cycles()`.
- The AQ data decoder (`brcm_aeat9955_decoder.c`) converts the raw 18-bit value into Q31 for FOC loops.

## 3. Observations and nitpicks

1. **SPI framing**
   - 8-bit mode keeps the byte order straightforward but doubles the number of SPI events per logical word. At 8 MHz the extra inter-frame gap costs ~125 ns per parity byte.
   - The read-angle helper issues five bytes (40 bits). In 16-bit mode you could accomplish the same exchange with three 16-bit words (48 bits). The bit count is slightly larger, but each transfer is atomic and parity is handled by hardware instead of software.
   - Because the STM32 LL driver allows `SPI_WORD_SET(16)`, switching would remove manual parity generation and simplify error checking. It also opens the door to reading the 18-bit angle with two transfers (command, then data) if CS remains asserted between words. However, the AEAT-9955 still needs the "dummy" frame per datasheet, so the real gain is hardware-managed parity, not fewer frames.

2. **Parity helper**
   - `aeat9955_parity()` calculates parity only over the 8-bit register/value. In SPI4-16a, parity must cover the full 15 bits (opcode + address). When using 8-bit SPI the current packing effectively sends parity separately so it works, but if you migrate to 16-bit frames you need to extend the computation to include the opcode bits.

3. **RTIO buffering**
   - `static uint8_t tx_buf[]` lives in `.data`, so multiple outstanding submissions would contend for the same buffer. For ISR-triggered sampling with queue depth >1 you should either make the buffer part of `aeat9955_encoded_data` or allocate it per SQE.
   - The encoded packet only writes the timestamp. The raw payload (`edata->buf`) is populated by the RTIO SPI transfer, but there is no explicit metadata describing layout. For multi-rate consumers (e.g., logging vs. FOC) consider adding a small header with status bits and the decoded absolute angle to avoid duplication in every consumer.

4. **Interrupt triggering**
   - The driver expects to be called from a non-blocking context (`aeat9955_submit()` is async-friendly). For deterministic ISR usage, ensure the RTIO context is pre-armed (no dynamic allocation) and the SPI queue depth matches the worst-case ISR burst.

5. **Error GPIO**
   - The ERROR pin is configured but never read. If parity or framing faults occur, polling this pin (or wiring it to an EXTI) would shorten fault detection compared to the SPI status bit alone.

## 4. 8-bit vs. 16-bit mode

| Aspect | 8-bit mode (current) | 16-bit mode (proposed) |
| --- | --- | --- |
| Parity generation | Software (`aeat9955_parity()`) | Automatic (CR1 `SPI_CR1_SSM` + hardware parity) |
| Frame count for angle read | 5 bytes (40 bits) | 3 words (48 bits) |
| Endianness | Manual byte/bit shifting (`sys_get_be24`) | Natural (18 bits extracted from 16-bit data word + extra byte) |
| Peripheral config | Simple (default) | Need to configure `SPI_CR1.DFF` = 16-bit and ensure DMA/RTIO alignment |
| Compatibility | Matches other 8-bit peripherals on same bus | Requires homogeneous 16-bit devices or dynamic reconfiguration |

Given the minor difference in total bits and the convenience of hardware parity, moving to 16-bit mode is attractive **if**:

- This SPI bus is dedicated to AEAT-9955, or you can tolerate reconfiguring word size per transaction.
- You are comfortable decoding the returned 18-bit value from a 16-bit-aligned buffer (the sensor still sends 18 bits, so you must read two words and shift).

If additional devices share the bus and are strictly 8-bit, staying in 8-bit mode avoids reconfiguration overhead.

## 5. Recommendations

1. **Decouple TX buffer per transaction** to permit pipelined RTIO submissions.
2. **Add a compact result header** (status flags + decoded angle) so the ISR and control loop can consume data without re-parsing the raw bytes.
3. **Evaluate 16-bit SPI**: prototype by setting `SPI_WORD_SET(16)` and adjusting parity handling. Measure end-to-end latency (command issue to CQE availability) to decide if the hardware parity benefit outweighs reconfiguration costs.
4. **Leverage the ERROR pin**: either poll after each transfer or hook it to an interrupt to flag magnet loss or parity faults faster.
5. **Document timing budget**: align the ISR scheduling (timer/ADC) with the AEAT-9955 sample latency. The application note’s worst-case conversion delays should be captured alongside the RTIO queuing overhead.

## 6. Next steps

- Implement a prototype branch with 16-bit framing and run on the F407/G474 reference hardware. Collect latency measurements with the timer ISR triggering sequence.
- Extend unit/integration tests in `drivers/sensor/brcm_aeat-9955/` to cover both SPI word sizes and parity fault injection (flip a bit in dummy registers).
- Update `docs/stm32_spi_rtio_review.md` (or a new AEAT-9955 note) with empirical data to support FOC tuning.

---

For deeper protocol details, see Broadcom Application Note “AEAT-9955 Magnetic Angle Encoder” (linked in the request).