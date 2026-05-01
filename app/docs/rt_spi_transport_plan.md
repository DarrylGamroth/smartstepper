# Real-time SPI Engine Plan

## Goal

Create a deterministic, ISR-safe SPI acquisition path for motor encoders without depending on Zephyr's generic SPI API or RTIO in the control loop.

The driver must be reusable across absolute SPI encoders, not AEAT-9955-specific. The transport layer should move short command/response transfers using static buffers and a single in-flight request. Encoder-specific layers should own command bytes, decode, parity/CRC/status checks, and latency semantics.

Primary targets:

- STM32H7: use SPI FIFO efficiently.
- STM32H7: support non-FIFO mode later for parity with upstream devicetree configuration.
- STM32F4: support the older SPI peripheral without FIFO.

## Motivation

Current encoder acquisition uses Zephyr sensor + RTIO + upstream STM32 SPI. This was chosen because normal Zephyr SPI APIs are not safe to call from the ADC/control ISR.

Observed issues:

- With overlapping RTIO reads, H7 hardware faulted inside the STM32 SPI completion path.
- With one in-flight read, transport is stable but sample rate/latency is worse.
- When current is applied, AEAT-9955 parity errors still occur, likely from switching-noise or sampling-time sensitivity.

The new path should solve the transport determinism problem. It will not by itself solve physical signal-integrity issues, but it will make those errors observable as clean frame/status counters instead of lockups.

## Architecture

Layering:

```text
motor control ISR / encoder pipeline
        |
        v
fast encoder frontend
  - AEAT-9955 fast frontend
  - MT6835 fast frontend
  - future encoder frontends
        |
        v
real-time SPI transport
  - request short TX/RX transfer
  - owns CS timing
  - owns SPI ISR
  - stores latest raw frame or SPSC trace entry
        |
        v
STM32 backend
  - H7 FIFO backend
  - H7 non-FIFO backend
  - F4 non-FIFO backend
```

The real-time SPI transport is a custom Zephyr device API, not an implementation of
Zephyr's `spi_driver_api`. It deliberately exposes a tiny RTIO-style lifecycle:
`request()` starts a short, statically buffered transfer from ISR context and
`collect()` consumes the completed result later.

It should still use Zephyr conventions where useful:

- Devicetree for hardware instance, pins, CS GPIO, frame size, and backend selection.
- Kconfig for compile-time feature selection and diagnostics.
- Zephyr device model only if it stays lightweight and does not force kernel APIs into ISR paths.

## Non-Goals

- Do not replace the Zephyr sensor drivers immediately.
- Do not implement Zephyr's generic `spi_driver_api`.
- Do not support arbitrary scatter/gather buffers.
- Do not support concurrent clients.
- Do not support dynamic frame allocation.
- Do not perform shell/log formatting in interrupt context.
- Do not make the transport AEAT-only.

## Transport API

Proposed core API:

```c
struct rt_spi_transfer {
	const uint8_t *tx;
	uint8_t len;
};

struct rt_spi_result {
	uint32_t timestamp_cycles;
	uint8_t raw[RT_SPI_MAX_FRAME_BYTES];
	uint8_t len;
	uint16_t flags;
};

int rt_spi_init(const struct device *dev);
int rt_spi_request(const struct device *dev,
		   const struct rt_spi_transfer *transfer);
int rt_spi_collect(const struct device *dev,
		   struct rt_spi_result *result);
void rt_spi_get_stats(const struct device *dev,
				  struct rt_spi_stats *stats);
```

Expected behavior:

- `request()` is ISR-safe.
- `request()` returns `-EBUSY` if a transaction is already active.
- `collect()` is ISR-safe.
- Completion writes a single latest-sample slot and optionally a diagnostic ring.
- The engine never blocks.
- The engine never allocates.
- The engine never calls kernel queue/semaphore APIs from ISR.

## Encoder Frontend API

Each encoder frontend provides static frame configuration and decode helpers:

```c
struct encoder_fast_vtable {
	uint8_t frame_len;
	uint8_t pipeline_delay_samples;
	void (*prepare_frame)(uint8_t *tx);
	int (*decode)(const uint8_t *raw, struct motor_encoder_sample *sample);
};
```

Examples:

- AEAT-9955:
  - frame length: 3 bytes.
  - decode 18-bit angle.
  - verify parity over full returned frame.
  - check status/error bit.
  - account for one-sample protocol delay if used in pipelined mode.

- MT6835:
  - frame length: 6 bytes.
  - decode 21-bit angle.
  - verify CRC8 over raw bytes.
  - check `STATUS[2:0]`.
  - no extra SPI-cycle delay if angle latches on CS low.

The frontend should be usable by both the fast path and existing sensor driver decode tests where practical.

## STM32H7 FIFO Backend

Use LL register access, not the generic Zephyr SPI transaction layer.

H7 backend behavior:

1. `request()` verifies idle state.
2. Assert CS.
3. Configure transfer size for requested frame length.
4. Enable RX/TX/EOT/error interrupts.
5. Fill TX FIFO with command/dummy bytes.
6. ISR drains RX FIFO and feeds TX FIFO until complete.
7. On EOT, clear flags, deassert CS, publish raw frame, return to idle.
8. On OVR/MODF/UDR/SUSP/TIFRE, clear flags, deassert CS, publish transport error, return to idle.

H7-specific notes:

- Prefer FIFO threshold that minimizes ISR entries for 3-6 byte frames.
- Avoid DMA initially; setup overhead and cache coherency are not worth it for tiny frames.
- Keep NSS software-controlled through GPIO unless hardware NSS can be proven to meet encoder timing.
- Ensure EOT/OVR flag clear order follows STM32H7 reference manual and upstream driver practice.

## STM32H7 Non-FIFO Backend

The first H7 implementation should be FIFO-only and fully validated before this mode is added.

H7 non-FIFO mode exists for parity with the upstream STM32 SPI driver, where FIFO behavior is controlled by devicetree. It should use the same public transport API and the same encoder frontend code.

Candidate devicetree property:

- `fifo-enable`: boolean, matching the spirit of the upstream STM32 SPI binding.

Expected behavior:

- If `fifo-enable` is present on an H7 node, use the H7 FIFO backend.
- If `fifo-enable` is absent on an H7 node, use the H7 non-FIFO backend.
- F4 ignores `fifo-enable` or rejects it at binding/build time because the peripheral has no H7 FIFO.

H7 non-FIFO backend behavior:

1. `request()` verifies idle state.
2. Assert CS.
3. Configure transfer size for requested frame length.
4. Enable RX/TX/EOT/error interrupts.
5. ISR writes one byte/word as TX space becomes available.
6. ISR reads one byte/word as RX data becomes available.
7. On EOT, clear flags, deassert CS, publish raw frame, return to idle.
8. On OVR/MODF/UDR/SUSP/TIFRE, clear flags, deassert CS, publish transport error, return to idle.

H7 non-FIFO notes:

- Follow upstream handling for the H7 cases where completion flags can be awkward without FIFO.
- Keep 8-bit transfers initially unless a specific encoder benefits from 16-bit frames.
- This mode is not the performance baseline; it is compatibility and fallback.

## STM32F4 Non-FIFO Backend

F4 backend behavior:

1. `request()` verifies idle state.
2. Assert CS.
3. Enable RXNE/TXE/error interrupts.
4. ISR writes next byte when TXE is set.
5. ISR reads next byte when RXNE is set.
6. When all bytes are received, wait for not busy, deassert CS, publish sample.
7. On OVR/MODF, clear flags, deassert CS, publish transport error, return to idle.

F4-specific notes:

- No FIFO assumptions.
- Keep frame length small and fixed.
- Use 8-bit transfers initially.
- Avoid blocking on BSY for long loops inside ISR; bound any final wait or defer completion to a later ISR edge if needed.

## Devicetree

Add a binding such as:

```yaml
compatible: "rubus,stm32-rt-spi"
```

Candidate properties:

- `reg`: SPI peripheral base from parent bus or direct MMIO node.
- `interrupts`: SPI IRQ.
- `pinctrl-0`, `pinctrl-names`.
- `cs-gpios`.
- `st,soft-nss`.
- `spi-frequency`.
- `spi-cpol`.
- `spi-cpha`.
- `max-frame-len`.
- `rx-sample-delay-ns` or `trigger-delay-cycles` if needed later.
- `fifo-enable`: enable H7 FIFO operation when supported.

CS/NSS policy should follow the upstream STM32 SPI binding model:

- `cs-gpios` present: use soft NSS internally and control CS with GPIO.
- `st,soft-nss` present: use soft NSS without asserting a CS/NSS pin.
- neither property present: use hardware NSS output.

Kconfig should not select board-level CS behavior. It should only enable the
driver/backend and unavoidable SoC-family workarounds, such as STM32 errata
handling. Hardware-vs-GPIO CS is board wiring and belongs in devicetree.

Encoder frontend nodes should reference the engine:

```dts
rtspi0: rt-spi@... {
	compatible = "rubus,stm32-rt-spi";
	cs-gpios = <&gpiox y GPIO_ACTIVE_LOW>;
	spi-frequency = <1000000>;
	max-frame-len = <8>;
	fifo-enable;
};

encoder1: encoder@0 {
	compatible = "brcm,aeat-9955-fast";
	transport = <&rtspi0>;
	encoder-direction-sign = <(-1)>;
};
```

## Integration With Existing Code

Keep current RTIO/sensor path available:

- sensor shell diagnostics,
- register access,
- commissioning checks,
- non-control telemetry,
- fallback when fast path is disabled.

Add fast path into the motor encoder pipeline:

```text
timer/direct encoder trigger ISR
  -> rt_spi_request()

ADC/control ISR
  -> rt_spi_collect()
  -> encoder frontend decode
  -> angle_observer_update()
```

The control loop should consume the same `struct motor_encoder_sample` regardless of source.

## Telemetry

Add low-cost counters:

- request count,
- busy count,
- completion count,
- transport error count,
- overrun count,
- frame decode error count,
- parity/CRC/status error count,
- stale sample count,
- max transaction cycles.

Add optional raw-frame ring:

- compile-time gated,
- runtime enable/disable,
- stores timestamp, raw bytes, transport flags, decode flags,
- dump through shell outside ISR.

This replaces ad-hoc encoder debug paths over time.

## Safety Rules

The fast path must obey:

- no `k_msgq_put()` from direct ISR,
- no `k_sem_*()` from direct ISR,
- no logging from SPI ISR,
- no shell calls from ISR,
- no heap allocation,
- no unbounded loops,
- no generic SPI API calls from control ISR,
- no shared mutable TX buffer across transactions unless only one in-flight transaction is impossible by construction.

## Implementation Phases

### Phase 0: Requirements and Register Audit

- Identify exact SPI instance used on H7 AEAT hardware.
- Identify target F4 SPI peripheral constraints.
- Record required SPI mode, max clock, CS timing, frame length, and encoder latch behavior for AEAT-9955 and MT6835.
- Compare upstream STM32 SPI clear/disable/EOT sequences against STM32H7 and STM32F4 reference manuals.

Deliverable:

- Driver requirements note in this document or a companion markdown file.

#### Phase 0 Audit - 2026-05-01

Current AEAT-9955 hardware uses `spi3` on `smartstepper_v2/stm32h743xx`.

Resolved hardware facts:

- SPI peripheral: STM32H7 `SPI3`.
- MMIO base: `0x40003c00`.
- IRQ: `51`.
- Clock enables from STM32H7 devicetree: `STM32_CLOCK(APB1, 15)` plus `STM32_SRC_PLL1_Q SPI123_SEL(0)`.
- Pins from `boards/rubus/smartstepper_v2/smartstepper_v2.dts`:
  - SCK: `PC10`
  - MISO: `PC11`
  - MOSI: `PC12`
  - NSS: `PA15`
- Current AEAT profile overlay: `app/configs/motor_aeat9955_067a.overlay`.
- Current SPI configuration:
  - `fifo-enable` present.
  - no `cs-gpios`.
  - no `st,soft-nss`.
  - therefore upstream policy is hardware NSS output.
- Encoder node: `aeat9955@0`, `compatible = "brcm,aeat-9955"`.
- Encoder alias: `encoder1 = &aeat9955`.
- Current control-loop SPI command frame:
  - 3 bytes.
  - `tx[0] = AEAT9955_CMD_READ_SPI16 | ((~POPCOUNT(AEAT9955_REG_POS) & 1U) << 7)`.
  - `tx[1] = AEAT9955_REG_POS`.
  - `tx[2] = 0x00`.
- Current AEAT decode:
  - raw response length: 3 bytes.
  - position: bits `[21:4]` of the 24-bit response.
  - status/error bit: response byte 0 bit 6.
  - parity is odd/even check over the full 24-bit encoder response as implemented by `aeat9955_decode_position()`.
  - pipeline delay: one sample when using the AEAT pipelined position-read protocol.

Implementation constraint from this audit:

- The first fast transport instance should reference `&spi3` and follow the same CS policy as upstream STM32 SPI:
  - `cs-gpios` on the SPI node means software NSS plus GPIO CS.
  - `st,soft-nss` means software NSS without CS assertion.
  - neither means hardware NSS output.
- For the initial H7 validation path, `fifo-enable` on the low-latency transport enables the H7 FIFO backend.
- The existing sensor driver remains present for shell/property access; the fast transport must not call Zephyr SPI APIs from ISR context.

### Phase 1: Transport Interface Skeleton

- Add low-latency SPI transport headers.
- Add common state machine:
  - idle,
  - active,
  - complete,
  - error.
- Add stats struct.
- Add compile-time max frame length.
- Add a fake/native unit-test backend for state-machine tests if practical.

Validation:

- Native/unit tests for request/collect/busy/error state transitions.

### Phase 2: STM32H7 FIFO Backend

- Implement H7 LL backend.
- Use software CS.
- Use interrupt-driven FIFO refill/drain.
- Publish latest raw sample.
- Add transport stats.

Validation:

- Build H7 target.
- Logic analyzer: verify CS timing and frame bytes.
- HIL: request/collect at 20 kHz trigger rate without hard faults.
- HIL: raw trace shows no transport drops with motor disabled.

### Phase 3: AEAT-9955 Fast Frontend

- Implement AEAT frame preparation.
- Reuse or share AEAT decode/parity/status logic.
- Account for AEAT pipeline delay explicitly.
- Wire into motor encoder pipeline behind Kconfig/devicetree selection.

Validation:

- Compare fast-path angle against existing sensor shell at rest.
- Velocity-generated run: raw trace angle changes with expected sign/speed.
- Current-enabled run: parity errors are counted, not fatal transport crashes.

### Phase 4: H7 FIFO HIL Hardening

- Run repeated prepare/alignment cycles.
- Run generated velocity with fast-path trace.
- Run current-enabled encoder diagnostics.
- Verify CS timing, FIFO service timing, and raw frame integrity with logic analyzer.
- Tune SPI frequency, trigger phase, and optional raw-frame telemetry decimation.

Validation:

- No transport hard faults.
- No missed request/collect accounting.
- Transport errors, if injected or observed, are counted and recover cleanly.
- AEAT signal-integrity errors are visible as frame/parity errors only.

### Phase 5: STM32H7 Non-FIFO Backend

- Implement H7 non-FIFO backend selected by absence of `fifo-enable`.
- Follow upstream completion/flag-clear handling.
- Keep same API and encoder frontends.

Validation:

- Build H7 target with `fifo-enable` removed.
- Logic analyzer: verify CS timing and frame bytes.
- HIL smoke test at a conservative trigger rate.

### Phase 6: STM32F4 Byte Backend

- Implement F4 non-FIFO backend.
- Keep same transport API.
- Avoid H7-only register assumptions.

Validation:

- Build F4 board/profile.
- If hardware exists: logic analyzer frame check and basic request/collect test.
- If no hardware: compile-only plus unit-test backend coverage.

### Phase 7: MT6835 Fast Frontend

- Implement MT6835 frame preparation.
- Reuse CRC/status decode.
- Wire into motor encoder pipeline.

Validation:

- Compare fast-path angle against existing sensor shell at rest.
- Velocity-generated run: angle velocity matches commanded direction/speed.
- Current-enabled run: CRC/status counters remain clean or report cleanly.

### Phase 8: Control Integration

- Select encoder source by devicetree/Kconfig:
  - RTIO sensor path,
  - real-time SPI fast path.
- Keep control loop input type unchanged.
- Ensure angle observer owns wrap/offset/latency compensation.
- Ensure control ISR sees bounded stale/fault behavior.

Validation:

- Generated modes still work with fast encoder diagnostics enabled.
- Encoder current mode enters without RTIO/SPI hard fault.
- Encoder velocity/position modes fail gracefully on frame errors.

### Phase 9: Diagnostics and Operator Shell

- Add shell commands:
  - `motor encoder fast status`
  - `motor encoder fast stats`
  - `motor encoder fast reset`
  - `motor encoder trace start/stop/dump`
- Keep all formatting out of ISR.

Validation:

- Trace dump works after high-rate capture.
- Counters distinguish transport errors from parity/CRC/status errors.

### Phase 10: Cleanup and Default Policy

- Decide default encoder path per motor profile overlay.
- Keep RTIO path for shell/sensor diagnostics unless explicitly removed later.
- Document known-good HIL workflows.
- Remove obsolete experimental SPI code if no longer needed.

Validation:

- H7 AEAT build.
- H7 MT6835 build.
- Unit tests.
- HIL smoke test for generated velocity and encoder diagnostic capture.

## Acceptance Criteria

The work is complete when:

- H7 low-latency SPI transport runs at the required trigger rate without hard faults.
- AEAT parity/status errors are counted and do not corrupt transport state.
- MT6835 CRC/status errors are counted and do not corrupt transport state.
- Control ISR uses no Zephyr blocking/kernel queue APIs for encoder acquisition.
- Existing sensor shell path remains available for diagnostics.
- Encoder frontend decode is shared or behaviorally identical between sensor and fast paths.
- Raw trace can prove frame correctness and sample timing.

## Open Decisions

- Whether to put the transport under `drivers/rt_spi/`, `drivers/misc/`, or `app/src/`.
- Whether to expose it as a Zephyr device with a custom API or plain app-level singleton.
- Whether H7 and F4 should be separate source files selected by devicetree compatible, or one source with SoC conditionals.
- Whether CS should be normal GPIO only, or optionally hardware NSS after timing is verified.
- Whether the fast path should support one latest-sample slot only or always include an optional SPSC ring.

## Recommendation

Start with a custom Zephyr device using a private API:

```text
drivers/rt_spi/
  rt_spi.h
  rt_spi_stm32_h7.c
  rt_spi_stm32_f4.c
  Kconfig
  CMakeLists.txt
dts/bindings/rt_spi/rubus,stm32-rt-spi.yaml
```

Then add encoder frontends outside the transport:

```text
drivers/encoder/
  aeat9955_fast.c
  mt6835_fast.c
```

This keeps the real-time transport reusable while avoiding the complexity of a full Zephyr SPI driver.
