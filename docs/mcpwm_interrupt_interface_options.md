# MCPWM Interrupt Interface Design Options

## Background
The STM32 MCPWM fork currently exposes runtime control through the standard Zephyr `mcpwm_driver_api` entry points, plus a private break-callback helper that is not wired into the API table. There is no way to subscribe to timer output compare (OC) interrupts, even though the inline fast-path helpers depend on predictable timing. We want to introduce callbacks for OC events (per-channel) and make the break callback part of the public API while staying true to Zephyr design principles:

- Keep driver APIs small, capability-based, and explicit about optional features.
- Prefer device-API methods over ad-hoc globals when run-time registration is needed.
- Allow deferred configuration when possible (configure first, enable later) and expose feature probes (e.g. via `mcpwm_capabilities` or `device_is_ready`).
- Offer low-latency paths when justified, without locking out platforms that cannot supply direct interrupts.

The sections below weigh alternative interface designs for output compare callbacks, break-fault notification, and ISR binding strategy. Each option notes how well it fits Zephyr's conventions, expected complexity, and potential migration impact.

## Design Goals
- **Channel-level callbacks**: Applications should subscribe/unsubscribe per OC channel.
- **Optional feature flagging**: Drivers should advertise whether OC callbacks or break handling are supported.
- **Minimal latency**: Critical motor-control loops may require direct ISR dispatch.
- **Compatibility**: Existing API users should not break; new features must be additive.

## Output Compare Callback Registration

### Option A: Extend `.configure()` to accept callbacks
- **Concept**: Change `mcpwm->configure(dev, channel, flags)` to accept an optional descriptor struct that includes callback function and context pointer.
- **Pros**:
  - Single call sets OC mode and callback atomically.
  - Mirrors patterns like `counter_set_channel_alarm()` where configuration and handler are bundled.
  - No new API entry point required.
- **Cons**:
  - Breaks existing signature (ABI change for all controllers), requiring shim layers or a second API version.
  - `configure` semantics become overloaded (it currently modifies flags only).
  - Hard to update handler without reconfiguring PWM polarity/flags.
  - Zephyr convention generally keeps callbacks separate (e.g. GPIO uses `pin_interrupt_configure` + `gpio_add_callback`).
- **Zephyr fit**: Weak; most device APIs avoid signature changes that disrupt common calls.

### Option B: New API `set_compare_callback()`
- **Concept**: Add `int (*set_compare_callback)(const struct device *dev, uint32_t channel, mcpwm_compare_cb_t cb, void *user_data)` to the driver API table.
- **Pros**:
  - Backwards-compatible; existing code keeps working.
  - Clear separation of responsibility (configure channel first, then register handler).
  - Matches Zephyr precedent (GPIO `gpio_add_callback()`, counter `counter_set_channel_alarm()`, UART async callbacks).
  - Allows `NULL` to unregister without altering PWM configuration.
  - Device can report `-ENOTSUP` when OC interrupts are unavailable.
- **Cons**:
  - Slightly more boilerplate (two calls during bring-up).
  - Requires per-channel bookkeeping in the driver (callback tables, enable bits).
- **Zephyr fit**: Strong; aligns with capability-based optional hooks.

### Option C: Channel descriptor objects with `mcpwm_channel_config()`
- **Concept**: Introduce a new struct (mode, callback, DMA preferences) passed to a new `channel_config` entry point modeled after `pwm_set()` from upstream.
- **Pros**:
  - Extensible container for future options (dead-time per channel, complementary mapping).
  - Maintains atomic configuration and callback update without ABI churn on `.configure()`.
- **Cons**:
  - Adds a second configuration pathway alongside existing `.configure()`; the driver would need to support both until a wider refactor lands.
  - More complex state management (coexistence with current flags API).
  - Higher refactor cost for applications.
- **Zephyr fit**: Moderate; richer struct matches `pwm_led` style but diverges from current MCPWM API minimalism.

### Recommendation
Adopt **Option B** (`set_compare_callback`) as the primary path. It extends the API minimally, follows Zephyr conventions, and can coexist with existing `configure` and inline helpers. We can revisit a descriptor-based refactor once upstream PWM alignment is complete.

### DMA Request Enable Strategy
- **Current state**: Channel 4 enables DMA requests unconditionally inside `mcpwm_stm32_enable()` via `LL_TIM_EnableDMAReq_CC4()`.
- **Goal**: Allow applications to enable or disable DMA per channel and event source (compare, update, trigger) without hard-coding it in the driver.

#### Option DMA-1: Devicetree-controlled DMA masks
- **Concept**: Introduce DT properties (e.g. `st,cc-dma-mask`, `st,update-dma`, `st,trigger-dma`) that map to DIER bits. Driver reads them during init and configures baseline DMA enables.
- **Pros**:
  - Keeps DMA policy in board configuration; no runtime API impact.
  - Matches Zephyr expectation that static hardware wiring (DMA channel availability) lives in DT.
  - Easy for advanced timers where DMA routing is fixed.
- **Cons**:
  - Harder to toggle DMA on/off dynamically per motion phase.
  - Requires DT updates for every deployment that changes DMA usage.

#### Option DMA-2: Runtime API alongside callbacks
- **Concept**: Extend the new compare callback API with trigger configuration (e.g. `set_compare_action(dev, ch, flags, cb, user_data)` where `flags` include IRQ and/or DMA enable bits), or add a dedicated `set_compare_trigger()` function.
- **Pros**:
  - Fine-grained runtime control; applications can switch between IRQ and DMA paths.
  - Enables advanced scenarios (e.g. calibrate via IRQ, then switch to DMA streaming).
- **Cons**:
  - Slightly more complex API; need to validate interactions with callbacks.
  - Requires additional state tracking inside the driver (DIER bits per channel).

#### Option DMA-3: Hybrid (DT defaults + runtime arming)
- **Concept**: Use Devicetree to declare which compare events are wired for DMA, but keep the DIER bits disabled until firmware explicitly arms them through a lightweight runtime helper.
- **Pros**:
  - Sensible defaults without sacrificing flexibility; DMA routing remains discoverable in DT.
  - Avoids spurious DMA triggers during bring-up because requests stay masked until the control loop enables them.
  - Aligns with Zephyr PWM capture approach where DT ties pins but API toggles capture dynamically.
- **Cons**:
  - Requires one extra call at runtime to arm DMA; firmware must remember to disarm when the peripheral pipeline is torn down.

### DMA Recommendation
Adopt **Option DMA-3**: define a Devicetree property (e.g. `st,cc-dma-channels = <4>`) listing the compare outputs that should be permitted to raise DMA requests. The driver leaves all DIER bits masked during init, then uses a private helper inside `mcpwm_stm32_enable()/disable()` to arm or disarm the corresponding DMA request when a channel is toggled. This keeps the static wiring declarative, avoids exporting a new runtime API, and ensures requests are only asserted while the downstream peripherals are ready—preventing spurious DMA bursts without forcing applications to manage the bit directly.

## Break Fault Callback Integration

### Option D: Promote existing helper into API
- **Concept**: Export the current `mcpwm_stm32_register_break_callback()` through the API table (drop `static`, add function pointer).
- **Pros**:
  - No behavioural change; just exposes existing functionality to consumers.
  - Keeps API coherent with the new OC callback pattern (two symmetric setters).
  - Allows other SoCs (e.g. H7 with multiple break inputs) to reuse the hook.
- **Cons**:
  - Requires documentation explaining that the hook runs in ISR context and must be quick.
  - Drivers without break inputs must return `-ENOTSUP`.
- **Zephyr fit**: Strong; pattern mirrors `pwm_capture_cb_set()`.

### Option E: Fold break and compare into single event callback
- **Concept**: Provide one event handler that receives an enum (`MCPWM_EVENT_BREAK`, `MCPWM_EVENT_COMPARE_CHx`).
- **Pros**:
  - Fewer registration functions; consistent event dispatch.
  - Easier for apps to multiplex behaviour in one callback.
- **Cons**:
  - Conflates high-priority break faults with routine compare events; handler complexity increases.
  - Harder to disable one without the other; needs per-event enable bits anyway.
- **Zephyr fit**: Moderate; Zephyr typically keeps fault handlers separate (e.g. I2C, PWM capture).

### Recommendation
Proceed with **Option D** to keep the break callback explicit. Pairing it with the new compare setter gives callers fine control over critical fault handling.

## ISR Binding Strategy

### Option F: Shared ISR per timer (current pattern)
- **Concept**: Continue using a single IRQ handler (`mcpwm_stm32_brk_isr` plus a new compare ISR) invoked via `IRQ_CONNECT`, then fan out to registered callbacks.
- **Pros**:
  - Simple to maintain; consistent with current driver structure.
  - Works on SoCs lacking Zephyr direct-interrupt support.
  - Easier to guard with PM and runtime power domain code.
- **Cons**:
  - Adds small dispatch overhead before reaching the user handler.
  - Requires checking each channel flag inside ISR.
  - On advanced timers (TIM1/TIM8) we must install handlers for multiple IRQ lines (`brk`, `up`, `trgcom`, `cc`) and ensure each calls into the shared dispatcher appropriately.
  - On general timers (single "global" IRQ) all events enter through one vector, so the dispatcher needs to distinguish event types via SR/DIER bits.

- **Concept**: Bind each OC interrupt line directly to a small trampoline that jumps straight to the user callback (no `irq_enable` call stack).
- **Concept**: Bind each OC interrupt line directly to a small trampoline that jumps straight to the user callback (no `irq_enable` call stack).
- **Pros**:
  - Lowest possible latency; avoids logics in shared ISR.
  - Can place trampoline in RAM for deterministic timing.
- **Cons**:
  - Consumes additional vector slots; not all STM32 timers expose separate IRQ lines for each channel.
  - Harder to enable/disable dynamically; requires compile-time decisions or runtime reconfiguration of direct interrupts (which Zephyr limits).
  - Direct ISRs cannot call into regular kernel services without additional work (must `z_soc_irq_enable`/`irq_unlock`).
  - Diverges from Zephyr’s preference for shared IRQ dispatch unless latency demands justify it.

- **Pros**:
  - Default remains portable; advanced timers opt-in to direct CC vectors when available (e.g. using `IRQ_CONNECT` for global timers but `IRQ_DIRECT_CONNECT` for TIM1/TIM8 `cc` lines).
- **Concept**: Keep the shared handler but allow an optional `CONFIG_MCPWM_STM32_DIRECT_OC_ISR` that swaps in `IRQ_DIRECT_CONNECT` trampolines when the SoC provides dedicated CC interrupts.
- **Pros**:
  - Default remains portable; advanced users opt-in to lower latency.
  - Maintains a single callback registration API.
- **Cons**:
  - Adds build-time option matrix to test.
  - More conditional code paths; must ensure trampolines still reference current callback tables safely.

### Recommendation
Start with **Option F** (shared ISR) while designing the callback infrastructure, making sure the dispatcher can operate on both multi-vector (advanced timer) and single-vector (general timer) instances. Add hooks so TIM1/TIM8 can optionally leverage `IRQ_DIRECT_CONNECT` for their dedicated `cc` line if latency measurements justify it, but keep the default portable.

## Next Steps
1. Extend `struct mcpwm_driver_api` with two optional hooks:
   ```c
   int (*set_compare_callback)(const struct device *dev, uint32_t channel,
                               mcpwm_compare_cb_t cb, void *user_data);
   int (*set_break_callback)(const struct device *dev,
                             mcpwm_break_cb_t cb, void *user_data);
   ```
   Provide default `NULL` entries so other drivers remain unaffected.
2. Seed a private DMA arming helper from the new `st,cc-dma-channels` property so the driver toggles DIER bits when channels are enabled or disabled.
3. Implement these APIs in `mcpwm_stm32.c`, allocating per-channel callback tables, wiring them into the existing shared ISR, and honoring the internal DMA helper.
4. Update the header (`include/drivers/pwm/mcpwm_stm32.h`) with typedefs, capability flags, and documentation for ISR contexts and DMA behaviour.
5. Optionally add Kconfig toggles for future direct-ISR experiments once baseline functionality is in place.

This staged approach keeps the API evolution incremental, mirrors Zephyr best practices, and leaves room for performance-oriented enhancements after correctness is validated.
