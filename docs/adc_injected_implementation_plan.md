# STM32 Ultra Low-Latency ADC Driver Implementation Plan

## Executive Summary

This driver provides hardware-triggered injected channel ADC functionality for STM32 motor control applications while **maintaining maximum compatibility with upstream Zephyr code**.

**Key Strategy:** Reuse upstream hardware setup code, remove regular ADC operations, adapt for injected channels.

**Benefits:**
- ✅ Proven hardware initialization (clock, calibration, boost mode, etc.)
- ✅ Support for multiple STM32 families (F4, H7, etc.) with minimal effort
- ✅ Can pull upstream fixes for hardware setup issues
- ✅ Focused on injected mode - no unnecessary regular ADC code

**What to Keep from Upstream (Hardware Setup):**
- ADC enable/disable functions
- Calibration routines (H7 offset/linearity, F4 simple)
- Clock configuration (including H7 boost mode)
- Channel sampling time setup
- Differential channel support
- Power management (suspend/resume hooks)
- **~400 lines of hardware setup code**

**What to Remove (Regular ADC Operations):**
- `adc_context` and blocking/async read infrastructure (~50 lines)
- `start_read()`, `adc_read()`, `adc_read_async()` (~100 lines)
- Regular channel sequencer code (~50 lines)
- Regular ADC ISR (EOC/EOS handling) (~30 lines)
- PM policy locks in ISR (~5 lines)
- DMA support (~100 lines)
- **~335 lines removed**

**What to Add (Injected-Specific):**
- Injected sequence configuration from DT (~50 lines)
- Injected trigger setup (~30 lines)
- Zero-latency JEOS ISR (ISR_DIRECT_DECLARE) (~40 lines)
- Callback management (~20 lines)
- Enable/disable/set_callback API (~50 lines)
- Modified init to configure injected sequencer (~20 lines)
- **~210 lines custom code**

**Net Result:** ~400 reused + ~210 custom = ~610 lines total
- **65% reuse** of hardware setup code from upstream
- Clean separation: hardware setup vs injected operations
- Can pull upstream fixes for hardware issues, maintain injected code separately

## Overview
Create a dedicated ultra low-latency ADC driver for STM32F4/H7 using injected channel mode with zero-latency interrupts. This driver will be independent of the Zephyr ADC API and optimized for real-time motor control applications.

## Current Implementation Status

### ✅ Complete (Reused from Upstream)
- All ADC initialization (clock, pinctrl, calibration, boost mode)
- All helper functions (enable, disable, clock setup)
- All channel configuration (sampling time, differential mode, etc.)
- All power management hooks
- All family-specific code (F1/F3/F4/F7/H5/H7/L4/G0/U5/N6/etc.)
- All error handling and validation
- Device instantiation macros

### ✅ Complete (Custom Injected Code)
- API header file (`adc_injected.h`) with syscalls
- Basic enable/disable functions
- API table pointing to `adc_injected` subsystem (3 functions)
- Zero-latency ISR architecture defined

### ✅ Removed (Not Needed for Injected Mode)
- `adc_context` structure and all references (~50 lines removed)
- `start_read()`, `adc_read()`, `adc_read_async()` functions (~100 lines removed)
- Regular `adc_stm32_isr()` function (~30 lines removed, replaced with injected ISR)
- `pm_policy_state_lock_put()` calls in ISR (not used without `adc_context`)
- DMA support code (not applicable for injected mode)

**Total Removed:** ~200 lines of upstream code not needed for injected operation

### ⚠️ TODO (Minimal Custom Code Needed)
1. **Remove remaining regular ADC code** (~10 lines)
   - Remove `pm_policy` lock calls in existing ISR (lines 894-900)
   - Remove `adc_stm32_isr()` function completely (will be replaced)

2. **Channel configuration from devicetree** (~30 lines)
   - Parse DT for channel properties (acquisition time, differential mode)
   - Call upstream channel setup helpers for each channel
   - Called from `adc_stm32_init()` during initialization

3. **Injected sequence configuration from devicetree** (~30 lines)
   - Parse DT for injected channel list and ranks
   - Configure LL_ADC_INJ_SetSequencerRanks()
   - Configure trigger source/edge from DT
   - Call from `adc_stm32_init()` after channel setup

4. **Callback storage in driver data** (~10 lines)
   - Add `callback` and `user_data` fields to `adc_stm32_data`
   - Implement `adc_stm32_inj_set_callback()` to store them

5. **Zero-latency injected ISR (REQUIRED)** (~40 lines)
   - Declare with ISR_DIRECT_DECLARE (priority 0)
   - Read injected data registers based on num_channels from DT
   - Convert to Q31 format (resolution from DT)
   - Call registered callback directly from ISR
   - Callback executes FOC algorithm, updates PWM
   - Clear JEOS flag
   - No kernel calls in ISR (callback can't use kernel either)

6. **Devicetree binding updates** (~20 lines)
   - Make `st,stm32-adc-injected.yaml` inherit from upstream binding
   - Add `st,adc-injected-channels` property
   - Add `st,adc-trigger-source` property
   - Add `st,adc-trigger-edge` property

7. **Build system integration** (~10 lines)
   - Add CMakeLists.txt entry
   - Add Kconfig option

**Total TODO:** ~150 lines of custom code to add

## Supported Hardware
- **STM32F407**: ADC version 1 (ADC_VER_V1)
- **STM32H7xx**: ADC version 5 (ADC_VER_V5_X, ADC_VER_V5_V90, ADC_VER_V5_3)

## Design Goals

1. **Zero-latency interrupts (REQUIRED)**: ISR priority 0, un-maskable by kernel, < 500ns latency
2. **Ultra-low latency**: Total latency < 3µs (trigger to PWM update)
3. **Hardware-triggered**: TIM1 OC4 (or configurable) triggers ADC conversions
4. **Direct callback from ISR**: Zero-latency ISR reads ADC, invokes callback directly
5. **FOC in callback**: Complete algorithm runs in callback (called from zero-latency ISR)
6. **Independent from Zephyr ADC API**: Custom driver, not compatible with standard ADC API
7. **No DMA required**: Direct register reads in ISR

## Architecture Decisions

### Custom Subsystem API
**API Structure: Simple, purpose-built interface for motor control**

The driver exposes a custom API (not compatible with Zephyr's generic ADC API) optimized for zero-latency operation.

#### Configuration Flow

All sequence configuration (channel list, trigger source, trigger edge) is **defined statically in devicetree**. This eliminates runtime configuration overhead and ensures deterministic behavior.

**Devicetree defines:**
- Which channels to convert (in order)
- Hardware trigger source (e.g., `TIM1_CC4`)
- Trigger edge (rising/falling/both)
- Per-channel settings (acquisition time, etc.)

**Runtime API calls:**
1. `adc_injected_set_callback(dev, callback, user_data)` - Optional: register callback for notifications
2. `adc_injected_enable(dev)` - Enable hardware-triggered conversions
3. `adc_injected_disable(dev)` - Disable conversions

#### API Functions

**`adc_injected_set_callback(dev, callback, user_data)`**
- Optional: register callback for conversion complete notification
- Callback receives ADC values directly as parameters: `callback(dev, values[], count, user_data)`
- Values are Q31 fixed-point (0x7FFFFFFF = full-scale)
- **CRITICAL**: Callback runs in zero-latency ISR context (priority 0)
- **NO kernel calls allowed** in callback (no LOG_*, k_*, device APIs)
- Callback can update global variables or hardware registers directly

**`adc_injected_enable(dev)`**
- Enables hardware-triggered injected channel conversions
- Performs work similar to upstream's `start_read()`:
  - Ensures ADC peripheral is enabled
  - Enables JEOS (injected end-of-sequence) interrupt
  - Starts injected conversions (H7: explicit `LL_ADC_INJ_StartConversion()`)
- After this, ADC converts automatically on each hardware trigger event
- **No runtime sequence configuration** - uses sequence from devicetree

**`adc_injected_disable(dev)`**
- Stops responding to hardware triggers
- Disables JEOS interrupt
- Disables ADC peripheral

#### Key Design Points

1. **Pure devicetree configuration**: All channel setup (acquisition time, ranks, trigger) configured during init from DT
2. **No runtime channel configuration**: Everything comes from devicetree, no manual setup needed
3. **Minimal API surface**: Only 3 functions (vs. standard ADC API's larger surface)
4. **Values passed to callback**: No global variables, thread-safe design
5. **Zero-latency ISR**: Reads ADC registers, converts to Q31, calls callback directly
6. **Automatic initialization**: Driver configures all channels from DT during `adc_stm32_init()`

#### Complete Usage Example

```c
/* Devicetree fragment (defines the sequence statically) */
&adc1 {
    compatible = "st,stm32-adc-injected";
    status = "okay";
    st,adc-trigger-source = "tim1_cc4";  // Hardware trigger
    st,adc-trigger-edge = "rising";
    
    // Sequence: channel 3, then 10, then 11 (in that order)
    channel@3 {  // Rank 1
        reg = <3>;
        zephyr,acquisition-time = <ADC_ACQ_TIME(ADC_ACQ_TIME_TICKS, 2)>;
    };
    channel@10 { // Rank 2
        reg = <10>;
    };
    channel@11 { // Rank 3
        reg = <11>;
    };
};

/* Application code */
#include <zephyr/drivers/adc_injected.h>

// Callback runs in ZERO-LATENCY ISR CONTEXT - NO kernel calls allowed!
void adc_complete_callback(const struct device *dev, 
                          const q31_t *values, 
                          uint8_t count, 
                          void *user_data)
{
    // values[0] = channel 3 (phase A current)
    // values[1] = channel 10 (phase B current)  
    // values[2] = channel 11 (DC bus voltage)
    
    // CRITICAL: This runs in ISR context (priority 0)
    // Cannot use LOG_*, k_*, or any kernel functions!
    // Can only update global variables or hardware registers
    
    // Store to global for main thread to read:
    extern volatile q31_t g_phase_a, g_phase_b, g_v_bus;
    g_phase_a = values[0];
    g_phase_b = values[1];
    g_v_bus = values[2];
}

void motor_control_init(void)
{
    const struct device *adc = DEVICE_DT_GET(DT_NODELABEL(adc1));
    
    if (!device_is_ready(adc)) {
        return;
    }
    
    /* All channel configuration done automatically during driver init from DT:
     * - Channel 3, 10, 11 configured with acquisition times
     * - Trigger source (TIM1_CC4) and edge (rising) configured
     * - Injected sequence ranks assigned
     * No manual setup needed!
     */
    
    // Optional: set callback for notifications
    adc_injected_set_callback(adc, adc_complete_callback, NULL);
    
    // Enable hardware-triggered conversions
    // From now on, ADC converts on each TIM1_CC4 event
    adc_injected_enable(adc);
}

void motor_control_stop(void)
{
    const struct device *adc = DEVICE_DT_GET(DT_NODELABEL(adc1));
    
    // Stop conversions
    adc_injected_disable(adc);
}
```

**Data Flow:**
1. TIM1 generates CC4 event (hardware trigger) at 20kHz
2. ADC automatically converts channels 3, 10, 11 in sequence
3. Zero-latency ISR reads results, converts to Q31 format
4. ISR calls registered callback **directly** (callback runs in ISR context)
5. Callback stores values to globals or updates PWM (no kernel calls allowed)
6. ISR returns
7. Main thread can read global variables updated by callback

### Starting Point: Upstream Driver
**Strategy: Selectively reuse hardware setup code from upstream, implement injected-specific operations**

This driver **reuses proven hardware initialization code** while implementing a focused injected channel interface for motor control.

**Implementation approach:**
1. Copy `adc_stm32.c` to `adc_stm32_injected.c` **completely**
2. Change `DT_DRV_COMPAT` from `st_stm32_adc` to `st_stm32_adc_injected`
3. **Remove only these unused parts** (to avoid confusion):
   - `adc_context` references (not used for injected channels)
   - `start_read()`, `adc_read()`, `adc_read_async()` functions (regular ADC API, not used)
   - Regular ADC ISR `adc_stm32_isr()` (will be replaced with injected JEOS ISR)
   - PM policy lock/unlock calls in ISR (not relevant for injected mode)
   - DMA support code (not used for injected channels)
4. Keep **all essential upstream code**:
   - All helper functions (enable, disable, calibrate, clock setup, etc.)
   - All data structures (`adc_stm32_cfg`, `adc_stm32_data`)
   - All initialization code (`adc_stm32_init()`)
   - All channel setup code (`adc_stm32_channel_setup()`)
   - PM device support (suspend/resume hooks)
   - All family-specific code (F1/F3/F4/F7/H7/L4/G0/etc.)
5. Add **only 3 injected-specific functions**:
   - `adc_stm32_inj_set_callback()` - Store callback for notifications
   - `adc_stm32_inj_enable()` - Enable JEOS interrupt and start conversions
   - `adc_stm32_inj_disable()` - Disable JEOS interrupt and stop ADC
6. Add **custom injected ISR**:
   - `adc_stm32_inj_isr()` or `ISR_DIRECT_DECLARE(adc_stm32_inj_isr_direct)`
   - Handles only JEOS (injected end-of-sequence) interrupt
   - Reads injected data registers, converts to Q31, stores values
   - Triggers callback notification (flag or work queue)
7. Create new API table `api_stm32_driver_api` with type `adc_injected`:
   - Points to upstream `adc_stm32_channel_setup()` (reused as-is)
   - Points to injected-specific callback/enable/disable functions

**Why reuse hardware setup code:**
- ✓ **Proven initialization** - calibration, boost mode, clocking already working for all families
- ✓ **Multi-family support** - F4/H7/etc. differences already handled
- ✓ **Can pull upstream fixes** - hardware setup bugs get fixed by upstream
- ✓ **Focus effort** - spend time on injected-specific code, not hardware quirks
- ✓ **Reduced code size** - only ~610 lines total vs ~1400 for full driver

**Code removed from upstream (not needed for injected mode):**
- `adc_context` and all related code (not used for hardware-triggered injected mode)
- `start_read()`, `adc_read()`, `adc_read_async()` (regular ADC API, not applicable)
- `adc_stm32_isr()` (regular ADC ISR - replaced with injected-specific ISR)
- `pm_policy_state_lock_put()` calls in ISR (not relevant without `adc_context`)
- DMA support code (not used for injected channels, read directly in ISR)

**Code reused from upstream (no modifications):**
- All helper functions: `adc_stm32_enable()`, `adc_stm32_disable()`, `adc_stm32_calibrate()`, etc.
- Clock configuration: `adc_stm32_set_clock()`, boost mode setup for H7
- Channel setup: `adc_stm32_channel_setup()` handles all families correctly
- Sampling time management for all STM32 variants
- Differential channel support (H7/L4/etc.)
- Power management hooks (suspend/resume, not ISR locks)
- All device tree macros and instance generation

**Custom code added (injected-specific):**
- `adc_stm32_inj_configure_channels()` - Configures all channels from DT during init
- `adc_stm32_inj_configure_sequence()` - Configures injected sequence/trigger from DT during init
- `adc_stm32_inj_set_callback()` - Stores callback pointer in driver data
- `adc_stm32_inj_enable()` - Calls upstream `adc_stm32_enable()`, then enables JEOS interrupt
- `adc_stm32_inj_disable()` - Disables JEOS interrupt, calls upstream `adc_stm32_disable()`
- `ISR_DIRECT_DECLARE()` - Zero-latency ISR, reads injected values, calls callback
- API table pointing to `adc_injected` subsystem (3 functions, no channel_setup)
- DT parsing macros for channel properties and sequence configuration

**What this means:**
- The driver is ~65% reused hardware setup, ~35% custom injected code
- Hardware setup code can pull upstream fixes (calibration bugs, new families, etc.)
- Injected operations are independent and optimized for motor control
- Clean architecture: proven low-level hardware code + focused high-level injected API

**Key Differences Between F4 and H7:**
| Feature | F407 (ADC v1) | H7 (ADC v5) |
|---------|---------------|-------------|
| Calibration | Not required | Required (offset & linear) |
| Boost Mode | N/A | Required for >25MHz |
| Resolution | 12-bit only | 8/10/12/14/16-bit |
| Clock Source | APB2/2,4,6,8 | Kernel clock (async) |
| Trigger Sources | Different LL constants | Different LL constants |
| JEOS Flag | Same LL function | Same LL function |
| **Data Read** | **LL_ADC_INJ_ReadConversionData32** | **LL_ADC_INJ_ReadConversionData32** |

**Note:** Both families support `LL_ADC_INJ_ReadConversionData32()` - use this consistently!

## Implementation Phases

**Strategy: Reuse upstream hardware setup, build injected-specific operations on top**

### Phase 1: Foundation (Day 1)

#### 1.0 Copy Upstream Driver (Keep Everything!)
**First step: Create `adc_stm32_injected.c` by copying `zephyr/drivers/adc/adc_stm32.c`**

```bash
cd chopper/drivers/adc
cp /workspace/zephyr/drivers/adc/adc_stm32.c adc_stm32_injected.c
```

**Then selectively keep/remove/modify:**
1. Change line 12: `#define DT_DRV_COMPAT st_stm32_adc` → `#define DT_DRV_COMPAT st_stm32_adc_injected`
2. **Keep hardware setup code:**
   - `adc_stm32_enable()`, `adc_stm32_disable()`
   - `adc_stm32_calibrate()` (all families)
   - `adc_stm32_set_clock()`, `adc_stm32h7_setup_boost()`
   - `adc_stm32_channel_setup()`, sampling time functions
   - PM suspend/resume hooks
   - All family-specific initialization
3. **Remove regular ADC operations:**
   - `adc_context` structure and all references
   - `start_read()`, `adc_read()`, `adc_read_async()`
   - Regular channel sequencer code
   - Regular ADC ISR (`adc_stm32_isr`)
   - DMA support code
   - PM policy locks in ISR
4. **Add injected-specific code:**
   - Injected sequence configuration from DT
   - Injected trigger setup
   - Injected JEOS ISR
   - API functions: `channel_setup`, `set_callback`, `enable`, `disable`
5. **Modify init function:**
   - Keep upstream hardware setup
   - Add injected sequencer configuration
   - Add trigger source/edge setup
6. Create API table pointing to `adc_injected` subsystem

#### 1.1 Device Tree Bindings
Create custom bindings in `dts/bindings/adc/st,stm32-adc-injected.yaml`:

```yaml
description: STM32 ADC Injected Channel Configuration (F4/H7)

compatible: "st,stm32-adc-injected"

include: [adc-controller.yaml, pinctrl-device.yaml]

properties:
  trigger-source:
    type: string
    required: true
    description: Hardware trigger source for injected channels
    enum:
      - "tim1_trgo"
      - "tim1_cc4"
      - "tim8_trgo"
      - "tim8_cc4"
      # Add more as needed
  
  trigger-edge:
    type: string
    default: "rising"
    enum:
      - "rising"
      - "falling"
      - "both"
  
  scan-direction:
    type: string
    default: "forward"
    enum:
      - "forward"
      - "backward"

child-binding:
  description: ADC Injected Channel Configuration
  
  properties:
    reg:
      type: int
      required: true
      description: ADC channel number (0-19)
    
    st,offset:
      type: int
      description: Offset value for this channel (12-bit)
    
    differential:
      type: boolean
      description: Configure channel in differential mode
```

#### 1.2 Device Tree Examples

**STM32H7 Example:**
```dts
&adc1 {
    compatible = "st,stm32-adc-injected";
    status = "okay";
    trigger-source = "tim1_cc4";
    trigger-edge = "rising";
    
    /* H7-specific ADC configuration */
    st,adc-clock-source = "SYNC";
    st,adc-prescaler = <4>;
    
    #address-cells = <1>;
    #size-cells = <0>;
    
    /* Phase A current */
    channel@3 {
        reg = <3>;
        zephyr,gain = "ADC_GAIN_1";
        zephyr,reference = "ADC_REF_INTERNAL";
        zephyr,acquisition-time = <ADC_ACQ_TIME(ADC_ACQ_TIME_TICKS, 2)>;
    };
    
    /* Phase B current */
    channel@10 {
        reg = <10>;
    };
    
    /* DC Bus voltage */
    channel@11 {
        reg = <11>;
    };
};
```

**STM32F407 Example:**
```dts
&adc1 {
    compatible = "st,stm32-adc-injected";
    status = "okay";
    trigger-source = "tim1_cc4";
    trigger-edge = "rising";
    
    #address-cells = <1>;
    #size-cells = <0>;
    
    /* Phase A current - ADC1_IN0 */
    channel@0 {
        reg = <0>;
        zephyr,gain = "ADC_GAIN_1";
        zephyr,reference = "ADC_REF_INTERNAL";
        zephyr,acquisition-time = <ADC_ACQ_TIME(ADC_ACQ_TIME_TICKS, 15)>;
    };
    
    /* Phase B current - ADC1_IN1 */
    channel@1 {
        reg = <1>;
    };
    
    /* DC Bus voltage - ADC1_IN2 */
    channel@2 {
        reg = <2>;
    };
};
```

#### 1.3 Driver Structure
The driver keeps upstream structures unchanged and reuses them:

```c
#define DT_DRV_COMPAT st_stm32_adc_injected  /* ONLY CHANGE from upstream */

/* All upstream includes kept as-is */
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/pinctrl.h>
/* ... all other includes unchanged ... */

/* All upstream structures kept as-is */
struct adc_stm32_data {
    const struct device *dev;
    adc_data_size_t *buffer;
    adc_data_size_t *repeat_buffer;
    uint8_t resolution;
    uint32_t channels;
    uint8_t channel_count;
    uint8_t samples_count;
    int8_t acq_time_index[2];
    
    /* TODO: Add injected-specific fields here later:
     * adc_injected_callback_t callback;
     * void *user_data;
     * volatile q31_t values[4];
     */
};

struct adc_stm32_cfg {
    /* All upstream config fields unchanged */
    ADC_TypeDef *base;
    void (*irq_cfg_func)(void);
    const struct stm32_pclken pclken;
    /* ... all other fields from upstream ... */
    
    /* NEW: Injected-specific configuration from devicetree */
    uint8_t num_channels;         /* Number of injected channels (1-4) */
    uint32_t channel_ranks[4];    /* LL_ADC_INJ_RANK_1..4 for each channel */
    uint32_t channel_ids[4];      /* ADC channel numbers (0-19) */
    uint32_t channel_acq_times[4]; /* Acquisition time for each channel */
    bool channel_differential[4]; /* Differential mode flags */
    uint32_t trigger_source;      /* LL_ADC_INJ_TRIG_xxx constant */
    uint32_t trigger_edge;        /* LL_ADC_INJ_TRIG_EXT_xxx constant */
};

/* All upstream helper functions kept unchanged - examples: */
static int adc_stm32_enable(ADC_TypeDef *adc) { /* upstream code */ }
static void adc_stm32_disable(ADC_TypeDef *adc) { /* upstream code */ }
static int adc_stm32_calibrate(const struct device *dev) { /* upstream code */ }
static int adc_stm32_channel_setup(const struct device *dev,
                                   const struct adc_channel_cfg *cfg) 
{ /* upstream code */ }
/* ... hundreds more lines of upstream code ... */

/* NEW: Add only 3 injected-specific functions at end */
static int adc_stm32_inj_set_callback(const struct device *dev,
                                      adc_injected_callback_t callback,
                                      void *user_data)
{
    /* TODO: Store in adc_stm32_data */
    return -ENOSYS;
}

static int adc_stm32_inj_enable(const struct device *dev)
{
    const struct adc_stm32_cfg *config = dev->config;
    ADC_TypeDef *adc = config->base;
    int err;
    
    /* Reuse upstream enable function */
    err = adc_stm32_enable(adc);
    if (err < 0) {
        return err;
    }
    
    /* Add injected-specific: enable JEOS interrupt */
    LL_ADC_EnableIT_JEOS(adc);
    
    /* Start injected conversions (H7 requires explicit start) */
    #if !DT_HAS_COMPAT_STATUS_OKAY(st_stm32f1_adc) && \
        !DT_HAS_COMPAT_STATUS_OKAY(st_stm32f4_adc)
    LL_ADC_INJ_StartConversion(adc);
    #endif
    
    return 0;
}

static int adc_stm32_inj_disable(const struct device *dev)
{
    const struct adc_stm32_cfg *config = dev->config;
    ADC_TypeDef *adc = config->base;
    
    /* Disable JEOS interrupt */
    LL_ADC_DisableIT_JEOS(adc);
    
    /* Reuse upstream disable function */
    adc_stm32_disable(adc);
    
    return 0;
}

/* MODIFIED: Change API table to point to adc_injected subsystem */
static DEVICE_API(adc_injected, api_stm32_driver_api) = {
    .set_callback = adc_stm32_inj_set_callback,
    .enable = adc_stm32_inj_enable,
    .disable = adc_stm32_inj_disable,
};

/* All upstream device instantiation macros kept unchanged */
```

**Key insight:** The driver is ~1400 lines from upstream + ~50 lines custom = minimal delta!

### Phase 2: Initialization (Day 1-2)

#### 2.1 ADC Peripheral Setup
**No changes needed** - reuse upstream `adc_stm32_init()` completely!

The upstream initialization already handles:
- Clock enable for all STM32 families
- Pinctrl configuration
- H7: Kernel clock, boost mode, calibration (offset + linearity)
- F4/F1: Common register setup, prescaler
- Internal regulator startup delays
- Deep power-down disable (U5/N6)
- All error handling and validation

The upstream init is called automatically by the device instantiation macros at the end of the file. We don't modify it at all.

**What makes it work for injected channels:**
- The init doesn't know or care about regular vs injected
- It just sets up the ADC peripheral in a ready state
- Our custom `adc_stm32_inj_enable()` adds the injected-specific parts:
  - Enables JEOS interrupt
  - Starts injected conversions
  - (Injected channel sequence configured in devicetree, applied during init)

#### 2.2 Channel Configuration from Devicetree
**All channel configuration happens automatically during init from devicetree**

The driver parses DT child nodes during initialization and configures each channel:
- Channel number (from `reg` property)
- Acquisition time (from `zephyr,acquisition-time` property)
- Differential mode (from `differential` property if present)
- Resolution (from `zephyr,resolution` property)

```c
/* Parse DT and configure channels during init */
static int adc_stm32_inj_configure_channels(const struct device *dev)
{
    const struct adc_stm32_cfg *config = dev->config;
    ADC_TypeDef *adc = config->base;
    
    /* Configure each channel from DT */
    for (uint8_t i = 0; i < config->num_channels; i++) {
        uint32_t channel = config->channel_ids[i];
        uint32_t acq_time = config->channel_acq_times[i];
        bool differential = config->channel_differential[i];
        
        /* Reuse upstream helper for hardware setup */
        adc_stm32_setup_channel(adc, channel, acq_time, differential);
    }
    
    return 0;
}
```

This is called automatically from `adc_stm32_init()` - no runtime API needed!

#### 2.3 Injected Channel Sequence Configuration
**Add devicetree parsing for injected channel sequence**

This code configures the injected sequencer from DT during initialization:

```c
/* Add this helper function to parse DT and configure injected sequence */
static int adc_stm32_inj_configure_sequence(const struct device *dev)
{
    const struct adc_stm32_cfg *config = dev->config;
    ADC_TypeDef *adc = config->base;
    
    /* Set number of injected channels (from DT, stored in config) */
    LL_ADC_INJ_SetSequencerLength(adc, config->num_channels);
    
    /* Configure each rank (channel assignment and sampling time)
     * The ranks (LL_ADC_INJ_RANK_1..4) are stored in config->channel_ranks[]
     * during device instantiation from DT child nodes
     */
    for (uint8_t i = 0; i < config->num_channels; i++) {
        uint32_t rank = config->channel_ranks[i];
        uint32_t channel = config->channel_ids[i];  /* ADC channel number (0-19) */
        
        LL_ADC_INJ_SetSequencerRanks(adc, rank, channel);
    }
    
    /* Configure trigger source from DT (e.g., TIM1_CC4) */
    LL_ADC_INJ_SetTriggerSource(adc, config->trigger_source);
    
    /* Configure trigger edge from DT (rising/falling/both) */
    LL_ADC_INJ_SetTriggerEdge(adc, config->trigger_edge);
    
    return 0;
}

/* Add call to this in adc_stm32_init() after upstream init completes */
static int adc_stm32_init(const struct device *dev)
{
    /* ... all upstream initialization code ... */
    
    /* Configure channels from devicetree (acquisition time, differential mode) */
    err = adc_stm32_inj_configure_channels(dev);
    if (err < 0) {
        return err;
    }
    
    /* Configure injected sequence from devicetree (ranks, trigger) */
    err = adc_stm32_inj_configure_sequence(dev);
    if (err < 0) {
        return err;
    }
    
    return 0;
}
```

**Devicetree Parsing (during device instantiation):**
The config structure fields are populated from DT using macros:
```c
#define ADC_STM32_INJ_INIT(index)                                           \
    static const struct adc_stm32_cfg adc_stm32_cfg_##index = {            \
        /* ... upstream fields ... */                                       \
        .num_channels = DT_INST_CHILD_NUM(index),                          \
        .channel_ranks = DT_INST_FOREACH_CHILD_SEP(index,                  \
            ADC_CHANNEL_RANK_INIT, (,)),                                    \
        .channel_ids = DT_INST_FOREACH_CHILD_SEP(index,                    \
            ADC_CHANNEL_ID_INIT, (,)),                                      \
        .trigger_source = DT_INST_PROP(index, trigger_source),             \
        .trigger_edge = DT_INST_PROP(index, trigger_edge),                 \
    };
```

The devicetree binding specifies:
- Injected channel list (up to 4 channels from child nodes)
- Hardware trigger source (TIM1_CC4, etc.)
- Trigger edge (rising/falling/both)

These get parsed during device instantiation and used by init to configure the hardware.

### Phase 3: Injected ISR Implementation (Day 2)

#### 3.1 Remove Regular ADC ISR
**Delete the upstream regular ADC ISR** - it handles EOC/EOS for regular channels, not needed:

```c
/* REMOVE THIS FUNCTION (upstream regular ADC ISR, ~30 lines): */
static void adc_stm32_isr(const struct device *dev)
{
    /* ... handles regular channel conversions ... */
    /* ... pm_policy_state_lock_put() calls ... */
    /* ... not used for injected mode ... */
}
```

#### 3.2 Add Zero-Latency Injected JEOS ISR
Implement zero-latency ISR with ISR_DIRECT_DECLARE:

```c
/* Zero-latency ISR - handles ONLY injected end-of-sequence */
ISR_DIRECT_DECLARE(adc_stm32_inj_isr_direct)
{
    const struct device *dev = DEVICE_DT_INST_GET(0);
    const struct adc_stm32_cfg *config =
        (const struct adc_stm32_cfg *)dev->config;
    const struct adc_stm32_data *data =
        (const struct adc_stm32_data *)dev->data;
    ADC_TypeDef *adc = config->base;
    
    /* Optional: GPIO toggle for timing measurement */
    /* GPIOB->BSRR = GPIO_PIN_0;  Set high at entry */
    
    /* Check for injected end-of-sequence */
    if (LL_ADC_IsActiveFlag_JEOS(adc)) {
        LL_ADC_ClearFlag_JEOS(adc);
        
        /* Read injected data registers based on configured sequence
         * The number of channels and their rank assignments come from DT
         * config->num_channels tells us how many to read
         * config->channel_ranks[] maps channel index to LL_ADC_INJ_RANK_x
         */
        uint32_t num_channels = config->num_channels;
        uint32_t raw_values[4];  /* Max 4 injected channels */
        q31_t values[4];
        
        /* Read each configured rank (works on both F4 and H7) */
        for (uint32_t i = 0; i < num_channels; i++) {
            uint32_t rank = config->channel_ranks[i];  /* From DT: LL_ADC_INJ_RANK_1..4 */
            raw_values[i] = LL_ADC_INJ_ReadConversionData32(adc, rank);
            
            /* Convert to Q31 format based on resolution
             * 12-bit: raw << 19
             * 14-bit: raw << 17
             * 16-bit: raw << 15
             */
            uint8_t shift = 31 - data->resolution;  /* resolution stored in bits */
            values[i] = (q31_t)(raw_values[i] << shift);
        }
        
        /* Call registered callback directly from zero-latency ISR
         * Callback must follow zero-latency constraints:
         * - No kernel calls (k_*, LOG_*, etc.)
         * - No blocking operations
         * - Can update PWM registers directly
         * - FOC algorithm runs here at 20kHz
         */
        extern adc_injected_callback_t g_adc_callback;
        extern void *g_adc_user_data;
        
        if (g_adc_callback) {
            /* Callback receives Q31 values and executes FOC */
            g_adc_callback(NULL, values, num_channels, g_adc_user_data);
        }
    }
    
    /* Check for overrun */
    if (LL_ADC_IsActiveFlag_OVR(adc)) {
        LL_ADC_ClearFlag_OVR(adc);
        extern volatile uint32_t g_adc_overrun_count;
        g_adc_overrun_count++;
    }
    
    /* GPIOB->BSRR = (GPIO_PIN_0 << 16);  Set low at exit */
    
    return 1;  /* Must return 1 if interrupt handled */
}

/* ISR setup - zero-latency configuration */
#define ADC_STM32_INJ_IRQ_CONFIG(index)                                     \
static void adc_stm32_inj_irq_config_##index(void)                          \
{                                                                            \
    IRQ_DIRECT_CONNECT(DT_INST_IRQN(index),                                 \
                       0,  /* Priority 0 = zero-latency */                   \
                       adc_stm32_inj_isr_direct,                            \
                       0);                                                   \
    irq_enable(DT_INST_IRQN(index));                                        \
}
```

#### 3.3 Testing Zero-Latency ISR
Validation points:
- ✓ Verify trigger timing with scope
- ✓ Confirm all channels read correctly
- ✓ Validate ADC values with known inputs
- ✓ Measure ISR latency (< 500ns from trigger to ISR entry)
- ✓ Verify no kernel function calls in ISR
- ✓ Check Q31 conversion accuracy
- ✓ Confirm ISR execution time < 2µs total
- ✓ Use GPIO toggles for timing (no logging in ISR)

### Phase 4: Motor Control Integration (Day 3)

This phase shows complete end-to-end usage of the injected ADC API in a real motor control application.

#### 4.1 Complete Motor Control Application Example

```c
/* motor_control.c - Complete FOC motor control with injected ADC */

#include <zephyr/kernel.h>
#include <zephyr/drivers/adc_injected.h>
#include <zephyr/drivers/mcpwm.h>
#include <zephyr/logging/log.h>
#include <arm_math.h>

LOG_MODULE_REGISTER(motor_control, LOG_LEVEL_INF);

/* Device tree references */
static const struct device *adc_dev = DEVICE_DT_GET(DT_NODELABEL(adc1));
static const struct device *mcpwm_dev = DEVICE_DT_GET(DT_NODELABEL(mcpwm1));

/* Motor control state */
struct motor_state {
    /* Current measurements (Q31 format: 0x7FFFFFFF = full scale) */
    q31_t i_phase_a;
    q31_t i_phase_b;
    q31_t v_bus;
    
    /* Control setpoints */
    q31_t i_d_ref;  /* d-axis current reference (field) */
    q31_t i_q_ref;  /* q-axis current reference (torque) */
    
    /* PI controller states */
    q31_t i_d_integral;
    q31_t i_q_integral;
    
    /* Rotor position (electrical angle, 0 to 0x7FFFFFFF = 0 to 2π) */
    q31_t theta_elec;
    
    /* Statistics */
    uint32_t update_count;
    uint32_t overrun_count;
};

static struct motor_state motor = {0};

/* ADC callback - called from zero-latency ISR context!
 * CRITICAL: This callback runs in ISR context at priority 0
 * - No kernel calls allowed (k_*, LOG_*, etc.)
 * - No blocking operations
 * - Can access hardware registers directly
 * - Must be fast (< 2µs execution time)
 */
static void adc_values_ready(const struct device *dev,
                             const q31_t *values,
                             uint8_t count,
                             void *user_data)
{
    /* Store latest ADC values */
    motor.i_phase_a = values[0];  /* Phase A current */
    motor.i_phase_b = values[1];  /* Phase B current */
    motor.v_bus = values[2];      /* DC bus voltage */
    
    motor.update_count++;
    
    /* ===== FOC Algorithm - runs at ADC sample rate (20kHz) ===== */
    
    /* Calculate phase C current (Kirchhoff's law: i_a + i_b + i_c = 0) */
    q31_t i_c = -(motor.i_phase_a + motor.i_phase_b);
    
    /* Clarke transform: 3-phase ABC -> 2-phase αβ stationary frame
     * α = i_a
     * β = (i_a + 2*i_b) / √3
     */
    q31_t i_alpha = motor.i_phase_a;
    q31_t i_beta = (q31_t)(((int64_t)motor.i_phase_a + 2*(int64_t)motor.i_phase_b) * 0x4A5FC72C >> 31);
    
    /* Park transform: stationary αβ -> rotating dq frame
     * Uses current rotor position (from encoder)
     * d = α*cos(θ) + β*sin(θ)
     * q = -α*sin(θ) + β*cos(θ)
     */
    q31_t cos_theta = arm_cos_q31(motor.theta_elec);
    q31_t sin_theta = arm_sin_q31(motor.theta_elec);
    
    q31_t i_d, i_q;
    arm_mult_q31(&i_alpha, &cos_theta, &i_d, 1);
    q31_t temp;
    arm_mult_q31(&i_beta, &sin_theta, &temp, 1);
    i_d += temp;
    
    arm_negate_q31(&sin_theta, &sin_theta, 1);
    arm_mult_q31(&i_alpha, &sin_theta, &i_q, 1);
    arm_mult_q31(&i_beta, &cos_theta, &temp, 1);
    i_q += temp;
    
    /* PI controllers for d and q axes
     * v_d = Kp*(i_d_ref - i_d) + Ki*∫(i_d_ref - i_d)
     * v_q = Kp*(i_q_ref - i_q) + Ki*∫(i_q_ref - i_q)
     */
    q31_t e_d = motor.i_d_ref - i_d;
    q31_t e_q = motor.i_q_ref - i_q;
    
    const q31_t Kp = 0x10000000;  /* 0.125 in Q31 */
    const q31_t Ki = 0x00800000;  /* Small integral gain */
    
    motor.i_d_integral += (q31_t)(((int64_t)e_d * Ki) >> 31);
    motor.i_q_integral += (q31_t)(((int64_t)e_q * Ki) >> 31);
    
    q31_t v_d = (q31_t)(((int64_t)e_d * Kp) >> 31) + motor.i_d_integral;
    q31_t v_q = (q31_t)(((int64_t)e_q * Kp) >> 31) + motor.i_q_integral;
    
    /* Inverse Park: rotating dq -> stationary αβ
     * α = d*cos(θ) - q*sin(θ)
     * β = d*sin(θ) + q*cos(θ)
     */
    q31_t v_alpha, v_beta;
    arm_mult_q31(&v_d, &cos_theta, &v_alpha, 1);
    arm_mult_q31(&v_q, &sin_theta, &temp, 1);
    v_alpha -= temp;
    
    arm_mult_q31(&v_d, &sin_theta, &v_beta, 1);
    arm_mult_q31(&v_q, &cos_theta, &temp, 1);
    v_beta += temp;
    
    /* Space Vector PWM: αβ -> ABC duty cycles
     * Inverse Clarke transform to get 3-phase duty cycles
     * 
     * duty_a = (v_alpha) / V_bus
     * duty_b = (-v_alpha/2 + sqrt(3)*v_beta/2) / V_bus
     * duty_c = (-v_alpha/2 - sqrt(3)*v_beta/2) / V_bus
     * 
     * Then add 0.5 offset to convert from [-0.5, 0.5] to [0, 1]
     */
    
    /* Scale by bus voltage and convert to duty cycle [0, 1] in Q31 */
    q31_t v_scale = (q31_t)(((int64_t)v_alpha << 31) / motor.v_bus);
    q31_t duty_a = v_scale + 0x40000000; /* +0.5 offset */
    
    /* duty_b = (-v_alpha/2 + sqrt(3)*v_beta/2) / V_bus + 0.5 */
    q31_t half_alpha = v_alpha >> 1;
    q31_t sqrt3_beta;
    arm_mult_q31(&v_beta, (q31_t *)&(const q31_t){0x6ED9EBA1}, &sqrt3_beta, 1); /* sqrt(3)/2 */
    q31_t v_b = -half_alpha + sqrt3_beta;
    v_scale = (q31_t)(((int64_t)v_b << 31) / motor.v_bus);
    q31_t duty_b = v_scale + 0x40000000;
    
    /* duty_c = (-v_alpha/2 - sqrt(3)*v_beta/2) / V_bus + 0.5 */
    q31_t v_c = -half_alpha - sqrt3_beta;
    v_scale = (q31_t)(((int64_t)v_c << 31) / motor.v_bus);
    q31_t duty_c = v_scale + 0x40000000;
    
    /* Update PWM outputs with calculated duty cycles
     * IMPORTANT: Cannot use mcpwm_set_duty_cycle() from ISR context!
     * Must write TIM registers directly.
     * 
     * Assuming TIM1 ARR (auto-reload) = 1000 for 20kHz at 200MHz
     * duty_cycle (Q31) needs conversion to CCR value (0-1000)
     */
    
    /* Convert Q31 duty cycle (0x00000000 to 0x7FFFFFFF = 0 to 1.0)
     * to timer compare value (0 to ARR)
     */
    uint32_t arr = TIM1->ARR;  /* Get auto-reload value */
    
    /* duty_a is Q31 (0 to 0x7FFFFFFF), convert to 0-ARR range */
    uint32_t ccr_a = (uint32_t)(((uint64_t)duty_a * arr) >> 31);
    uint32_t ccr_b = (uint32_t)(((uint64_t)duty_b * arr) >> 31);
    uint32_t ccr_c = (uint32_t)(((uint64_t)duty_c * arr) >> 31);
    
    /* Update timer compare registers directly */
    TIM1->CCR1 = ccr_a;  /* Phase A PWM */
    TIM1->CCR2 = ccr_b;  /* Phase B PWM */
    TIM1->CCR3 = ccr_c;  /* Phase C PWM */
}

/* Initialize motor control subsystem */
int motor_control_init(void)
{
    int ret;
    
    /* Verify ADC device is ready */
    if (!device_is_ready(adc_dev)) {
        LOG_ERR("ADC device not ready");
        return -ENODEV;
    }
    
    LOG_INF("Initializing motor control ADC...");
    
    /* All ADC channel configuration already done during driver init from DT:
     * - Channel 3, 10, 11 configured with acquisition times
     * - Injected sequence ranks assigned (RANK_1, RANK_2, RANK_3)
     * - Trigger source (TIM1_CC4) and edge (rising) configured
     * - Differential/single-ended mode set per channel
     * 
     * No manual setup needed - devicetree defines everything!
     */
    
    /* Register callback for ADC completion notifications
     * Callback receives Q31 values directly
     * Called from main thread context (not ISR)
     */
    ret = adc_injected_set_callback(adc_dev, adc_values_ready, NULL);
    if (ret < 0) {
        LOG_ERR("Failed to set callback: %d", ret);
        return ret;
    }
    
    LOG_INF("ADC channels configured, callback registered");
    
    return 0;
}

/* Start motor control (enable ADC conversions) */
int motor_control_start(void)
{
    int ret;
    
    LOG_INF("Starting motor control...");
    
    /* Enable hardware-triggered ADC conversions
     * After this call:
     * - ADC responds to TIM1_CC4 trigger events
     * - ISR reads values on each JEOS interrupt
     * - Callback is invoked with new values
     */
    ret = adc_injected_enable(adc_dev);
    if (ret < 0) {
        LOG_ERR("Failed to enable ADC: %d", ret);
        return ret;
    }
    
    LOG_INF("Motor control running (ADC enabled)");
    
    return 0;
}

/* Stop motor control (disable ADC conversions) */
int motor_control_stop(void)
{
    int ret;
    
    LOG_INF("Stopping motor control...");
    
    /* Disable ADC conversions
     * After this call:
     * - ADC stops responding to trigger events
     * - No more ISR invocations
     * - No more callbacks
     */
    ret = adc_injected_disable(adc_dev);
    if (ret < 0) {
        LOG_ERR("Failed to disable ADC: %d", ret);
        return ret;
    }
    
    /* Also disable MCPWM outputs for safety */
    mcpwm_set_duty_cycle(mcpwm_dev, 1, 0);  /* Phase A off */
    mcpwm_set_duty_cycle(mcpwm_dev, 2, 0);  /* Phase B off */
    mcpwm_set_duty_cycle(mcpwm_dev, 3, 0);  /* Phase C off */
    
    ret = mcpwm_stop(mcpwm_dev);
    if (ret < 0) {
        LOG_ERR("Failed to stop MCPWM: %d", ret);
    }
    
    LOG_INF("Motor control stopped");
    
    return 0;
}

/* Main motor control thread - processes ADC values and updates PWM */
static void motor_control_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);
    
    /* Main control loop - monitoring and setpoint updates
     * Note: FOC algorithm runs in ADC callback at 20kHz
     * This thread handles slower tasks: monitoring, setpoint changes, etc.
     */
    while (1) {
        k_sleep(K_MSEC(10));  /* 100Hz for monitoring */
        
        /* Update rotor position from encoder (example) */
        // motor.theta_elec = encoder_get_electrical_angle();
        
        /* Adjust current setpoints based on speed controller (example) */
        // motor.i_q_ref = speed_controller_output();
        // motor.i_d_ref = field_weakening_output();
        
        /* Monitor for faults */
        if (motor.overrun_count > 0) {
            LOG_WRN("ADC overruns detected: %u", motor.overrun_count);
        }
    }
}

K_THREAD_DEFINE(motor_thread, 2048, motor_control_thread, 
                NULL, NULL, NULL, 5, 0, 0);

/* Application main */
int main(void)
{
    int ret;
    
    LOG_INF("Motor Control Application Starting...");
    
    /* Initialize motor control */
    ret = motor_control_init();
    if (ret < 0) {
        LOG_ERR("Motor control init failed: %d", ret);
        return ret;
    }
    
    /* Start motor control */
    ret = motor_control_start();
    if (ret < 0) {
        LOG_ERR("Motor control start failed: %d", ret);
        return ret;
    }
    
    /* Main loop - monitor status */
    while (1) {
        k_sleep(K_SECONDS(1));
        
        LOG_INF("Status: updates=%u, overruns=%u, i_a=0x%08X, i_b=0x%08X, v_bus=0x%08X",
                motor.update_count, motor.overrun_count,
                motor.i_phase_a, motor.i_phase_b, motor.v_bus);
    }
    
    return 0;
}
```

#### 4.2 Devicetree Configuration

```dts
/* nucleo_h753zi.overlay */

&adc1 {
    compatible = "st,stm32-adc-injected";
    status = "okay";
    
    /* Hardware trigger: TIM1_CC4 event at 20kHz */
    st,adc-trigger-source = "tim1_cc4";
    st,adc-trigger-edge = "rising";
    
    /* H7-specific configuration */
    st,adc-clock-source = "SYNC";
    st,adc-prescaler = <4>;
    
    pinctrl-0 = <&adc1_inp3_pa3 &adc1_inp10_pc0 &adc1_inp11_pc1>;
    pinctrl-names = "default";
    
    #address-cells = <1>;
    #size-cells = <0>;
    
    /* Injected sequence: 3 channels in this order */
    channel@3 {   /* Rank 1: Phase A current */
        reg = <3>;
        zephyr,gain = "ADC_GAIN_1";
        zephyr,reference = "ADC_REF_INTERNAL";
        zephyr,acquisition-time = <ADC_ACQ_TIME(ADC_ACQ_TIME_TICKS, 2)>;
        zephyr,resolution = <12>;
    };
    
    channel@10 {  /* Rank 2: Phase B current */
        reg = <10>;
        zephyr,acquisition-time = <ADC_ACQ_TIME(ADC_ACQ_TIME_TICKS, 2)>;
    };
    
    channel@11 {  /* Rank 3: DC bus voltage */
        reg = <11>;
        zephyr,acquisition-time = <ADC_ACQ_TIME(ADC_ACQ_TIME_TICKS, 2)>;
    };
};

&mcpwm1 {
    status = "okay";
    compatible = "st,stm32-mcpwm";
    
    /* MCPWM uses TIM1 internally for 3-phase generation */
    timers = <&tim1>;
    
    /* 20kHz PWM frequency */
    pwm-frequency = <20000>;
    
    /* Dead time for complementary outputs (in nanoseconds) */
    dead-time-ns = <1000>;
    
    /* Phase output pins */
    pinctrl-0 = <&tim1_ch1_pa8 &tim1_ch1n_pa7
                 &tim1_ch2_pa9 &tim1_ch2n_pb0
                 &tim1_ch3_pa10 &tim1_ch3n_pb1>;
    pinctrl-names = "default";
};
```

#### 4.3 API Usage Summary

**Initialization Sequence:**
1. `device_is_ready(adc_dev)` - Verify device available (driver auto-configured from DT during init)
2. `adc_injected_set_callback(dev, cb, user_data)` - Optional: register callback
3. `adc_injected_enable(dev)` - Enable hardware-triggered conversions

**Runtime Operation:**
- TIM1 generates CC4 events at 20kHz (configured in TIM1 setup)
- ADC automatically converts channels 3, 10, 11 on each trigger
- ISR reads values, converts to Q31, stores in driver data
- Callback invoked with Q31 values at 20kHz rate
- **FOC algorithm runs in callback**: Clarke, Park, PI, inverse Park, SVPWM
- **PWM updated directly from callback** at 20kHz synchronous with ADC
- Main thread handles slower tasks (monitoring, setpoint updates)

**Shutdown Sequence:**
1. `adc_injected_disable(dev)` - Stop conversions
2. Disable PWM outputs (for safety in motor control)

**Key Design Validated:**
- ✓ Pure devicetree configuration - no runtime channel setup needed
- ✓ Callback receives values directly (Q31 format)
- ✓ All channel properties configured automatically during driver init
- ✓ Clean separation: device ready → register callback → enable
- ✓ Simple enable/disable for start/stop control
- ✓ Minimal API surface (3 functions)

### Phase 5: Complete System Integration (Day 4)

This phase shows the complete integration of the zero-latency ISR calling the FOC callback from Phase 4.

#### 5.1 Architecture Review

**Data Flow:**
1. TIM1_CC4 triggers ADC conversion (hardware)
2. ADC completes, generates JEOS interrupt at priority 0
3. Zero-latency ISR executes (< 500ns latency)
   - Reads ADC values from hardware
   - Converts to Q31 format
   - Calls registered callback **directly**
4. Callback executes (still in ISR context, < 2µs)
   - FOC algorithm (Clarke, Park, PI, inverse Park, SVPWM)
   - Updates TIM1->CCR1/2/3 registers directly
5. ISR returns
6. Total latency: < 3µs trigger to PWM update

**Key Points:**
- ISR and callback both run at priority 0 (zero-latency)
- No kernel involvement in critical path
- Callback can't use kernel functions (LOG_*, k_*, device APIs)
- All motor state in global variables
- Main thread only updates setpoints and monitors

#### 5.2 Callback Implementation

The callback from Phase 4 runs in zero-latency ISR context:

```c
/* Global motor state (accessible from both ISR and main thread) */
volatile struct {
    /* Current measurements (updated by callback) */
    q31_t i_phase_a;
    q31_t i_phase_b;
    q31_t v_bus;
    
    /* Control setpoints (updated by main thread) */
    q31_t i_d_ref;
    q31_t i_q_ref;
    
    /* PI controller states */
    q31_t i_d_integral;
    q31_t i_q_integral;
    
    /* Rotor position (updated by encoder or main thread) */
    q31_t theta_elec;
    
    /* Statistics */
    uint32_t update_count;
    uint32_t overrun_count;
} g_motor;

/* Callback runs in zero-latency ISR context - NO kernel calls! */
static void adc_values_ready(const struct device *dev,
                             const q31_t *values,
                             uint8_t count,
                             void *user_data)
{
    /* Store ADC values */
    g_motor.i_phase_a = values[0];
    g_motor.i_phase_b = values[1];
    g_motor.v_bus = values[2];
    
    /* FOC Algorithm - same as Phase 4 example
     * - Clarke transform
     * - Park transform  
     * - PI controllers
     * - Inverse Park
     * - Space Vector PWM
     * - Direct TIM register writes
     * (See Phase 4 for complete implementation)
     */
    
    g_motor.update_count++;
}
```

#### 5.3 Callback Constraints in Zero-Latency Context

**CRITICAL: Callback runs in zero-latency ISR context (priority 0)**

Since the callback is invoked directly from the zero-latency ISR, it inherits all ISR constraints:

**Allowed Operations:**
- ✓ Direct register reads/writes (ADC, GPIO, TIM, etc.)
- ✓ Simple arithmetic and bit operations
- ✓ Array indexing with global/static variables
- ✓ Atomic operations (`atomic_*`) - use carefully
- ✓ Function calls to inline or static functions meeting these constraints
- ✓ ARM CMSIS-DSP functions (arm_cos_q31, arm_sin_q31, etc.)
- ✓ Lookup table access
- ✓ Direct PWM register updates (TIM1->CCR1/2/3)

**Forbidden Operations (Will Cause Crash):**
- ✗ Any `k_*` kernel functions (k_sleep, k_work_submit, etc.)
- ✗ `LOG_*` macros (no logging in callback!)
- ✗ Mutexes, semaphores, queues
- ✗ Memory allocation (malloc, k_malloc, etc.)
- ✗ Floating point (unless FPU context saved - adds 1-2µs overhead)
- ✗ Any calls that might block or use kernel services
- ✗ Device API calls (mcpwm_set_duty_cycle won't work - use direct register writes)

**Why this architecture:**
- Zero-latency ISR ensures < 500ns from trigger to callback entry
- Callback provides clean separation between ADC reading and FOC algorithm
- All FOC state in global variables accessible from callback
- Main thread can update setpoints, monitor statistics
- Total latency (trigger → callback complete) < 3µs

#### 5.4 Main Thread Responsibilities

```c
/* Main thread - handles non-realtime tasks */
static void motor_control_thread(void *p1, void *p2, void *p3)
{
    while (1) {
        k_sleep(K_MSEC(10));  /* 100Hz for monitoring */
        
        /* Update rotor position from encoder */
        g_motor.theta_elec = encoder_get_electrical_angle();
        
        /* Adjust setpoints from speed controller */
        g_motor.i_q_ref = speed_controller_output();
        g_motor.i_d_ref = field_weakening_output();
        
        /* Monitor for faults */
        if (g_motor.overrun_count > 0) {
            LOG_WRN("ADC overruns: %u", g_motor.overrun_count);
        }
    }
}
```

#### 5.5 Performance Targets

| Metric | Target | Measurement Method |
|--------|--------|--------------------|
| **ISR Entry Latency** | < 500ns | Scope: trigger to ISR entry (GPIO toggle) |
| **Callback Entry** | < 1µs | Scope: trigger to callback start |
| **Callback Execution** | < 2µs | FOC algorithm complete |
| **Total ADC→PWM Latency** | < 3µs | Scope: trigger to PWM register update |
| **Jitter** | < 100ns | Statistical analysis over 10k samples |
| **Update Rate** | 20kHz sustained | No overruns over 1 hour |

**Why Zero-Latency ISR Calling Callback:**
- ISR reads ADC with < 500ns latency (priority 0, un-maskable)
- Callback gets values immediately (no queueing or deferred work)
- FOC algorithm runs at 20kHz synchronous with ADC
- PWM updated in same execution context (< 3µs total)
- Clean architecture: ISR handles hardware, callback handles algorithm
- Main thread only updates setpoints and monitors (no real-time work)

### Phase 6: Testing & Validation (Day 4-5)

#### 6.1 Functional Testing

**Test 1: Channel Configuration**
```c
/* Verify devicetree-driven channel setup works */
void test_channel_setup(void)
{
    const struct device *adc = DEVICE_DT_GET(DT_NODELABEL(adc1));
    
    /* DT_FOREACH_CHILD should configure all channels */
    int ret = motor_control_init();
    assert(ret == 0);
    
    /* Verify channels are configured (check internal registers if accessible) */
}
```

**Test 2: Callback Invocation**
```c
/* Verify callback is invoked at correct rate */
static uint32_t callback_count = 0;

void test_callback(const struct device *dev, const q31_t *values,
                   uint8_t count, void *user_data)
{
    callback_count++;
}

void test_callback_rate(void)
{
    adc_injected_set_callback(adc_dev, test_callback, NULL);
    adc_injected_enable(adc_dev);
    
    callback_count = 0;
    k_sleep(K_MSEC(100));  /* Wait 100ms */
    
    /* At 20kHz, should have ~2000 callbacks in 100ms */
    assert(callback_count >= 1900 && callback_count <= 2100);
}
```

**Test 3: ADC Value Accuracy**
```c
/* Verify ADC reads correct values with known inputs */
void test_adc_accuracy(void)
{
    /* Apply known voltage to ADC input (e.g., 1.65V = mid-scale) */
    /* Expected Q31 value: 0x40000000 for 12-bit at mid-scale */
    
    /* Read values in callback, compare to expected */
}
```

#### 6.2 Timing Validation

**Latency Measurement with GPIO:**
```c
/* Add to ISR for precise timing measurement */
static void adc_stm32_inj_isr(const struct device *dev)
{
    /* Set GPIO high immediately at ISR entry */
    gpio_pin_set_raw(debug_gpio_dev, DEBUG_PIN, 1);
    
    const struct adc_stm32_cfg *config = dev->config;
    struct adc_stm32_data *data = dev->data;
    ADC_TypeDef *adc = config->base;
    
    if (LL_ADC_IsActiveFlag_JEOS(adc)) {
        LL_ADC_ClearFlag_JEOS(adc);
        
        /* Read values, convert to Q31, store, trigger callback... */
        
        /* Set GPIO low at ISR exit */
        gpio_pin_set_raw(debug_gpio_dev, DEBUG_PIN, 0);
    }
}
```

**Oscilloscope Measurements:**
- **Channel 1:** TIM1_CC4 output (ADC trigger)
- **Channel 2:** Debug GPIO (ISR execution)
- **Measure:** Time from trigger rising edge to GPIO rising edge
- **Target:** < 2µs for regular ISR, < 500ns for zero-latency

**Jitter Measurement:**
- Capture 10,000+ samples
- Measure min/max/average ISR latency
- Target: < 100ns jitter (variation)

#### 6.3 Stress Testing

**Test 1: Sustained Operation**
```c
void test_sustained_operation(void)
{
    /* Run motor control for extended period */
    adc_injected_enable(adc_dev);
    
    /* Monitor for 1 hour */
    for (int i = 0; i < 3600; i++) {
        k_sleep(K_SECONDS(1));
        
        /* Check for errors */
        assert(motor.overrun_count == 0);
        assert(motor.update_count > 0);
        
        /* Reset counter to prevent overflow */
        motor.update_count = 0;
    }
    
    adc_injected_disable(adc_dev);
}
```

**Test 2: System Load**
```c
/* Verify ADC works correctly under high system load */
void test_with_load(void)
{
    /* Start ADC */
    adc_injected_enable(adc_dev);
    
    /* Add system load: other interrupts, threads, etc. */
    /* Verify ADC continues to work correctly */
    /* Verify no missed conversions */
    
    assert(motor.overrun_count == 0);
}
```

#### 6.4 Integration Testing

**Motor Spin Test:**
1. Connect motor to controller
2. Enable motor control: `motor_control_start()`
3. Set torque command: `motor.i_q_ref = 0x10000000;`
4. Verify:
   - ✓ Motor spins smoothly
   - ✓ No audible noise or vibration
   - ✓ Current readings are stable
   - ✓ No overruns or faults

**Current Control Validation:**
1. Apply step change to `i_q_ref`
2. Measure actual current response
3. Verify:
   - ✓ Rise time < 5ms
   - ✓ No overshoot
   - ✓ Steady-state error < 5%

#### 6.5 Debugging Techniques

**Method 1: GPIO Timing Markers**
```c
/* Add GPIO toggles at strategic points */
static void adc_values_ready(const struct device *dev,
                             const q31_t *values,
                             uint8_t count,
                             void *user_data)
{
    gpio_pin_set_raw(debug_gpio_dev, 0, 1);  /* Start of callback */
    
    /* FOC algorithm... */
    
    gpio_pin_set_raw(debug_gpio_dev, 1, 1);  /* Clarke done */
    
    /* Park transform... */
    
    gpio_pin_set_raw(debug_gpio_dev, 2, 1);  /* Park done */
    
    /* PI controllers... */
    
    gpio_pin_set_raw(debug_gpio_dev, 0, 0);  /* End of callback */
    gpio_pin_set_raw(debug_gpio_dev, 1, 0);
    gpio_pin_set_raw(debug_gpio_dev, 2, 0);
}
```

**Method 2: Circular Buffer Logging**
```c
/* Log data from ISR for later analysis */
struct adc_log_entry {
    uint32_t timestamp;
    q31_t values[4];
    uint8_t flags;
} __attribute__((packed));

volatile struct adc_log_entry g_adc_log[1024];
volatile uint32_t g_adc_log_idx = 0;

/* In ISR - log minimal data */
static void log_adc_sample(const q31_t *values, uint8_t flags)
{
    uint32_t idx = g_adc_log_idx % 1024;
    g_adc_log[idx].timestamp = k_cycle_get_32();
    g_adc_log[idx].values[0] = values[0];
    g_adc_log[idx].values[1] = values[1];
    g_adc_log[idx].values[2] = values[2];
    g_adc_log[idx].flags = flags;
    g_adc_log_idx++;
}

/* In main thread - dump to console */
void dump_adc_log(void)
{
    static uint32_t last_idx = 0;
    uint32_t curr_idx = g_adc_log_idx;
    
    while (last_idx < curr_idx) {
        uint32_t idx = last_idx % 1024;
        LOG_INF("T=%u: [%08X,%08X,%08X] flags=0x%02X",
                g_adc_log[idx].timestamp,
                g_adc_log[idx].values[0],
                g_adc_log[idx].values[1],
                g_adc_log[idx].values[2],
                g_adc_log[idx].flags);
        last_idx++;
    }
}
```

**Method 3: Statistical Analysis**
```c
/* Track statistics for analysis */
struct adc_stats {
    uint32_t min_latency;
    uint32_t max_latency;
    uint32_t total_latency;
    uint32_t sample_count;
    uint32_t overrun_count;
} g_stats = {.min_latency = UINT32_MAX};

/* Update in ISR */
void update_stats(uint32_t latency)
{
    if (latency < g_stats.min_latency) g_stats.min_latency = latency;
    if (latency > g_stats.max_latency) g_stats.max_latency = latency;
    g_stats.total_latency += latency;
    g_stats.sample_count++;
}

/* Print from main thread */
void print_stats(void)
{
    uint32_t avg = g_stats.total_latency / g_stats.sample_count;
    LOG_INF("ADC Stats: min=%u, max=%u, avg=%u, overruns=%u",
            g_stats.min_latency, g_stats.max_latency,
            avg, g_stats.overrun_count);
}
```

## Device Tree Examples (Complete)

### STM32H753ZI Example (Nucleo-H753ZI)

```dts
/ {
    /* Debug GPIOs for timing measurement */
    debug_gpios {
        compatible = "gpio-leds";
        isr_timing: isr_timing {
            gpios = <&gpiob 0 GPIO_ACTIVE_HIGH>;
        };
    };
};

&adc1 {
    compatible = "st,stm32-adc-injected";
    status = "okay";
    
    /* Hardware trigger configuration */
    trigger-source = "tim1_cc4";
    trigger-edge = "rising";
    
    pinctrl-0 = <&adc1_inp3_pa3 &adc1_inp10_pc0 &adc1_inp11_pc1>;
    pinctrl-names = "default";
    
    /* H7-specific ADC configuration */
    st,adc-clock-source = "SYNC";
    st,adc-prescaler = <4>;
    
    #address-cells = <1>;
    #size-cells = <0>;
    
    /* Phase A current - ADC1_INP3 */
    channel@3 {
        reg = <3>;
        zephyr,gain = "ADC_GAIN_1";
        zephyr,reference = "ADC_REF_INTERNAL";
        zephyr,acquisition-time = <ADC_ACQ_TIME(ADC_ACQ_TIME_TICKS, 2)>;
        zephyr,resolution = <12>;
    };
    
    /* Phase B current - ADC1_INP10 */
    channel@10 {
        reg = <10>;
        zephyr,acquisition-time = <ADC_ACQ_TIME(ADC_ACQ_TIME_TICKS, 2)>;
    };
    
    /* DC Bus voltage - ADC1_INP11 */
    channel@11 {
        reg = <11>;
        zephyr,acquisition-time = <ADC_ACQ_TIME(ADC_ACQ_TIME_TICKS, 2)>;
    };
};
```

### STM32F407VG Example

```dts
/ {
    debug_gpios {
        compatible = "gpio-leds";
        isr_timing: isr_timing {
            gpios = <&gpiob 0 GPIO_ACTIVE_HIGH>;
        };
    };
};

&adc1 {
    compatible = "st,stm32-adc-injected";
    status = "okay";
    
    /* Hardware trigger configuration */
    trigger-source = "tim1_cc4";
    trigger-edge = "rising";
    
    pinctrl-0 = <&adc1_in0_pa0 &adc1_in1_pa1 &adc1_in2_pa2>;
    pinctrl-names = "default";
    
    #address-cells = <1>;
    #size-cells = <0>;
    
    /* Phase A current - ADC1_IN0 */
    channel@0 {
        reg = <0>;
        zephyr,gain = "ADC_GAIN_1";
        zephyr,reference = "ADC_REF_INTERNAL";
        zephyr,acquisition-time = <ADC_ACQ_TIME(ADC_ACQ_TIME_TICKS, 15)>;
    };
    
    /* Phase B current - ADC1_IN1 */
    channel@1 {
        reg = <1>;
        zephyr,acquisition-time = <ADC_ACQ_TIME(ADC_ACQ_TIME_TICKS, 15)>;
    };
    
    /* DC Bus voltage - ADC1_IN2 */
    channel@2 {
        reg = <2>;
        zephyr,acquisition-time = <ADC_ACQ_TIME(ADC_ACQ_TIME_TICKS, 15)>;
    };
};
```

## Build System Integration

### Kconfig
```kconfig
config ADC_STM32_INJECTED
    bool "STM32 ADC Injected Channel Driver"
    depends on SOC_SERIES_STM32H7X || SOC_SERIES_STM32F4X
    select PINCTRL
    help
      Ultra low-latency ADC driver using injected channels
      with zero-latency interrupt support.
      
      Supports STM32F4 and STM32H7 families.

if ADC_STM32_INJECTED

config ADC_STM32_INJECTED_ZERO_LATENCY
    bool "Use zero-latency interrupts"
    default y
    help
      Enable zero-latency ISR (priority 0) for minimal latency.
      When enabled, ISR cannot use any kernel functions.

endif # ADC_STM32_INJECTED
```

### CMakeLists.txt
```cmake
zephyr_library_sources_ifdef(
    CONFIG_ADC_STM32_INJECTED
    adc_stm32_injected.c
)
```

## Timeline & Milestones

### Day 1: Foundation (Phase 1)
- ✓ Device tree bindings defined
- ✓ Driver structure created
- ✓ Channel configuration working

### Day 2: Core Implementation (Phase 2-3)
- ✓ ADC initialization complete
- ✓ Zero-latency ISR implemented (ISR_DIRECT_DECLARE)
- ✓ ISR calls registered callback directly
- ✓ Basic validation passing
- ✓ ISR latency < 500ns confirmed
- ✓ No kernel calls in ISR verified

### Day 3: Motor Control Reference (Phase 4)
- ✓ Complete FOC example in callback
- ✓ Callback follows zero-latency constraints
- ✓ FOC algorithm validated (Clarke, Park, PI, SVPWM)
- ✓ Q31 math verified
- ✓ PWM registers updated from callback

### Day 4: Integration & Optimization (Phase 5)
- ✓ Zero-latency ISR + callback integration complete
- ✓ Motor control functional at 20kHz
- ✓ System stability verified
- ✓ Latency targets met (< 3µs total)
- ✓ No kernel calls in callback verified

### Day 5: Testing & Validation (Phase 6)
- ✓ Functional tests passing
- ✓ Timing validation complete (< 500ns ISR, < 3µs total)
- ✓ Stress testing passed (1 hour no overruns)
- ✓ Integration tests successful
- ✓ Performance documented

## Risk Mitigation

### Risk 1: Zero-latency ISR crashes
**Mitigation:**
- Develop with regular ISR first
- Extensive testing before migration
- Watchdog timer enabled
- GPIO tracing for debugging

### Risk 2: ADC timing issues
**Mitigation:**
- Validate trigger timing with scope first
- Start with slower PWM frequency
- Use overrun detection
- Monitor with GPIO toggles

### Risk 3: Interference with other peripherals
**Mitigation:**
- Document all peripheral interactions
- Test with full system load
- Monitor for DMA conflicts
- Verify timing margins

## Success Criteria

- [ ] Zero-latency ISR implemented with ISR_DIRECT_DECLARE
- [ ] ISR priority 0 (un-maskable by kernel)
- [ ] ISR entry latency < 500ns (trigger to ISR entry)
- [ ] ISR execution < 2µs (complete FOC + PWM update)
- [ ] Total ADC→PWM latency < 3µs
- [ ] Zero kernel calls in ISR (verified)
- [ ] 20kHz conversion rate sustained
- [ ] No data loss over 1 hour continuous operation
- [ ] Jitter < 100ns (measured with scope)
- [ ] Motor control stable and responsive
- [ ] Clean integration with TIM1 PWM

## F4 vs H7 Implementation Notes

### ADC LL Function Compatibility

**Good news:** `LL_ADC_INJ_ReadConversionData32()` works on both F4 and H7!
- Use this consistently for reading injected channel data
- Returns uint32_t with conversion data
- No need for family-specific #ifdefs for data reading

### Other LL Function Differences

**Common (work on both):**
- `LL_ADC_IsActiveFlag_JEOS()`
- `LL_ADC_ClearFlag_JEOS()`
- `LL_ADC_IsActiveFlag_OVR()`
- `LL_ADC_ClearFlag_OVR()`
- `LL_ADC_EnableIT_JEOS()`
- `LL_ADC_INJ_SetTriggerSource()`
- `LL_ADC_INJ_SetTriggerEdge()`
- `LL_ADC_INJ_SetSequencerLength()`
- `LL_ADC_INJ_SetSequencerRanks()`

**Family-specific:**
- **H7 only**: `LL_ADC_IsActiveFlag_ADRDY()`, calibration functions
- **H7 only**: `LL_ADC_INJ_StartConversion()` (F4 auto-starts)
- **H7 boost mode**: Special clock setup for >25MHz

**Key takeaway:** Upstream driver already handles all the family differences correctly. Keep that code!

### Timing Considerations

**F4 ADC Timing (APB2 = 84MHz, prescaler /4 = 21MHz):**
- Conversion time: ~1.7µs (3 cycles + 15 sample cycles @ 21MHz)
- 3 channels: ~5.1µs total
- ISR latency target: <500ns

**H7 ADC Timing (Kernel clock 120MHz, prescaler /4 = 30MHz):**
- Conversion time: ~1.4µs (2.5 cycles + 2.5 sample cycles @ 30MHz with boost)
- 3 channels: ~4.2µs total
- ISR latency target: <500ns

Both families can easily meet the 20kHz update rate (50µs period).

## Conclusion

This plan provides a structured approach to implementing an ultra low-latency ADC driver for STM32F4 and STM32H7 motor control applications. By starting with the proven upstream driver foundation and carefully migrating to zero-latency interrupts, we minimize risk while achieving the required performance.

The key advantages of this approach:
1. **Dual family support**: Works on both F407 and H7 with minimal code differences
2. **Reuses validated initialization code**: H7 boost mode, F4 common registers
3. **Independent of Zephyr ADC API**: Simplified, purpose-built
4. **Configurable via device tree**: Easy hardware adaptation
5. **Incremental development path**: Regular ISR → zero-latency
6. **No DMA complexity**: Direct register reads
7. **Direct ISR processing**: Minimum latency for motor control
