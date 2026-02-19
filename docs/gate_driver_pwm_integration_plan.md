# Gate Driver PWM Integration Plan

## Overview
Integrate PWM control directly into the DRV8328 gate driver to manage INHx (PWM) and INLx (enable) signals without needing complementary PWM outputs.

## Problem Statement
- DRV8328 is a 3-phase gate driver but should support 1-3 phase operation
- Each phase requires:
  - **INHx**: PWM signal for motor control
  - **INLx**: Enable signal (held high during operation, low when disabled)
- Operating modes:
  - **3x mode (3 half-bridges)**: Uses GPIO for INLx since complementary outputs aren't needed
  - **6x mode (6 switches)**: Uses complementary PWM (OCxN) for INLx to get independent control
- STM32 complementary outputs (OCx/OCxN) cannot operate independently
  - OCxN always derives from same OCxREF as OCx
  - Cannot have OCx=PWM while OCxN=forced high
  - Solution: Use GPIO in 3x mode, complementary PWM in 6x mode
- Current approach uses GPIO for INLx, but this requires manual coordination

## Proposed Solution
Pass PWM channels to the gate driver in devicetree and manage both INHx and INLx from the driver.

## Architecture

### Devicetree Changes

**Current structure:**
```dts
&drv8328 {
    pwms = <&pwm1 1 PWM_POLARITY_NORMAL>;  /* Single PWM (deprecated) */
    inlx-gpios = <&gpioe 8 GPIO_ACTIVE_HIGH>;  /* Separate GPIO (manual control) */
};
```

**Recommended structure (3x mode - GPIO for INL, 2-channel H-bridge):**
```dts
&drv8328_a {
    /* PWM channels for INHx (motor control) - number of entries determines channel count */
    pwms = <&pwm1 1 PWM_POLARITY_NORMAL>,  /* INH1 - TIM1 CH1 */
           <&pwm1 2 PWM_POLARITY_NORMAL>;  /* INH2 - TIM1 CH2 */
    
    /* GPIO pins for INLx (gate driver enable) */
    inl-gpios = <&gpioe 8 GPIO_ACTIVE_HIGH>,   /* INL1 */
                <&gpioe 10 GPIO_ACTIVE_HIGH>;  /* INL2 */
    
    mode = "3x";  /* 2 half-bridges for H-bridge configuration */
};
```

**Alternative structure (6x mode - Complementary PWM for INL):**
```dts
&drv8328 {
    /* PWM channels for INHx (high-side control) */
    /* Hardware automatically generates complementary outputs on CH1N/CH2N/CH3N for INLx */
    pwms = <&pwm1 1 PWM_POLARITY_NORMAL>,  /* INH1 - TIM1 CH1, INL1 - TIM1 CH1N */
           <&pwm1 2 PWM_POLARITY_NORMAL>,  /* INH2 - TIM1 CH2, INL2 - TIM1 CH2N */
           <&pwm1 3 PWM_POLARITY_NORMAL>;  /* INH3 - TIM1 CH3, INL3 - TIM1 CH3N */
    
    mode = "6x";  /* 6 independent switches with complementary outputs */
};
```

**Note:** The MCPWM driver uses phandle references without period/flags. The PWM period is configured at the timer level in the `&timers1` and `&timers8` nodes via the `period` property in the `mcpwm` child node.

### Binding Updates

**File: `chopper/dts/bindings/ti,drv8328.yaml`**

```yaml
properties:
  pwms:
    description: |
      MCPWM channels for INHx (high-side control) signals.
      One PWM channel per half-bridge (3 for 3x mode, 3 for 6x mode).
      Format: <&mcpwm_device channel flags>
    type: phandle-array
    required: true

  inl-gpios:
    description: |
      GPIO pins for INLx (low-side enable) signals.
      One GPIO per half-bridge. Only used in 3x mode.
      These will be set high when channel is enabled, low when disabled.
    type: phandle-array
    
  mode:
    description: |
      Operating mode: "3x" for 3 half-bridges (GPIO INL), "6x" for 6 switches (PWM INL)
    type: string
    required: true
    enum:
      - "3x"
      - "6x"
    
  sleep-gpios:
    description: Sleep control pin
    type: phandle-array
    
  fault-gpios:
    description: Fault status pin
    type: phandle-array
    
  drvoff-gpios:
    description: Gate driver disable pin (DRV8328C/D only)
    type: phandle-array
```

### Driver Implementation

**File: `chopper/drivers/gate_driver/ti_drv8328.c`**

#### Data Structures

```c
#include <drivers/mcpwm.h>

#define DRV8328_MAX_CHANNELS 3

struct drv8328_channel_config {
    const struct device *pwm_dev;       /* MCPWM device for INHx (and INLx in 6x mode) */
    uint32_t channel;                   /* MCPWM channel number */
    struct gpio_dt_spec inl_gpio;       /* GPIO spec (3x mode only) */
};

struct drv8328_config {
    struct gpio_dt_spec sleep_gpio;
    struct gpio_dt_spec fault_gpio;
    struct gpio_dt_spec drvoff_gpio;
    
    struct drv8328_channel_config channels[DRV8328_MAX_CHANNELS];
    uint8_t num_channels;
    bool mode_6x;  /* true for 6x mode (PWM INL), false for 3x mode (GPIO INL) */
};

struct drv8328_data {
    const struct device *dev;
    struct gpio_callback fault_cb;
    bool initialized;
    bool sleep_mode;
    bool fault_state;
    drv8328_fault_cb_t user_fault_cb;
    void *user_data;
};
```

#### Initialization

```c
static int drv8328_init(const struct device *dev)
{
    const struct drv8328_config *config = dev->config;
    struct drv8328_data *data = dev->data;
    int ret;
    
    /* ... existing GPIO init ... */
    
    /* Validate number of channels (1-3 supported) */
    if (config->num_channels < 1 || config->num_channels > DRV8328_MAX_CHANNELS) {
        LOG_ERR("Invalid number of channels: %d (must be 1-3)", config->num_channels);
        return -EINVAL;
    }
    
    /* Initialize PWM channels */
    for (int i = 0; i < config->num_channels; i++) {
        /* Verify MCPWM device */
        if (!device_is_ready(config->channels[i].pwm_dev)) {
            LOG_ERR("PWM channel %d device not ready", i);
            return -ENODEV;
        }
        
        /* Configure PWM channel (with complementary mode in 6x) */
        uint32_t flags = PWM_POLARITY_NORMAL;
        if (config->mode_6x) {
            flags |= MCPWM_COMPLEMENTARY_MODE;  /* Enable complementary outputs */
        }
        
        ret = mcpwm_configure(config->channels[i].pwm_dev,
                             config->channels[i].channel,
                             flags);
        if (ret < 0) {
            LOG_ERR("Failed to configure PWM channel %d: %d", i, ret);
            return ret;
        }
        
        /* In 3x mode, also configure GPIO for INL */
        if (!config->mode_6x) {
            if (!gpio_is_ready_dt(&config->channels[i].inl_gpio)) {
                LOG_ERR("INL GPIO channel %d not ready", i);
                return -ENODEV;
            }
            
            ret = gpio_pin_configure_dt(&config->channels[i].inl_gpio, GPIO_OUTPUT_INACTIVE);
            if (ret < 0) {
                LOG_ERR("Failed to configure INL GPIO %d: %d", i, ret);
                return ret;
            }
        }
    }
    
    LOG_INF("DRV8328 initialized with %d channel(s)", config->num_channels);
    return 0;
}
```

#### Channel Control Functions

```c
/**
 * Enable a half-bridge channel
 * - Enables INH PWM output (motor control handles duty cycle via mcpwm_set_duty_cycle)
 * - Forces INL high (enable low-side gate driver)
 */
int drv8328_enable_channel(const struct device *dev, uint8_t channel)
{
    const struct drv8328_config *config = dev->config;
    struct drv8328_data *data = dev->data;
    int ret;
    
    if (channel >= config->num_channels) {
        return -EINVAL;
    }
    
    /* In 3x mode, set GPIO high first */
    if (!config->mode_6x) {
        ret = gpio_pin_set_dt(&config->channels[channel].inl_gpio, 1);
        if (ret < 0) {
            LOG_ERR("Failed to set INL GPIO high for channel %d: %d", channel, ret);
            return ret;
        }
    }
    
    /* Enable PWM (complementary output automatically enabled in 6x mode) */
    ret = mcpwm_enable(config->channels[channel].pwm_dev,
                       config->channels[channel].channel);
    if (ret < 0) {
        LOG_ERR("Failed to enable INH PWM for channel %d: %d", channel, ret);
        return ret;
    }
    
    LOG_DBG("Channel %d enabled", channel);
    return 0;
}

/**
 * Disable a half-bridge channel (freewheel mode)
 * - Disables INH PWM output
 * - Forces INL low (disable low-side gate driver)
 */
int drv8328_disable_channel(const struct device *dev, uint8_t channel)
{
    const struct drv8328_config *config = dev->config;
    struct drv8328_data *data = dev->data;
    int ret;
    
    if (channel >= config->num_channels) {
        return -EINVAL;
    }
    
    /* Disable PWM (complementary output automatically disabled in 6x mode) */
    ret = mcpwm_disable(config->channels[channel].pwm_dev,
                        config->channels[channel].channel);
    if (ret < 0) {
        LOG_ERR("Failed to disable PWM for channel %d: %d", channel, ret);
        return ret;
    }
    
    /* In 3x mode, also set GPIO low */
    if (!config->mode_6x) {
        ret = gpio_pin_set_dt(&config->channels[channel].inl_gpio, 0);
        if (ret < 0) {
            LOG_ERR("Failed to set INL GPIO low for channel %d: %d", channel, ret);
            return ret;
        }
    }
    
    LOG_DBG("Channel %d disabled", channel);
    return 0;
}

/**
 * Emergency disable all channels
 */
int drv8328_disable_all_channels(const struct device *dev)
{
    const struct drv8328_config *config = dev->config;
    
    LOG_WRN("Emergency stop - disabling all gate driver channels");
    
    for (int i = 0; i < config->num_channels; i++) {
        drv8328_disable_channel(dev, i);
    }
    
    return 0;
}

/**
 * Get PWM device for direct duty cycle control
 */
const struct device *drv8328_get_pwm_device(const struct device *dev, uint8_t channel)
{
    const struct drv8328_config *config = dev->config;
    
    if (channel >= config->num_channels) {
        return NULL;
    }
    
    return config->channels[channel].pwm_dev;
}

/**
 * Get PWM channel number
 */
int drv8328_get_pwm_channel(const struct device *dev, uint8_t channel)
{
    const struct drv8328_config *config = dev->config;
    
    if (channel >= config->num_channels) {
        return -EINVAL;
    }
    
    return config->channels[channel].channel;
}

/**
 * Set sleep mode
 */
int drv8328_set_sleep_mode(const struct device *dev, bool sleep)
{
    const struct drv8328_config *config = dev->config;
    struct drv8328_data *data = dev->data;
    int ret;

    if (config->sleep_gpio.port == NULL) {
        LOG_WRN("Sleep GPIO not configured");
        return -ENOTSUP;
    }

    if (sleep) {
        /* Entering sleep: disable all PWM outputs first */
        LOG_INF("Entering sleep mode - disabling PWM outputs");
        for (int i = 0; i < config->num_channels; i++) {
            drv8328_disable_channel(dev, i);
        }
        
        /* Assert nSLEEP low */
        ret = gpio_pin_set_dt(&config->sleep_gpio, 0);
        if (ret < 0) {
            LOG_ERR("Failed to enter sleep mode: %d", ret);
            return ret;
        }
        data->sleep_mode = true;
    } else {
        /* Exiting sleep: de-assert nSLEEP high */
        ret = gpio_pin_set_dt(&config->sleep_gpio, 1);
        if (ret < 0) {
            LOG_ERR("Failed to exit sleep mode: %d", ret);
            return ret;
        }
        
        /* Wait tWAKE time before device is ready */
        k_usleep(100);
        data->sleep_mode = false;
        
        LOG_INF("Exited sleep mode - PWM channels remain disabled");
    }

    return 0;
}

/**
 * Callback for PWM break/fault interrupt
 * Called from MCPWM break interrupt handler
 */
void drv8328_pwm_break_callback(const struct device *dev)
{
    struct drv8328_data *data = dev->data;
    
    /* PWM hardware already disabled via break input */
    /* Disable any GPIO-controlled INL pins */
    drv8328_disable_all_channels(dev);
    
    /* Set fault state */
    data->fault_state = true;
    
    LOG_ERR("DRV8328 PWM break fault detected");
    
    /* Call user callback if registered */
    if (data->user_fault_cb != NULL) {
        data->user_fault_cb(dev, data->user_data);
    }
}
```

#### Devicetree Macro

```c
/* Helper macro to initialize a single channel config from devicetree */
#define DRV8328_CHANNEL_INIT(inst, idx) \
    { \
        .pwm_dev = DEVICE_DT_GET(DT_PWMS_CTLR_BY_IDX(DT_DRV_INST(inst), pwms, idx)), \
        .channel = DT_PWMS_CHANNEL_BY_IDX(DT_DRV_INST(inst), pwms, idx), \
        .inl_gpio = GPIO_DT_SPEC_INST_GET_BY_IDX_OR(inst, inl_gpios, idx, {0}), \
    },

/* Main device initialization macro */
#define DRV8328_INIT(inst) \
    static const struct drv8328_config drv8328_config_##inst = { \
        .sleep_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, sleep_gpios, {0}), \
        .fault_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, fault_gpios, {0}), \
        .drvoff_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, drvoff_gpios, {0}), \
        .channels = { \
            LISTIFY(DT_INST_PROP_LEN(inst, pwms), DRV8328_CHANNEL_INIT, (), inst) \
        }, \
        .num_channels = DT_INST_PROP_LEN(inst, pwms), \
        .mode_6x = DT_INST_ENUM_IDX(inst, mode) == 1, /* 0="3x", 1="6x" */ \
    }; \
    \
    static struct drv8328_data drv8328_data_##inst; \
    \
    DEVICE_DT_INST_DEFINE(inst, \
                          drv8328_init, \
                          NULL, \
                          &drv8328_data_##inst, \
                          &drv8328_config_##inst, \
                          POST_KERNEL, \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, \
                          NULL);

DT_INST_FOREACH_STATUS_OKAY(DRV8328_INIT)
```

### Motor Control Integration

**File: `chopper/app/src/motor_hardware.c`**

```c
#include <drivers/gate_driver/ti_drv8328.h>
#include <drivers/pwm/mcpwm_stm32.h>

/* Cache PWM device pointers for fast ISR access (2 H-bridges, 2 channels each) */
static const struct device *pwm_a_dev;  /* H-bridge A */
static const struct device *pwm_b_dev;  /* H-bridge B */

/* Enable motor control (turn on both H-bridges) */
int motor_hardware_enable(void)
{
    const struct device *drv_a = DEVICE_DT_GET(DT_NODELABEL(drv8328_a));
    const struct device *drv_b = DEVICE_DT_GET(DT_NODELABEL(drv8328_b));
    int ret;
    
    /* Enable H-bridge A (2 channels) */
    ret = drv8328_enable_channel(drv_a, 0);
    if (ret < 0) {
        LOG_ERR("Failed to enable H-bridge A channel 0");
        return ret;
    }
    ret = drv8328_enable_channel(drv_a, 1);
    if (ret < 0) {
        LOG_ERR("Failed to enable H-bridge A channel 1");
        return ret;
    }
    
    /* Enable H-bridge B (2 channels) */
    ret = drv8328_enable_channel(drv_b, 0);
    if (ret < 0) {
        LOG_ERR("Failed to enable H-bridge B channel 0");
        return ret;
    }
    ret = drv8328_enable_channel(drv_b, 1);
    if (ret < 0) {
        LOG_ERR("Failed to enable H-bridge B channel 1");
        return ret;
    }
    
    /* Cache PWM devices for ISR duty cycle updates */
    pwm_a_dev = drv8328_get_pwm_device(drv_a, 0);
    pwm_b_dev = drv8328_get_pwm_device(drv_b, 0);
    
    return 0;
}

/* Disable motor control (freewheel - turn off both gate drivers) */
int motor_hardware_disable(void)
{
    const struct device *drv_a = DEVICE_DT_GET(DT_NODELABEL(drv8328_a));
    const struct device *drv_b = DEVICE_DT_GET(DT_NODELABEL(drv8328_b));
    
    drv8328_disable_all_channels(drv_a);
    drv8328_disable_all_channels(drv_b);
    
    return 0;
}

/* In motor ISR - use optimized inline duty cycle functions */
void motor_isr(void)
{
    /* ... FOC calculations ... */
    
    /* Output PWM values to 2 H-bridges using cached device pointers
     * H-bridge A: controls winding A (2-phase bipolar)
     * H-bridge B: controls winding B (2-phase bipolar)
     */
    mcpwm_stm32_set_duty_cycle_2phase_f32(pwm_a_dev, Da_high_pu, Da_low_pu);
    mcpwm_stm32_set_duty_cycle_2phase_f32(pwm_b_dev, Db_high_pu, Db_low_pu);
}

/* PWM break callback - hardware fault detected */
void pwm_break_callback(const struct device *pwm_dev, void *user_data)
{
    /* Determine which gate driver triggered the break */
    const struct device *drv_a = DEVICE_DT_GET(DT_NODELABEL(drv8328_a));
    const struct device *drv_b = DEVICE_DT_GET(DT_NODELABEL(drv8328_b));
    
    /* Notify both gate drivers of break event (safe to call multiple times) */
    drv8328_pwm_break_callback(drv_a);
    drv8328_pwm_break_callback(drv_b);
    
    /* Post error to motor state machine */
    motor_api_post_error(ERROR_HARDWARE_BREAK);
}

/* DRV8328 fault callback - nFAULT pin triggered */
void drv8328_fault_handler(const struct device *drv, void *user_data)
{
    /* Post error to motor state machine */
    motor_api_post_error(ERROR_GATE_DRIVER_FAULT);
}
```

## Implementation Steps

1. **Update DTS binding** (`ti,drv8328.yaml`)
   - Add `pwms` property for INHx channels
   - Add `inl-gpios` property for INLx enable signals
   - Add `mode` property for 3x/6x selection

2. **Update devicetree** (`smartstepper_v2.dts`)
   - Define PWM channels for INHx and INLx
   - Update DRV8328 node with new properties

3. **Update driver** (`ti_drv8328.c`)
   - Add channel configuration structures
   - Implement channel enable/disable functions
   - Add initialization for INL forced output mode

4. **Add public header** (`include/drivers/gate_driver/ti_drv8328.h`)
   - Export channel control API
   - Add emergency stop function

5. **Update motor control** (`motor_hardware.c`, `motor_states.c`)
   - Call gate driver enable/disable instead of direct PWM control
   - Integrate emergency stop with fault handling

6. **Testing**
   - **3x mode tests**:
     - Verify INL GPIO stays high during PWM operation
     - Verify INL goes low when channel disabled
     - Test single-phase operation (1 PWM in devicetree)
     - Test dual-phase operation (2 PWMs in devicetree)
     - Test three-phase operation (3 PWMs in devicetree)
   - **6x mode tests**:
     - Verify complementary outputs on OCxN pins
     - Measure dead-time insertion
     - Verify synchronized complementary switching
   - **Fault handling tests**:
     - Trigger nFAULT pin, verify all channels disabled
     - Trigger PWM break, verify callback invoked
     - Test fault reset and channel re-enable
     - Verify fault callback registration
   - **Sleep mode tests**:
     - Enter sleep, verify PWM outputs disabled
     - Exit sleep, verify channels remain disabled until explicitly enabled
     - Measure tWAKE delay
   - **DRVOFF tests** (DRV8328C/D only):
     - Verify gate drivers can be disabled independently
     - Test DRVOFF with PWM still enabled (no output)
   - **ISR performance**:
     - Verify `mcpwm_stm32_set_duty_cycle_2phase_f32()` works with cached PWM pointers
     - Measure ISR overhead with gate driver integration

## API Design

**File: `include/drivers/gate_driver/ti_drv8328.h`**

```c
#ifndef DRIVERS_GATE_DRIVER_TI_DRV8328_H_
#define DRIVERS_GATE_DRIVER_TI_DRV8328_H_

#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>

/**
 * @brief Fault callback function type
 * 
 * Called when nFAULT pin goes active (low)
 * PWM outputs are automatically disabled by this callback
 */
typedef void (*drv8328_fault_cb_t)(const struct device *dev, void *user_data);

/**
 * @brief Enable a half-bridge channel
 * 
 * In 3x mode: Sets INL GPIO high, then enables PWM
 * In 6x mode: Enables PWM (complementary output auto-enabled)
 * 
 * @param dev DRV8328 device
 * @param channel Channel index (0-2)
 * @return 0 on success, negative errno on failure
 */
int drv8328_enable_channel(const struct device *dev, uint8_t channel);

/**
 * @brief Disable a half-bridge channel
 * 
 * In 3x mode: Disables PWM, then sets INL GPIO low
 * In 6x mode: Disables PWM (complementary output auto-disabled)
 * 
 * @param dev DRV8328 device
 * @param channel Channel index (0-2)
 * @return 0 on success, negative errno on failure
 */
int drv8328_disable_channel(const struct device *dev, uint8_t channel);

/**
 * @brief Emergency stop - disable all channels immediately
 * 
 * Disables all configured PWM channels and INL GPIOs
 * Called by fault handler or externally for safety shutdown
 * 
 * @param dev DRV8328 device
 * @return 0 on success, negative errno on failure
 */
int drv8328_disable_all_channels(const struct device *dev);

/**
 * @brief Get PWM device for direct duty cycle control
 * 
 * Returns the MCPWM device reference for a channel to allow
 * optimized inline duty cycle updates (e.g., mcpwm_stm32_set_duty_cycle_2phase)
 * 
 * @param dev DRV8328 device
 * @param channel Channel index (0-2)
 * @return PWM device pointer, or NULL if invalid channel
 */
const struct device *drv8328_get_pwm_device(const struct device *dev, uint8_t channel);

/**
 * @brief Get PWM channel number for a gate driver channel
 * 
 * @param dev DRV8328 device
 * @param channel Channel index (0-2)
 * @return PWM channel number, or -EINVAL if invalid
 */
int drv8328_get_pwm_channel(const struct device *dev, uint8_t channel);

/**
 * @brief Set sleep mode (low power state)
 * 
 * Entering sleep: Disables all PWM outputs, then asserts nSLEEP low
 * Exiting sleep: De-asserts nSLEEP high, waits tWAKE, channels remain disabled
 * 
 * @param dev DRV8328 device
 * @param sleep true to enter sleep, false to wake
 * @return 0 on success, negative errno on failure
 */
int drv8328_set_sleep_mode(const struct device *dev, bool sleep);

/**
 * @brief Reset fault condition
 * 
 * Generates nSLEEP low pulse (1-1.2μs) to clear fault latch
 * Waits tWAKE (100μs) before returning
 * 
 * @param dev DRV8328 device
 * @return 0 on success, negative errno on failure
 */
int drv8328_reset_fault(const struct device *dev);

/**
 * @brief Get fault status
 * 
 * Returns cached fault state from interrupt
 * 
 * @param dev DRV8328 device
 * @param fault Pointer to store fault status (true = fault active)
 * @return 0 on success, negative errno on failure
 */
int drv8328_get_fault_status(const struct device *dev, bool *fault);

/**
 * @brief Enable/disable gate drivers (DRV8328C/D only)
 * 
 * Controls DRVOFF pin (active low)
 * Independent of PWM enable - use for external fault management
 * 
 * @param dev DRV8328 device
 * @param enable true to enable gate drivers, false to disable
 * @return 0 on success, -ENOTSUP if DRVOFF not configured
 */
int drv8328_enable_gate_drivers(const struct device *dev, bool enable);

/**
 * @brief Register fault callback
 * 
 * Callback is invoked from GPIO interrupt context when nFAULT goes active
 * PWM outputs are disabled before callback is invoked
 * 
 * @param dev DRV8328 device
 * @param callback Fault callback function
 * @param user_data User data passed to callback
 * @return 0 on success, negative errno on failure
 */
int drv8328_register_fault_callback(const struct device *dev,
                                   drv8328_fault_cb_t callback,
                                   void *user_data);

/**
 * @brief Callback for PWM break/fault interrupt
 * 
 * Call this from MCPWM break interrupt handler
 * Disables all gate driver channels and triggers fault callback
 * 
 * @param dev DRV8328 device
 */
void drv8328_pwm_break_callback(const struct device *dev);

#endif /* DRIVERS_GATE_DRIVER_TI_DRV8328_H_ */
```

## Benefits

1. **Cleaner abstraction**: Gate driver owns all its control signals
2. **Safer operation**: Coordinated enable/disable of INHx and INLx
3. **Hardware independence**: Motor control doesn't need to know about PWM details
4. **Better fault handling**: Gate driver can emergency-stop all channels
5. **Simplified motor code**: Single function call to enable/disable motor
6. **Direct PWM access**: Optimized inline duty cycle functions can access PWM devices

## Design Decisions

### Fault Handling
- **nFAULT GPIO interrupt**: Calls `drv8328_disable_all_channels()` to disable all PWM outputs before invoking user callback
- **PWM break callback**: Hardware automatically disables PWM, driver disables GPIO INL pins and invokes user callback
- **User responsibility**: Application must call `drv8328_reset_fault()` and re-enable channels after fault recovery

### Sleep Mode Integration
- **Entering sleep**: Driver disables all PWM outputs, then asserts nSLEEP low
- **Exiting sleep**: Driver de-asserts nSLEEP, waits tWAKE (100μs), channels remain disabled
- **Zephyr PM compatibility**: `drv8328_set_sleep_mode()` can be called from PM callbacks

### DRVOFF Pin (DRV8328C/D)
- **Independent control**: `drv8328_enable_gate_drivers()` controls DRVOFF separately from PWM enable
- **Use case**: External fault management systems can disable gate drivers without affecting PWM state

### Direct PWM Access
- **Optimized ISR**: `drv8328_get_pwm_device()` provides PWM device pointers for inline duty cycle functions
- **No abstraction overhead**: Motor ISR calls `mcpwm_stm32_set_duty_cycle_2phase_f32()` directly with cached pointers
- **Channel management**: Gate driver handles enable/disable, ISR handles duty cycle updates

### Error Handling
- **No rollback**: If enabling one channel fails, previously enabled channels remain enabled
- **Application responsibility**: Call `drv8328_disable_all_channels()` if initialization fails

## Potential Issues

1. **6x mode timing**: Complementary PWM outputs need precise synchronization
   - **Solution**: Use same timer for all channels (TIM1 has 3 complementary pairs)
   
2. **3x mode coordination**: INL GPIO must be high before INH starts PWMing
   - **Solution**: Enable INL first in `drv8328_enable_channel()`, then enable PWM (no delay needed)
   
3. **Mode detection**: Driver must correctly identify 3x vs 6x mode from devicetree
   - **Solution**: Use explicit `mode` enum property in devicetree binding, check with `DT_INST_ENUM_IDX`

## Example Devicetree Configuration

**3x Mode (GPIO for INL):**

**File: `chopper/app/boards/smartstepper_v2.overlay`**

```dts
/ {
    /* First DRV8328 - H-bridge A (2 phases) */
    drv8328_a: drv8328_a {
        compatible = "ti,drv8328";
        status = "okay";
        
        /* MCPWM channels for motor control (2 channels for H-bridge A) */
        pwms = <&pwm1 1 PWM_POLARITY_NORMAL>,  /* INH1 - PE9 (TIM1_CH1) */
               <&pwm1 2 PWM_POLARITY_NORMAL>;  /* INH2 - PE11 (TIM1_CH2) */
        
        /* GPIO pins for gate driver enable (INLx) */
        inl-gpios = <&gpioe 8 GPIO_ACTIVE_HIGH>,   /* INL1 - PE8 */
                    <&gpioe 10 GPIO_ACTIVE_HIGH>;  /* INL2 - PE10 */
        
        /* Control signals */
        sleep-gpios = <&gpiog 14 GPIO_ACTIVE_HIGH>;  /* nSLEEP */
        fault-gpios = <&gpioi 4 (GPIO_ACTIVE_LOW | GPIO_PULL_UP)>;  /* nFAULT */
        
        mode = "3x";
    };
    
    /* Second DRV8328 - H-bridge B (2 phases) */
    drv8328_b: drv8328_b {
        compatible = "ti,drv8328";
        status = "okay";
        
        /* MCPWM channels for motor control (2 channels for H-bridge B) */
        pwms = <&pwm8 1 PWM_POLARITY_NORMAL>,  /* INH1 - PC6 (TIM8_CH1) */
               <&pwm8 2 PWM_POLARITY_NORMAL>;  /* INH2 - PC8 (TIM8_CH2) */
        
        /* GPIO pins for gate driver enable (INLx) */
        inl-gpios = <&gpioc 7 GPIO_ACTIVE_HIGH>,   /* INL1 - PC7 */
                    <&gpioc 9 GPIO_ACTIVE_HIGH>;   /* INL2 - PC9 */
        
        /* Control signals */
        sleep-gpios = <&gpiog 15 GPIO_ACTIVE_HIGH>;  /* nSLEEP */
        fault-gpios = <&gpioi 5 (GPIO_ACTIVE_LOW | GPIO_PULL_UP)>;  /* nFAULT */
        
        mode = "3x";
    };
};
```

**6x Mode (Complementary PWM for INL):**

**File: `chopper/app/boards/smartstepper_v2.overlay`**

```dts
/ {
    /* First DRV8328 - H-bridge A (2 channels with complementary outputs) */
    drv8328_a: drv8328_a {
        compatible = "ti,drv8328";
        status = "okay";
        
        /* MCPWM channels (complementary mode enabled, auto-generates INLx on CHxN) */
        pwms = <&pwm1 1 PWM_POLARITY_NORMAL>,  /* INH1 - PE9 (TIM1_CH1), INL1 - PE8 (TIM1_CH1N) */
               <&pwm1 2 PWM_POLARITY_NORMAL>;  /* INH2 - PE11 (TIM1_CH2), INL2 - PE10 (TIM1_CH2N) */
        
        /* Control signals */
        sleep-gpios = <&gpiog 14 GPIO_ACTIVE_HIGH>;  /* nSLEEP */
        fault-gpios = <&gpioi 4 (GPIO_ACTIVE_LOW | GPIO_PULL_UP)>;  /* nFAULT */
        
        mode = "6x";
    };
    
    /* Second DRV8328 - H-bridge B (2 channels with complementary outputs) */
    drv8328_b: drv8328_b {
        compatible = "ti,drv8328";
        status = "okay";
        
        /* MCPWM channels (complementary mode enabled, auto-generates INLx on CHxN) */
        pwms = <&pwm8 1 PWM_POLARITY_NORMAL>,  /* INH1 - PC6 (TIM8_CH1), INL1 - PC7 (TIM8_CH1N) */
               <&pwm8 2 PWM_POLARITY_NORMAL>;  /* INH2 - PC8 (TIM8_CH2), INL2 - PC9 (TIM8_CH2N) */
        
        /* Control signals */
        sleep-gpios = <&gpiog 15 GPIO_ACTIVE_HIGH>;  /* nSLEEP */
        fault-gpios = <&gpioi 5 (GPIO_ACTIVE_LOW | GPIO_PULL_UP)>;  /* nFAULT */
        
        mode = "6x";
    };
};
```

## Recommendation

**For 3x mode (3 half-bridges):**
- Use **GPIO for INL** (simpler, proven)
- Use **MCPWM for INH** (necessary for motor control)
- Benefits:
  - Avoids complexity of complementary PWM management
  - Provides clean gate driver abstraction
  - Centralizes all motor hardware control in the gate driver
  - Simplifies motor control code (single enable/disable call)
  - No special timer configuration required

**For 6x mode (6 independent switches):**
- Use **complementary PWM for both INH and INL** (required for independent control)
- Use **single timer (TIM1)** with all 3 complementary pairs
- Benefits:
  - True 6-switch control capability
  - Hardware-synchronized complementary outputs
  - Dead-time insertion handled by timer hardware
  - Enables advanced motor control strategies
