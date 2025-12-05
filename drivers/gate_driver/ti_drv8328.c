/*
 * Copyright (c) 2025 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_drv8328

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>

LOG_MODULE_REGISTER(ti_drv8328, CONFIG_KERNEL_LOG_LEVEL);

/* Forward declaration for fault callback */
static void drv8328_fault_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);

/* User fault callback function pointer type */
typedef void (*drv8328_fault_cb_t)(const struct device *dev, void *user_data);

struct drv8328_config
{
    struct gpio_dt_spec sleep_gpio;
    struct gpio_dt_spec fault_gpio;
    struct gpio_dt_spec drvoff_gpio;
    struct gpio_dt_spec inlx_gpio;
    struct pwm_dt_spec pwm_spec;
};

struct drv8328_data
{
    const struct device *dev;
    struct gpio_callback fault_cb;
    bool initialized;
    bool sleep_mode;
    bool fault_state;
    drv8328_fault_cb_t user_fault_cb;
    void *user_data;
};

static int drv8328_init(const struct device *dev)
{
    const struct drv8328_config *config = dev->config;
    struct drv8328_data *data = dev->data;
    int ret = 0;

    LOG_INF("Initializing DRV8328 gate driver: %s", dev->name);

    /* Initialize sleep GPIO if specified */
    if (config->sleep_gpio.port != NULL)
    {
        if (!gpio_is_ready_dt(&config->sleep_gpio))
        {
            LOG_ERR("Sleep GPIO device %s not ready", config->sleep_gpio.port->name);
            return -ENODEV;
        }

        ret = gpio_pin_configure_dt(&config->sleep_gpio, GPIO_OUTPUT_ACTIVE);
        if (ret < 0)
        {
            LOG_ERR("Failed to configure sleep GPIO: %d", ret);
            return ret;
        }

        /* Start in active mode (not sleeping) */
        ret = gpio_pin_set_dt(&config->sleep_gpio, 1);
        if (ret < 0)
        {
            LOG_ERR("Failed to set sleep GPIO: %d", ret);
            return ret;
        }
        data->sleep_mode = false;
        LOG_DBG("Sleep GPIO configured and set to active");
    }

    /* Initialize fault GPIO if specified */
    if (config->fault_gpio.port != NULL)
    {
        if (!gpio_is_ready_dt(&config->fault_gpio))
        {
            LOG_ERR("Fault GPIO device %s not ready", config->fault_gpio.port->name);
            return -ENODEV;
        }

        ret = gpio_pin_configure_dt(&config->fault_gpio, GPIO_INPUT);
        if (ret < 0)
        {
            LOG_ERR("Failed to configure fault GPIO: %d", ret);
            return ret;
        }

        /* Configure fault interrupt */
        ret = gpio_pin_interrupt_configure_dt(&config->fault_gpio, GPIO_INT_EDGE_TO_ACTIVE);
        if (ret < 0)
        {
            LOG_ERR("Failed to configure fault GPIO interrupt: %d", ret);
            return ret;
        }

        /* Initialize callback */
        data->dev = dev;
        gpio_init_callback(&data->fault_cb, drv8328_fault_callback, BIT(config->fault_gpio.pin));
        
        ret = gpio_add_callback(config->fault_gpio.port, &data->fault_cb);
        if (ret < 0)
        {
            LOG_ERR("Failed to add fault GPIO callback: %d", ret);
            return ret;
        }

        LOG_DBG("Fault GPIO configured as interrupt input");
    }

    /* Initialize DRVOFF GPIO if specified (for DRV8328C/D variants) */
    if (config->drvoff_gpio.port != NULL)
    {
        if (!gpio_is_ready_dt(&config->drvoff_gpio))
        {
            LOG_ERR("DRVOFF GPIO device %s not ready", config->drvoff_gpio.port->name);
            return -ENODEV;
        }

        ret = gpio_pin_configure_dt(&config->drvoff_gpio, GPIO_OUTPUT_INACTIVE);
        if (ret < 0)
        {
            LOG_ERR("Failed to configure DRVOFF GPIO: %d", ret);
            return ret;
        }

        /* Start with gate drivers enabled (DRVOFF inactive) */
        ret = gpio_pin_set_dt(&config->drvoff_gpio, 0);
        if (ret < 0)
        {
            LOG_ERR("Failed to set DRVOFF GPIO: %d", ret);
            return ret;
        }
        LOG_DBG("DRVOFF GPIO configured and set to enable gate drivers");
    }

    /* Initialize INLx GPIO if specified */
    if (config->inlx_gpio.port != NULL)
    {
        if (!gpio_is_ready_dt(&config->inlx_gpio))
        {
            LOG_ERR("INLx GPIO device %s not ready", config->inlx_gpio.port->name);
            return -ENODEV;
        }

        ret = gpio_pin_configure_dt(&config->inlx_gpio, GPIO_OUTPUT);
        if (ret < 0)
        {
            LOG_ERR("Failed to configure INLx GPIO: %d", ret);
            return ret;
        }

        /* Force INLx high for INHx control mode */
        ret = gpio_pin_set_dt(&config->inlx_gpio, 1);
        if (ret < 0)
        {
            LOG_ERR("Failed to set INLx GPIO high: %d", ret);
            return ret;
        }
        LOG_DBG("INLx GPIO configured and forced high");
    }

    /* Verify PWM device if specified */
    if (config->pwm_spec.dev != NULL)
    {
        if (!pwm_is_ready_dt(&config->pwm_spec))
        {
            LOG_ERR("PWM device %s not ready", config->pwm_spec.dev->name);
            return -ENODEV;
        }
        LOG_DBG("PWM device verified and ready");
    }

    data->initialized = true;
    data->fault_state = false;
    data->user_fault_cb = NULL;
    data->user_data = NULL;

    LOG_INF("DRV8328 gate driver initialized successfully");
    return 0;
}

/* Fault interrupt callback */
static void drv8328_fault_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    struct drv8328_data *data = CONTAINER_OF(cb, struct drv8328_data, fault_cb);
    
    /* Set fault state */
    data->fault_state = true;
    
    LOG_ERR("DRV8328 fault detected on device %s", dev->name);
    
    /* Call user callback if registered */
    if (data->user_fault_cb != NULL) {
        data->user_fault_cb(dev, data->user_data);
    }
}

static int drv8328_set_sleep_mode(const struct device *dev, bool sleep)
{
    const struct drv8328_config *config = dev->config;
    struct drv8328_data *data = dev->data;
    int ret;

    if (config->sleep_gpio.port == NULL)
    {
        LOG_WRN("Sleep GPIO not configured");
        return -ENOTSUP;
    }

    ret = gpio_pin_set_dt(&config->sleep_gpio, sleep ? 0 : 1);
    if (ret < 0)
    {
        LOG_ERR("Failed to set sleep mode: %d", ret);
        return ret;
    }

    data->sleep_mode = sleep;
    LOG_INF("DRV8328 %s mode", sleep ? "sleep" : "active");
    return 0;
}

static int drv8328_reset_fault(const struct device *dev)
{
    const struct drv8328_config *config = dev->config;
    struct drv8328_data *data = dev->data;
    int ret;

    if (config->sleep_gpio.port == NULL)
    {
        LOG_WRN("Sleep GPIO not configured - cannot reset fault");
        return -ENOTSUP;
    }

    LOG_INF("Resetting DRV8328 fault");

    /* Generate fault reset pulse: high-to-low-to-high transition */
    /* Duration: 1-1.2μs as per datasheet section 8.4.3 */
    
    /* Ensure sleep pin is high first */
    ret = gpio_pin_set_dt(&config->sleep_gpio, 1);
    if (ret < 0)
    {
        LOG_ERR("Failed to set sleep GPIO high: %d", ret);
        return ret;
    }

    /* Pull sleep pin low for fault reset pulse */
    ret = gpio_pin_set_dt(&config->sleep_gpio, 0);
    if (ret < 0)
    {
        LOG_ERR("Failed to pull sleep GPIO low: %d", ret);
        return ret;
    }

    /* Wait 1.1μs (middle of 1-1.2μs range) */
    k_busy_wait(1);  /* 1 microsecond */

    /* Pull sleep pin high to complete reset pulse */
    ret = gpio_pin_set_dt(&config->sleep_gpio, 1);
    if (ret < 0)
    {
        LOG_ERR("Failed to set sleep GPIO high: %d", ret);
        return ret;
    }

    /* Clear fault state */
    data->fault_state = false;
    data->sleep_mode = false;  /* Device is now in active mode */

    /* Wait tWAKE time before device is ready for inputs */
    k_usleep(100);  /* Conservative wait time */

    LOG_INF("DRV8328 fault reset completed");
    return 0;
}

static int drv8328_get_fault_status(const struct device *dev, bool *fault)
{
    const struct drv8328_data *data = dev->data;

    /* Return cached fault state from interrupt */
    *fault = data->fault_state;
    return 0;
}

static int drv8328_enable_gate_drivers(const struct device *dev, bool enable)
{
    const struct drv8328_config *config = dev->config;
    int ret;

    if (config->drvoff_gpio.port == NULL)
    {
        LOG_WRN("DRVOFF GPIO not configured");
        return -ENOTSUP;
    }

    /* DRVOFF is active low, so invert the enable signal */
    ret = gpio_pin_set_dt(&config->drvoff_gpio, enable ? 0 : 1);
    if (ret < 0)
    {
        LOG_ERR("Failed to set gate driver enable: %d", ret);
        return ret;
    }

    LOG_INF("Gate drivers %s", enable ? "enabled" : "disabled");
    return 0;
}

static int drv8328_set_inlx_state(const struct device *dev, bool state)
{
    const struct drv8328_config *config = dev->config;
    int ret;

    if (config->inlx_gpio.port == NULL)
    {
        LOG_WRN("INLx GPIO not configured");
        return -ENOTSUP;
    }

    ret = gpio_pin_set_dt(&config->inlx_gpio, state ? 1 : 0);
    if (ret < 0)
    {
        LOG_ERR("Failed to set INLx state: %d", ret);
        return ret;
    }

    LOG_DBG("INLx set to %s", state ? "HIGH" : "LOW");
    return 0;
}

static int drv8328_register_fault_callback(const struct device *dev, 
                                          drv8328_fault_cb_t callback, 
                                          void *user_data)
{
    struct drv8328_data *data = dev->data;
    
    if (!data->initialized) {
        LOG_ERR("DRV8328 device not initialized");
        return -ENODEV;
    }
    
    data->user_fault_cb = callback;
    data->user_data = user_data;
    
    LOG_DBG("User fault callback registered for device %s", dev->name);
    return 0;
}

/* Device tree initialization macro */
#define DRV8328_INIT(inst)                                                \
    static const struct drv8328_config drv8328_config_##inst = {          \
        .sleep_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, sleep_gpios, {0}),   \
        .fault_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, fault_gpios, {0}),   \
        .drvoff_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, drvoff_gpios, {0}), \
        .inlx_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, inlx_gpios, {0}),     \
        .pwm_spec = PWM_DT_SPEC_INST_GET_OR(inst, pwms, {0}),             \
    };                                                                    \
                                                                          \
    static struct drv8328_data drv8328_data_##inst;                       \
                                                                          \
    DEVICE_DT_INST_DEFINE(inst,                                           \
                          drv8328_init,                                   \
                          NULL,                                           \
                          &drv8328_data_##inst,                           \
                          &drv8328_config_##inst,                         \
                          POST_KERNEL,                                    \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,            \
                          NULL);

DT_INST_FOREACH_STATUS_OKAY(DRV8328_INIT)
