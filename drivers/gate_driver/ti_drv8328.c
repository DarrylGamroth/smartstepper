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
#include <zephyr/pm/device.h>
#include <drivers/mcpwm.h>
#include <drivers/gate_driver/ti_drv8328.h>

LOG_MODULE_REGISTER(ti_drv8328, CONFIG_KERNEL_LOG_LEVEL);

/* Forward declaration for fault callback */
static void drv8328_fault_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);

/* User fault callback function pointer type */
typedef void (*drv8328_fault_cb_t)(const struct device *dev, void *user_data);

/* Per-channel configuration */
struct drv8328_channel_config
{
    struct mcpwm_dt_spec pwm_spec;
    struct gpio_dt_spec inl_gpio;
};

struct drv8328_config
{
    struct gpio_dt_spec sleep_gpio;
    struct gpio_dt_spec fault_gpio;
    struct gpio_dt_spec drvoff_gpio;
    const struct drv8328_channel_config *channels;
    uint8_t num_channels;
    bool mode_6x;
};

struct drv8328_data
{
    const struct device *dev;
    struct gpio_callback fault_cb;
    bool fault_state;
    drv8328_fault_cb_t user_fault_cb;
    void *user_data;
};

static int drv8328_init(const struct device *dev)
{
    const struct drv8328_config *config = dev->config;
    struct drv8328_data *data = dev->data;
    int ret = 0;

    LOG_INF("Initializing DRV8328 gate driver: %s (%d channels, %s mode)",
            dev->name, config->num_channels, config->mode_6x ? "6x" : "3x");

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
        data->user_fault_cb = NULL;
        data->user_data = NULL;
        data->fault_state = false;

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

    /* Initialize channels */
    for (uint8_t i = 0; i < config->num_channels; i++)
    {
        const struct drv8328_channel_config *ch = &config->channels[i];

        /* Verify PWM device */
        if (!device_is_ready(ch->pwm_spec.dev))
        {
            LOG_ERR("PWM device for channel %u not ready", i);
            return -ENODEV;
        }

        /* Configure PWM channel using devicetree spec */
        ret = mcpwm_configure_dt(&ch->pwm_spec);
        if (ret < 0)
        {
            LOG_ERR("Failed to configure PWM for channel %u: %d", i, ret);
            return ret;
        }

        /* In 3x mode, configure INL GPIO */
        if (!config->mode_6x && ch->inl_gpio.port != NULL)
        {
            if (!gpio_is_ready_dt(&ch->inl_gpio))
            {
                LOG_ERR("INL GPIO device for channel %u not ready", i);
                return -ENODEV;
            }

            ret = gpio_pin_configure_dt(&ch->inl_gpio, GPIO_OUTPUT_INACTIVE);
            if (ret < 0)
            {
                LOG_ERR("Failed to configure INL GPIO for channel %u: %d", i, ret);
                return ret;
            }

            LOG_DBG("Channel %u: INL GPIO configured (3x mode)", i);
        }

        LOG_DBG("Channel %u initialized (PWM dev: %s, channel: %u)",
                i, ch->pwm_spec.dev->name, ch->pwm_spec.channel);
    }

    return 0;
}

/* Fault interrupt callback */
static void drv8328_fault_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    struct drv8328_data *data = CONTAINER_OF(cb, struct drv8328_data, fault_cb);

    /* Set fault state */
    data->fault_state = true;

    LOG_ERR("DRV8328 fault detected on device %s", dev->name);

    /* Emergency stop all channels */
    drv8328_disable_all_channels(dev);

    /* Call user callback if registered */
    if (data->user_fault_cb != NULL) {
        data->user_fault_cb(dev, data->user_data);
    }
}

int drv8328_set_sleep_mode(const struct device *dev, bool sleep)
{
    const struct drv8328_config *config = dev->config;
    int ret;

    if (config->sleep_gpio.port == NULL)
    {
        LOG_WRN("Sleep GPIO not configured");
        return -ENOTSUP;
    }

    /* When entering sleep, disable all channels first */
    if (sleep)
    {
        ret = drv8328_disable_all_channels(dev);
        if (ret < 0)
        {
            LOG_ERR("Failed to stop channels before sleep: %d", ret);
            return ret;
        }
    }

    ret = gpio_pin_set_dt(&config->sleep_gpio, sleep ? 0 : 1);
    if (ret < 0)
    {
        LOG_ERR("Failed to set sleep mode: %d", ret);
        return ret;
    }

    /* When exiting sleep, wait tWAKE */
    if (!sleep)
    {
        k_usleep(100);  /* tWAKE = 100μs typical */
    }

    LOG_INF("DRV8328 %s mode", sleep ? "sleep" : "active");
    return 0;
}

int drv8328_reset_fault(const struct device *dev)
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

    k_busy_wait(1);

    /* Pull sleep pin high to complete reset pulse */
    ret = gpio_pin_set_dt(&config->sleep_gpio, 1);
    if (ret < 0)
    {
        LOG_ERR("Failed to set sleep GPIO high: %d", ret);
        return ret;
    }

    /* Clear fault state */
    data->fault_state = false;

    /* Wait tWAKE time before device is ready for inputs */
    k_usleep(100);

    LOG_INF("DRV8328 fault reset completed");
    return 0;
}

int drv8328_get_fault_status(const struct device *dev, bool *fault)
{
    const struct drv8328_data *data = dev->data;

    /* Return cached fault state from interrupt */
    *fault = data->fault_state;
    return 0;
}

int drv8328_register_fault_callback(const struct device *dev,
                                          drv8328_fault_cb_t callback,
                                          void *user_data)
{
    struct drv8328_data *data = dev->data;

    data->user_fault_cb = callback;
    data->user_data = user_data;

    LOG_DBG("User fault callback registered for device %s", dev->name);
    return 0;
}

/* Channel control functions */
int drv8328_enable_channel(const struct device *dev, uint8_t channel)
{
    const struct drv8328_config *config = dev->config;
    int ret;

    if (channel >= config->num_channels)
    {
        LOG_ERR("Invalid channel %u (max: %u)", channel, config->num_channels - 1);
        return -EINVAL;
    }

    const struct drv8328_channel_config *ch = &config->channels[channel];

    /* Enable PWM (complementary in 6x mode, single-ended in 3x mode) */
    ret = mcpwm_enable_dt(&ch->pwm_spec);
    if (ret < 0)
    {
        LOG_ERR("Failed to enable PWM for channel %u: %d", channel, ret);
        return ret;
    }

    if (!config->mode_6x)
    {
        /* 3x mode: Set INL GPIO high after enabling PWM on INH */
        if (ch->inl_gpio.port != NULL)
        {
            ret = gpio_pin_set_dt(&ch->inl_gpio, 1);
            if (ret < 0)
            {
                LOG_ERR("Failed to set INL GPIO high for channel %u: %d", channel, ret);
                /* Rollback: disable PWM on failure */
                mcpwm_disable_dt(&ch->pwm_spec);
                return ret;
            }
        }
    }

    LOG_DBG("Channel %u enabled (%s mode)", channel, config->mode_6x ? "6x" : "3x");
    return 0;
}

int drv8328_disable_channel(const struct device *dev, uint8_t channel)
{
    const struct drv8328_config *config = dev->config;
    int ret;

    if (channel >= config->num_channels)
    {
        LOG_ERR("Invalid channel %u (max: %u)", channel, config->num_channels - 1);
        return -EINVAL;
    }

    const struct drv8328_channel_config *ch = &config->channels[channel];

    /* Disable PWM first (safe shutdown) */
    ret = mcpwm_disable_dt(&ch->pwm_spec);
    if (ret < 0)
    {
        LOG_ERR("Failed to disable PWM for channel %u: %d", channel, ret);
        return ret;
    }

    if (!config->mode_6x)
    {
        /* 3x mode: Set INL GPIO low after disabling PWM */
        if (ch->inl_gpio.port != NULL)
        {
            ret = gpio_pin_set_dt(&ch->inl_gpio, 0);
            if (ret < 0)
            {
                LOG_ERR("Failed to set INL GPIO low for channel %u: %d", channel, ret);
                return ret;
            }
        }
    }

    LOG_DBG("Channel %u disabled (%s mode)", channel, config->mode_6x ? "6x" : "3x");
    return 0;
}

int drv8328_disable_all_channels(const struct device *dev)
{
    const struct drv8328_config *config = dev->config;
    int ret;
    int errors = 0;

    for (uint8_t i = 0; i < config->num_channels; i++)
    {
        ret = drv8328_disable_channel(dev, i);
        if (ret < 0)
        {
            LOG_ERR("Failed to disable channel %u during disable all channels: %d", i, ret);
            errors++;
        }
    }

    return errors > 0 ? -EIO : 0;
}

/* PWM access functions for ISR optimization */
const struct device *drv8328_get_pwm_device(const struct device *dev, uint8_t channel)
{
    const struct drv8328_config *config = dev->config;

    if (channel >= config->num_channels)
    {
        LOG_ERR("Invalid channel %u", channel);
        return NULL;
    }

    return config->channels[channel].pwm_spec.dev;
}

int drv8328_get_pwm_channel(const struct device *dev, uint8_t channel, uint32_t *pwm_channel)
{
    const struct drv8328_config *config = dev->config;

    if (channel >= config->num_channels)
    {
        LOG_ERR("Invalid channel %u", channel);
        return -EINVAL;
    }

    if (pwm_channel == NULL)
    {
        return -EINVAL;
    }

    *pwm_channel = config->channels[channel].pwm_spec.channel;
    return 0;
}

int drv8328_enable_gate_drivers(const struct device *dev, bool enable)
{
    const struct drv8328_config *config = dev->config;
    int ret;

    if (config->drvoff_gpio.port == NULL)
    {
        LOG_WRN("DRVOFF GPIO not configured");
        return -ENOTSUP;
    }

    /* DRVOFF is active high - when high, drivers are disabled */
    ret = gpio_pin_set_dt(&config->drvoff_gpio, enable ? 0 : 1);
    if (ret < 0)
    {
        LOG_ERR("Failed to set gate driver enable: %d", ret);
        return ret;
    }

    LOG_INF("Gate drivers %s", enable ? "enabled" : "disabled");
    return 0;
}

#ifdef CONFIG_PM_DEVICE
static int drv8328_pm_action(const struct device *dev, enum pm_device_action action)
{
    int ret = 0;

    switch (action) {
    case PM_DEVICE_ACTION_SUSPEND:
        LOG_DBG("Suspending DRV8328 %s", dev->name);

        /* Enter sleep mode (disables all channels and asserts sleep GPIO) */
        ret = drv8328_set_sleep_mode(dev, true);
        if (ret < 0) {
            LOG_ERR("Failed to enter sleep mode during suspend: %d", ret);
            return ret;
        }
        break;

    case PM_DEVICE_ACTION_RESUME:
        LOG_DBG("Resuming DRV8328 %s", dev->name);

        /* Exit sleep mode (wakes device) */
        /* Application is responsible for re-enabling channels */
        ret = drv8328_set_sleep_mode(dev, false);
        if (ret < 0) {
            LOG_ERR("Failed to exit sleep mode during resume: %d", ret);
            return ret;
        }
        break;

    case PM_DEVICE_ACTION_TURN_OFF:
        LOG_DBG("Turning off DRV8328 %s", dev->name);

        /* Enter sleep mode (disables all channels and asserts sleep GPIO) */
        ret = drv8328_set_sleep_mode(dev, true);
        if (ret < 0) {
            LOG_ERR("Failed to turn off device: %d", ret);
            return ret;
        }
        break;

    case PM_DEVICE_ACTION_TURN_ON:
        LOG_DBG("Turning on DRV8328 %s", dev->name);

        /* Exit sleep mode (wakes device) */
        /* Application is responsible for enabling channels */
        ret = drv8328_set_sleep_mode(dev, false);
        if (ret < 0) {
            LOG_ERR("Failed to turn on device: %d", ret);
            return ret;
        }
        break;

    default:
        return -ENOTSUP;
    }

    return ret;
}
#endif /* CONFIG_PM_DEVICE */

/* Device tree initialization macro */

/* Helper macro to get PWM device */
#define DRV8328_PWM_DEV(node_id, prop, idx) \
    DEVICE_DT_GET(DT_PWMS_CTLR_BY_IDX(node_id, idx)),

/* Helper macro to get PWM channel */
#define DRV8328_PWM_CHANNEL(node_id, prop, idx) \
    DT_PWMS_CHANNEL_BY_IDX(node_id, idx),

/* Helper macro to get INL GPIO (or empty if not present) */
#define DRV8328_INL_GPIO(node_id, prop, idx) \
    GPIO_DT_SPEC_GET_BY_IDX_OR(node_id, inl_gpios, idx, {0}),

/* Macro to define a single channel configuration */
#define DRV8328_CHANNEL_CONFIG(node_id, prop, idx) \
    { \
        .pwm_spec = MCPWM_DT_SPEC_GET_BY_IDX(node_id, idx), \
        .inl_gpio = GPIO_DT_SPEC_GET_BY_IDX_OR(node_id, inl_gpios, idx, {0}), \
    },

#define DRV8328_MODE_3X 0
#define DRV8328_MODE_6X 1

#define DRV8328_INIT(inst)                                                          \
    static const struct drv8328_channel_config drv8328_channels_##inst[] = {       \
        DT_INST_FOREACH_PROP_ELEM(inst, pwms, DRV8328_CHANNEL_CONFIG)              \
    };                                                                              \
                                                                                    \
    static const struct drv8328_config drv8328_config_##inst = {                   \
        .sleep_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, sleep_gpios, {0}),            \
        .fault_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, fault_gpios, {0}),            \
        .drvoff_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, drvoff_gpios, {0}),          \
        .channels = drv8328_channels_##inst,                                       \
        .num_channels = DT_INST_PROP_LEN(inst, pwms),                              \
        .mode_6x = (DT_INST_ENUM_IDX(inst, mode) == DRV8328_MODE_6X),              \
    };                                                                              \
                                                                                    \
    static struct drv8328_data drv8328_data_##inst;                                \
                                                                                    \
    PM_DEVICE_DT_INST_DEFINE(inst, drv8328_pm_action);                             \
                                                                                    \
    DEVICE_DT_INST_DEFINE(inst,                                                    \
                          drv8328_init,                                            \
                          PM_DEVICE_DT_INST_GET(inst),                             \
                          &drv8328_data_##inst,                                    \
                          &drv8328_config_##inst,                                  \
                          POST_KERNEL,                                             \
                          CONFIG_GATE_DRIVER_INIT_PRIORITY,                        \
                          NULL);

DT_INST_FOREACH_STATUS_OKAY(DRV8328_INIT)
