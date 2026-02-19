# Motor Parameter Persistence Implementation Plan

## Overview

This document outlines how to implement persistent parameter storage using Zephyr's Settings subsystem with EEPROM backend. Parameters saved to EEPROM will survive power cycles and be automatically restored on boot.

## Hardware Setup

**Target Hardware**: Smart Stepper V2 Controller
- STM32H743 microcontroller
- ST M24C16 EEPROM (2KB / 16 kilobit)
- I2C3 bus, base address 0x50
- 16-byte page writes
- 8-bit addressing

### M24C16 Addressing Quirk

The M24C16 appears at **8 consecutive I2C addresses** (0x50-0x57) due to its internal paging:

```
I2C Scan Results:
0x50 = Page 0 (bytes 0x000-0x0FF)    256 bytes
0x51 = Page 1 (bytes 0x100-0x1FF)    256 bytes
0x52 = Page 2 (bytes 0x200-0x2FF)    256 bytes
0x53 = Page 3 (bytes 0x300-0x3FF)    256 bytes
0x54 = Page 4 (bytes 0x400-0x4FF)    256 bytes
0x55 = Page 5 (bytes 0x500-0x5FF)    256 bytes
0x56 = Page 6 (bytes 0x600-0x6FF)    256 bytes
0x57 = Page 7 (bytes 0x700-0x7FF)    256 bytes
```

**Why This Happens:**
- M24C16 uses 8-bit internal addressing (can only address 256 bytes per "page")
- To access 2KB total, it uses **address bits A0-A2** as page selectors
- These appear as different I2C device addresses: 0b1010[A2][A1][A0][R/W]
- The Zephyr AT24 driver automatically handles this - you just use address 0x50
- Driver internally translates large offsets to the correct page address

**Your devicetree is correct:**
```dts
eeprom: eeprom@50 {
    compatible = "st,m24c16", "st,m24xxx", "atmel,at24";
    reg = <0x50>;              // Base address
    size = <DT_SIZE_K(2)>;     // Driver knows total is 2KB across all pages
    address-width = <8>;       // 8-bit addressing per page
}
```

When you write to byte offset 0x250, the driver:
1. Calculates page: 0x250 / 256 = page 2
2. Uses I2C address: 0x50 + 2 = 0x52
3. Uses offset within page: 0x250 % 256 = 0x50

**Bottom line:** You always use address `0x50` in devicetree and code. The driver handles the page mapping automatically.

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Boot Sequence                             │
├─────────────────────────────────────────────────────────────┤
│  1. Initialize motor_params with defaults                   │
│  2. motor_control_api_init(&motor_params)                   │
│  3. settings_subsys_init()                                   │
│  4. settings_load()  ───► Read from EEPROM ───┐             │
│                                                 │             │
│                                                 ▼             │
│                          ┌───────────────────────────────┐   │
│                          │  motor_settings_set()         │   │
│                          │  - Called for each param      │   │
│                          │  - Calls motor_api_set_param()│   │
│                          │  - Posts to event queue       │   │
│                          └───────────────────────────────┘   │
│  5. Continue normal init...                                  │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│                Runtime Parameter Updates                     │
├─────────────────────────────────────────────────────────────┤
│                                                               │
│  Shell/Protocol changes parameter:                           │
│    motor_api_set_param("Id_setpoint_A", 1.5)                │
│                                                               │
│  User explicitly saves:                                       │
│    motor params save  (shell command)                        │
│         │                                                     │
│         ▼                                                     │
│    settings_save() ───► Write to EEPROM ◄─┐                 │
│                                             │                 │
│                          ┌──────────────────┘                │
│                          │  motor_settings_export()          │
│                          │  - Reads all params via API       │
│                          │  - Returns values to Settings     │
│                          └───────────────────────────────────┘
│                                                               │
└─────────────────────────────────────────────────────────────┘
```

## Implementation Steps

### Step 1: Devicetree Configuration

The EEPROM is already configured in `boards/rubus/smartstepper_v2/smartstepper_v2.dts`:

```dts
&i2c3 {
    /* ... existing config ... */
    
    eeprom: eeprom@50 {
        compatible = "st,m24c16", "st,m24xxx", "atmel,at24";
        reg = <0x50>;
        size = <DT_SIZE_K(2)>;    /* 2KB (16 kilobit) */
        pagesize = <16>;          /* 16-byte page writes */
        address-width = <8>;      /* 8-bit addressing */
        timeout = <5>;
        status = "okay";

        partitions {
            compatible = "fixed-partitions";
            #address-cells = <1>;
            #size-cells = <1>;

            /* Reserve 1KB for motor parameter settings */
            settings_partition: partition@0 {
                label = "settings_storage";
                reg = <0x0000 0x0400>;  /* 1KB at start of EEPROM */
            };

            /* Remaining 1KB available for calibration data, logs, etc. */
            user_data: partition@400 {
                label = "user_data";
                reg = <0x0400 0x0400>;  /* 1KB remaining */
            };
        };
    };
};

/ {
    chosen {
        /* ... existing entries ... */
        zephyr,settings-partition = &settings_partition;
    };
};
```

**Storage Capacity:**
- Total EEPROM: 2KB (2048 bytes)
- Settings partition: 1KB (1024 bytes)
- User data partition: 1KB (1024 bytes)
- NVS overhead: ~8-12 bytes per entry (key + metadata)
- Float parameter: 4 bytes (binary format)
- **Capacity: ~60-70 float parameters** with typical key names

### Step 2: Kconfig Configuration

Add to `app/prj.conf`:

```kconfig
# Settings Subsystem (Parameter Persistence)
CONFIG_SETTINGS=y
CONFIG_SETTINGS_RUNTIME=y

# EEPROM Backend
CONFIG_EEPROM=y
CONFIG_EEPROM_AT24=y

# Flash map (required for partition access)
CONFIG_FLASH_MAP=y

# NVS (Non-Volatile Storage) - works with EEPROM
CONFIG_NVS=y
CONFIG_SETTINGS_NVS=y

# I2C (if not already enabled)
CONFIG_I2C=y
```

**Why NVS for EEPROM?**
- Designed for byte-writable storage (perfect for EEPROM)
- No erase cycles needed (unlike flash filesystems)
- Settings subsystem has native NVS backend
- Append-only structure with garbage collection
- Binary format for efficient storage

### Step 3: Create Settings Handler

Create new file `app/src/motor_settings.c`:

```c
/*
 * Copyright (c) 2025 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/settings/settings.h>
#include <zephyr/kernel.h>
#include <string.h>

#include "motor_control_api.h"
#include "config.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(motor_settings, CONFIG_APP_LOG_LEVEL);

/**
 * @brief Settings load callback
 * 
 * Called by Settings subsystem for each stored parameter during boot.
 * Uses motor_api_set_param() to restore values via event queue.
 * 
 * @param name Parameter name (e.g., "Id_setpoint_A")
 * @param len Length of stored data
 * @param read_cb Callback to read data from storage
 * @param cb_arg Argument for read callback
 * @return 0 on success, negative errno on failure
 */
static int motor_settings_set(const char *name, size_t len,
                              settings_read_cb read_cb, void *cb_arg)
{
    const char *next;
    float value;
    int rc;

    if (len != sizeof(float)) {
        LOG_ERR("Invalid data length for %s: %zu (expected %zu)",
                name, len, sizeof(float));
        return -EINVAL;
    }

    /* Read float value from storage */
    rc = read_cb(cb_arg, &value, sizeof(value));
    if (rc < 0) {
        LOG_ERR("Failed to read setting %s: %d", name, rc);
        return rc;
    }

    /* Apply parameter using motor control API
     * This posts an event to the queue, ensuring thread-safe application */
    rc = motor_api_set_param(name, value);
    if (rc < 0) {
        LOG_WRN("Failed to restore parameter %s: %d", name, rc);
        return rc;
    }

    LOG_INF("Restored %s = %.3f", name, (double)value);
    return 0;
}

/**
 * @brief Settings export callback
 * 
 * Called by Settings subsystem during settings_save().
 * Reads current parameter values via motor_api_get_param() and
 * passes them to Settings for storage.
 * 
 * @param cb Callback to write data to storage
 * @return 0 on success, negative errno on failure
 */
static int motor_settings_export(int (*cb)(const char *name,
                                           const void *value,
                                           size_t val_len))
{
    float value;
    int rc;
    int saved_count = 0;

    /* Export all double-buffered control parameters */
    const char *param_names[] = {
        "Id_setpoint_A",
        "Iq_setpoint_A",
        /* Add more parameters as they're added to param_table[] */
    };

    for (size_t i = 0; i < ARRAY_SIZE(param_names); i++) {
        rc = motor_api_get_param(param_names[i], &value);
        if (rc == 0) {
            /* Build full key: "motor/<param_name>" */
            char key[64];
            snprintf(key, sizeof(key), "motor/%s", param_names[i]);
            
            rc = cb(key, &value, sizeof(value));
            if (rc < 0) {
                LOG_ERR("Failed to save %s: %d", param_names[i], rc);
            } else {
                saved_count++;
                LOG_DBG("Saved %s = %.3f", param_names[i], (double)value);
            }
        }
    }

    LOG_INF("Saved %d parameters to EEPROM", saved_count);
    return 0;
}

/**
 * @brief Settings commit callback
 * 
 * Called after all settings are loaded. Can be used for validation
 * or applying batch updates.
 */
static int motor_settings_commit(void)
{
    LOG_INF("Motor settings loaded and committed");
    return 0;
}

/* Register settings handler with Zephyr Settings subsystem
 * 
 * Handler name: "motor"
 * Keys will be stored as: "motor/Id_setpoint_A", "motor/Iq_setpoint_A", etc.
 */
SETTINGS_STATIC_HANDLER_DEFINE(motor, "motor", NULL,
                               motor_settings_set,
                               motor_settings_commit,
                               motor_settings_export);

/**
 * @brief Initialize settings subsystem
 * 
 * Must be called after motor_control_api_init() but before settings_load()
 */
int motor_settings_init(void)
{
    int rc;

    /* Initialize Zephyr Settings subsystem */
    rc = settings_subsys_init();
    if (rc) {
        LOG_ERR("Settings subsystem init failed: %d", rc);
        return rc;
    }

    LOG_INF("Settings subsystem initialized (EEPROM backend)");
    return 0;
}

/**
 * @brief Load saved parameters from EEPROM
 * 
 * Restores all previously saved parameters. Should be called during
 * boot sequence after motor_settings_init().
 */
int motor_settings_load(void)
{
    int rc;

    rc = settings_load();
    if (rc) {
        LOG_ERR("Failed to load settings: %d", rc);
        return rc;
    }

    LOG_INF("Settings loaded from EEPROM");
    return 0;
}

/**
 * @brief Save current parameters to EEPROM
 * 
 * Saves all parameters returned by motor_settings_export().
 * Can be called from shell command or automatically on parameter changes.
 */
int motor_settings_save(void)
{
    int rc;

    rc = settings_save();
    if (rc) {
        LOG_ERR("Failed to save settings: %d", rc);
        return rc;
    }

    LOG_INF("Settings saved to EEPROM");
    return 0;
}
```

Create header `app/include/motor_settings.h`:

```c
/*
 * Copyright (c) 2025 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_SETTINGS_H_
#define MOTOR_SETTINGS_H_

/**
 * @brief Initialize settings subsystem
 * @return 0 on success, negative errno on failure
 */
int motor_settings_init(void);

/**
 * @brief Load saved parameters from EEPROM
 * @return 0 on success, negative errno on failure
 */
int motor_settings_load(void);

/**
 * @brief Save current parameters to EEPROM
 * @return 0 on success, negative errno on failure
 */
int motor_settings_save(void);

#endif /* MOTOR_SETTINGS_H_ */
```

### Step 4: Update CMakeLists.txt

Add to `app/CMakeLists.txt`:

```cmake
target_sources(app PRIVATE
    src/main.c
    src/motor_states.c
    src/motor_isr.c
    src/motor_hardware.c
    src/motor_control_api.c
    src/motor_settings.c        # Add this line
    src/shell_commands.c
    # ... other sources
)
```

### Step 5: Update Main Initialization

Modify `app/src/main.c`:

```c
#include "motor_settings.h"

int main(void)
{
    /* Initialize motor parameters with defaults */
    struct motor_parameters motor_params = {
        /* ... existing initialization ... */
    };

    config_print_parameters();

    /* Initialize motor control API */
    if (motor_control_api_init(&motor_params) < 0) {
        LOG_ERR("Failed to initialize motor control API");
        return 0;
    }
    shell_set_motor_params(&motor_params);

    /* Initialize settings subsystem */
    if (motor_settings_init() < 0) {
        LOG_ERR("Failed to initialize settings");
        return 0;
    }

    /* Load saved parameters from EEPROM
     * This will call motor_api_set_param() for each saved parameter */
    if (motor_settings_load() < 0) {
        LOG_WRN("Failed to load settings, using defaults");
        /* Not fatal - continue with default values */
    }

    /* Hardware initialization */
    if (motor_hardware_init_gpio() < 0) {
        LOG_ERR("Failed to initialize GPIO");
        return 0;
    }

    if (motor_hardware_check_devices() < 0) {
        LOG_ERR("Device readiness check failed");
        return 0;
    }

    /* State machine initialization */
    smf_set_initial(SMF_CTX(&motor_params), &motor_states[MOTOR_STATE_HW_INIT]);

    /* Main control loop */
    while (1) {
        struct motor_event evt;
        if (motor_api_peek_event(&evt) == 0) {
            LOG_DBG("Event pending: type=%d", evt.type);
        }

        if (smf_run_state(SMF_CTX(&motor_params)) < 0) {
            LOG_ERR("State machine error");
            break;
        }

        k_sleep(K_MSEC(100));
    }

    return 0;
}
```

### Step 6: Add Shell Commands

Add to `app/src/shell_commands.c`:

```c
#include "motor_settings.h"

/* motor params save - Save current parameters to EEPROM */
static int cmd_motor_params_save(const struct shell *sh, size_t argc, char **argv)
{
    shell_print(sh, "Saving parameters to EEPROM...");
    
    if (motor_settings_save() < 0) {
        shell_error(sh, "Failed to save parameters");
        return -EIO;
    }
    
    shell_print(sh, "Parameters saved successfully");
    return 0;
}

/* motor params load - Reload parameters from EEPROM */
static int cmd_motor_params_load(const struct shell *sh, size_t argc, char **argv)
{
    shell_print(sh, "Loading parameters from EEPROM...");
    
    if (motor_settings_load() < 0) {
        shell_error(sh, "Failed to load parameters");
        return -EIO;
    }
    
    shell_print(sh, "Parameters loaded successfully");
    return 0;
}

/* motor params reset - Clear EEPROM and restore defaults */
static int cmd_motor_params_reset(const struct shell *sh, size_t argc, char **argv)
{
    shell_print(sh, "Resetting parameters to defaults...");
    
    /* Delete all motor settings from EEPROM */
    int rc = settings_delete("motor");
    if (rc < 0) {
        shell_error(sh, "Failed to clear EEPROM: %d", rc);
        return rc;
    }
    
    /* Reload defaults (from motor_params initialization in main) */
    shell_print(sh, "Parameters reset. Reboot to apply defaults.");
    return 0;
}

/* Add to params subcommand table */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_params,
    SHELL_CMD(get, NULL, "Get parameter value", cmd_motor_params_get),
    SHELL_CMD(set, NULL, "Set parameter value", cmd_motor_params_set),
    SHELL_CMD(list, NULL, "List all parameters", cmd_motor_params_list),
    SHELL_CMD(save, NULL, "Save parameters to EEPROM", cmd_motor_params_save),
    SHELL_CMD(load, NULL, "Reload parameters from EEPROM", cmd_motor_params_load),
    SHELL_CMD(reset, NULL, "Reset parameters to defaults", cmd_motor_params_reset),
    SHELL_SUBCMD_SET_END
);
```

## Usage Examples

### First-Time Setup

```shell
motor:~$ motor params set Id_setpoint_A 0.5
Id_setpoint_A = 0.500

motor:~$ motor params set Iq_setpoint_A 1.2
Iq_setpoint_A = 1.200

motor:~$ motor params save
Saving parameters to EEPROM...
Parameters saved successfully
```

### After Reboot

```
[00:00:00.123,000] <inf> motor_settings: Restored Id_setpoint_A = 0.500
[00:00:00.124,000] <inf> motor_settings: Restored Iq_setpoint_A = 1.200
[00:00:00.125,000] <inf> motor_settings: Motor settings loaded and committed
```

### Reset to Defaults

```shell
motor:~$ motor params reset
Resetting parameters to defaults...
Parameters reset. Reboot to apply defaults.

motor:~$ kernel reboot cold
```

## EEPROM Considerations

### Write Cycle Limits

**ST M24C16 Specs:**
- 1,000,000 write cycles per page (datasheet guaranteed)
- Typical usage: 10-100 writes over product lifetime (commissioning/tuning)
- **Wear leveling not needed** for such infrequent writes
- Safety margin: >10,000x expected usage

**Expected Write Scenarios:**
1. **Initial commissioning**: 5-10 saves (setting up motor parameters)
2. **Periodic tuning**: 1-2 saves per year (PI gain adjustments)
3. **Calibration updates**: 2-5 saves per year (after motor changes)
4. **Total lifetime**: <100 writes typical

With 1,000,000 cycle rating, even writing every day for 30 years uses only 10,950 cycles.

**Best Practices:**
1. **Manual Save**: Require explicit `motor params save` command (recommended)
2. **No Auto-Save**: Not needed - user controls when to persist changes
3. **Confirmation**: Prompt before saving to prevent accidental writes

### Auto-Save Implementation (Not Recommended)

Auto-save is **not recommended** for this application:

**Reasons to avoid auto-save:**
1. Parameters are tuned infrequently (commissioning/maintenance only)
2. User may be experimenting with values - don't persist bad settings
3. Prevents accidental corruption from temporary test values
4. Explicit save gives user control

**Manual save workflow (recommended):**
```shell
# User experiments with parameters
motor:~$ motor params set Iq_setpoint_A 1.5
motor:~$ motor state start
# Test motor response...
motor:~$ motor state stop

# Only save when satisfied
motor:~$ motor params save
Parameters saved to EEPROM

# Bad value? Just reboot to restore last saved state
motor:~$ kernel reboot cold
```

### Storage Layout in EEPROM

```
EEPROM Layout (ST M24C16 - 2KB):

0x0000 ┌────────────────────────────────────┐
       │  Settings Partition (1KB)          │
       │  ┌──────────────────────────────┐  │
       │  │ NVS Sector 0 (512 bytes)     │  │
       │  │ ┌──────────────────────────┐ │  │
       │  │ │ ATE: 0xFF (empty)        │ │  │
       │  │ │ motor/Id_setpoint_A      │ │  │
       │  │ │   value: 0.500000 (4B)   │ │  │
       │  │ │ motor/Iq_setpoint_A      │ │  │
       │  │ │   value: 1.200000 (4B)   │ │  │
       │  │ │ motor/Rs_ohm             │ │  │
       │  │ │   value: 0.350000 (4B)   │ │  │
       │  │ │ ...                      │ │  │
       │  │ └──────────────────────────┘ │  │
       │  └──────────────────────────────┘  │
       │  ┌──────────────────────────────┐  │
       │  │ NVS Sector 1 (512 bytes)     │  │
0x0400 │  │ - Overflow/rotation space    │  │
       │  └──────────────────────────────┘  │
       └────────────────────────────────────┘
       │  User Data Partition (1KB)         │
       │  ┌──────────────────────────────┐  │
       │  │ - Calibration timestamps     │  │
       │  │ - Fault event log            │  │
0x0800 │  │ - Factory calibration backup │  │
       │  └──────────────────────────────┘  │
       └────────────────────────────────────┘

Format: Binary with CRC protection
ATE (Allocation Table Entry) tracks free space
```

### NVS Configuration

NVS (Non-Volatile Storage) is designed for byte-writable storage like EEPROM:

```kconfig
# NVS configuration for 1KB partition
CONFIG_NVS_SECTOR_SIZE=512       # 512 bytes per sector (2 sectors)
CONFIG_NVS_SECTOR_COUNT=2        # Minimum 2 sectors
```

**How NVS Works with EEPROM:**
1. **Append-Only Writes**: New entries appended, old invalidated (no erase needed)
2. **Garbage Collection**: When sector fills, valid entries copied to next sector
3. **CRC Protection**: Each entry has CRC for corruption detection
4. **Atomic Updates**: Power-loss safe writes
5. **Binary Format**: Efficient storage (4 bytes per float)

**Sector Management:**
- Sector 0 is active, receives new writes
- When Sector 0 fills, valid entries compacted to Sector 1
- Sectors swap roles (ping-pong buffering)
- Old entries naturally pruned during compaction

## Testing

### Verify EEPROM Access

```shell
motor:~$ device list
- eeprom@50 (READY)

motor:~$ i2c scan i2c@40005400
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- -- -- -- -- -- -- -- -- -- -- -- --
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
50: 50 -- -- -- -- -- -- -- -- -- -- -- -- -- -- --  # EEPROM detected
60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
70: -- -- -- -- -- -- -- -- -- -- -- --
```

### Verify Settings Storage

```shell
# Enable debug logging
motor:~$ log enable dbg motor_settings

# Save parameters
motor:~$ motor params set Id_setpoint_A 2.5
motor:~$ motor params save
[00:01:23.456,000] <dbg> motor_settings: Saved Id_setpoint_A = 2.500
[00:01:23.457,000] <inf> motor_settings: Saved 2 parameters to EEPROM

# Reboot and verify restore
motor:~$ kernel reboot cold
[00:00:00.123,000] <inf> motor_settings: Restored Id_setpoint_A = 2.500
```

## Troubleshooting

### EEPROM Not Detected

1. Check I2C bus and pins in devicetree
2. Verify pull-up resistors (4.7kΩ typical)
3. Check I2C address (A0/A1/A2 pins on EEPROM)
4. Use `i2c scan` to verify device responds

### Settings Not Persisting

1. Check partition is defined in devicetree
2. Verify `zephyr,settings-partition` chosen node
3. Enable debug logging: `CONFIG_SETTINGS_LOG_LEVEL_DBG=y`
4. Check EEPROM write-protect pin (if present)

### Corrupted Settings

```shell
# Clear all settings and start fresh
motor:~$ motor params reset
motor:~$ kernel reboot cold
```

NVS includes CRC checks - corrupted data is automatically rejected.

## Future Enhancements

### 1. Calibration Data Storage

Extend to store motor-specific calibration:
```c
/* Save resistance, inductance from calibration routine */
settings_save_one("motor/Rs_ohm", &motor_params.Rs_ohm, sizeof(float));
settings_save_one("motor/Ls_H", &motor_params.Ls_H, sizeof(float));
settings_save_one("motor/flux_linkage", &motor_params.flux_linkage_Wb, sizeof(float));
```

### 2. Configuration Profiles

Store multiple parameter sets:
```c
/* Save/load profiles */
settings_save_one("motor/profile/high_torque/Iq_max", ...);
settings_save_one("motor/profile/high_speed/Iq_max", ...);
```

### 3. Factory Reset

```c
int motor_settings_factory_reset(void)
{
    /* Clear all settings */
    settings_delete("motor");
    
    /* Restore compile-time defaults */
    motor_params_reset_to_defaults();
    
    /* Save defaults */
    motor_settings_save();
}
```

## Summary

The Settings subsystem integration:

✅ **Minimal Code**: ~200 lines for full persistence
✅ **Thread-Safe**: Uses existing motor_control_api
✅ **Flexible**: Works with EEPROM, flash, filesystem
✅ **Wear-Leveling**: NVS protects EEPROM lifetime
✅ **Robust**: CRC checks, atomic updates, power-loss safe
✅ **Extensible**: Easy to add more parameters
✅ **Standards-Based**: Uses Zephyr Settings subsystem

Your refactored API design makes this integration straightforward - the Settings subsystem just calls your existing `set_param()` and `get_param()` functions.
