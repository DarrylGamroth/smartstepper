# Devicetree vs Kconfig Parameter Analysis

## Current Architecture

Your motor control application currently uses **devicetree (DT) for almost all configuration parameters**. This includes:

### Parameters in Devicetree (via board overlays)

#### 1. User Parameters (`user_parameters` node)
- **Control Loop Settings**: PWM frequency, control loop frequency, current loop bandwidth
- **Calibration Settings**: Alignment current/duration, R/L estimation parameters
- **Operational Limits**: Max modulation index, velocity limits, acceleration limits
- **Filter Settings**: Offset filter pole frequency

#### 2. Motor Parameters (`motor_parameters` node)
- **Electrical Properties**: Ld, Lq, Rs, flux linkage, pole pairs
- **Physical Properties**: Inertia, max speed, max current

#### 3. Hardware Configuration
- **Current Sensing**: Resistor values, gains, channels, ADC buffer indices
- **Voltage Sensing**: Divider resistors, ADC channels
- **Observer Settings**: Angle observer bandwidth
- **Fault Detection**: Thresholds (encoder fault, overcurrent)

---

## Analysis: Which Parameters Should Be in Kconfig?

### ✅ **Keep in Devicetree** (Hardware-Specific, Per-Board)

These parameters are **hardware-dependent** and vary between board designs:

| Parameter Category | Examples | Reasoning |
|-------------------|----------|-----------|
| **Hardware Topology** | Current sense resistors, gains, ADC channels, pin assignments | Different board revisions have different hardware |
| **Motor Electrical** | Ld, Lq, Rs, flux linkage, pole pairs | Different motors connected to different boards |
| **Voltage/Current Sensing** | Divider resistors, shunt resistors, ADC buffer indices | Board-specific circuitry |
| **PWM/Timer Config** | Timer prescalers, DMA channels, pin mappings | Hardware peripheral assignments |

**Why DT?**: These parameters describe the physical hardware. Changing boards (smartstepper vs smartstepper_v2 vs nucleo) requires different values. DT overlays naturally handle per-board configuration.

---

### ⚠️ **Consider Moving to Kconfig** (Software Features/Algorithms)

These are **software configuration options** that could apply across all boards:

| Parameter | Current Location | Should Move? | Type | Reasoning |
|-----------|------------------|--------------|------|-----------|
| **RLS Parameter Estimation** | Not configured | ✅ **YES** | Kconfig | Feature toggle (you just added this) |
| **Logging Levels** | Kconfig | ✅ Already correct | Kconfig | Build-time software feature |
| **Debug Options** | Kconfig | ✅ Already correct | Kconfig | Build-time optimization |
| **Control Algorithm Choice** | N/A | ⚠️ **Consider** | Kconfig | If you add alternative FOC algorithms |
| **Observer Type** | Hardcoded | ⚠️ **Consider** | Kconfig | If you implement multiple observers (PLL, Luenberger, etc.) |

---

### 🤔 **Gray Area** (Could Go Either Way)

These parameters are **tuning parameters** that could be board-specific OR runtime-configurable:

| Parameter | Current | DT Pros | Kconfig Pros | Recommendation |
|-----------|---------|---------|--------------|----------------|
| **Current Loop Bandwidth** | DT | Different motors need different tuning | Could be compile-time option for performance | **Keep in DT** - motor-dependent |
| **Alignment Current/Duration** | DT | Safe defaults per motor | Could be build configuration | **Keep in DT** - motor-dependent |
| **R/L Estimation Params** | DT | Motor-specific safe values | Could be compile-time calibration options | **Keep in DT** - motor-dependent |
| **Max Modulation Index** | DT | H-bridge topology dependent | Could be software limit | **Keep in DT** - hardware-dependent |
| **Velocity Limits** | DT | Motor mechanical limits | Could be runtime configurable | **Keep in DT** - safety critical |
| **Fault Thresholds** | DT | Hardware current limits | Could be software policy | **Keep in DT** - hardware protection |
| **Observer Bandwidth** | DT | Motor/encoder dependent | Could be tuning parameter | **Keep in DT** - system-specific |
| **RLS Decimation** | DT | Motor time constants | Performance vs accuracy tradeoff | **Keep in DT** - system-specific |
| **Thermal Model Params** | DT | Motor-specific thermal properties | Could be algorithm tuning | **Keep in DT** - motor-specific |

---

## Pros and Cons Summary

### Devicetree Advantages ✅

1. **Per-Board Configuration**: Each board overlay can have different values
2. **Runtime Introspection**: Parameters accessible via devicetree API at runtime
3. **Hardware Description**: Natural fit for hardware topology and electrical properties
4. **No Recompilation**: Can change motor/board without rebuilding (in theory, with overlays)
5. **Type Safety**: Devicetree bindings provide validation and documentation
6. **Tool Support**: `west build -t menuconfig` can't easily handle per-board values

### Devicetree Disadvantages ❌

1. **Less Discoverable**: Harder to find than Kconfig menu
2. **No Conditional Compilation**: Can't disable code at compile time
3. **Limited Validation**: Harder to express complex constraints
4. **Learning Curve**: Less familiar to embedded developers than Kconfig
5. **Static After Build**: Can't change without new firmware (but this is also a safety feature)

### Kconfig Advantages ✅

1. **Feature Toggles**: Enable/disable entire subsystems at compile time
2. **Conditional Compilation**: Reduce code size by removing unused features
3. **Familiar Interface**: `menuconfig` is standard in Linux/Zephyr world
4. **Validation**: Can express dependencies between options
5. **Documentation**: Help text appears in menuconfig
6. **Build Optimization**: Dead code elimination for disabled features

### Kconfig Disadvantages ❌

1. **Build-Time Only**: Must recompile to change values
2. **Not Board-Specific**: Same Kconfig applies to all boards (requires workarounds)
3. **No Runtime Access**: Values baked into code as `#define`
4. **Not for Hardware**: Poor fit for describing hardware topology

---

## Recommendations

### 1. **Feature Flags → Kconfig** ✅

Move these to Kconfig:

```kconfig
menu "Motor Control Features"

config RLS_PARAMETER_ESTIMATION
    bool "Enable RLS online parameter estimation"
    default n  # You already added this!

config MOTOR_THERMAL_MODEL
    bool "Enable motor thermal model"
    default n

config ENCODER_FAULT_DETECTION
    bool "Enable encoder fault detection"
    default y

config VBUS_REGEN_BRAKING
    bool "Enable Vbus-regulated regenerative braking"
    default y

endmenu
```

### 2. **Hardware/Motor Parameters → Keep in Devicetree** ✅

Continue using DT for:
- Motor electrical parameters (Ld, Lq, Rs, flux linkage)
- Current/voltage sensing hardware
- PWM frequencies and timing
- Calibration parameters (alignment, R/L estimation)
- Operational limits (max current, max speed)
- Observer tuning (bandwidth)

### 3. **Consider Runtime Configuration (Future)** 🔮

For production systems, you might want:
- **NVS/EEPROM storage** for tuning parameters (can override DT defaults)
- **Shell commands** for runtime adjustment (already partially implemented)
- **Parameter persistence** across reboots

This would keep:
- Safe defaults in DT
- Feature enables in Kconfig  
- Runtime overrides in NVS

---

## Specific Recommendations for Your System

### Move to Kconfig (Feature Flags)

```kconfig
config RLS_PARAMETER_ESTIMATION
    # Already added! ✓

config MOTOR_CONTROL_DEBUG
    bool "Enable motor control debug outputs"
    default n
    help
      Enable additional debug telemetry and logging

config USE_ANGLE_OBSERVER
    bool "Use angle observer for position estimation"
    default y
    help
      Enable Luenberger observer for angle/speed estimation
```

### Keep in Devicetree (Everything Else)

Your current architecture is sound! The parameters in DT are hardware/motor-specific and benefit from per-board configuration.

---

## Example: Hybrid Approach

```c
/* Feature enabled via Kconfig */
#ifdef CONFIG_RLS_PARAMETER_ESTIMATION
    /* Parameters configured via Devicetree */
    uint32_t decimation = RLS_DECIMATION;  /* From DT */
    float32_t lambda = RLS_LAMBDA;         /* From DT */
    
    rls_motor_est_init(&params->rls_d, lambda, decimation);
#endif
```

This gives you:
- **Kconfig**: Compile-time feature toggle (removes code when disabled)
- **Devicetree**: Runtime parameters specific to motor/hardware

---

## Adding NVS Storage: Three-Tier Parameter Architecture

### Parameter Source Hierarchy

With NVS storage, you create a **three-tier parameter hierarchy**:

```
┌─────────────────────────────────────────────┐
│  1. Kconfig (Compile Time)                  │  Highest Priority
│     - Feature enables/disables              │  (Removes code)
│     - Debug/optimization options            │
└─────────────────────────────────────────────┘
           ↓
┌─────────────────────────────────────────────┐
│  2. Devicetree (Build Time)                 │  Medium Priority
│     - Hardware topology                     │  (Safe defaults)
│     - Motor electrical parameters           │
│     - Factory-calibrated values             │
└─────────────────────────────────────────────┘
           ↓
┌─────────────────────────────────────────────┐
│  3. NVS Storage (Runtime Persistent)        │  Lowest Priority
│     - User tuning overrides                 │  (User customization)
│     - Calibration results                   │
│     - Learned parameters (RLS)              │
└─────────────────────────────────────────────┘
```

### Design Pattern: Override with Fallback

```c
/* Parameter loading priority */
float32_t load_parameter(const char *key, float32_t dt_default) {
    float32_t value;
    
    /* Try to load from NVS first */
    if (nvs_read_float(key, &value) == 0) {
        LOG_INF("Loaded %s from NVS: %.3f", key, value);
        return value;
    }
    
    /* Fall back to devicetree default */
    LOG_INF("Using %s from DT: %.3f", key, dt_default);
    return dt_default;
}
```

---

## Which Parameters Should Be NVS-Overridable?

### ✅ **Good Candidates for NVS Override**

Parameters that benefit from **field tuning** or **learned values**:

| Parameter Category | Examples | Why NVS? |
|-------------------|----------|----------|
| **Calibration Results** | Current offsets, encoder zero position | Measured in-situ, should persist |
| **RLS Learned Parameters** | Rs, Ld, Lq estimates | Improve over time, motor-specific |
| **Tuning Parameters** | PI gains, observer bandwidth | May need field adjustment |
| **Operational Limits** | Max speed, max current | Customer-specific derating |
| **User Preferences** | Default velocity, acceleration | Application-specific |
| **Safety Margins** | Overcurrent threshold, voltage limits | May need tightening after testing |

### ❌ **Should NOT Be NVS-Overridable** (Safety-Critical)

Parameters that must remain **hardware-enforced** or **factory-locked**:

| Parameter | Why Fixed? |
|-----------|------------|
| **Hardware Topology** | Current sense resistors, gains, ADC channels | Hardware reality, corruption = damage |
| **PWM Frequency** | Timer configuration, DMA setup | System stability critical |
| **Motor Pole Pairs** | Commutation accuracy | Incorrect = runaway motor |
| **Absolute Max Current** | Hardware current limit | Safety critical - exceeding damages hardware |
| **Voltage Sense Scaling** | Divider resistors | Corruption = overvoltage damage |

### ⚠️ **Conditional NVS Override** (With Validation)

Parameters that can be overridden but need **strict validation**:

| Parameter | Validation Rule | Fallback |
|-----------|-----------------|----------|
| **PI Gains** | Kp > 0, Ki > 0, reasonable range | DT defaults |
| **Current Limits** | Must be < hardware absolute max | DT defaults |
| **Velocity Limits** | Must be < motor mechanical max | DT defaults |
| **Observer Bandwidth** | Must be < Nyquist limit | DT defaults |

---

## Proposed NVS Parameter Structure

### Storage Keys (Namespace: `motor_params`)

```c
/* Calibration data (always use NVS after calibration) */
#define NVS_KEY_CURRENT_OFFSET_A    "cal/ia_offset"
#define NVS_KEY_CURRENT_OFFSET_B    "cal/ib_offset"
#define NVS_KEY_ENCODER_ZERO        "cal/enc_zero"
#define NVS_KEY_RS_MEASURED         "cal/rs_ohm"
#define NVS_KEY_LS_MEASURED         "cal/ls_h"

/* User tuning overrides (optional, fallback to DT) */
#define NVS_KEY_PI_KP_D             "tune/pi_kp_d"
#define NVS_KEY_PI_KI_D             "tune/pi_ki_d"
#define NVS_KEY_PI_KP_Q             "tune/pi_kp_q"
#define NVS_KEY_PI_KI_Q             "tune/pi_ki_q"
#define NVS_KEY_OBS_BANDWIDTH       "tune/obs_bw_hz"
#define NVS_KEY_MAX_CURRENT         "tune/i_max_a"
#define NVS_KEY_MAX_SPEED           "tune/v_max_hz"

/* RLS learned parameters (updated periodically) */
#define NVS_KEY_RLS_LD              "rls/ld_h"
#define NVS_KEY_RLS_LQ              "rls/lq_h"
#define NVS_KEY_RLS_RS              "rls/rs_ohm"
#define NVS_KEY_RLS_TIMESTAMP       "rls/timestamp"

/* System state */
#define NVS_KEY_PARAM_VERSION       "sys/version"
#define NVS_KEY_CALIBRATION_VALID   "sys/cal_valid"
```

### Parameter Loading Flow

```c
int motor_params_init(struct motor_parameters *params)
{
    int ret;
    bool nvs_valid = false;
    
    /* 1. Load devicetree defaults */
    params->Ia_offset = 0.0f;  /* DT doesn't have offset - must calibrate */
    params->Ib_offset = 0.0f;
    params->Rs_measured_ohm = MOTOR_RESISTANCE_OHM;  /* DT default */
    params->Ls_measured_H = MOTOR_INDUCTANCE_D_H;    /* DT default */
    
    /* 2. Initialize NVS subsystem */
    ret = nvs_init();
    if (ret < 0) {
        LOG_WRN("NVS init failed, using DT defaults only");
        return 0;  /* Non-fatal - can operate with DT defaults */
    }
    
    /* 3. Check if calibration data exists */
    uint8_t cal_valid = 0;
    if (nvs_read_u8(NVS_KEY_CALIBRATION_VALID, &cal_valid) == 0 && cal_valid == 1) {
        nvs_valid = true;
    }
    
    /* 4. Load calibration data from NVS (required for operation) */
    if (nvs_valid) {
        nvs_read_float(NVS_KEY_CURRENT_OFFSET_A, &params->Ia_offset);
        nvs_read_float(NVS_KEY_CURRENT_OFFSET_B, &params->Ib_offset);
        nvs_read_float(NVS_KEY_RS_MEASURED, &params->Rs_measured_ohm);
        nvs_read_float(NVS_KEY_LS_MEASURED, &params->Ls_measured_H);
        LOG_INF("Loaded calibration from NVS");
    } else {
        LOG_WRN("No valid calibration in NVS - must run calibration");
        /* Motor won't be able to run until calibrated */
    }
    
    /* 5. Load optional tuning overrides from NVS (fallback to DT) */
    float32_t pi_kp_d = load_parameter_with_validation(
        NVS_KEY_PI_KP_D, 
        MOTOR_INDUCTANCE_D_H * CURRENT_LOOP_BANDWIDTH_RPS,  /* DT default */
        0.001f,  /* min */
        100.0f   /* max */
    );
    pi_set_kp(&params->pi_Id, pi_kp_d);
    
    /* 6. Load RLS learned parameters if available */
    #ifdef CONFIG_RLS_PARAMETER_ESTIMATION
    if (nvs_read_float(NVS_KEY_RLS_LD, &params->Ld_est) != 0) {
        params->Ld_est = MOTOR_INDUCTANCE_D_H;  /* Fallback to DT */
    }
    if (nvs_read_float(NVS_KEY_RLS_LQ, &params->Lq_est) != 0) {
        params->Lq_est = MOTOR_INDUCTANCE_Q_H;  /* Fallback to DT */
    }
    #endif
    
    return 0;
}
```

### Parameter Update and Persistence

```c
int motor_params_update_calibration(struct motor_parameters *params)
{
    int ret;
    
    /* Update parameters in memory */
    params->Ia_offset = filter_fo_get_out(&params->filter_Ia);
    params->Ib_offset = filter_fo_get_out(&params->filter_Ib);
    
    /* Persist to NVS */
    ret = nvs_write_float(NVS_KEY_CURRENT_OFFSET_A, params->Ia_offset);
    if (ret < 0) return ret;
    
    ret = nvs_write_float(NVS_KEY_CURRENT_OFFSET_B, params->Ib_offset);
    if (ret < 0) return ret;
    
    ret = nvs_write_float(NVS_KEY_RS_MEASURED, params->Rs_measured_ohm);
    if (ret < 0) return ret;
    
    ret = nvs_write_float(NVS_KEY_LS_MEASURED, params->Ls_measured_H);
    if (ret < 0) return ret;
    
    /* Mark calibration as valid */
    ret = nvs_write_u8(NVS_KEY_CALIBRATION_VALID, 1);
    if (ret < 0) return ret;
    
    LOG_INF("Calibration saved to NVS");
    return 0;
}

#ifdef CONFIG_RLS_PARAMETER_ESTIMATION
int motor_params_update_rls(struct motor_parameters *params)
{
    /* Only persist if converged */
    if (!rls_motor_est_is_converged(&params->rls_d) ||
        !rls_motor_est_is_converged(&params->rls_q)) {
        return -EAGAIN;
    }
    
    /* Update estimates */
    params->Ld_est = rls_motor_est_get_L(&params->rls_d);
    params->Lq_est = rls_motor_est_get_L(&params->rls_q);
    params->Rs_measured_ohm = (rls_motor_est_get_Rs(&params->rls_d) + 
                               rls_motor_est_get_Rs(&params->rls_q)) * 0.5f;
    
    /* Persist to NVS (rate-limited, e.g., every 60 seconds) */
    nvs_write_float(NVS_KEY_RLS_LD, params->Ld_est);
    nvs_write_float(NVS_KEY_RLS_LQ, params->Lq_est);
    nvs_write_float(NVS_KEY_RLS_RS, params->Rs_measured_ohm);
    nvs_write_u32(NVS_KEY_RLS_TIMESTAMP, k_uptime_get_32());
    
    LOG_INF("RLS estimates saved to NVS: Ld=%.1fuH, Lq=%.1fuH, Rs=%.1fmΩ",
            params->Ld_est * 1e6f, params->Lq_est * 1e6f, 
            params->Rs_measured_ohm * 1e3f);
    
    return 0;
}
#endif
```

---

## Impact on Current Design

### Architectural Changes

| Component | Before (DT-only) | After (DT + NVS) |
|-----------|------------------|------------------|
| **Initialization** | Load all from DT macros | Load DT defaults, override with NVS |
| **Calibration States** | Store in RAM only | Persist to NVS at end of calibration |
| **Parameter Updates** | Runtime only (lost at reset) | Persist to NVS, survive reboots |
| **Factory Reset** | N/A | Erase NVS, fall back to DT defaults |
| **Field Tuning** | Requires firmware update | Runtime tuning with persistence |

### New State Machine Requirements

Add state transitions for parameter management:

```
MOTOR_STATE_HW_INIT
    ↓
MOTOR_STATE_LOAD_PARAMS  ← NEW: Load from NVS or use DT defaults
    ↓
MOTOR_STATE_CHECK_CALIBRATION  ← NEW: Verify calibration valid
    ↓ (if invalid)
MOTOR_STATE_OFFSET_MEAS → MOTOR_STATE_ROVERL_MEAS → MOTOR_STATE_RS_EST
    ↓ (save to NVS)
MOTOR_STATE_CALIBRATION_COMPLETE  ← NEW: Persist calibration
    ↓
MOTOR_STATE_CTRL_INIT
```

### Shell Command Enhancements

```bash
# View current parameters and their source
motor params list

# Output:
#   Ia_offset: 0.123 A (NVS)
#   Ib_offset: 0.098 A (NVS)
#   Rs: 5.6 mΩ (NVS, RLS learned)
#   Ld: 3.4 uH (DT default)
#   PI Kp_d: 0.021 (NVS override)

# Override parameter (persists to NVS)
motor params set pi_kp_d 0.025

# Factory reset (erase NVS, use DT defaults)
motor params reset

# Force recalibration
motor calibrate

# Save current runtime params to NVS
motor params save
```

---

## Safety Considerations

### Parameter Validation

```c
bool validate_parameter(const char *key, float32_t value)
{
    /* PI gains must be positive */
    if (strcmp(key, NVS_KEY_PI_KP_D) == 0 || strcmp(key, NVS_KEY_PI_KI_D) == 0) {
        if (value <= 0.0f || value > 100.0f) {
            LOG_ERR("Invalid PI gain: %.3f", value);
            return false;
        }
    }
    
    /* Current limit must be below hardware absolute max */
    if (strcmp(key, NVS_KEY_MAX_CURRENT) == 0) {
        if (value > MOTOR_MAX_CURRENT_A || value <= 0.0f) {
            LOG_ERR("Invalid current limit: %.3f A", value);
            return false;
        }
    }
    
    /* Speed limit must be below motor mechanical max */
    if (strcmp(key, NVS_KEY_MAX_SPEED) == 0) {
        if (value > MOTOR_MAX_SPEED_HZ || value <= 0.0f) {
            LOG_ERR("Invalid speed limit: %.3f Hz", value);
            return false;
        }
    }
    
    return true;
}

int nvs_write_float_safe(const char *key, float32_t value)
{
    if (!validate_parameter(key, value)) {
        return -EINVAL;
    }
    return nvs_write_float(key, value);
}
```

### Corruption Detection

```c
struct nvs_param_header {
    uint32_t magic;      /* 0x4D4F544F = "MOTO" */
    uint16_t version;    /* Parameter schema version */
    uint16_t crc;        /* CRC16 of parameter block */
};

bool nvs_params_valid(void)
{
    struct nvs_param_header hdr;
    
    if (nvs_read(&hdr, sizeof(hdr)) != sizeof(hdr)) {
        return false;
    }
    
    if (hdr.magic != 0x4D4F544F) {
        LOG_WRN("NVS magic mismatch");
        return false;
    }
    
    /* Verify CRC of parameter block */
    uint16_t calc_crc = compute_nvs_crc();
    if (hdr.crc != calc_crc) {
        LOG_ERR("NVS CRC mismatch: stored=0x%04x, calc=0x%04x", 
                hdr.crc, calc_crc);
        return false;
    }
    
    return true;
}
```

---

## Implementation Roadmap

### Phase 1: Basic NVS Infrastructure
1. Add NVS subsystem to `prj.conf`
2. Create `motor_params_nvs.c/.h` module
3. Implement load/save for calibration data only
4. Update calibration state machines to persist results

### Phase 2: Parameter Override
1. Implement parameter validation framework
2. Add NVS override for PI gains, limits
3. Update shell commands for param management
4. Add factory reset command

### Phase 3: RLS Integration
1. Persist RLS learned parameters periodically
2. Load RLS parameters on startup if available
3. Add convergence-based save logic
4. Add staleness detection (re-learn if motor changed)

### Phase 4: Advanced Features
1. Parameter change logging/audit trail
2. Parameter export/import for multiple motors
3. Cloud backup/sync (if networked)
4. A/B parameter testing framework

---

## Storage Requirements

### Estimated NVS Usage

| Category | Parameters | Size per Parameter | Total |
|----------|------------|-------------------|-------|
| **Calibration** | 5 (offsets, Rs, Ls, encoder zero) | 4 bytes | 20 bytes |
| **PI Gains** | 4 (Kp/Ki d/q) | 4 bytes | 16 bytes |
| **Limits** | 3 (current, speed, voltage) | 4 bytes | 12 bytes |
| **RLS** | 4 (Ld, Lq, Rs, timestamp) | 4 bytes | 16 bytes |
| **Observer** | 2 (bandwidth, pole pairs) | 4 bytes | 8 bytes |
| **Metadata** | 1 header | 8 bytes | 8 bytes |
| **Total** | | | **~80 bytes** |

With overhead and wear-leveling, allocate **256 bytes minimum** or **1 KB recommended**.

---

## Conclusion (Updated with NVS)

**Adding NVS storage enhances your design significantly** ✅

### Revised Architecture:

1. **Kconfig** → Feature enables (compile-time)
2. **Devicetree** → Hardware defaults and safe factory values
3. **NVS** → Runtime overrides, calibration results, learned parameters

### Key Benefits:

- ✅ Field tuning without firmware updates
- ✅ Calibration persistence (offsets, Rs, Ls)
- ✅ RLS parameter learning with persistence
- ✅ User-specific operational limits
- ✅ Factory reset capability

### Design Principles:

1. **DT provides safe defaults** - always fall back if NVS corrupted
2. **NVS stores overrides only** - don't duplicate all DT params
3. **Validate all NVS writes** - prevent dangerous configurations
4. **Detect corruption** - CRC check, magic numbers, version control
5. **Rate-limit writes** - NVS has finite write cycles

### Action Items (Updated):

1. ✅ **Done**: Added `CONFIG_RLS_PARAMETER_ESTIMATION` to Kconfig
2. ⚠️ **Next**: Implement NVS parameter storage module
3. ⚠️ **Next**: Add parameter validation framework
4. ⚠️ **Next**: Update calibration states to persist to NVS
5. 🔮 **Future**: Implement shell commands for parameter management

**Your current architecture is well-designed!** ✅

- **Keep 95% of parameters in Devicetree** - they're hardware/motor-specific
- **Use Kconfig only for feature flags** - algorithm enables, debug options
- **Don't move tuning parameters to Kconfig** - they vary per motor/board

The main improvement would be adding more **feature flags** to Kconfig (like the RLS option you just added) to enable compile-time code elimination for unused features.

### Action Items

1. ✅ **Done**: Added `CONFIG_RLS_PARAMETER_ESTIMATION` to Kconfig
2. ⚠️ **Consider**: Add Kconfig options for other major features (thermal model, fault detection modes)
3. ✅ **Keep**: All motor/hardware parameters in devicetree
4. 🔮 **Future**: Consider NVS storage for runtime parameter persistence

