# Motor Control Shell Implementation Plan

## Overview

This document outlines the implementation plan for adding shell commands to the chopper motor control application. The shell interface will allow runtime interaction with the motor control system for parameter tuning, state control, and diagnostics.

## Architecture

### Design Pattern: Double Buffering

To avoid shared state issues between the shell thread and the ADC ISR (20kHz), we use a **double buffering approach**:

- **Active Buffer**: Used by ADC ISR for motor control
- **Shadow Buffer**: Modified by shell commands
- **Atomic Swap**: Shell triggers parameter update via flag, ISR swaps buffers at safe point

This eliminates the need for mutexes in the critical ISR path while ensuring thread-safe parameter updates.

```c
struct motor_parameters_buffer {
    float Id_setpoint_A;
    float Iq_setpoint_A;
    float maxVsMag_pu;
    struct pi_controller pi_Id;
    struct pi_controller pi_Iq;
    // ... other runtime-modifiable parameters
};

struct motor_parameters {
    // Double buffered parameters
    struct motor_parameters_buffer buffers[2];
    volatile uint8_t active_buffer;      // 0 or 1 (read by ISR)
    volatile bool update_pending;        // Set by shell, cleared by ISR
    
    /* Note: On single-core Cortex-M, volatile is sufficient for thread-to-ISR
     * communication. Hardware guarantees atomic read/write of aligned 8-bit
     * and 32-bit values. Volatile prevents compiler reordering. */
    
    // Non-buffered state (read-only from shell)
    float Id_ref_A;
    float Iq_ref_A;
    float Vd_V;
    float Vq_V;
    // ... measurements and state
};
```

**Update Sequence:**
1. Shell command modifies shadow buffer (inactive buffer)
2. Shell sets `update_pending` flag
3. At next ADC ISR entry (safe point), ISR checks flag
4. ISR swaps `active_buffer` index (single byte write is atomic on Cortex-M)
5. ISR clears `update_pending` flag
6. ISR uses new active buffer for control

## Command Structure

```
motor
├── state       - State machine control
│   ├── start       - Transition to ONLINE state
│   ├── stop        - Transition to IDLE state  
│   ├── calibrate   - Run full calibration sequence (OFFSET→ROVERL→RS_EST→ALIGN)
│   ├── align       - Force ALIGN state
│   ├── error       - Force ERROR state (for testing)
│   ├── clear       - Clear error and return to IDLE
│   └── status      - Print current state and error code
├── params      - Parameter management
│   ├── get <name>  - Get parameter value
│   ├── set <name> <value> - Set parameter value
│   └── list        - List all parameters with current values
├── current     - Current control (similar to spinner's cloop)
│   ├── id <amps>   - Set Id setpoint
│   ├── iq <amps>   - Set Iq setpoint
│   ├── dq <id> <iq> - Set both Id and Iq setpoints
│   └── get         - Print Id/Iq setpoints and measured values
├── pi          - PI controller tuning
│   ├── get <id|iq> - Get PI gains (Kp, Ki)
│   ├── set <id|iq> <kp> <ki> - Set PI gains directly
│   └── bandwidth <id|iq> <hz> - Calculate and set gains from bandwidth
└── info        - System information
    ├── config      - Print configuration summary (all DT params)
    ├── measured    - Print measured parameters (R, L, offsets)
    ├── live        - Print live telemetry (currents, voltages, angle, speed)
    └── stats       - Print performance statistics (ISR timing, faults)
```

## Parameters for Get/Set

### Read-Only Parameters (Telemetry)
These are accessed directly from active buffer (safe read):

- `id_ref` - D-axis current reference (A) - actual PI input
- `iq_ref` - Q-axis current reference (A) - actual PI input
- `id_meas` - Measured D-axis current (A)
- `iq_meas` - Measured Q-axis current (A)
- `angle_mech` - Mechanical angle (deg)
- `angle_elec` - Electrical angle (deg)
- `speed` - Mechanical speed (Hz)
- `vbus` - Bus voltage (V)
- `vd` - D-axis voltage output (V)
- `vq` - Q-axis voltage output (V)
- `state` - Current state machine state (enum as string)
- `error` - Error code (enum as string)
- `ia_offset` - Phase A current offset (A)
- `ib_offset` - Phase B current offset (A)
- `rs_measured` - Measured stator resistance (Ω)
- `l_measured` - Measured inductance (H)
- `roverl_measured` - Measured R/L time constant (rad/s)

### Read-Write Parameters (Double Buffered)
These are modified in shadow buffer, swapped by ISR:

- `id_setpoint` - D-axis current setpoint (A) - shell sets this
- `iq_setpoint` - Q-axis current setpoint (A) - shell sets this
- `max_current` - Maximum current limit (A) - used by trajectory generator
- `max_vsmag` - Maximum voltage magnitude (per-unit) - PI output limit
- `pi_id_kp` - Id PI proportional gain
- `pi_id_ki` - Id PI integral gain
- `pi_iq_kp` - Iq PI proportional gain
- `pi_iq_ki` - Iq PI integral gain

## Configuration Summary Output

`motor info config` should print all devicetree-configured parameters:

```
Motor Configuration:
  Pole Pairs:          4
  Max Current:         2.00 A
  Rated Voltage:       24.0 V
  Control Frequency:   20000 Hz
  PWM Frequency:       20000 Hz
  
Angle Observer:
  Bandwidth:           100.0 Hz
  
Calibration Sequence:
  Offset Measurement:
    Duration:          1000 samples (50 ms)
  
  RoverL Measurement:
    Current:           0.200 A
    Frequency:         100.0 Hz
    Duration:          1.000 s
    Settling Time:     50 ms
  
  Rs Estimation:
    Current:           1.000 A
    Rampup Time:       1.000 s
    Coarse Filter BW:  1.0 Hz
    Fine Filter BW:    0.1 Hz
  
  Alignment:
    Current:           0.500 A
    Duration:          2.000 s

PI Controllers:
  Id Controller:
    Bandwidth:         500.0 Hz
    Kp:                0.xxxx (calculated from L)
    Ki:                xxxx.x (calculated from R)
  
  Iq Controller:
    Bandwidth:         500.0 Hz
    Kp:                0.xxxx
    Ki:                xxxx.x

Fault Detection:
  Encoder Fault:       100 samples (5 ms @ 20kHz)
  Overcurrent:         3.60 A (120% of max)
  Overvoltage:         28.0 V
  Vbus Regen Limit:    26.0 V
  
Dynamic Braking:
  Enabled:             Yes
  Voltage Margin:      2.0 V
```

## Measured Parameters Output

`motor info measured` should print calibration results:

```
Measured Parameters:
  Current Offsets:
    Ia offset:         0.0123 A
    Ib offset:        -0.0087 A
  
  Stator Resistance:   0.4250 Ω
  
  Inductance:          125.3 µH
  
  R/L Time Constant:   3392 rad/s (τ = 295 µs)
  
  Alignment Offset:    -45.2 deg
```

## Live Telemetry Output

`motor info live` should print real-time data (snapshot from active buffer):

```
Live Telemetry:
  State:               ONLINE
  Error Code:          NONE
  
  Currents:
    Id setpoint:       0.00 A
    Iq setpoint:       0.50 A
    Id reference:      0.00 A (after trajectory)
    Iq reference:      0.50 A
    Id measured:       0.02 A
    Iq measured:       0.48 A
    Ia (phase):        0.34 A
    Ib (phase):       -0.34 A
  
  Voltages:
    Vbus:              24.3 V
    Vd output:         0.12 V
    Vq output:         3.45 V
    Va (phase):        2.98 V
    Vb (phase):       -2.98 V
  
  Position:
    Mech angle:        123.4 deg
    Elec angle:        45.6 deg (predicted)
    Speed:             10.5 Hz (315 RPM)
  
  Encoder:
    Fault counter:     0
    Last read:         OK
```

## Implementation Files

### New Files

1. **`/workspace/chopper/app/src/shell_commands.c`**
   - All shell command handler implementations
   - Parameter access functions (get/set with double buffering)
   - State machine control wrappers

2. **`/workspace/chopper/app/include/shell_commands.h`**
   - Public API for accessing motor parameters from shell
   - Function declarations for state control
   - Buffer swap interface

### Modified Files

1. **`/workspace/chopper/app/CMakeLists.txt`**
   ```cmake
   target_sources(app PRIVATE
       src/main.c
       src/shell_commands.c  # Add this
   )
   ```

2. **`/workspace/chopper/app/prj.conf`**
   ```kconfig
   # Shell subsystem
   CONFIG_SHELL=y
   CONFIG_SHELL_BACKEND_SERIAL=y
   CONFIG_SHELL_PROMPT_UART="motor:~$ "
   CONFIG_SHELL_CMD_BUFF_SIZE=256
   CONFIG_SHELL_PRINTF_BUFF_SIZE=256
   CONFIG_SHELL_HELP=y
   CONFIG_SHELL_HISTORY=y
   CONFIG_SHELL_HISTORY_BUFFER=128
   ```

3. **`/workspace/chopper/app/src/main.c`**
   - Convert single `motor_parameters` to double-buffered structure
   - Add buffer swap logic at top of `adc_callback()`
   - Add accessor functions for shell (declared in shell_commands.h)
   - Make `motor_params` accessible via getter function

## Shell Command Examples

### State Control
```c
static int cmd_motor_state_start(const struct shell *shell, size_t argc, char **argv)
{
    enum motor_state current = motor_get_current_state();
    
    if (current != MOTOR_STATE_IDLE) {
        shell_error(shell, "Motor must be in IDLE state. Current: %s",
                    motor_state_to_string(current));
        return -EINVAL;
    }
    
    motor_request_state(MOTOR_STATE_ONLINE);
    shell_print(shell, "Transitioning to ONLINE state");
    
    return 0;
}

static int cmd_motor_state_calibrate(const struct shell *shell, size_t argc, char **argv)
{
    motor_request_state(MOTOR_STATE_OFFSET_MEAS);
    shell_print(shell, "Starting calibration sequence");
    shell_print(shell, "Sequence: OFFSET → ROVERL → RS_EST → ALIGN → IDLE");
    
    return 0;
}

static int cmd_motor_state_status(const struct shell *shell, size_t argc, char **argv)
{
    enum motor_state state = motor_get_current_state();
    enum motor_error error = motor_get_error_code();
    
    shell_print(shell, "Current State: %s", motor_state_to_string(state));
    shell_print(shell, "Error Code:    %s", motor_error_to_string(error));
    
    return 0;
}
```

### Parameter Get/Set
```c
static int cmd_motor_params_get(const struct shell *shell, size_t argc, char **argv)
{
    if (argc != 2) {
        shell_help(shell);
        return -EINVAL;
    }
    
    const char *param = argv[1];
    float value;
    
    if (motor_param_get(param, &value) != 0) {
        shell_error(shell, "Unknown parameter: %s", param);
        shell_print(shell, "Use 'motor params list' to see available parameters");
        return -EINVAL;
    }
    
    shell_print(shell, "%s = %.4f", param, (double)value);
    return 0;
}

static int cmd_motor_params_set(const struct shell *shell, size_t argc, char **argv)
{
    if (argc != 3) {
        shell_help(shell);
        return -EINVAL;
    }
    
    const char *param = argv[1];
    float value = strtof(argv[2], NULL);
    
    if (motor_param_set(param, value) != 0) {
        shell_error(shell, "Failed to set parameter: %s", param);
        return -EINVAL;
    }
    
    shell_print(shell, "%s set to %.4f", param, (double)value);
    return 0;
}
```

### Current Control
```c
static int cmd_motor_current_dq(const struct shell *shell, size_t argc, char **argv)
{
    if (argc != 3) {
        shell_help(shell);
        return -EINVAL;
    }
    
    float id = strtof(argv[1], NULL);
    float iq = strtof(argv[2], NULL);
    
    // Validate limits
    float max_current = motor_get_max_current();
    float magnitude = sqrtf(id*id + iq*iq);
    
    if (magnitude > max_current) {
        shell_error(shell, "Current magnitude %.2f A exceeds limit %.2f A",
                    (double)magnitude, (double)max_current);
        return -EINVAL;
    }
    
    motor_set_current_setpoints(id, iq);
    shell_print(shell, "Current setpoints: Id=%.2f A, Iq=%.2f A",
                (double)id, (double)iq);
    
    return 0;
}

static int cmd_motor_current_get(const struct shell *shell, size_t argc, char **argv)
{
    struct motor_current_state curr;
    motor_get_current_state(&curr);
    
    shell_print(shell, "Setpoints:  Id=%.3f A  Iq=%.3f A",
                (double)curr.id_setpoint, (double)curr.iq_setpoint);
    shell_print(shell, "References: Id=%.3f A  Iq=%.3f A",
                (double)curr.id_ref, (double)curr.iq_ref);
    shell_print(shell, "Measured:   Id=%.3f A  Iq=%.3f A",
                (double)curr.id_meas, (double)curr.iq_meas);
    
    return 0;
}
```

### PI Controller Tuning
```c
static int cmd_motor_pi_bandwidth(const struct shell *shell, size_t argc, char **argv)
{
    if (argc != 3) {
        shell_help(shell);
        return -EINVAL;
    }
    
    const char *controller = argv[1];  // "id" or "iq"
    float bandwidth_hz = strtof(argv[2], NULL);
    
    if (strcmp(controller, "id") != 0 && strcmp(controller, "iq") != 0) {
        shell_error(shell, "Controller must be 'id' or 'iq'");
        return -EINVAL;
    }
    
    // Calculate gains from bandwidth and motor parameters
    float L = motor_get_inductance();
    float R = motor_get_resistance();
    float kp = 2.0f * PI_F32 * bandwidth_hz * L;
    float ki = 2.0f * PI_F32 * bandwidth_hz * R;
    
    motor_set_pi_gains(controller, kp, ki);
    
    shell_print(shell, "PI_%s gains set for %.1f Hz bandwidth:",
                controller, (double)bandwidth_hz);
    shell_print(shell, "  Kp = %.4f", (double)kp);
    shell_print(shell, "  Ki = %.1f", (double)ki);
    
    return 0;
}

static int cmd_motor_pi_get(const struct shell *shell, size_t argc, char **argv)
{
    if (argc != 2) {
        shell_help(shell);
        return -EINVAL;
    }
    
    const char *controller = argv[1];  // "id" or "iq"
    
    if (strcmp(controller, "id") != 0 && strcmp(controller, "iq") != 0) {
        shell_error(shell, "Controller must be 'id' or 'iq'");
        return -EINVAL;
    }
    
    float kp, ki;
    motor_get_pi_gains(controller, &kp, &ki);
    
    shell_print(shell, "PI_%s gains:", controller);
    shell_print(shell, "  Kp = %.4f", (double)kp);
    shell_print(shell, "  Ki = %.1f", (double)ki);
    
    return 0;
}

static int cmd_motor_pi_set(const struct shell *shell, size_t argc, char **argv)
{
    if (argc != 4) {
        shell_help(shell);
        return -EINVAL;
    }
    
    const char *controller = argv[1];  // "id" or "iq"
    float kp = strtof(argv[2], NULL);
    float ki = strtof(argv[3], NULL);
    
    if (strcmp(controller, "id") != 0 && strcmp(controller, "iq") != 0) {
        shell_error(shell, "Controller must be 'id' or 'iq'");
        return -EINVAL;
    }
    
    motor_set_pi_gains(controller, kp, ki);
    
    shell_print(shell, "PI_%s gains set:", controller);
    shell_print(shell, "  Kp = %.4f", (double)kp);
    shell_print(shell, "  Ki = %.1f", (double)ki);
    
    return 0;
}

static int cmd_motor_current_id(const struct shell *shell, size_t argc, char **argv)
{
    if (argc != 2) {
        shell_help(shell);
        return -EINVAL;
    }
    
    float id = strtof(argv[1], NULL);
    float iq;
    
    // Get current Iq setpoint to preserve it
    motor_param_get("iq_setpoint", &iq);
    
    // Validate current magnitude
    float max_current = motor_get_max_current();
    float magnitude = sqrtf(id*id + iq*iq);
    
    if (magnitude > max_current) {
        shell_error(shell, "Current magnitude %.2f A exceeds limit %.2f A",
                    (double)magnitude, (double)max_current);
        return -EINVAL;
    }
    
    motor_param_set("id_setpoint", id);
    shell_print(shell, "Id setpoint set to %.2f A", (double)id);
    
    return 0;
}

static int cmd_motor_current_iq(const struct shell *shell, size_t argc, char **argv)
{
    if (argc != 2) {
        shell_help(shell);
        return -EINVAL;
    }
    
    float iq = strtof(argv[1], NULL);
    float id;
    
    // Get current Id setpoint to preserve it
    motor_param_get("id_setpoint", &id);
    
    // Validate current magnitude
    float max_current = motor_get_max_current();
    float magnitude = sqrtf(id*id + iq*iq);
    
    if (magnitude > max_current) {
        shell_error(shell, "Current magnitude %.2f A exceeds limit %.2f A",
                    (double)magnitude, (double)max_current);
        return -EINVAL;
    }
    
    motor_param_set("iq_setpoint", iq);
    shell_print(shell, "Iq setpoint set to %.2f A", (double)iq);
    
    return 0;
}

static int cmd_motor_params_list(const struct shell *shell, size_t argc, char **argv)
{
    shell_print(shell, "Read-Write Parameters (Double Buffered):");
    shell_print(shell, "=========================================");
    
    // Iterate through descriptor table
    for (size_t i = 0; i < ARRAY_SIZE(motor_param_table); i++) {
        float value;
        if (motor_param_get(motor_param_table[i].name, &value) == 0) {
            shell_print(shell, "  %-20s %.4f", 
                       motor_param_table[i].name, (double)value);
        }
    }
    
    shell_print(shell, "");
    shell_print(shell, "Read-Only Parameters (Telemetry):");
    shell_print(shell, "==================================");
    
    // Print telemetry values
    struct motor_current_state curr;
    motor_get_current_state(&curr);
    
    shell_print(shell, "  %-20s %.3f A", "id_ref", (double)curr.id_ref);
    shell_print(shell, "  %-20s %.3f A", "iq_ref", (double)curr.iq_ref);
    shell_print(shell, "  %-20s %.3f A", "id_meas", (double)curr.id_meas);
    shell_print(shell, "  %-20s %.3f A", "iq_meas", (double)curr.iq_meas);
    shell_print(shell, "  %-20s %.1f deg", "angle_mech", (double)motor_get_angle_mech());
    shell_print(shell, "  %-20s %.1f deg", "angle_elec", (double)motor_get_angle_elec());
    shell_print(shell, "  %-20s %.2f Hz", "speed", (double)motor_get_speed());
    shell_print(shell, "  %-20s %.1f V", "vbus", (double)motor_get_vbus());
    shell_print(shell, "  %-20s %s", "state", motor_state_to_string(motor_get_current_state()));
    shell_print(shell, "  %-20s %s", "error", motor_error_to_string(motor_get_error_code()));
    
    return 0;
}

static int cmd_motor_state_stop(const struct shell *shell, size_t argc, char **argv)
{
    enum motor_state current = motor_get_current_state();
    
    if (current != MOTOR_STATE_ONLINE) {
        shell_warn(shell, "Motor is not running. Current state: %s",
                   motor_state_to_string(current));
    }
    
    motor_request_state(MOTOR_STATE_IDLE);
    shell_print(shell, "Transitioning to IDLE state");
    
    return 0;
}

static int cmd_motor_state_align(const struct shell *shell, size_t argc, char **argv)
{
    motor_request_state(MOTOR_STATE_ALIGN);
    shell_print(shell, "Starting alignment sequence");
    
    return 0;
}

static int cmd_motor_state_error(const struct shell *shell, size_t argc, char **argv)
{
    motor_request_state(MOTOR_STATE_ERROR);
    shell_print(shell, "Forcing ERROR state (for testing)");
    
    return 0;
}

static int cmd_motor_state_clear(const struct shell *shell, size_t argc, char **argv)
{
    enum motor_state current = motor_get_current_state();
    
    if (current != MOTOR_STATE_ERROR) {
        shell_warn(shell, "Motor is not in ERROR state. Current: %s",
                   motor_state_to_string(current));
        return 0;
    }
    
    motor_clear_error();
    shell_print(shell, "Error cleared, returning to IDLE state");
    
    return 0;
}

static int cmd_motor_info_config(const struct shell *shell, size_t argc, char **argv)
{
    const struct motor_config *cfg = motor_get_config();
    
    shell_print(shell, "Motor Configuration:");
    shell_print(shell, "  Pole Pairs:          %u", cfg->pole_pairs);
    shell_print(shell, "  Max Current:         %.2f A", (double)cfg->max_current_A);
    shell_print(shell, "  Rated Voltage:       %.1f V", (double)cfg->rated_voltage_V);
    shell_print(shell, "  Control Frequency:   %u Hz", cfg->control_freq_hz);
    shell_print(shell, "  PWM Frequency:       %u Hz", cfg->pwm_freq_hz);
    shell_print(shell, "");
    
    shell_print(shell, "Angle Observer:");
    shell_print(shell, "  Bandwidth:           %.1f Hz", (double)cfg->observer_bw_hz);
    shell_print(shell, "");
    
    shell_print(shell, "Calibration Sequence:");
    shell_print(shell, "  Offset Measurement:");
    shell_print(shell, "    Duration:          %u samples (%.1f ms)", 
               cfg->offset_meas_samples,
               (double)(cfg->offset_meas_samples * 1000.0f / cfg->control_freq_hz));
    shell_print(shell, "");
    
    shell_print(shell, "  RoverL Measurement:");
    shell_print(shell, "    Current:           %.3f A", (double)cfg->roverl_current_A);
    shell_print(shell, "    Frequency:         %.1f Hz", (double)cfg->roverl_freq_hz);
    shell_print(shell, "    Duration:          %.3f s", (double)cfg->roverl_duration_s);
    shell_print(shell, "    Settling Time:     %.1f ms", (double)(cfg->roverl_settling_ms));
    shell_print(shell, "");
    
    shell_print(shell, "  Rs Estimation:");
    shell_print(shell, "    Current:           %.3f A", (double)cfg->rs_est_current_A);
    shell_print(shell, "    Rampup Time:       %.3f s", (double)cfg->rs_est_rampup_s);
    shell_print(shell, "    Coarse Filter BW:  %.1f Hz", (double)cfg->rs_est_coarse_bw_hz);
    shell_print(shell, "    Fine Filter BW:    %.1f Hz", (double)cfg->rs_est_fine_bw_hz);
    shell_print(shell, "");
    
    shell_print(shell, "  Alignment:");
    shell_print(shell, "    Current:           %.3f A", (double)cfg->align_current_A);
    shell_print(shell, "    Duration:          %.3f s", (double)cfg->align_duration_s);
    shell_print(shell, "");
    
    shell_print(shell, "PI Controllers:");
    shell_print(shell, "  Id Controller:");
    shell_print(shell, "    Bandwidth:         %.1f Hz", (double)cfg->pi_id_bw_hz);
    shell_print(shell, "");
    shell_print(shell, "  Iq Controller:");
    shell_print(shell, "    Bandwidth:         %.1f Hz", (double)cfg->pi_iq_bw_hz);
    shell_print(shell, "");
    
    shell_print(shell, "Fault Detection:");
    shell_print(shell, "  Encoder Fault:       %u samples (%.1f ms @ %u Hz)", 
               cfg->encoder_fault_samples,
               (double)(cfg->encoder_fault_samples * 1000.0f / cfg->control_freq_hz),
               cfg->control_freq_hz);
    shell_print(shell, "  Overcurrent:         %.2f A (%.0f%% of max)", 
               (double)cfg->overcurrent_limit_A,
               (double)(100.0f * cfg->overcurrent_limit_A / cfg->max_current_A));
    shell_print(shell, "  Overvoltage:         %.1f V", (double)cfg->overvoltage_limit_V);
    shell_print(shell, "  Vbus Regen Limit:    %.1f V", (double)cfg->vbus_regen_limit_V);
    shell_print(shell, "");
    
    shell_print(shell, "Dynamic Braking:");
    shell_print(shell, "  Enabled:             %s", cfg->dynamic_braking_en ? "Yes" : "No");
    shell_print(shell, "  Voltage Margin:      %.1f V", (double)cfg->dynamic_braking_margin_V);
    
    return 0;
}

static int cmd_motor_info_measured(const struct shell *shell, size_t argc, char **argv)
{
    const struct motor_measured_params *meas = motor_get_measured_params();
    
    shell_print(shell, "Measured Parameters:");
    shell_print(shell, "  Current Offsets:");
    shell_print(shell, "    Ia offset:         %.4f A", (double)meas->ia_offset_A);
    shell_print(shell, "    Ib offset:         %.4f A", (double)meas->ib_offset_A);
    shell_print(shell, "");
    shell_print(shell, "  Stator Resistance:   %.4f Ω", (double)meas->rs_ohm);
    shell_print(shell, "");
    shell_print(shell, "  Inductance:          %.1f µH", (double)(meas->l_H * 1e6f));
    shell_print(shell, "");
    shell_print(shell, "  R/L Time Constant:   %.0f rad/s (τ = %.0f µs)", 
               (double)meas->roverl_radps,
               (double)(1e6f / meas->roverl_radps));
    shell_print(shell, "");
    shell_print(shell, "  Alignment Offset:    %.1f deg", (double)meas->align_offset_deg);
    
    return 0;
}

static int cmd_motor_info_live(const struct shell *shell, size_t argc, char **argv)
{
    const struct motor_live_data *live = motor_get_live_data();
    
    shell_print(shell, "Live Telemetry:");
    shell_print(shell, "  State:               %s", motor_state_to_string(live->state));
    shell_print(shell, "  Error Code:          %s", motor_error_to_string(live->error));
    shell_print(shell, "");
    
    shell_print(shell, "  Currents:");
    shell_print(shell, "    Id setpoint:       %.2f A", (double)live->id_setpoint_A);
    shell_print(shell, "    Iq setpoint:       %.2f A", (double)live->iq_setpoint_A);
    shell_print(shell, "    Id reference:      %.2f A (after trajectory)", (double)live->id_ref_A);
    shell_print(shell, "    Iq reference:      %.2f A", (double)live->iq_ref_A);
    shell_print(shell, "    Id measured:       %.2f A", (double)live->id_meas_A);
    shell_print(shell, "    Iq measured:       %.2f A", (double)live->iq_meas_A);
    shell_print(shell, "    Ia (phase):        %.2f A", (double)live->ia_A);
    shell_print(shell, "    Ib (phase):        %.2f A", (double)live->ib_A);
    shell_print(shell, "");
    
    shell_print(shell, "  Voltages:");
    shell_print(shell, "    Vbus:              %.1f V", (double)live->vbus_V);
    shell_print(shell, "    Vd output:         %.2f V", (double)live->vd_V);
    shell_print(shell, "    Vq output:         %.2f V", (double)live->vq_V);
    shell_print(shell, "    Va (phase):        %.2f V", (double)live->va_V);
    shell_print(shell, "    Vb (phase):        %.2f V", (double)live->vb_V);
    shell_print(shell, "");
    
    shell_print(shell, "  Position:");
    shell_print(shell, "    Mech angle:        %.1f deg", (double)live->angle_mech_deg);
    shell_print(shell, "    Elec angle:        %.1f deg (predicted)", (double)live->angle_elec_deg);
    shell_print(shell, "    Speed:             %.2f Hz (%.0f RPM)", 
               (double)live->speed_hz,
               (double)(live->speed_hz * 60.0f));
    shell_print(shell, "");
    
    shell_print(shell, "  Encoder:");
    shell_print(shell, "    Fault counter:     %u", live->encoder_fault_count);
    shell_print(shell, "    Last read:         %s", live->encoder_ok ? "OK" : "FAULT");
    
    return 0;
}

static int cmd_motor_info_stats(const struct shell *shell, size_t argc, char **argv)
{
    const struct motor_stats *stats = motor_get_stats();
    
    shell_print(shell, "Performance Statistics:");
    shell_print(shell, "  ISR Execution:");
    shell_print(shell, "    Total count:       %u", stats->isr_count);
    shell_print(shell, "    Max cycles:        %u (%.1f µs @ %u MHz)", 
               stats->isr_max_cycles,
               (double)(stats->isr_max_cycles * 1e6f / sys_clock_hw_cycles_per_sec()),
               (unsigned)(sys_clock_hw_cycles_per_sec() / 1000000));
    shell_print(shell, "    Avg cycles:        %u (%.1f µs)", 
               stats->isr_avg_cycles,
               (double)(stats->isr_avg_cycles * 1e6f / sys_clock_hw_cycles_per_sec()));
    shell_print(shell, "");
    
    shell_print(shell, "  Buffer Management:");
    shell_print(shell, "    Swaps:             %u", stats->buffer_swap_count);
    shell_print(shell, "");
    
    shell_print(shell, "  Fault Counters:");
    shell_print(shell, "    Encoder faults:    %u", stats->encoder_fault_total);
    shell_print(shell, "    Overcurrent:       %u", stats->overcurrent_total);
    shell_print(shell, "    Overvoltage:       %u", stats->overvoltage_total);
    shell_print(shell, "    Undervoltage:      %u", stats->undervoltage_total);
    
    return 0;
}
```

## Command Registration

```c
/* State control subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(
    sub_motor_state,
    SHELL_CMD(start, NULL, "Start motor (IDLE→ONLINE)", cmd_motor_state_start),
    SHELL_CMD(stop, NULL, "Stop motor (ONLINE→IDLE)", cmd_motor_state_stop),
    SHELL_CMD(calibrate, NULL, "Run calibration sequence", cmd_motor_state_calibrate),
    SHELL_CMD(align, NULL, "Run alignment", cmd_motor_state_align),
    SHELL_CMD(error, NULL, "Force ERROR state", cmd_motor_state_error),
    SHELL_CMD(clear, NULL, "Clear error", cmd_motor_state_clear),
    SHELL_CMD(status, NULL, "Show current state", cmd_motor_state_status),
    SHELL_SUBCMD_SET_END
);

/* Parameter management subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(
    sub_motor_params,
    SHELL_CMD_ARG(get, NULL, "Get parameter value\nUsage: params get <name>",
                  cmd_motor_params_get, 2, 0),
    SHELL_CMD_ARG(set, NULL, "Set parameter value\nUsage: params set <name> <value>",
                  cmd_motor_params_set, 3, 0),
    SHELL_CMD(list, NULL, "List all parameters", cmd_motor_params_list),
    SHELL_SUBCMD_SET_END
);

/* Current control subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(
    sub_motor_current,
    SHELL_CMD_ARG(id, NULL, "Set Id setpoint\nUsage: current id <amps>",
                  cmd_motor_current_id, 2, 0),
    SHELL_CMD_ARG(iq, NULL, "Set Iq setpoint\nUsage: current iq <amps>",
                  cmd_motor_current_iq, 2, 0),
    SHELL_CMD_ARG(dq, NULL, "Set Id and Iq setpoints\nUsage: current dq <id> <iq>",
                  cmd_motor_current_dq, 3, 0),
    SHELL_CMD(get, NULL, "Show current values", cmd_motor_current_get),
    SHELL_SUBCMD_SET_END
);

/* PI controller tuning subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(
    sub_motor_pi,
    SHELL_CMD_ARG(get, NULL, "Get PI gains\nUsage: pi get <id|iq>",
                  cmd_motor_pi_get, 2, 0),
    SHELL_CMD_ARG(set, NULL, "Set PI gains\nUsage: pi set <id|iq> <kp> <ki>",
                  cmd_motor_pi_set, 4, 0),
    SHELL_CMD_ARG(bandwidth, NULL, "Set PI bandwidth\nUsage: pi bandwidth <id|iq> <hz>",
                  cmd_motor_pi_bandwidth, 3, 0),
    SHELL_SUBCMD_SET_END
);

/* System info subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(
    sub_motor_info,
    SHELL_CMD(config, NULL, "Show configuration", cmd_motor_info_config),
    SHELL_CMD(measured, NULL, "Show measured parameters", cmd_motor_info_measured),
    SHELL_CMD(live, NULL, "Show live telemetry", cmd_motor_info_live),
    SHELL_CMD(stats, NULL, "Show performance stats", cmd_motor_info_stats),
    SHELL_SUBCMD_SET_END
);

/* Top-level motor command */
SHELL_STATIC_SUBCMD_SET_CREATE(
    sub_motor,
    SHELL_CMD(state, &sub_motor_state, "State machine control", NULL),
    SHELL_CMD(params, &sub_motor_params, "Parameter management", NULL),
    SHELL_CMD(current, &sub_motor_current, "Current control", NULL),
    SHELL_CMD(pi, &sub_motor_pi, "PI controller tuning", NULL),
    SHELL_CMD(info, &sub_motor_info, "System information", NULL),
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(motor, &sub_motor, "Motor Control Commands", NULL);
```

## Double Buffer Implementation Details

### Buffer Structure in main.c

```c
/* Parameters that can be modified at runtime via shell */
struct motor_params_buffer {
    float Id_setpoint_A;
    float Iq_setpoint_A;
    float maxVsMag_pu;
    struct pi_controller pi_Id;
    struct pi_controller pi_Iq;
};

struct motor_parameters {
    /* Double-buffered runtime parameters */
    struct motor_params_buffer buffers[2];
    volatile uint8_t active_buffer;      // Index 0 or 1
    volatile bool update_pending;        // Flag for buffer swap
    
    /* State machine state */
    struct smf_ctx ctx;
    enum motor_error error_code;
    uint32_t state_counter;
    
    /* Non-buffered measurements (written by ISR, read by shell) */
    float Ia_offset;
    float Ib_offset;
    float Id_ref_A;              // After trajectory
    float Iq_ref_A;              // After trajectory
    float Vd_V;
    float Vq_V;
    float Rs_measured;
    float L_measured;
    float RoverL_measured;
    
    /* Observer state */
    struct angle_observer observer;
    
    /* Filters and trajectories (ISR only) */
    struct filter_fo filter_Ia;
    struct filter_fo filter_Ib;
    struct filter_fo filter_rs_est_V;
    struct filter_fo filter_rs_est_I;
    struct traj traj_Id;
    
    /* RoverL measurement accumulators */
    float roverl_sum_Vd_Id;
    float roverl_sum_Vq_Id;
    float roverl_sum_Id2;
    float roverl_phase_deg;
    
    /* Encoder fault tracking */
    uint32_t encoder_fault_counter;
};
```

### Buffer Swap in ADC ISR

```c
static void adc_callback(const struct device *dev, const q31_t *values,
                        uint8_t count, void *user_data)
{
    gpio_pin_set_dt(&trig, 1);
    
    struct motor_parameters *params = (struct motor_parameters *)user_data;
    
    /* Check for pending parameter update */
    if (params->update_pending) {
        params->update_pending = false;
        /* Swap active buffer index (single byte write is atomic on Cortex-M) */
        params->active_buffer = 1 - params->active_buffer;
    }
    
    /* Get pointer to active buffer */
    uint8_t idx = params->active_buffer;
    struct motor_params_buffer *active = &params->buffers[idx];
    
    /* Use active buffer for control */
    float Id_setpoint_A = active->Id_setpoint_A;
    float Iq_setpoint_A = active->Iq_setpoint_A;
    // ... rest of ISR uses active buffer
    
    /* ... existing ISR code ... */
    
    gpio_pin_set_dt(&trig, 0);
}
```

### Shell Parameter Update

#### Option 1: Offset-Based Access (Recommended - Zephyr Idiomatic)

**Advantages:**
- Zero lookup overhead
- Direct memory access
- Smallest code size
- Perfect for binary protocols
- **Zephyr-idiomatic**: Uses same pattern as JSON descriptors, Settings subsystem
- Leverage existing macros like `offsetof()` and `CONTAINER_OF()`

```c
/* Parameter metadata table - similar to Zephyr's json_obj_descr pattern */
struct motor_param_descr {
    const char *name;           // For shell display
    uint16_t offset;            // Offset into motor_params_buffer
    uint8_t size;               // sizeof(float) or sizeof(struct)
    uint8_t type;               // PARAM_TYPE_FLOAT, PARAM_TYPE_PI, etc
};

enum motor_param_type {
    MOTOR_PARAM_TYPE_FLOAT,
    MOTOR_PARAM_TYPE_PI_CONTROLLER,
};

/* Helper macro - similar to JSON_OBJ_DESCR_PRIM */
#define MOTOR_PARAM_DESCR_FLOAT(field_name_) \
    { \
        .name = #field_name_, \
        .offset = offsetof(struct motor_params_buffer, field_name_), \
        .size = sizeof(float), \
        .type = MOTOR_PARAM_TYPE_FLOAT, \
    }

/* Parameter descriptor table */
static const struct motor_param_descr motor_param_table[] = {
    MOTOR_PARAM_DESCR_FLOAT(Id_setpoint_A),
    MOTOR_PARAM_DESCR_FLOAT(Iq_setpoint_A),
    MOTOR_PARAM_DESCR_FLOAT(maxVsMag_pu),
    /* PI controllers would use a different macro */
};

/* Direct offset-based access (fastest) */
int motor_param_set_by_offset(uint16_t offset, float value)
{
    struct motor_parameters *params = motor_get_params();
    
    uint8_t active_idx = params->active_buffer;
    uint8_t shadow_idx = 1 - active_idx;
    struct motor_params_buffer *shadow = &params->buffers[shadow_idx];
    
    /* Direct memory write via offset */
    *(float *)((uint8_t *)shadow + offset) = value;
    
    params->update_pending = true;
    return 0;
}

/* String-based wrapper for shell */
int motor_param_set(const char *name, float value)
{
    /* Linear search through descriptor table */
    for (size_t i = 0; i < ARRAY_SIZE(motor_param_table); i++) {
        if (strcmp(name, motor_param_table[i].name) == 0) {
            return motor_param_set_by_offset(motor_param_table[i].offset, value);
        }
    }
    return -EINVAL;  // Unknown parameter
}

/* For binary protocols (MODBUS, CAN) - use offset directly */
int motor_param_set_by_index(uint8_t index, float value)
{
    if (index >= ARRAY_SIZE(motor_param_table)) {
        return -EINVAL;
    }
    return motor_param_set_by_offset(motor_param_table[index].offset, value);
}

/* Direct offset-based read (fastest) */
int motor_param_get_by_offset(uint16_t offset, float *value)
{
    struct motor_parameters *params = motor_get_params();
    
    uint8_t active_idx = params->active_buffer;
    struct motor_params_buffer *active = &params->buffers[active_idx];
    
    /* Direct memory read via offset */
    *value = *(float *)((uint8_t *)active + offset);
    
    return 0;
}

/* String-based wrapper for shell */
int motor_param_get(const char *name, float *value)
{
    /* Linear search through descriptor table */
    for (size_t i = 0; i < ARRAY_SIZE(motor_param_table); i++) {
        if (strcmp(name, motor_param_table[i].name) == 0) {
            return motor_param_get_by_offset(motor_param_table[i].offset, value);
        }
    }
    return -EINVAL;  // Unknown parameter
}

/* For binary protocols (MODBUS, CAN) - use index directly */
int motor_param_get_by_index(uint8_t index, float *value)
{
    if (index >= ARRAY_SIZE(motor_param_table)) {
        return -EINVAL;
    }
    return motor_param_get_by_offset(motor_param_table[index].offset, value);
}
```

**Usage in protocols:**

```c
// Shell: Uses string interface
motor_param_set("iq_setpoint", 0.5f);
float iq;
motor_param_get("iq_setpoint", &iq);

// MODBUS: Uses index (register address maps to index)
motor_param_set_by_index(MODBUS_REG_IQ_SETPOINT, 0.5f);
float iq_readback;
motor_param_get_by_index(MODBUS_REG_IQ_SETPOINT, &iq_readback);

// CAN: Uses index (object dictionary maps to index)
motor_param_set_by_index(CAN_OBJ_IQ_SETPOINT, 0.5f);
float iq_status;
motor_param_get_by_index(CAN_OBJ_IQ_SETPOINT, &iq_status);

// Settings subsystem: Uses descriptor table for save/load
float value;
for (size_t i = 0; i < ARRAY_SIZE(motor_param_table); i++) {
    motor_param_get(motor_param_table[i].name, &value);
    settings_save_one("motor/param", motor_param_table[i].name, &value, sizeof(float));
}
```

#### Option 2: Enum-Based Keys (Alternative)

**Advantages:**
- Compile-time type checking
- Autocomplete in IDE
- Typos caught at compile time
- Good for internal APIs

**Disadvantages:**
- Requires manual switch statement for each parameter
- More code to maintain vs descriptor table
- Less flexible than offset-based approach

```c
/* Parameter ID enum */
enum motor_param_id {
    MOTOR_PARAM_ID_SETPOINT = 0,
    MOTOR_PARAM_IQ_SETPOINT,
    MOTOR_PARAM_MAX_VSMAG,
    MOTOR_PARAM_PI_ID_KP,
    MOTOR_PARAM_PI_ID_KI,
    MOTOR_PARAM_PI_IQ_KP,
    MOTOR_PARAM_PI_IQ_KI,
    MOTOR_PARAM_COUNT
};

/* Requires manual switch for each parameter */
int motor_param_set_by_id(enum motor_param_id id, float value)
{
    struct motor_parameters *params = motor_get_params();
    uint8_t shadow_idx = 1 - params->active_buffer;
    struct motor_params_buffer *shadow = &params->buffers[shadow_idx];
    
    switch (id) {
    case MOTOR_PARAM_ID_SETPOINT:
        shadow->Id_setpoint_A = value;
        break;
    case MOTOR_PARAM_IQ_SETPOINT:
        shadow->Iq_setpoint_A = value;
        break;
    /* ... manually add each case ... */
    default:
        return -EINVAL;
    }
    
    params->update_pending = true;
    return 0;
}
```

#### Option 3: Hash-Based Keys (Compile-Time Optimization)

**Advantages:**
- O(1) hash lookup with perfect hash
- No runtime string comparison overhead
- Still supports text-based protocols

**Disadvantages:**
- Requires constexpr or build-time hash generation
- More complex than descriptor table
- Hash collisions possible (unless using perfect hash)
#define HASH_OFFSET 2166136261u
#define HASH_PRIME 16777619u

static constexpr uint32_t hash_string(const char *str) {
    uint32_t hash = HASH_OFFSET;
    while (*str) {
        hash ^= (uint8_t)*str++;
        hash *= HASH_PRIME;
    }
    return hash;
}

/* Pre-computed hashes as constants */
#define PARAM_HASH_ID_SETPOINT  hash_string("id_setpoint")
#define PARAM_HASH_IQ_SETPOINT  hash_string("iq_setpoint")
// ... etc

int motor_param_set_by_hash(uint32_t hash, float value)
{
    struct motor_parameters *params = motor_get_params();
    
    uint8_t active_idx = params->active_buffer;
    uint8_t shadow_idx = 1 - active_idx;
    struct motor_params_buffer *shadow = &params->buffers[shadow_idx];
    
    /* Hash-based switch (compiler optimizes to jump table) */
    switch (hash) {
    case PARAM_HASH_ID_SETPOINT:
        shadow->Id_setpoint_A = value;
        break;
    case PARAM_HASH_IQ_SETPOINT:
        shadow->Iq_setpoint_A = value;
        break;
    // ... etc
    default:
        return -EINVAL;
    }
    
    params->update_pending = true;
    return 0;
}

/* String wrapper computes hash once */
int motor_param_set(const char *name, float value)
{
    uint32_t hash = hash_string(name);
    return motor_param_set_by_hash(hash, value);
}
```

```c
/* Compile-time hash function (FNV-1a) */

}
```

### Recommendation: **Offset-Based Approach (Option 1)**

**Use offset-based descriptors** following Zephyr patterns (JSON, Settings, etc.):

1. **Zephyr-idiomatic**: Matches json_obj_descr, settings, and other subsystems
2. **Table-driven**: Single source of truth for parameter metadata
3. **Minimal code**: No manual switch statements to maintain
4. **Flexible**: Descriptor table can include validation, limits, help text
5. **Efficient**: Direct memory access via offsetof()
6. **Shell integration**: Easy to generate `params list` from descriptor table

**Performance Comparison:**

Zephyr provides several subsystems that can simplify implementation:

### 1. Settings Subsystem (Parameter Persistence)

**Purpose**: Automatically persist parameters to NVM (flash/EEPROM)

```kconfig
CONFIG_SETTINGS=y
CONFIG_SETTINGS_RUNTIME=y
CONFIG_FCB=y  # Flash Circular Buffer backend
CONFIG_FLASH=y
CONFIG_FLASH_MAP=y
```

**Usage**:
```c
#include <zephyr/settings/settings.h>

/* Automatic save/load of motor parameters using descriptor table */
static int motor_settings_set(const char *name, size_t len,
                              settings_read_cb read_cb, void *cb_arg)
{
    const char *next;
    float value;
    
    /* Iterate through parameter table to find matching parameter */
    for (size_t i = 0; i < ARRAY_SIZE(motor_param_table); i++) {
        if (settings_name_steq(name, motor_param_table[i].name, &next) && !next) {
            read_cb(cb_arg, &value, sizeof(value));
            motor_param_set_by_offset(motor_param_table[i].offset, value);
            return 0;
        }
    }
    return -ENOENT;
}

static int motor_settings_export(int (*cb)(const char *name, const void *value,
                                           size_t val_len))
{
    float value;
    
    /* Export all parameters from descriptor table */
    for (size_t i = 0; i < ARRAY_SIZE(motor_param_table); i++) {
        motor_param_get(motor_param_table[i].name, &value);
        cb(motor_param_table[i].name, &value, sizeof(value));
    }
    return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(motor, "motor", NULL, motor_settings_set,
                               motor_settings_commit, motor_settings_export);

/* Shell command to save parameters */
motor params save  // Calls settings_save()
```

### 2. Stats Subsystem (Performance Monitoring)

**Purpose**: Built-in performance statistics collection

```kconfig
CONFIG_STATS=y
CONFIG_STATS_NAMES=y
```

**Usage**:
```c
#include <zephyr/stats/stats.h>

/* Define stats group for motor control */
STATS_SECT_START(motor_stats)
STATS_SECT_ENTRY32(isr_count)           // Total ISR invocations
STATS_SECT_ENTRY32(isr_max_cycles)      // Max ISR duration
STATS_SECT_ENTRY32(buffer_swaps)        // Parameter updates
STATS_SECT_ENTRY32(encoder_faults)      // Encoder errors
STATS_SECT_ENTRY32(overcurrent_faults)  // Overcurrent events
STATS_SECT_END;

STATS_SECT_DECL(motor_stats) motor_statistics;
STATS_NAME_START(motor_stats)
STATS_NAME(motor_stats, isr_count)
STATS_NAME(motor_stats, isr_max_cycles)
STATS_NAME(motor_stats, buffer_swaps)
STATS_NAME(motor_stats, encoder_faults)
STATS_NAME(motor_stats, overcurrent_faults)
STATS_NAME_END(motor_stats);

/* In ISR */
STATS_INC(motor_statistics, isr_count);

/* Shell automatically provides stats commands */
stats show motor_stats
stats reset motor_stats
```

### 3. Ring Buffers (Data Logging)

**Purpose**: Efficient circular buffer for telemetry logging

```c
#include <zephyr/sys/ring_buffer.h>

/* High-speed telemetry ring buffer */
RING_BUF_DECLARE(telemetry_log, 4096);

struct telemetry_sample {
    uint32_t timestamp_us;
    float id_meas;
    float iq_meas;
    float vd;
    float vq;
    float speed;
};

/* In ADC ISR - log telemetry (zero-copy) */
uint8_t *buf;
uint32_t claimed = ring_buf_put_claim(&telemetry_log, &buf, sizeof(struct telemetry_sample));

if (claimed == sizeof(struct telemetry_sample)) {
    struct telemetry_sample *sample = (struct telemetry_sample *)buf;
    sample->timestamp_us = k_cycle_get_32() / (sys_clock_hw_cycles_per_sec() / 1000000);
    sample->id_meas = Id;
    sample->iq_meas = Iq;
    sample->vd = Vd;
    sample->vq = Vq;
    sample->speed = speed;
    
    ring_buf_put_finish(&telemetry_log, claimed);
}
/* If buffer full (claimed == 0), sample is dropped - acceptable for telemetry */

/* Shell command to dump log */
motor info log  // Reads and prints ring buffer
```

### 4. Logging Subsystem (Structured Logging)

**Purpose**: Better than raw printk, supports filtering and backends

```kconfig
CONFIG_LOG=y
CONFIG_LOG_MODE_DEFERRED=y  # Don't block ISR
CONFIG_LOG_BACKEND_UART=y
CONFIG_LOG_BUFFER_SIZE=4096
```

**Usage**:
```c
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(motor_control, LOG_LEVEL_INF);

/* Structured logging with automatic timestamps */
LOG_INF("Calibration complete: Rs=%.4f Ω, L=%.2f µH",
        (double)params->Rs_measured, (double)(params->L_measured * 1e6f));

LOG_WRN("Overcurrent detected: Ia=%.2f A", (double)Ia);

LOG_ERR("Encoder fault: %u consecutive failures", fault_count);

/* Runtime control via shell */
log enable motor_control debug
log disable motor_control
```

### 5. Zephyr Shell Dictionary (Auto-completion)

**Purpose**: Built-in parameter name completion

```c
/* Shell automatically provides tab-completion for subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(
    sub_motor_params,
    SHELL_CMD_ARG(get, NULL, "Get parameter\n"
                  "Available: id_setpoint, iq_setpoint, max_vsmag, ...",
                  cmd_motor_params_get, 2, 0),
    // Shell provides autocomplete automatically!
    SHELL_SUBCMD_SET_END
);
```

### 6. Thread-to-ISR Communication Patterns

**Problem**: Message queues and zbus cannot be used in ISRs due to potential blocking.

**Solution**: Double buffering is the correct pattern for thread-to-ISR communication.

#### Why Message Queues Don't Work for Motor Control

```c
/* ❌ WRONG: Cannot use in ISR */
K_MSGQ_DEFINE(motor_cmd_queue, sizeof(struct motor_cmd), 16, 4);

static void adc_callback(const struct device *dev, const q31_t *values,
                        uint8_t count, void *user_data)
{
    struct motor_cmd cmd;
    
    /* ❌ k_msgq_get() can block - NOT allowed in ISR! */
    if (k_msgq_get(&motor_cmd_queue, &cmd, K_NO_WAIT) == 0) {
        // This won't work reliably
    }
}
```

**Why it fails:**
- ISRs cannot block (K_NO_WAIT still has overhead)
- Message queue synchronization primitives too heavy for 20kHz
- Adds jitter and latency to critical control loop
- Can cause priority inversion issues

#### ✅ Correct Patterns for ISR Communication

**1. Double Buffering (Recommended for our motor control)**
```c
/* Lock-free, minimal ISR overhead (single-core Cortex-M) */
struct motor_params_buffer buffers[2];
volatile uint8_t active_buffer;
volatile bool update_pending;

/* Thread writes to shadow buffer */
void thread_update_params(void) {
    uint8_t shadow_idx = 1 - active_buffer;
    buffers[shadow_idx].iq_setpoint = new_value;
    update_pending = true;  // Signal ISR
}

/* ISR reads from active buffer */
void isr_handler(void) {
    if (update_pending) {
        update_pending = false;
        active_buffer = 1 - active_buffer;  // Swap (atomic on Cortex-M)
    }
    float iq = buffers[active_buffer].iq_setpoint;  // Use active
}
```

**2. Volatile Variables (For simple single values)**
```c
/* For single scalar parameters (single-core Cortex-M) */
volatile bool emergency_stop_flag;
volatile int32_t target_speed_mhz;  // Fixed-point

/* Thread sets */
target_speed_mhz = (int32_t)(speed_hz * 1000);

/* ISR reads */
if (emergency_stop_flag) {
    // Immediate shutdown
}
float speed = (float)target_speed_mhz / 1000.0f;

/* Note: Aligned 32-bit reads/writes are atomic on Cortex-M.
 * For 64-bit values, use double buffering instead. */
```

**3. Volatile Flags (For ISR-to-thread signaling)**
```c
/* ISR signals thread via volatile flags */
static volatile bool error_pending;
static volatile uint32_t error_code;  // ISR writes, thread reads

/* Dedicated error handling thread polls flag */
static void error_thread(void)
{
    while (1) {
        if (error_pending) {
            error_pending = false;
            LOG_ERR("Motor error occurred, code: %d", error_code);
            // Handle error in thread context
            // Can use message queues, mutexes, etc. here
        }
        k_msleep(10);  // Poll periodically
    }
}

/* In ISR - just set flag */
static void adc_callback(...)
{
    if (error_detected) {
        error_code = detected_error;
        error_pending = true;  // Minimal overhead
    }
}

/* Note: Cannot use k_work_submit() from direct ISR as it may reschedule.
 * Use volatile flags with thread polling for ISR-to-thread communication. */
```

**4. Circular Buffers (For high-speed data logging)**
```c
/* Lock-free ring buffer for telemetry */
#include <zephyr/sys/ring_buffer.h>

RING_BUF_DECLARE(telemetry_log, 4096);

/* ISR writes samples (zero-copy with claim/finish) */
static void adc_callback(...)
{
    uint8_t *buf;
    uint32_t claimed = ring_buf_put_claim(&telemetry_log, &buf, sizeof(struct sample));
    
    if (claimed == sizeof(struct sample)) {
        struct sample *s = (struct sample *)buf;
        s->id = Id;
        s->iq = Iq;
        s->timestamp = k_cycle_get_32();
        
        ring_buf_put_finish(&telemetry_log, claimed);
    }
    /* If full, sample dropped - acceptable for high-speed logging */
}

/* Thread reads samples */
void logging_thread(void)
{
    struct sample s;
    while (ring_buf_get(&telemetry_log, (uint8_t *)&s, sizeof(s)) == sizeof(s)) {
        // Process/log sample
    }
}
```

#### Comparison of Communication Methods

| Method | ISR-Safe | Latency | Overhead | Use Case |
|--------|----------|---------|----------|----------|
| **Double Buffer** | ✅ Yes | ~50µs | Minimal (volatile) | **Motor params (our case)** |
| Volatile Variables | ✅ Yes | Immediate | Minimal | Single values, flags |
| Volatile Flags + Poll | ✅ Yes (one-way) | ~ms | Minimal ISR | ISR→Thread events |
| Ring Buffer | ✅ Yes | Variable | Low | High-speed logging |
| Work Queue | ❌ No (may reschedule) | ~ms | Medium | Thread↔Thread only |
| Message Queue | ❌ No | ~ms | High | Thread↔Thread only |
| Zbus | ❌ No | ~ms | Medium | Thread↔Thread pub/sub |
| Mutex | ❌ No | N/A | High | Thread↔Thread only |

**Note on Single-Core Cortex-M**: Atomic operations not required for aligned
32-bit or smaller variables. Hardware guarantees atomicity; volatile prevents
compiler reordering.

#### When to Use Each Pattern

**Double Buffering**: 
- ✅ ISR needs consistent snapshot of multiple related parameters
- ✅ Updates are infrequent relative to ISR rate (our motor control)
- ✅ Parameters form coherent structure (motor_params_buffer)

**Volatile Variables**:
- ✅ Single independent value (emergency stop flag)
- ✅ Value fits in 32 bits (atomic on Cortex-M)
- ✅ Can use for float (32-bit aligned)
- ❌ Don't use for 64-bit types (use double buffering)

**Volatile Flags with Thread Polling**:
- ✅ ISR needs to signal non-urgent event to thread
- ✅ Error handling, notifications (can tolerate ms latency)
- ✅ Minimal ISR overhead (just flag write)
- ❌ Not for real-time control (polling adds latency)

**Ring Buffer**:
- ✅ ISR produces data that thread consumes
- ✅ Can tolerate some data loss (overruns)
- ✅ High-frequency telemetry logging

**Message Queue/Zbus** (Thread-only):
- ✅ Communication between shell thread and state machine thread
- ❌ NEVER use in ISR path
- Example: Shell → State Machine → Parameter validation → Double buffer update

#### Recommended Architecture for Motor Control

```
┌──────────────┐
│ Shell Thread │  Uses message queue for commands
└──────┬───────┘
       │ k_msgq_put()
       ↓
┌──────────────────────┐
│ State Machine Thread │  Validates and processes commands
└──────┬───────────────┘
       │ motor_param_set_by_id()
       ↓
┌────────────────────┐
│  Double Buffer     │  Lock-free parameter storage
│  (Shadow/Active)   │
└──────┬─────────────┘
       │ atomic swap
       ↓
┌──────────────────┐
│  ADC ISR (20kHz) │  Reads active buffer only
│  Motor Control   │
└──────────────────┘
```

**Why this works:**
1. Shell → State Machine: Can use blocking message queue (both are threads)
2. State Machine → Double Buffer: Lock-free update of shadow buffer
3. Double Buffer → ISR: Atomic swap, zero ISR overhead
4. ISR never blocks, never touches message queues

#### Code Example: Complete Flow

```c
/* Message queue for shell → state machine */
K_MSGQ_DEFINE(shell_cmd_queue, sizeof(struct motor_cmd), 16, 4);

/* Shell command handler (thread context) */
static int cmd_motor_current_iq(const struct shell *shell, size_t argc, char **argv)
{
    float iq = strtof(argv[1], NULL);
    
    /* ✅ OK: Shell directly calls parameter API (can also use message queue) */
    if (motor_param_set("iq_setpoint", iq) != 0) {
        shell_error(shell, "Failed to set Iq setpoint");
        return -EINVAL;
    }
    
    shell_print(shell, "Iq setpoint set to %.2f A", (double)iq);
    return 0;
}

/* Alternative: State machine thread (if using message queue) */
static enum smf_state_result motor_state_online_run(void *obj)
{
    struct motor_cmd cmd;
    
    /* ✅ OK: Non-blocking check for commands */
    while (k_msgq_get(&shell_cmd_queue, &cmd, K_NO_WAIT) == 0) {
        /* Validate command */
        if (cmd.type == CMD_SET_PARAM) {
            /* ✅ OK: Thread updates shadow buffer via double buffering */
            motor_param_set(cmd.param_name, cmd.value);
        }
    }
    
    return SMF_EVENT_HANDLED;
}

/* ADC ISR (never touches message queue) */
static void adc_callback(...)
{
    /* ✅ OK: ISR swaps buffers (atomic on single-core Cortex-M) */
    if (update_pending) {
        update_pending = false;
        active_buffer = 1 - active_buffer;
    }
    
    /* ✅ OK: ISR reads from active buffer */
    struct motor_params_buffer *active = &buffers[active_buffer];
    float iq_setpoint = active->Iq_setpoint_A;
    
    /* Motor control continues... */
}
```

**Summary**: Use message queues/zbus for thread-to-thread communication, but always use double buffering (or volatile variables) for thread-to-ISR communication in real-time control systems. On single-core Cortex-M processors, volatile is sufficient for aligned 32-bit or smaller variables.

### 7. JSON Parsing (For HTTP/MQTT)

**Purpose**: Built-in JSON encoder/decoder

```kconfig
CONFIG_JSON_LIBRARY=y
```

**Usage**:
```c
#include <zephyr/data/json.h>

/* Parse JSON from HTTP POST */
static const struct json_obj_descr motor_cmd_descr[] = {
    JSON_OBJ_DESCR_PRIM(struct motor_cmd_json, id_setpoint, JSON_TOK_FLOAT),
    JSON_OBJ_DESCR_PRIM(struct motor_cmd_json, iq_setpoint, JSON_TOK_FLOAT),
};

struct motor_cmd_json cmd;
int ret = json_obj_parse(payload, len, motor_cmd_descr,
                        ARRAY_SIZE(motor_cmd_descr), &cmd);
if (ret == 0) {
    motor_param_set("id_setpoint", cmd.id_setpoint);
    motor_param_set("iq_setpoint", cmd.iq_setpoint);
}

/* Encode JSON for HTTP response */
JSON_OBJ_DESCR_PRIM(struct motor_status, id_meas, JSON_TOK_FLOAT);
// ... encode telemetry to JSON
```

### 8. Retention Subsystem (Survive Reboot)

**Purpose**: Keep parameters in RAM across soft resets

```kconfig
CONFIG_RETENTION=y
CONFIG_RETAINED_MEM=y
```

**Usage**:
```c
/* Parameters survive soft reset but not power cycle */
static struct motor_params_buffer retained_params
    __attribute__((section(".noinit")));

/* On boot, check if valid */
if (retained_params.magic == MOTOR_PARAMS_MAGIC) {
    /* Restore from retention */
    memcpy(&motor_params.buffers[0], &retained_params,
           sizeof(struct motor_params_buffer));
}
```

## Leveraging Zephyr Built-In Subsystems

Zephyr provides several subsystems that can simplify implementation:

### 1. Settings Subsystem (Parameter Persistence)

**Purpose**: Automatically persist parameters to NVM (flash/EEPROM)

```kconfig
CONFIG_SETTINGS=y
CONFIG_SETTINGS_RUNTIME=y
CONFIG_FCB=y  # Flash Circular Buffer backend
CONFIG_FLASH=y
CONFIG_FLASH_MAP=y
```

**Usage**:
```c
#include <zephyr/settings/settings.h>

/* Automatic save/load of motor parameters using descriptor table */
static int motor_settings_set(const char *name, size_t len,
                              settings_read_cb read_cb, void *cb_arg)
{
    const char *next;
    float value;
    
    /* Iterate through parameter table to find matching parameter */
    for (size_t i = 0; i < ARRAY_SIZE(motor_param_table); i++) {
        if (settings_name_steq(name, motor_param_table[i].name, &next) && !next) {
            read_cb(cb_arg, &value, sizeof(value));
            motor_param_set_by_offset(motor_param_table[i].offset, value);
            return 0;
        }
    }
    return -ENOENT;
}

static int motor_settings_export(int (*cb)(const char *name, const void *value,
                                           size_t val_len))
{
    float value;
    
    /* Export all parameters from descriptor table */
    for (size_t i = 0; i < ARRAY_SIZE(motor_param_table); i++) {
        motor_param_get(motor_param_table[i].name, &value);
        cb(motor_param_table[i].name, &value, sizeof(value));
    }
    return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(motor, "motor", NULL, motor_settings_set,
                               motor_settings_commit, motor_settings_export);

/* Shell command to save parameters */
motor params save  // Calls settings_save()
```

### 2. Stats Subsystem (Performance Monitoring)

**Purpose**: Built-in performance statistics collection

```kconfig
CONFIG_STATS=y
CONFIG_STATS_NAMES=y
```

**Usage**:
```c
#include <zephyr/stats/stats.h>

/* Define stats group for motor control */
STATS_SECT_START(motor_stats)
STATS_SECT_ENTRY32(isr_count)           // Total ISR invocations


## Benefits of Double Buffering

1. **No Locks in ISR**: Zero overhead in critical 20kHz control loop
2. **Thread Safe**: Volatile buffer swap ensures consistency (single-core Cortex-M)
3. **Simple**: Clear separation between active and shadow buffers
4. **Fast**: Single byte write for buffer swap
5. **Safe**: No partial updates visible to ISR

## Testing Plan

1. **Basic Commands**: Test all shell commands with motor disabled
2. **Parameter Changes**: Verify buffer swapping works during motor operation
3. **State Transitions**: Test state machine control via shell
4. **Error Handling**: Verify error states and recovery
5. **Concurrent Access**: Test multiple rapid parameter changes
6. **Safety Limits**: Verify current/voltage limit checking

## Protocol Independence - Multi-Interface Support

The double buffering architecture is **protocol-agnostic** and works identically for any communication interface. All external protocols share the same challenge: they execute in different contexts (threads, callbacks, ISRs) than the motor control loop and need thread-safe parameter access.

### Universal Architecture

```
┌─────────────────┐
│  Shell Thread   │──┐
└─────────────────┘  │
                     │
┌─────────────────┐  │    ┌──────────────────┐
│ MODBUS Thread   │──┼───→│  Shadow Buffer   │
└─────────────────┘  │    │  (Write Only)    │
                     │    └──────────────────┘
┌─────────────────┐  │            │
│  MQTT Thread    │──┤            │ atomic_set_bit(update_pending)
└─────────────────┘  │            ↓
                     │    ┌──────────────────┐
┌─────────────────┐  │    │  Update Pending  │
│  HTTP Thread    │──┤    │      Flag        │
└─────────────────┘  │    └──────────────────┘
                     │            │
┌─────────────────┐  │            │ ISR checks flag
│  CAN Thread     │──┘            ↓
└─────────────────┘      ┌──────────────────┐
                         │  Active Buffer   │←─── ADC ISR (20kHz)
                         │  (Read Only)     │     Motor Control
                         └──────────────────┘
```

**Key Point**: All protocols use the same `motor_param_set()` / `motor_param_get()` API. The buffer management is completely transparent to the communication layer.

### Protocol-Specific Implementations

#### 1. Shell (Interactive CLI)
```c
// Shell command handler
static int cmd_set_iq(const struct shell *shell, size_t argc, char **argv)
{
    float iq = strtof(argv[1], NULL);
    
    // Uses common API
    if (motor_param_set("iq_setpoint", iq) != 0) {
        shell_error(shell, "Failed to set Iq");
        return -EINVAL;
    }
    
    shell_print(shell, "Iq setpoint = %.2f A", (double)iq);
    return 0;
}
```

#### 2. Modbus RTU/TCP (Industrial Control)
```c
// Modbus register map
#define MODBUS_REG_IQ_SETPOINT  1000
#define MODBUS_REG_ID_SETPOINT  1001
#define MODBUS_REG_MAX_CURRENT  1002
// ... more registers

// Modbus holding register write callback
static int modbus_write_holding_register(uint16_t addr, uint16_t value)
{
    float float_val;
    
    switch (addr) {
    case MODBUS_REG_IQ_SETPOINT:
        // Convert fixed-point to float (e.g., value in mA)
        float_val = (float)value / 1000.0f;
        return motor_param_set("iq_setpoint", float_val);
        
    case MODBUS_REG_ID_SETPOINT:
        float_val = (float)value / 1000.0f;
        return motor_param_set("id_setpoint", float_val);
        
    case MODBUS_REG_MAX_CURRENT:
        float_val = (float)value / 1000.0f;
        return motor_param_set("max_current", float_val);
        
    default:
        return -EINVAL;
    }
}

// Modbus holding register read callback
static int modbus_read_holding_register(uint16_t addr, uint16_t *value)
{
    float float_val;
    
    switch (addr) {
    case MODBUS_REG_IQ_SETPOINT:
        if (motor_param_get("iq_setpoint", &float_val) != 0)
            return -EINVAL;
        *value = (uint16_t)(float_val * 1000.0f);  // Convert to mA
        return 0;
        
    // ... similar for other registers
    
    default:
        return -EINVAL;
    }
}
```

**Zephyr Modbus Integration**:
```kconfig
CONFIG_MODBUS=y
CONFIG_MODBUS_RTU_MODE=y
CONFIG_MODBUS_SERIAL_BACKEND=y
CONFIG_MODBUS_ROLE_SERVER=y
```

#### 3. MQTT (IoT / Cloud Connectivity)
```c
// MQTT topic subscription callback
static void mqtt_message_callback(struct mqtt_client *client,
                                  const struct mqtt_evt *evt)
{
    if (evt->type != MQTT_EVT_PUBLISH) {
        return;
    }
    
    const char *topic = evt->param.publish.message.topic.topic.utf8;
    const char *payload = evt->param.publish.message.payload.data;
    
    // Topic: motor/command/iq_setpoint
    // Payload: "0.5" (JSON or plain text)
    
    if (strstr(topic, "motor/command/") != NULL) {
        const char *param_name = topic + strlen("motor/command/");
        float value = strtof(payload, NULL);
        
        if (motor_param_set(param_name, value) == 0) {
            // Publish confirmation
            char response[64];
            snprintf(response, sizeof(response), 
                    "{\"param\":\"%s\",\"value\":%.3f}", 
                    param_name, (double)value);
            mqtt_publish(client, "motor/status/confirm", response);
        }
    }
}

// Periodic telemetry publishing
static void mqtt_publish_telemetry(struct mqtt_client *client)
{
    struct motor_current_state curr;
    motor_get_current_state(&curr);
    
    char json[256];
    snprintf(json, sizeof(json),
            "{\"id_meas\":%.3f,\"iq_meas\":%.3f,"
            "\"speed\":%.2f,\"vbus\":%.1f}",
            (double)curr.id_meas, (double)curr.iq_meas,
            (double)motor_get_speed(), (double)motor_get_vbus());
    
    mqtt_publish(client, "motor/telemetry", json);
}
```

**Zephyr MQTT Integration**:
```kconfig
CONFIG_MQTT_LIB=y
CONFIG_MQTT_LIB_TLS=y  # Optional for MQTT over TLS
CONFIG_NET_TCP=y
CONFIG_NET_SOCKETS=y
```

#### 4. HTTP REST API (Web Interface)
```c
// HTTP POST /api/motor/params endpoint
static int http_post_params(struct http_client_ctx *ctx,
                            const char *body)
{
    // Parse JSON: {"iq_setpoint": 0.5, "id_setpoint": 0.0}
    struct json_obj_descr param_descr[] = {
        JSON_OBJ_DESCR_PRIM(struct params, iq_setpoint, JSON_TOK_FLOAT),
        JSON_OBJ_DESCR_PRIM(struct params, id_setpoint, JSON_TOK_FLOAT),
    };
    
    struct params p;
    int ret = json_obj_parse(body, strlen(body), param_descr,
                             ARRAY_SIZE(param_descr), &p);
    
    if (ret == 0) {
        motor_param_set("iq_setpoint", p.iq_setpoint);
        motor_param_set("id_setpoint", p.id_setpoint);
        
        http_response_json(ctx, 200, "{\"status\":\"ok\"}");
    } else {
        http_response_json(ctx, 400, "{\"error\":\"invalid json\"}");
    }
    
    return 0;
}

// HTTP GET /api/motor/status endpoint
static int http_get_status(struct http_client_ctx *ctx)
{
    struct motor_current_state curr;
    motor_get_current_state(&curr);
    
    char json[512];
    snprintf(json, sizeof(json),
            "{\"state\":\"%s\","
            "\"id_setpoint\":%.3f,"
            "\"iq_setpoint\":%.3f,"
            "\"id_meas\":%.3f,"
            "\"iq_meas\":%.3f,"
            "\"speed\":%.2f,"
            "\"vbus\":%.1f}",
            motor_state_to_string(motor_get_current_state()),
            (double)curr.id_setpoint, (double)curr.iq_setpoint,
            (double)curr.id_meas, (double)curr.iq_meas,
            (double)motor_get_speed(), (double)motor_get_vbus());
    
    http_response_json(ctx, 200, json);
    return 0;
}
```

**Zephyr HTTP Server Integration**:
```kconfig
CONFIG_HTTP_SERVER=y
CONFIG_NET_TCP=y
CONFIG_NET_SOCKETS=y
CONFIG_HTTP_SERVER_MAX_CLIENTS=4
```

#### 5. CANopen (Automotive / Industrial Bus)
```c
// CANopen SDO (Service Data Object) read/write callbacks
static uint32_t canopen_sdo_read(uint16_t index, uint8_t subindex,
                                 void *buf, uint32_t size)
{
    float value;
    
    // Object dictionary mapping
    switch (index) {
    case 0x6040:  // Control word
        // Read state
        break;
    case 0x6071:  // Target torque (maps to Iq)
        if (motor_param_get("iq_setpoint", &value) == 0) {
            *(int16_t *)buf = (int16_t)(value * 1000.0f);  // mNm
            return sizeof(int16_t);
        }
        break;
    }
    
    return 0;  // Error
}

static uint32_t canopen_sdo_write(uint16_t index, uint8_t subindex,
                                  const void *buf, uint32_t size)
{
    float value;
    
    switch (index) {
    case 0x6040:  // Control word
        // State machine control
        break;
    case 0x6071:  // Target torque
        value = (float)(*(int16_t *)buf) / 1000.0f;
        return motor_param_set("iq_setpoint", value);
    }
    
    return 0;  // Error
}

// CANopen TPDO (Transmit PDO) for fast telemetry
static void canopen_send_tpdo1(void)
{
    struct motor_current_state curr;
    motor_get_current_state(&curr);
    
    // Pack telemetry into CAN frame
    struct can_frame frame = {
        .id = 0x180 + NODE_ID,  // TPDO1
        .dlc = 8,
    };
    
    *(int16_t *)&frame.data[0] = (int16_t)(curr.iq_meas * 1000.0f);
    *(int16_t *)&frame.data[2] = (int16_t)(curr.id_meas * 1000.0f);
    *(int16_t *)&frame.data[4] = (int16_t)(motor_get_speed() * 10.0f);
    *(uint16_t *)&frame.data[6] = (uint16_t)(motor_get_vbus() * 10.0f);
    
    can_send(can_dev, &frame, K_NO_WAIT);
}
```

**Zephyr CAN Integration**:
```kconfig
CONFIG_CAN=y
CONFIG_CANOPEN=y
CONFIG_CANOPEN_ROLE_NODE=y
```

### Unified Parameter Access API

All protocols use the same core API defined in `shell_commands.h`:

```c
/**
 * @brief Set motor parameter (writes to shadow buffer)
 * @param name Parameter name string
 * @param value New parameter value
 * @return 0 on success, -EINVAL if parameter unknown
 */
int motor_param_set(const char *name, float value);

/**
 * @brief Get motor parameter (reads from active buffer)
 * @param name Parameter name string
 * @param value Pointer to store parameter value
 * @return 0 on success, -EINVAL if parameter unknown
 */
int motor_param_get(const char *name, float *value);

/**
 * @brief Set current setpoints (convenience wrapper)
 */
void motor_set_current_setpoints(float id, float iq);

/**
 * @brief Get current state snapshot (telemetry)
 */
void motor_get_current_state(struct motor_current_state *state);

/**
 * @brief Request state machine transition
 */
void motor_request_state(enum motor_state new_state);

/**
 * @brief Get current state machine state
 */
enum motor_state motor_get_current_state(void);
```

### Benefits of This Architecture

1. **Protocol Independence**: Add new protocols without changing motor control code
2. **Consistent Behavior**: All interfaces see same parameters and updates
3. **Thread Safe**: Double buffering works regardless of protocol thread model
4. **Zero ISR Overhead**: No locks or synchronization in 20kHz control loop
5. **Easy Testing**: Can control motor from shell during development, then switch to production protocol
6. **Multiple Protocols**: Run shell + MODBUS + MQTT simultaneously if needed

### Example: Multi-Protocol System

```c
// All running concurrently:

// Shell for debugging
SHELL_CMD_REGISTER(motor, &sub_motor, "Motor Control Commands", NULL);

// MODBUS for PLC integration
modbus_server_init(&modbus_cfg);

// MQTT for cloud telemetry
mqtt_client_init(&mqtt_cfg);

// All use same motor_param_set/get API
// All updates go through same double buffer
// ISR doesn't know or care which protocol made the change
```

### Performance Considerations

- **Buffer swap overhead**: ~1-2 CPU cycles (single atomic read)
- **Update latency**: Max 50µs (one control loop period @ 20kHz)
- **Multiple protocol threads**: No problem, shadow buffer writes are serialized by each protocol's thread
- **Rapid updates**: Flag prevents multiple swaps in one ISR cycle, last update wins

### Configuration Example

```kconfig
# Enable multiple protocols simultaneously
CONFIG_SHELL=y
CONFIG_MODBUS=y
CONFIG_MQTT_LIB=y
CONFIG_HTTP_SERVER=y

# All share same parameter interface
CONFIG_MOTOR_PARAM_BUFFER_SIZE=128  # Size of parameter buffers
```

## Future Enhancements

- Parameter persistence to NVM (flash storage)
- Data logging to SD card
- Step response testing for PI tuning
- Frequency response measurement (Bode plots)
- Scripting support for automated testing sequences
- WebSocket support for real-time web dashboards
- EtherCAT slave implementation for high-speed industrial networks

## Helper Functions and Data Structures

### Required Helper Functions in shell_commands.h

```c
/**
 * @brief Convert motor state enum to string
 */
const char *motor_state_to_string(enum motor_state state);

/**
 * @brief Convert motor error enum to string
 */
const char *motor_error_to_string(enum motor_error error);

/**
 * @brief Get motor configuration (from devicetree)
 */
const struct motor_config *motor_get_config(void);

/**
 * @brief Get measured parameters (from calibration)
 */
const struct motor_measured_params *motor_get_measured_params(void);

/**
 * @brief Get live telemetry snapshot
 */
const struct motor_live_data *motor_get_live_data(void);

/**
 * @brief Get performance statistics
 */
const struct motor_stats *motor_get_stats(void);

/**
 * @brief Get motor inductance
 */
float motor_get_inductance(void);

/**
 * @brief Get motor resistance
 */
float motor_get_resistance(void);

/**
 * @brief Get maximum current limit
 */
float motor_get_max_current(void);

/**
 * @brief Get mechanical angle
 */
float motor_get_angle_mech(void);

/**
 * @brief Get electrical angle
 */
float motor_get_angle_elec(void);

/**
 * @brief Get motor speed
 */
float motor_get_speed(void);

/**
 * @brief Get bus voltage
 */
float motor_get_vbus(void);

/**
 * @brief Set PI controller gains
 */
void motor_set_pi_gains(const char *controller, float kp, float ki);

/**
 * @brief Get PI controller gains
 */
void motor_get_pi_gains(const char *controller, float *kp, float *ki);

/**
 * @brief Clear error condition
 */
void motor_clear_error(void);
```

### Data Structures

```c
/* Motor configuration from devicetree */
struct motor_config {
    uint8_t pole_pairs;
    float max_current_A;
    float rated_voltage_V;
    uint32_t control_freq_hz;
    uint32_t pwm_freq_hz;
    
    /* Observer */
    float observer_bw_hz;
    
    /* Calibration */
    uint32_t offset_meas_samples;
    float roverl_current_A;
    float roverl_freq_hz;
    float roverl_duration_s;
    float roverl_settling_ms;
    float rs_est_current_A;
    float rs_est_rampup_s;
    float rs_est_coarse_bw_hz;
    float rs_est_fine_bw_hz;
    float align_current_A;
    float align_duration_s;
    
    /* PI controllers */
    float pi_id_bw_hz;
    float pi_iq_bw_hz;
    
    /* Fault detection */
    uint32_t encoder_fault_samples;
    float overcurrent_limit_A;
    float overvoltage_limit_V;
    float vbus_regen_limit_V;
    
    /* Dynamic braking */
    bool dynamic_braking_en;
    float dynamic_braking_margin_V;
};

/* Measured parameters from calibration */
struct motor_measured_params {
    float ia_offset_A;
    float ib_offset_A;
    float rs_ohm;
    float l_H;
    float roverl_radps;
    float align_offset_deg;
};

/* Live telemetry data */
struct motor_live_data {
    enum motor_state state;
    enum motor_error error;
    
    float id_setpoint_A;
    float iq_setpoint_A;
    float id_ref_A;
    float iq_ref_A;
    float id_meas_A;
    float iq_meas_A;
    float ia_A;
    float ib_A;
    
    float vbus_V;
    float vd_V;
    float vq_V;
    float va_V;
    float vb_V;
    
    float angle_mech_deg;
    float angle_elec_deg;
    float speed_hz;
    
    uint32_t encoder_fault_count;
    bool encoder_ok;
};

/* Performance statistics */
struct motor_stats {
    uint32_t isr_count;
    uint32_t isr_max_cycles;
    uint32_t isr_avg_cycles;
    uint32_t buffer_swap_count;
    uint32_t encoder_fault_total;
    uint32_t overcurrent_total;
    uint32_t overvoltage_total;
    uint32_t undervoltage_total;
};

/* Current state snapshot */
struct motor_current_state {
    float id_setpoint;
    float iq_setpoint;
    float id_ref;
    float iq_ref;
    float id_meas;
    float iq_meas;
};
```

### Helper Function Implementations (in shell_commands.c)

```c
const char *motor_state_to_string(enum motor_state state)
{
    switch (state) {
    case MOTOR_STATE_INIT:        return "INIT";
    case MOTOR_STATE_IDLE:        return "IDLE";
    case MOTOR_STATE_OFFSET_MEAS: return "OFFSET_MEAS";
    case MOTOR_STATE_ROVERL_MEAS: return "ROVERL_MEAS";
    case MOTOR_STATE_RS_EST:      return "RS_EST";
    case MOTOR_STATE_ALIGN:       return "ALIGN";
    case MOTOR_STATE_ONLINE:      return "ONLINE";
    case MOTOR_STATE_ERROR:       return "ERROR";
    default:                      return "UNKNOWN";
    }
}

const char *motor_error_to_string(enum motor_error error)
{
    switch (error) {
    case MOTOR_ERROR_NONE:           return "NONE";
    case MOTOR_ERROR_OVERCURRENT:    return "OVERCURRENT";
    case MOTOR_ERROR_OVERVOLTAGE:    return "OVERVOLTAGE";
    case MOTOR_ERROR_UNDERVOLTAGE:   return "UNDERVOLTAGE";
    case MOTOR_ERROR_ENCODER_FAULT:  return "ENCODER_FAULT";
    case MOTOR_ERROR_CALIBRATION:    return "CALIBRATION";
    case MOTOR_ERROR_INTERNAL:       return "INTERNAL";
    default:                         return "UNKNOWN";
    }
}
```

## Implementation Checklist

### Phase 1: Core Infrastructure
- [ ] Create `shell_commands.h` with API declarations
- [ ] Create `shell_commands.c` with command implementations
- [ ] Add double buffering to `motor_parameters` structure in main.c
- [ ] Implement buffer swap logic in `adc_callback()`
- [ ] Add offset-based descriptor table and accessor functions
- [ ] Implement helper functions (state_to_string, error_to_string, etc.)

### Phase 2: Basic Commands
- [ ] Implement state control commands (start, stop, calibrate, status, etc.)
- [ ] Implement parameter get/set commands
- [ ] Implement `params list` command with descriptor table iteration
- [ ] Test basic shell interaction without motor running

### Phase 3: Runtime Control
- [ ] Implement current control commands (id, iq, dq, get)
- [ ] Implement PI tuning commands (get, set, bandwidth)
- [ ] Add current limit validation
- [ ] Test parameter updates during motor operation

### Phase 4: Diagnostics and Monitoring
- [ ] Implement `info config` command (devicetree parameters)
- [ ] Implement `info measured` command (calibration results)
- [ ] Implement `info live` command (real-time telemetry)
- [ ] Implement `info stats` command (performance counters)
- [ ] Add telemetry ring buffer (optional)

### Phase 5: Advanced Features (Optional)
- [ ] Add Settings subsystem for parameter persistence
- [ ] Add Stats subsystem integration
- [ ] Add ring buffer for high-speed logging
- [ ] Add JSON parsing for HTTP/MQTT protocols
- [ ] Implement protocol-agnostic interfaces (MODBUS, CAN, etc.)

### Phase 6: Testing and Validation
- [ ] Test all commands with motor disabled
- [ ] Test parameter updates during calibration
- [ ] Test parameter updates during motor operation
- [ ] Verify buffer swapping works correctly
- [ ] Test concurrent access from multiple sources
- [ ] Validate safety limits (current, voltage)
- [ ] Performance testing (ISR timing impact)
