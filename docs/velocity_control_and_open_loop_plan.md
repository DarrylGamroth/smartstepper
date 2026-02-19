# Velocity Control and Open-Loop Mode Implementation Plan

## Overview

This document outlines the plan for implementing velocity control modes (open-loop and closed-loop) to enable RLS parameter estimation testing without an encoder, and to provide a foundation for future velocity control capabilities.

**Primary Goal**: Enable RLS testing in open-loop mode without encoder feedback  
**Secondary Goal**: Establish architecture for velocity control (PI/PID/advanced controllers)

## Reference Systems

- **SimpleFOC**: Open-loop velocity control using angle generator
  - https://docs.simplefoc.com/velocity_openloop
  - Simple ramp-based velocity commands
  - Useful for initial motor characterization

## Architecture Design

### Control Modes Hierarchy

```
MOTOR_CONTROL_MODE
├── TORQUE_MODE (current)
│   ├── Id/Iq commanded directly via shell/API
│   └── No velocity control
│
├── VELOCITY_OPEN_LOOP (new)
│   ├── Uses angle_gen for electrical angle
│   ├── No feedback (encoder or observer)
│   ├── Velocity command → angle increment
│   └── Id/Iq commanded separately (usually Id=0, Iq=load dependent)
│
└── VELOCITY_CLOSED_LOOP (future)
    ├── Uses encoder or observer for feedback
    ├── PI/PID velocity controller
    ├── Velocity command → Iq reference
    └── Id command (field weakening, MTPA, etc.)
```

### State Machine Integration

Current state machine:
```
IDLE → CTRL_INIT → CALIBRATION → ONLINE → IDLE
                       ↓
                  (offset, Rs, R/L, align)
```

Enhanced state machine with ONLINE substates:
```
IDLE → CTRL_INIT → CALIBRATION → ONLINE → IDLE
                       ↓              ↓
                  (offset, Rs,   ONLINE.TORQUE_MODE (default)
                   R/L, align)   ONLINE.VELOCITY_OPEN_LOOP
                                 ONLINE.VELOCITY_CLOSED_LOOP (PI/PID/SpinTAC)
                                 ONLINE.POSITION (future)
```

**Key Decision**: Control modes are **substates of ONLINE**, not just properties. This provides:
- Clean entry/exit handlers for each mode
- Better separation of concerns
- Easier to add new control modes
- Explicit state transitions

### Data Structures

#### Velocity Controller API (Interface)

Define a common API that all velocity controllers must implement:

```c
/**
 * @brief Velocity controller context handle
 * 
 * Opaque handle to implementation-specific controller state.
 * Each implementation casts this to its own struct type.
 */
typedef void* velocity_controller_ctx_t;

/**
 * @brief Velocity controller API
 * 
 * Abstract interface for velocity control implementations.
 * Different implementations: open-loop, PI, PID, SpinTAC, etc.
 */
struct velocity_controller_api {
    /* Lifecycle */
    int (*init)(velocity_controller_ctx_t ctx, const void *config);  /* config is implementation-specific */
    void (*reset)(velocity_controller_ctx_t ctx);
    
    /* Setpoint */
    void (*set_target)(void *ctx, float32_t velocity_hz);
    float32_t (*get_target)(const void *ctx);
    
    /* Update (called from ISR) */
    void (*update)(velocity_controller_ctx_t ctx,
                   float32_t *angle_out,         /* Output: electrical angle */
                   float32_t *Iq_out,            /* Output: torque current command */
                   const void *update_params);   /* Implementation-specific inputs (NULL if not needed) */
    
    /* Configuration */
    int (*set_param)(velocity_controller_ctx_t ctx, const char *name, float32_t value);
    int (*get_param)(velocity_controller_ctx_t ctx, const char *name, float32_t *value);
    
    /* Status */
    bool (*is_stable)(velocity_controller_ctx_t ctx);
    float32_t (*get_error)(velocity_controller_ctx_t ctx);
};
```

**Key Design Points:**

1. **`init(velocity_controller_ctx_t ctx, const void *config)`**: 
   - `velocity_controller_ctx_t` is typedef for `void*` - clearer than raw void*
   - Each implementation defines its own config struct
   - Caller casts implementation-specific config to void*
   - Implementation casts back to its specific type

2. **`update(velocity_controller_ctx_t ctx, float32_t *angle_out, float32_t *Iq_out, const void *update_params)`**:
   - Outputs first, then implementation-specific inputs (common C pattern)
   - `update_params` can be NULL for open-loop (no feedback needed)
   - Can contain velocity feedback, angle feedback, etc. for closed-loop
   - Example: open-loop passes NULL, PI passes `velocity_pi_update_params*`

3. **`set_param/get_param(const char *name, float32_t value)`**:
   - Generic parameter access (replaces set_limits, set_bandwidth)
   - Shell commands can use this for all controllers
   - Implementation-specific parameters (e.g., "bandwidth", "max_velocity", "Kp")

#### Open-Loop Velocity Controller Implementation

```c
/**
 * @brief Open-loop velocity controller state
 * 
 * Simple angle generator, no feedback required.
 * Useful for testing, commissioning, and low-load applications.
 */
struct velocity_openloop_controller {
    angle_gen_t angle_gen;              /* Electrical angle generator */
    float32_t velocity_target_hz;       /* Mechanical Hz setpoint */
    float32_t max_velocity_hz;          /* Velocity limit */
    float32_t max_accel_hz_s;           /* Acceleration limit */
    float32_t pole_pairs;               /* Motor pole pairs (for elec conversion) */
    struct traj_f32 velocity_ramp;      /* Smooth ramping */
};

/**
 * @brief Open-loop controller initialization config
 */
struct velocity_openloop_config {
    float32_t control_freq_hz;          /* Control loop frequency */
    float32_t pole_pairs;               /* Motor pole pairs */
    float32_t max_velocity_hz;          /* Velocity limit */
    float32_t max_accel_hz_s;           /* Acceleration limit */
    float32_t initial_velocity_hz;      /* Starting velocity (usually 0) */
};

/* No update params needed for open-loop (it's truly open-loop) */

/* API implementation */
extern const struct velocity_controller_api velocity_openloop_api;

/* Shell command registration */
void velocity_openloop_register_shell_cmds(void);
```

#### PI Velocity Controller Implementation

```c
/**
 * @brief PI velocity controller state
 * 
 * Closed-loop velocity control using PI controller.
 * Requires encoder or observer feedback.
 */
struct velocity_pi_controller {
    struct pi_controller pi;            /* PI controller */
    float32_t velocity_target_hz;       /* Mechanical Hz setpoint */
    float32_t velocity_error_hz;        /* Tracking error */
    float32_t velocity_bandwidth_hz;    /* Loop bandwidth for tuning */
    float32_t motor_inertia_kgm2;       /* For auto-tuning */
    float32_t max_velocity_hz;          /* Velocity limit */
    float32_t max_accel_hz_s;           /* Acceleration limit */
    float32_t max_Iq_A;                 /* Current limit */
    struct traj_f32 velocity_ramp;      /* Smooth ramping */
};

/**
 * @brief PI controller initialization config
 */
struct velocity_pi_config {
    float32_t control_freq_hz;          /* Control loop frequency */
    float32_t motor_inertia_kgm2;       /* For auto-tuning */
    float32_t bandwidth_hz;             /* Desired closed-loop bandwidth */
    float32_t max_velocity_hz;          /* Velocity limit */
    float32_t max_accel_hz_s;           /* Acceleration limit */
    float32_t max_Iq_A;                 /* Current limit */
    /* Optional manual gains (if not auto-tuned) */
    float32_t Kp;                       /* 0 = auto-tune */
    float32_t Ki;                       /* 0 = auto-tune */
};

/**
 * @brief PI controller update parameters
 */
struct velocity_pi_update_params {
    float32_t velocity_fb_hz;           /* Velocity feedback (mech Hz) */
    float32_t angle_fb_rad;             /* Angle feedback (electrical rad) */
};

/* API implementation */
extern const struct velocity_controller_api velocity_pi_api;

/* Shell command registration */
void velocity_pi_register_shell_cmds(void);
```

#### PID Velocity Controller Implementation (Future)

```c
/**
 * @brief PID velocity controller state
 * 
 * Closed-loop velocity control with derivative term.
 * Better disturbance rejection than PI.
 */
struct velocity_pid_controller {
    struct pi_controller pi;            /* PI part */
    float32_t Kd;                       /* Derivative gain */
    float32_t velocity_prev_hz;         /* For derivative */
    struct filter_fo velocity_lpf;      /* D-term filter */
    /* ... similar fields to PI ... */
};

/**
 * @brief PID controller initialization config
 */
struct velocity_pid_config {
    /* Similar to PI config, plus: */
    float32_t Kd;                       /* Derivative gain */
    float32_t derivative_filter_hz;     /* D-term LPF cutoff */
};

/**
 * @brief PID controller update parameters
 */
struct velocity_pid_update_params {
    /* Same as PI */
    float32_t velocity_fb_hz;
    float32_t angle_fb_rad;
};

/* API implementation */
extern const struct velocity_controller_api velocity_pid_api;

/* Shell command registration */
void velocity_pid_register_shell_cmds(void);
```

#### SpinTAC or Advanced Controller (Future)

```c
/**
 * @brief Advanced adaptive velocity controller
 * 
 * Could be SpinTAC, adaptive control, model-predictive, etc.
 */
struct velocity_advanced_controller {
    /* Implementation-specific state */
    /* ... */
};

/* API implementation */
extern const struct velocity_controller_api velocity_advanced_api;
```

#### motor_parameters Extensions

```c
struct motor_parameters {
    /* ... existing fields ... */
    
    /* Velocity control - polymorphic design */
    const struct velocity_controller_api *velocity_api;  /* Current implementation */
    velocity_controller_ctx_t velocity_ctx;              /* Implementation context */
    
    /* Storage for different implementations */
    union {
        struct velocity_openloop_controller openloop;
        struct velocity_pi_controller pi;
        struct velocity_pid_controller pid;
        /* struct velocity_advanced_controller advanced; */
    } velocity_impl;
    
    /* Common velocity state */
    float32_t Id_cmd_A;                 /* D-axis current command */
    
    /* ... */
};
```

## Implementation Phases

### Phase 1: Open-Loop Velocity Mode (For RLS Testing)

**Goal**: Enable RLS testing without encoder

**Changes**:

1. **Create velocity controller API** (`velocity_controller_api.h`)
   - Define abstract interface (init, reset, set_target, update, etc.)
   - Allows multiple implementations with same interface

2. **Implement open-loop controller** (`velocity_openloop.h/c`)
   - `struct velocity_openloop_controller` with angle generator
   - Implement API functions (init, update, set_target, etc.)
   - Velocity ramping using `traj_f32`
   - Acceleration limits

3. **Add API to motor_parameters** (`config.h`)
   - Add `velocity_api` pointer (polymorphic interface)
   - Add `velocity_ctx` pointer (implementation context)
   - Add `velocity_impl` union for storage
   - Add `Id_cmd_A` for D-axis current command

4. **Add ONLINE substates** (`motor_states.c`)
   - `MOTOR_STATE_ONLINE_TORQUE` (default, existing behavior)
   - `MOTOR_STATE_ONLINE_VELOCITY_OPEN` (uses velocity_openloop_api)
   - `MOTOR_STATE_ONLINE_VELOCITY_CLOSED` (future, uses velocity_pi_api)
   - Entry handlers initialize appropriate controller
   - Run handlers call `velocity_api->update()`

5. **Modify ISR for velocity modes** (`motor_isr.c`)
   - In ONLINE state, check which substate is active
   - TORQUE substate: Use existing encoder/observer path
   - VELOCITY_OPEN substate:
     - Call `velocity_api->update(ctx, &angle, &Iq_cmd, NULL)` (NULL = no feedback)
     - Apply Id_cmd and Iq_cmd to PI controllers
     - Skip encoder updates (no feedback needed)
   - VELOCITY_CLOSED substate:
     - Get velocity feedback from encoder/observer
     - Prepare update_params struct with feedback
     - Call `velocity_api->update(ctx, &angle, &Iq_cmd, &update_params)`
     - Apply Id_cmd and Iq_cmd to PI controllers

6. **Add shell commands** (`shell_commands.c`)
   ```bash
   motor velocity mode <torque|openloop|closed>  # Trigger state transition
   motor velocity target <hz>                     # Set velocity target (mech Hz)
   motor velocity ramp <hz/s>                     # Set acceleration limit
   motor velocity get                             # Show velocity status
   motor velocity limits                          # Show velocity and current limits
   ```

7. **RLS gating modification** (`motor_isr.c`)
   - Check current substate
   - In VELOCITY_OPEN substate:
     - Bypass speed check (commanded velocity is known)
     - Keep current/voltage/residual checks
   - In TORQUE or VELOCITY_CLOSED substates:
     - Keep all gating checks (including speed)

**Testing Procedure**:
```bash
# 1. Calibrate motor (if not already done)
motor state calibrate

# 2. Switch to open-loop velocity mode
motor velocity mode openloop

# 3. Set velocity target (e.g., 5 Hz mechanical = 200 elec Hz for 40-pole stepper)
motor velocity target 5.0

# 4. Set Id/Iq currents
motor current id 0.0   # No field weakening
motor current iq 0.5   # Some torque for RLS excitation

# 5. Start motor
motor state start

# 6. Monitor RLS convergence
motor rls status
motor rls params

# 7. Wait for convergence (~10-15 seconds)
# 8. Check parameter estimates
motor rls params

# 9. Stop motor
motor state stop
```

### Phase 2: Velocity Ramping and Limits (For Smooth Operation)

**Goal**: Prevent sudden velocity changes

**Changes**:

1. **Add velocity ramping**
   - Use `traj_f32` for smooth velocity transitions
   - Configurable ramp rate (Hz/s)

2. **Add safety limits**
   - Maximum velocity check
   - Maximum acceleration check
   - Current limit enforcement

3. **Shell commands for tuning**
   ```bash
   motor velocity limit <hz>          # Set max velocity
   motor velocity accel <hz/s>        # Set max acceleration
   ```

### Phase 3: Closed-Loop Velocity Control (Post-Encoder Integration)

**Goal**: True velocity control with feedback

**Changes**:

1. **Add PI velocity controller**
   - PI controller in `velocity_controller` struct
   - Velocity error → Iq reference
   - Auto-tuning based on motor inertia and bandwidth

2. **Integrate feedback**
   - Encoder: Direct velocity measurement
   - Observer: Estimated velocity from angle observer

3. **Shell commands**
   ```bash
   motor velocity bandwidth <hz>      # Set velocity loop bandwidth
   motor velocity pid get             # Show PI gains
   motor velocity pid set <kp> <ki>   # Manual PI tuning
   ```

4. **Advanced features** (future)
   - Feed-forward compensation
   - MTPA (Maximum Torque Per Ampere)
   - Field weakening for high speeds
   - Observer-based sensorless control

## Devicetree Configuration

New properties in `rubus,user-parameters.yaml`:

```yaml
# Velocity control parameters
velocity-control-mode:
  type: int
  default: 0  # 0=torque, 1=velocity_open, 2=velocity_closed
  description: Default control mode

velocity-max-hz:
  type: int
  default: 50
  description: Maximum mechanical velocity in Hz

velocity-ramp-rate-hz-per-s:
  type: int
  default: 10
  description: Velocity ramp rate in Hz/s

velocity-bandwidth-hz:
  type: int
  default: 5
  description: Velocity controller bandwidth in Hz (for closed-loop)

velocity-max-current-a-milli:
  type: int
  default: 2000
  description: Maximum current for velocity mode in milliamps
```

## Code Organization

### New Files

#### Phase 1: Open-Loop
1. **`velocity_controller_api.h`** - Abstract velocity controller interface
2. **`velocity_openloop.h`** - Open-loop controller header and config structs
3. **`velocity_openloop.c`** - Open-loop controller implementation and API
4. **`velocity_openloop_shell.c`** - Open-loop specific shell commands

#### Phase 3: Closed-Loop (Future)
5. **`velocity_pi.h`** - PI controller header, config, and update param structs
6. **`velocity_pi.c`** - PI controller implementation and API
7. **`velocity_pi_shell.c`** - PI specific shell commands (bandwidth, gains, etc.)
8. **`velocity_pid.h`** - PID controller header (optional)
9. **`velocity_pid.c`** - PID controller implementation (optional)
10. **`velocity_pid_shell.c`** - PID specific shell commands (optional)

### Modified Files
1. **`config.h`** - Add velocity controller API pointer and union of implementations
2. **`motor_states.c`** - Add ONLINE substates (TORQUE, VELOCITY_OPEN_LOOP, VELOCITY_CLOSED_LOOP)
3. **`motor_isr.c`** - Call velocity controller API from ONLINE substates
4. **`shell_commands.c`** - Add top-level `motor velocity` subcommands and mode switching
5. **`rubus,user-parameters.yaml`** - Add velocity config properties
6. **`CMakeLists.txt`** - Add new velocity controller source files

## Key Design Decisions

### 1. Control Modes as Substates of ONLINE

**Decision**: Control modes are **substates of ONLINE** in the state machine hierarchy.

**Rationale**:
- All modes share the same calibration (offset, Rs, R/L) performed before ONLINE
- Clean entry/exit/run handlers for each mode
- Explicit state transitions (e.g., TORQUE → VELOCITY_OPEN_LOOP)
- Better separation of concerns than a mode flag
- Easier to add new control modes
- Consistent error handling and safety checks
- State machine library (SMF) provides hierarchical states naturally

### 2. Angle Source Selection

**Open-Loop**: `angle_gen_openloop` (independent from calibration angle generator)  
**Closed-Loop**: `angle_observer` (encoder or sensorless)

**Why separate angle generator?**
- Avoids conflicts with R/L calibration (uses `angle_gen_roverl`)
- Clean separation of concerns
- Easy to verify open-loop operation

### 3. Current Command vs. Torque Command

**Phase 1**: Direct Id/Iq commands (existing shell interface)  
**Phase 3**: Velocity controller outputs Iq, Id set separately

**Why not torque command?**
- Torque = (3/2) * pole_pairs * (λ_pm * Iq + (Ld - Lq) * Id * Iq)
- For SPM: Ld ≈ Lq, so torque ≈ k_t * Iq (simple)
- For IPM/hybrid stepper: Non-linear, MTPA needed
- Current control is more direct and flexible

### 4. Velocity Units

**User Interface**: Mechanical Hz (revolutions per second)  
**Internal**: Electrical rad/s for observers

**Rationale**:
- Hz is intuitive for users (RPM = Hz * 60)
- Electrical rad/s is natural for FOC math
- Conversion: ω_elec = 2π * pole_pairs * Hz_mech

### 5. RLS Gating in Open-Loop

**Keep all checks except speed observability:**
- ✅ Current magnitude check (ensure sufficient excitation)
- ✅ Voltage magnitude check (ensure valid measurements)
- ✅ PI saturation check (ensure voltage isn't clamped)
- ❌ Speed check (bypass - commanded velocity is known)
- ✅ Residual check (ensure good fit)

**Why bypass speed check?**
- In open-loop, back-EMF is commanded, not measured
- Cross-coupling terms are still correct (ω is known)
- Current derivative still valid
- Main observability concern (low speed) doesn't apply

## Shell Command Organization

### Design Pattern: Module-Based Shell Commands

**Principle**: Each velocity controller module registers its own shell commands, rather than centralizing everything in `shell_commands.c`.

**Benefits**:
- **Modularity**: Controller-specific commands stay with controller code
- **Extensibility**: Adding new controllers doesn't modify existing shell code
- **Maintainability**: Command implementation next to feature implementation
- **Discoverability**: Easy to find what commands a controller supports

### Shell Command Structure

```
motor velocity/
├── mode <torque|openloop|closed>     (shell_commands.c - state transitions)
├── get                                (shell_commands.c - generic status)
│
├── openloop/                          (velocity_openloop_shell.c)
│   ├── target <hz>
│   ├── ramp <hz/s>
│   ├── limit <hz>
│   └── status
│
├── pi/                                (velocity_pi_shell.c)
│   ├── target <hz>
│   ├── bandwidth <hz>
│   ├── gains get
│   ├── gains set <kp> <ki>
│   ├── limits <max_vel> <max_accel> <max_Iq>
│   └── status
│
└── pid/                               (velocity_pid_shell.c)
    ├── ... (similar to PI)
    └── derivative <kd> <filter_hz>
```

### Implementation Pattern

Each velocity controller module provides:

```c
/* In velocity_openloop.c */

static int cmd_velocity_openloop_target(const struct shell *sh, size_t argc, char **argv)
{
    /* Implementation */
}

static int cmd_velocity_openloop_status(const struct shell *sh, size_t argc, char **argv)
{
    /* Implementation */
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_velocity_openloop,
    SHELL_CMD(target, NULL, "Set velocity target", cmd_velocity_openloop_target),
    SHELL_CMD(ramp, NULL, "Set ramp rate", cmd_velocity_openloop_ramp),
    SHELL_CMD(status, NULL, "Show status", cmd_velocity_openloop_status),
    SHELL_SUBCMD_SET_END
);

void velocity_openloop_register_shell_cmds(void)
{
    /* Register with parent 'motor velocity' command tree */
    shell_register_subcmd("motor", "velocity", "openloop", &sub_velocity_openloop);
}
```

### Main Shell Commands (shell_commands.c)

Only high-level coordination commands stay in `shell_commands.c`:

```c
/* Mode switching (triggers state transitions) */
motor velocity mode <torque|openloop|closed>

/* Generic status (delegates to active controller) */
motor velocity get
```

## Shell Command Reference

### Phase 1 Commands

```bash
# Mode selection (shell_commands.c - triggers state transitions)
motor velocity mode torque         # Switch to ONLINE_TORQUE substate
motor velocity mode openloop       # Switch to ONLINE_VELOCITY_OPEN substate
motor velocity mode closed         # Switch to ONLINE_VELOCITY_CLOSED substate

# Generic status (shell_commands.c - delegates to active controller)
motor velocity get                 # Show current mode, target, actual, limits

# Open-loop specific commands (velocity_openloop_shell.c)
motor velocity openloop target <hz>         # Set velocity target (mech Hz)
motor velocity openloop ramp <hz/s>         # Set ramp rate
motor velocity openloop limit <hz>          # Set max velocity
motor velocity openloop accel <hz/s>        # Set max acceleration
motor velocity openloop status              # Show open-loop specific status

# Example usage:
motor velocity mode openloop                # Switch to open-loop mode
motor velocity openloop target 10.0         # Set 10 Hz target
motor velocity openloop ramp 5.0            # 5 Hz/s ramp rate
motor current dq 0.0 0.8                    # Set currents
motor state start                           # Start motor
```

### Phase 3 Commands (Closed-Loop)

```bash
# PI controller commands (velocity_pi_shell.c)
motor velocity pi target <hz>              # Set velocity target
motor velocity pi bandwidth <hz>           # Auto-tune PI based on bandwidth
motor velocity pi gains get                # Show Kp, Ki, limits
motor velocity pi gains set <kp> <ki>      # Manual PI tuning
motor velocity pi limits <vel> <accel> <Iq> # Set velocity, accel, current limits
motor velocity pi status                   # Show PI status (error, output, etc.)

# Example usage:
motor velocity mode closed                 # Switch to closed-loop mode
motor velocity pi bandwidth 10.0           # 10 Hz bandwidth (auto-tunes Kp, Ki)
motor velocity pi target 20.0              # Set 20 Hz target
motor current id 0.0                       # Set Id current
motor state start                          # Start motor
```

## Testing Strategy

### Phase 1 Testing (Open-Loop)

**Test 1: Basic Operation**
```bash
motor state calibrate              # Run calibration
motor velocity mode openloop       # Switch to open-loop
motor current dq 0.0 0.5           # Set Id=0, Iq=0.5A
motor velocity target 5.0          # 5 Hz mechanical
motor state start                  # Start motor
motor info live                    # Monitor currents/voltages
motor state stop                   # Stop motor
```

**Test 2: RLS Convergence**
```bash
motor state calibrate
motor velocity mode openloop
motor current dq 0.0 0.8           # Higher current for better excitation
motor velocity target 10.0         # 10 Hz mechanical
motor state start
# Wait 20 seconds for RLS convergence
motor rls status                   # Check convergence
motor rls params                   # Check estimates
motor rls temp                     # Check temperature tracking
motor state stop
```

**Test 3: Velocity Sweep**
```bash
motor state calibrate
motor velocity mode openloop
motor velocity ramp 2.0            # 2 Hz/s ramp
motor current dq 0.0 1.0
motor state start
motor velocity target 5.0          # Ramp to 5 Hz
# Wait 2.5 seconds
motor velocity target 15.0         # Ramp to 15 Hz
# Wait 5 seconds
motor velocity target 5.0          # Ramp back down
motor state stop
```

**Test 4: RLS Gating Validation**
```bash
motor state calibrate
motor velocity mode openloop
motor current dq 0.0 0.1           # Low current - should gate RLS
motor velocity target 5.0
motor state start
motor rls gating                   # Should show [LOW] current
motor current iq 0.8               # Increase current
# Wait a few seconds
motor rls gating                   # Should show [OK] current
motor state stop
```

### Success Criteria

**Phase 1 Complete When**:
- ✅ Motor spins smoothly in open-loop at commanded velocities
- ✅ RLS estimators converge within 15 seconds
- ✅ Parameter estimates are reasonable (Rs within 20% of calibrated value)
- ✅ Temperature tracking shows realistic values
- ✅ Gating logic properly enables/disables RLS
- ✅ No motor stalls or vibrations at steady velocity

## Future Enhancements

### Phase 4: Position Control (Post Phase 3)
- Position setpoint commands
- PID position controller
- Trajectory generation
- Homing sequences

### Phase 5: Advanced Control (Long-term)
- MTPA (Maximum Torque Per Ampere)
- Field weakening
- Sensorless control modes
- Adaptive control
- Model-predictive control (MPC)

### Phase 6: Motion Planning (Long-term)
- S-curve acceleration profiles
- Multi-segment trajectories
- Coordinated multi-axis motion
- Electronic camming

## Implementation Checklist

### Phase 1: Open-Loop Velocity

**API Layer:**
- [ ] Create `velocity_controller_api.h` with abstract interface
- [ ] Define API function pointers (init, reset, set_target, update, etc.)

**Open-Loop Implementation:**
- [ ] Create `velocity_openloop.h` with controller struct
- [ ] Create `velocity_openloop.c` with API implementation
- [ ] Implement `velocity_openloop_init()`
- [ ] Implement `velocity_openloop_set_target()` with ramping
- [ ] Implement `velocity_openloop_update()` with angle generation
- [ ] Implement `velocity_openloop_set_limits()`

**Integration:**
- [ ] Add velocity API fields to `motor_parameters` in `config.h`
- [ ] Add `velocity_impl` union to `motor_parameters`
- [ ] Add `Id_cmd_A` field to `motor_parameters`
- [ ] Add ONLINE substates to motor_states.c:
  - [ ] `MOTOR_STATE_ONLINE_TORQUE` (entry, run, exit)
  - [ ] `MOTOR_STATE_ONLINE_VELOCITY_OPEN` (entry, run, exit)
- [ ] Initialize velocity controller in ONLINE_VELOCITY_OPEN entry
- [ ] Call `velocity_api->update()` in ONLINE_VELOCITY_OPEN run
- [ ] Add substate switching in `motor_isr.c`
- [ ] Modify RLS gating for open-loop mode (bypass speed check)
- [ ] Add to `CMakeLists.txt`: `src/velocity_openloop.c`

**Configuration:**
- [ ] Add devicetree properties for velocity control
- [ ] Add macros in `config.h` for devicetree extraction

**Shell Commands:**
- [ ] Add shell commands: `motor velocity mode`
- [ ] Add shell commands: `motor velocity target`
- [ ] Add shell commands: `motor velocity ramp`
- [ ] Add shell commands: `motor velocity get`
- [ ] Add shell commands: `motor velocity limits`

**Testing:**
- [ ] Build and test basic open-loop operation
- [ ] Test velocity ramping and limits
- [ ] Test RLS convergence in open-loop mode
- [ ] Validate parameter estimates match calibration
- [ ] Test mode switching (torque ↔ openloop)

### Phase 2: Velocity Ramping (Optional for Phase 1)

- [ ] Add `traj_f32` for velocity ramping
- [ ] Implement acceleration limits
- [ ] Add shell commands: `motor velocity ramp`
- [ ] Add shell commands: `motor velocity limit`
- [ ] Test smooth velocity transitions

### Phase 3: Closed-Loop Velocity (Future)

**PI Implementation:**
- [ ] Create `velocity_pi.h` with controller struct
- [ ] Create `velocity_pi.c` with API implementation
- [ ] Implement `velocity_pi_init()` with auto-tuning
- [ ] Implement `velocity_pi_update()` with feedback
- [ ] Implement `velocity_pi_set_bandwidth()` for tuning

**Integration:**
- [ ] Add `velocity_pi_controller` to `velocity_impl` union
- [ ] Add `MOTOR_STATE_ONLINE_VELOCITY_CLOSED` substate
- [ ] Integrate encoder/observer feedback in ISR
- [ ] Add to `CMakeLists.txt`: `src/velocity_pi.c`

**Shell Commands:**
- [ ] Add shell commands: `motor velocity bandwidth`
- [ ] Add shell commands: `motor velocity pid get`
- [ ] Add shell commands: `motor velocity pid set`

**Testing:**
- [ ] Test closed-loop stability at various speeds
- [ ] Test load disturbance rejection
- [ ] Test bumpless transfer (openloop → closed)
- [ ] Validate bandwidth/gain relationship

## Notes and Considerations

### Open-Loop Limitations
- No load compensation (will slip under load)
- Requires accurate pole pair count
- No position tracking (angle drifts over time)
- Can't detect stalls or blockages
- Limited to low speeds for stability

### When to Use Open-Loop
- ✅ Initial motor characterization
- ✅ RLS parameter estimation testing
- ✅ Low-load applications (fans, pumps)
- ✅ Non-critical positioning (doesn't need to be exact)
- ❌ High-precision positioning
- ❌ Variable loads
- ❌ Safety-critical applications

### Transition to Closed-Loop
Once encoder is available:
1. Run open-loop at low speed
2. Switch to closed-loop mode (bumpless transfer)
3. Velocity controller takes over using encoder feedback
4. Gradually increase velocity to test stability

### Safety Considerations
- Overcurrent protection remains active in all modes
- Velocity limits enforced
- Watchdog timers for runaway detection
- Emergency stop always available
- RLS gating protects against bad estimates

## References

- SimpleFOC open-loop velocity: https://docs.simplefoc.com/velocity_openloop
- PMSM field-oriented control: Texas Instruments SPRABQ8
- Sensorless control methods: Application Note AN1946 (Microchip)
- RLS parameter estimation: "Recursive Identification and Parameter Estimation" by Goodwin & Sin
