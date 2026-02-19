# Online RLS Parameter Estimation for Motor Control

## Overview

This document describes the approach for continuous online parameter estimation using PRBS (Pseudo-Random Binary Sequence) injection and RLS (Recursive Least Squares) estimation. The goal is to track motor parameters (Rs, Ld, Lq) and system characteristics in real-time during normal operation.

**Measurement Philosophy**: All measurements leverage the PRBS injection signal that's already running for RLS. This document focuses on what can be measured without additional test sequences or dedicated excitation signals.

**Target Application: Hybrid Stepper Motor**
- High saliency (Ld ≠ Lq) → 3-parameter RLS essential
- No direct voltage measurement → Vd/Vq computed from PWM duty cycles + Vbus
- DOB/ESO used in velocity controller for disturbance/load estimation

**Available Measurements:**
- Phase currents: Ia, Ib (ADC)
- Bus voltage: Vbus (ADC)
- Computed: Vd, Vq (from duty cycles and Vbus, with optional dead-time compensation)

## Architecture

### Dual-Equation RLS with Staggered Decimation

**Strategy**: Run separate RLS estimators for d-axis and q-axis equations, but **stagger their execution** to avoid computational bursts.

**D-axis equation** (estimates Rs, Ld, Vbias, Vdt_sign):
```
Vd = Rs*Id + Ld*(dId/dt) + Vbias + Vdt_sign*sign(Id) - ω*Lq*Iq
```

**Q-axis equation** (estimates Rs, Lq, Vbias, Vdt_sign):
```
Vq = Rs*Iq + Lq*(dIq/dt) + Vbias + Vdt_sign*sign(Iq) + ω*Ld*Id
```

**Staggered execution** (configurable decimation from devicetree):
```c
// Config values from devicetree (rubus,user-parameters):
// rls-decimation = <16>;           // RLS update rate: ISR_freq / 16
// rls-stagger-offset = <8>;        // Q-axis offset for load spreading
// thermal-decimation = <2048>;     // Thermal model rate: ~10Hz at 20kHz ISR

const uint32_t rls_mask = params->rls_decimation - 1;        // e.g., 16-1 = 0xF
const uint32_t rls_offset = params->rls_stagger_offset;      // e.g., 8
const uint32_t thermal_mask = params->thermal_decimation - 1; // e.g., 2048-1 = 0x7FF

// PRBS injection: counts 0, 16, 32, 48, ...
if ((counter & rls_mask) == 0) {
    uint32_t prbs_bit = prbs_advance(&params->prbs_gen);  // Returns 0 or 1
    float32_t V_prbs = (2.0f * (float32_t)prbs_bit - 1.0f) * params->prbs_amplitude_V;  // Branchless: ±voltage
}

// D-axis PI controller with PRBS feedforward
pi_run_series(&params->pi_Id, Id_ref_A, Id_A, V_prbs, &Vd_V);

// Q-axis PI controller
pi_run_series(&params->pi_Iq, Iq_ref_A, Iq_A, 0.0f, &Vq_V);

// D-axis RLS update (after PI controllers to minimize control latency)
if ((counter & rls_mask) == 0) {
    // Uses params->Id_A from previous cycle for derivative calculation
    rls_motor_est_update(&params->rls_d, Vd_V, Id_A, params->Id_A, omega_elec,
                         params->Lq_est, Iq_A);  // Cross-coupling compensation
}

// Q-axis RLS: counts 8, 24, 40, 56, ... (staggered by offset, after PI controllers)
if ((counter & rls_mask) == rls_offset) {
    // Uses params->Iq_A from previous cycle for derivative calculation
    rls_motor_est_update(&params->rls_q, Vq_V, Iq_A, params->Iq_A, omega_elec,
                         params->Ld_est, Id_A);  // Cross-coupling compensation
}

// Thermal model: counts 0, 2048, 4096, ... (~10Hz at 20kHz ISR)
if ((counter & thermal_mask) == 0) {
    thermal_model_update(&params->thermal_model, Id_A, Iq_A, Rs_est, T_ambient);
}
```

**Benefits**:
- **Load spreading**: Peak computation never exceeds ~800 cycles in one ISR
- **Both Ld and Lq tracked**: Essential for high-saliency hybrid steppers
- **Independent convergence**: Each axis converges at its own rate
- **Flexible**: Can disable q-axis RLS for non-salient motors (set omega=0)

**Parameter synthesis**:
```c
// After both converge, average Rs estimates from both axes
float Rs_est = (rls_d.theta[0] + rls_q.theta[0]) / 2.0f;

// Use axis-specific inductances
float Ld_est = rls_d.theta[1];
float Lq_est = rls_q.theta[1];

// Average Vbias (should be similar from both)
float Vbias_est = (rls_d.theta[2] + rls_q.theta[2]) / 2.0f;
```

### Why RLS Instead of Kalman Filtering?

**RLS is optimal for this application because the motor voltage equation is linear in the unknown parameters:**

**Motor voltage equation (d-axis):**
```
Vd = Rs*Id + Ld*(dId/dt) + Vbias + Vdt_sign*sign(Id) - ω*Lq*Iq
     └─────────────────────────────────────────────┘
              Linear in parameters [Rs, Ld, Vbias, Vdt_sign]
```

**This is a linear regression problem**, not a state estimation problem:
- **Parameters are unknown constants** (or slowly varying): Rs, Ld, Vbias, Vdt_sign
- **Measurements are known**: Vd (from PI output), Id, dId/dt, ω, Iq
- **Relationship is linear**: `y = φᵀθ + noise`, perfect for RLS

**Comparison:**

| Method | Best For | Pros | Cons | Complexity |
|--------|----------|------|------|------------|
| **RLS** | **Parameter estimation** | Direct parameter update, simple, efficient | Assumes linear model | Low (~500 cycles) |
| **Kalman Filter** | State estimation | Optimal for dynamic states | Requires state-space model | Medium (~1000 cycles) |
| **Extended Kalman Filter (EKF)** | Nonlinear state estimation | Handles nonlinearities | Needs Jacobians, complex, expensive | High (~2000+ cycles) |
| **Unscented Kalman Filter (UKF)** | Nonlinear state estimation | No Jacobians needed | Sigma points expensive | Very High (~5000+ cycles) |

**Why NOT Kalman Filter for this problem:**

1. **No state dynamics**: Parameters (Rs, Ld) are constants, not states evolving over time
   - Kalman requires state transition model: `x[k+1] = A*x[k] + w`
   - For constant parameters: `A = I`, which reduces back to RLS

2. **Linear measurement model**: Voltage equation is already linear in parameters
   - No need for EKF's nonlinear prediction/update
   - RLS directly solves: `y = φᵀθ` (linear regression)

3. **Computational efficiency**: RLS exploits linearity
   - RLS: Direct parameter update via Kalman gain
   - KF: Would add unnecessary state prediction step
   - EKF: Would linearize already-linear equation (wasteful)

4. **Forgetting factor**: RLS handles slow parameter drift naturally
   - λ < 1 gives exponential weighting to recent data
   - Equivalent to time-varying process noise in KF (but simpler)

**When Kalman Filter WOULD be appropriate:**
- Estimating rotor position/velocity (dynamic states) → use angle observer instead
- Estimating flux linkage (state with dynamics) → requires back-EMF, not available here
- Simultaneous state + parameter estimation → might use dual EKF (but expensive)

**Conclusion**: RLS is the right tool for this job - parameter estimation from linear regression with measurement noise.

### Simplified Startup Sequence

**Offline Anchor (One-time calibration at known temperature):**
- Measure **Rs0** at **T0** (e.g., 25°C) using DC injection
- Store `(Rs0, T0)` in non-volatile memory
- Use copper temperature coefficient `α = 0.00393 /°C`
- This provides absolute temperature reference for all future Rs measurements

**Each startup sequence:**
1. **OFFSET_MEAS** (1s): Current sensor offset calibration
2. **RS_EST** (optional, 2s): Quick Rs check (validates against Rs0 scaled for ambient temperature)
   - Can be skipped if ambient temp unknown - RLS will converge within 10-15s anyway
3. **~~ROVERL_MEAS~~** (removed): RLS estimates Ld/Lq online, no dedicated test needed
4. **ALIGN** (1s): Rotor alignment to known position
5. **IDLE**: Ready for operation
6. **ONLINE**: Normal FOC with continuous RLS estimation running in background

**Total startup time: ~1-3 seconds** (vs 5-8s with ROVERL_MEAS, 10-20s for full TI sequence)
**RLS convergence: ~10-15 seconds** after entering ONLINE state (happens during normal operation)

### Core Components

#### 1. PRBS Generator (`prbs.h/c`)
- **Simple LFSR generator** - advances state and returns ±amplitude
- **No internal decimation** - ISR calls `prbs_advance()` at desired rate
- Output: ±1 binary sequence scaled to voltage amplitude
- Configurable amplitude: 0.5-2V typical (chosen to give 0.1-0.2A current excitation)
- **ISR handles timing**: Decimation by 16-32 from 20kHz (power-of-2 for efficiency)
- Bit duration: Each PRBS bit held for 16-32 control cycles (via ISR decimation)
- Injection: **Voltage feedforward** (4th parameter to PI controller)
  - `Vd_output = Vd_PI + Vd_prbs` (direct voltage injection)
  - Bypasses PI dynamics for full bandwidth excitation
  - Preserves reference tracking via PI feedback
- Minimizes torque ripple (d-axis voltage doesn't directly produce torque)
- **Based on liquid-dsp msequence.c** - proven implementation

#### 2. RLS Motor Estimator (`rls_motor_est.h/c`)
- **Update rate: 1-2 kHz** (decimated by 16-32 from 20kHz ISR) - reduces computational load
- **Power-of-2 decimation** enables efficient bit-mask implementation
- **4-parameter per axis: Rs, L, Vbias, Vdt_sign** (document implementation)
  - D-axis estimates: Rs, Ld, Vbias, Vdt_sign
  - Q-axis estimates: Rs, Lq, Vbias, Vdt_sign
  - `Vbias`: captures dead-time, diode drops, ADC offsets
  - `Vdt_sign * sign(I)`: captures current-sign-dependent voltage drop
- **Dual-axis approach for hybrid steppers** with significant Ld ≠ Lq
  - Separate RLS estimators run in staggered fashion (load spreading)
  - Cross-coupling compensation: -ω*Lq*Iq for d-axis, +ω*Ld*Id for q-axis
  - Parameter synthesis: average Rs from both axes, use axis-specific L
- Forgetting factor: λ = 0.999 to 0.9999 (slow tracking for thermal Rs drift)
- Convergence detection: monitor trace(P) < threshold per axis
- Residual tracking for diagnostic purposes

#### 3. Voltage Source for RLS
**Key Simplification**: Use PI controller outputs directly, not PWM reconstruction.

```c
// Use the COMMANDED voltages from PI controller (post-limiter)
Vd_measured = Vd_cmd;  // PI output for d-axis (after saturation)
Vq_measured = Vq_cmd;  // PI output for q-axis (after saturation)
```

**Why this is better:**
- **No dead-time compensation needed** - inverter non-linearities captured by Vbias parameter
- **Simpler implementation** - direct access to control variables
- **More accurate** - avoids PWM reconstruction errors
- **Let RLS learn the bias** - voltage offsets become estimated parameters

**Critical**: Must use post-limiter/post-saturation PI outputs, not pre-saturation values.

### State Machine Integration

**Current State Sequence:**
1. **OFFSET_MEAS** (1s): Current sensor offset calibration
2. **RS_EST** (2s): Initial Rs measurement via DC injection
3. **ROVERL_MEAS** (optional): R/L measurement if needed
4. **ALIGN**: Rotor alignment
5. **IDLE**: Ready for operation
6. **ONLINE**: Normal FOC with continuous RLS estimation

**RLS runs during ONLINE state** - continuous parameter tracking in background

#### 4. Thermal Model (`thermal_model.h/c`)
**Independent temperature monitoring module** - provides physics-based winding temperature estimate:

**Initialization:**
```c
thermal_model_init(&params->thermal_model,
                   CONTROL_LOOP_FREQUENCY_HZ,  // For decimation
                   thermal_time_constant_s,     // From devicetree, e.g., 180s
                   ambient_temp_C,              // Initial condition, e.g., 25°C
                   Rs0_ohm,                     // Calibration resistance
                   T0_C,                        // Calibration temperature
                   copper_temp_coeff);          // 0.00393/°C
```

**Update (called from ISR, decimated to 1-10 Hz):**
```c
thermal_model_update(&params->thermal_model,
                     Id_A,        // D-axis current
                     Iq_A,        // Q-axis current  
                     Rs_est_ohm,  // From RLS (real-time resistance)
                     ambient_temp_C);  // Optional: external sensor
// Returns: T_winding_thermal_C
```

**Features:**
- First-order thermal dynamics: τ(dT/dt) = I²R - (T - T_ambient)
- Uses RLS Rs estimate for accurate I²R power calculation
- Independent from RLS for validation/redundancy
- Runs at lower rate (1-10 Hz) to reduce computational load
- Compared against RLS temperature for diagnostic monitoring

## Gating and Validity Checks (Critical)

The RLS estimator **must freeze** during conditions that violate the assumed motor model. Poor gating leads to garbage parameter estimates.

### Always Freeze During:
1. **Braking/special PWM modes** - voltage model changes during Vbus-regulated braking
2. **Current PI saturation** - commanded voltage ≠ applied voltage
3. **Calibration states** - OFFSET_MEAS, RS_EST, ROVERL_MEAS, ALIGN
4. **Fault conditions** - overcurrent, overvoltage, encoder invalid
5. **State transitions** - wait for steady-state after mode changes

### Observability Requirements:
```c
// Only update RLS when signals are sufficiently excited
bool observable = (fabsf(Id) > ID_MIN) &&           // e.g., 0.5 × PRBS amplitude
                  (fabsf(dId_dt) > DID_DT_MIN) &&   // PRBS actually visible
                  (fabsf(omega_e) > OMEGA_MIN);     // Optional: avoid standstill bias

if (!observable) {
    // Skip RLS update - insufficient excitation
    return;
}
```

### Sanity Bounds:
```c
// Clamp parameters to physically plausible ranges
Rs_est = clampf(Rs_est, RS_MIN, RS_MAX);     // e.g., 0.5 to 50 Ω
Ld_est = clampf(Ld_est, LD_MIN, LD_MAX);     // e.g., 0.1 to 100 mH
Vbias_est = clampf(Vbias_est, -5.0f, 5.0f);  // e.g., ±5V

// Reject outlier measurements
float y_max = 0.8f * Vbus;  // Sanity check on voltage magnitude
if (fabsf(y) > y_max) {
    // Skip this update - measurement unrealistic
    return;
}
```

### Quality Metrics to Log:
- Residual: `e[k] = y[k] - φ[k]ᵀ * θ[k-1]`
- Normalized residual: `e / (ε + ||φ||)`
- Covariance diagonal: `P[0,0], P[1,1], P[2,2]` (confidence)
- Update acceptance rate: `accepted_updates / total_attempts`

## Data Structures

### PRBS Generator
```c
/**
 * @brief Pseudo-Random Binary Sequence generator using 12-bit maximal-length LFSR
 * 
 * Generates [0,1] sequence with 4095-sample period
 * Pure binary sequence generator - ISR converts to ±voltage via: 2*n - 1
 * 
 * Polynomial: x^12 + x^11 + x^10 + x^3 + 1 (maximal length, all frequencies)
 * Generator polynomial: 0x0e08 (bits 12,11,10,3 set) - matches liquid-dsp
 * 
 * Based on liquid-dsp msequence implementation
 */
struct prbs_gen {
    uint32_t m;              // Shift register length (12 for this application)
    uint32_t g;              // Generator polynomial (0x0e08 for 12-bit)
    uint32_t a;              // Initial state (typically 1)
    uint32_t state;          // Current shift register state
    uint32_t n;              // Sequence length: (2^m - 1) = 4095
};

/* Generator polynomial for 12-bit maximal-length sequence
 * Polynomial: x^12 + x^11 + x^10 + x^3 + 1 (liquid-dsp standard)
 * Binary representation: bits 12,11,10,3 set
 * Hex: 0x0e08 = 0b1110_0000_1000
 */
#define PRBS_GENPOLY_M12  0x0e08u

/**
 * API functions (following liquid-dsp msequence.c patterns):
 * 
 * void prbs_init(struct prbs_gen *prbs)
 *   Initialize generator with 12-bit polynomial (0x0e08, liquid-dsp)
 *   Initial state set to 1 (standard practice)
 * 
 * uint32_t prbs_advance(struct prbs_gen *prbs)
 *   Advance LFSR state using binary dot product feedback
 *   Returns: 0 or 1 (ISR converts to ±voltage via: 2*n - 1)
 *   Implementation: bit = bdotprod(state, g); state = (state << 1) | bit
 * 
 * uint32_t prbs_get_output(const struct prbs_gen *prbs)
 *   Get current output without advancing (0 or 1)
 *   Based on LSB of current state
 * 
 * void prbs_reset(struct prbs_gen *prbs)
 *   Reset state to initial value (1)
 */

/**
 * Example implementation (prbs.c):
 */
void prbs_init(struct prbs_gen *prbs)
{
    prbs->m = 12;
    prbs->g = PRBS_GENPOLY_M12;     // 0x0e08 (liquid-dsp)
    prbs->a = 1;                     // Initial state
    prbs->n = (1u << prbs->m) - 1;   // Sequence length: 4095
    prbs->state = prbs->a;
}

uint32_t prbs_advance(struct prbs_gen *prbs)
{
    // Binary dot product: XOR parity of (state & g)
    uint32_t b = __builtin_popcount(prbs->state & prbs->g) & 1;
    
    prbs->state <<= 1;           // Shift register left
    prbs->state |= b;            // Push feedback bit
    prbs->state &= prbs->n;      // Mask to sequence length
    
    // Return 0 or 1 (ISR converts to ±voltage using: 2*n - 1)
    return b;
}

uint32_t prbs_get_output(const struct prbs_gen *prbs)
{
    // Return current output (0 or 1) based on LSB without advancing
    return prbs->state & 1;
}

void prbs_reset(struct prbs_gen *prbs)
{
    prbs->state = prbs->a;  // Reset to initial state
}
```

### RLS Motor Estimator
```c
/**
 * @brief RLS estimator for motor parameter identification
 * 
 * Estimates θ = [Rs, L, Vbias, Vdt_sign]ᵀ where:
 * - Rs: Winding resistance (Ω)
 * - L: Inductance (H) - Ld for d-axis RLS, Lq for q-axis RLS
 * - Vbias: Voltage bias/offset (V)
 * - Vdt_sign: Sign-dependent voltage drop (V)
 */
struct rls_motor_est {
    /* Parameters being estimated */
    float32_t theta[4];              // [Rs, L, Vbias, Vdt_sign]
    
    /* Covariance matrix (symmetric, only upper triangle stored) */
    float32_t P[4][4];               // Parameter covariance
    float32_t P_min;                 // Minimum eigenvalue floor (1e-6)
    
    /* Configuration */
    float32_t lambda;                // Forgetting factor (0.9999 typical)
    float32_t control_freq;          // Control loop frequency (Hz)
    
    /* Temperature calibration (for Rs → temperature conversion) */
    float32_t Rs0_ohm;               // Resistance at reference temperature
    float32_t T0_C;                  // Reference temperature (°C)
    float32_t alpha;                 // Temperature coefficient (1/°C, ~0.00393 for copper)
    
    /* Convergence detection */
    bool converged;              // True when trace(P) < threshold
    float32_t convergence_threshold; // Convergence criterion for trace(P)
    uint32_t convergence_count;  // Update count at convergence
    
    /* Statistics */
    uint32_t num_updates;        // Total RLS updates performed
    uint32_t num_rejected;       // Updates rejected (numerical issues)
    float32_t residual_sum_sq;       // Sum of squared residuals (for RMS)
};
```

### Thermal Model
```c
/**
 * @brief First-order thermal model for winding temperature estimation
 * 
 * Model: τ * dT/dt = P_loss - (T - T_ambient) / R_th
 * Where: P_loss = I²*Rs (updated with RLS Rs estimate)
 */
struct thermal_model {
    /* State */
    float32_t T_winding_C;           // Estimated winding temperature (°C)
    
    /* Thermal parameters */
    float32_t tau_s;                 // Thermal time constant (seconds, ~120-300s typical)
    float32_t R_th;                  // Thermal resistance (°C/W)
    float32_t C_th;                  // Thermal capacitance (J/°C)
    float32_t T_ambient_C;           // Ambient temperature (°C)
    
    /* Configuration */
    float32_t control_freq;          // Control loop frequency (Hz)
    float32_t dt;                    // Integration timestep (1/control_freq)
    
    /* Temperature calibration (for I²R calculation) */
    float32_t Rs0_ohm;               // Resistance at reference temperature
    float32_t T0_C;                  // Reference temperature (°C)
    float32_t alpha;                 // Temperature coefficient (1/°C)
    
    /* Cross-validation */
    float32_t validation_threshold_C; // Max allowed difference vs RLS temperature (°C)
    bool validation_warning;      // True if RLS vs thermal discrepancy exceeds threshold
};
```

## RLS Implementation

### D-axis Voltage Equation (No Flux Term)
```
Vd = Rs*Id - ω*Lq*Iq + Ld*(dId/dt)
```

For estimation, discretize derivative:
```
Vd[k] = Rs*Id[k] + Ld*(Id[k] - Id[k-1])/Ts - ω*Lq*Iq[k]
```

### Parameter Vector (3-parameter - Recommended)
```
θ = [Rs, Ld, Vbias]ᵀ
```

### Regression Vector
```
φ[k] = [Id[k], (Id[k] - Id[k-1])/Ts, 1]ᵀ
```

**Optional 4-parameter (sign-dependent deadtime):**
```
θ = [Rs, Ld, Vbias, Vdt_sign]ᵀ
φ[k] = [Id[k], (Id[k] - Id[k-1])/Ts, 1, sign(Id[k])]ᵀ
```

### RLS Update Equations (Decimated)

**Run every N ISR cycles** (N = power-of-2, e.g., 16 or 32):
```c
// Check if update cycle (efficient bit-mask for power-of-2 decimation)
if ((isr_counter & RLS_DECIMATION_MASK) == 0) {
    // Prediction error
    e[k] = Vd_measured[k] - φ[k]ᵀ * θ[k-1]
    
    // Gain vector
    K[k] = P[k-1] * φ[k] / (λ + φ[k]ᵀ * P[k-1] * φ[k])
    
    // Parameter update
    θ[k] = θ[k-1] + K[k] * e[k]
    
    // Covariance update
    P[k] = (I - K[k] * φ[k]ᵀ) * P[k-1] / λ
}
```

**Note on Decimation Efficiency:**
- Power-of-2 decimation (16, 32, 64) allows bit-mask check: `(counter & RLS_DECIMATION_MASK) == 0`
- Non-power-of-2 (e.g., 20) requires modulo: `(counter % N) == 0` (slower)
- Recommended: Use N=16 (1.25kHz) or N=32 (625Hz) for optimal efficiency

### RLS Implementation Example (4-Parameter)

**For small systems (≤4 parameters), explicit code is recommended over CMSIS-DSP:**
- **More readable**: Clear what each operation does
- **Faster**: No function call overhead, better cache locality
- **Easier to debug**: Can inspect intermediate values
- **More maintainable**: Self-documenting code

**Complete RLS update implementation:**

```c
/**
 * @brief RLS parameter update for motor estimation
 * 
 * Estimates θ = [Rs, Ld, Vbias, Vdt_sign]ᵀ from voltage equation:
 * Vd = Rs*Id + Ld*dId/dt + Vbias + Vdt_sign*sign(Id) - ω*Lq*Iq
 * 
 * @param rls         RLS estimator state
 * @param V_meas      Measured voltage (Vd or Vq from PI output)
 * @param I           Current measurement (Id or Iq)
 * @param I_prev      Previous current (from params->Id_A or params->Iq_A)
 * @param omega       Electrical angular velocity (rad/s)
 * @param L_cross     Cross-coupling inductance (Lq for d-axis, Ld for q-axis)
 * @param I_cross     Cross-coupling current (Iq for d-axis, Id for q-axis)
 */
void rls_motor_est_update(struct rls_motor_est *rls,
                          float32_t V_meas, float32_t I, float32_t I_prev, float32_t omega,
                          float32_t L_cross, float32_t I_cross)
{
    /* Compensate cross-coupling term: V_compensated = V_meas + ω*L_cross*I_cross */
    float32_t V_compensated = V_meas + omega * L_cross * I_cross;
    
    /* Calculate current derivative internally */
    float32_t dI_dt = (I - I_prev) * rls->control_freq;
    
    /* Build regression vector φ[k] = [I, dI/dt, 1, sign(I)]ᵀ */
    float32_t phi[4];
    phi[0] = I;                           // Rs coefficient
    phi[1] = dI_dt;                       // L coefficient (Ld or Lq)
    phi[2] = 1.0f;                        // Vbias coefficient
    phi[3] = (I >= 0.0f) ? 1.0f : -1.0f;  // Vdt_sign coefficient
    
    /* Predicted voltage: y_pred = φᵀ * θ */
    float32_t y_pred = phi[0] * rls->theta[0] +  // Rs * I
                       phi[1] * rls->theta[1] +  // L * dI/dt
                   phi[2] * rls->theta[2] +  // Vbias
                   phi[3] * rls->theta[3];   // Vdt_sign * sign(I)
    
    /* Prediction error (using compensated voltage) */
    float32_t error = V_compensated - y_pred;
    
    /* Compute P * φ (4x4 * 4x1 = 4x1)
     * P is symmetric, only upper triangle stored for efficiency
     */
    float32_t P_phi[4];
    P_phi[0] = rls->P[0][0] * phi[0] + rls->P[0][1] * phi[1] + 
               rls->P[0][2] * phi[2] + rls->P[0][3] * phi[3];
    P_phi[1] = rls->P[0][1] * phi[0] + rls->P[1][1] * phi[1] + 
               rls->P[1][2] * phi[2] + rls->P[1][3] * phi[3];
    P_phi[2] = rls->P[0][2] * phi[0] + rls->P[1][2] * phi[1] + 
               rls->P[2][2] * phi[2] + rls->P[2][3] * phi[3];
    P_phi[3] = rls->P[0][3] * phi[0] + rls->P[1][3] * phi[1] + 
               rls->P[2][3] * phi[2] + rls->P[3][3] * phi[3];
    
    /* Denominator: λ + φᵀ * P * φ (scalar) */
    float32_t phi_P_phi = phi[0] * P_phi[0] + phi[1] * P_phi[1] + 
                          phi[2] * P_phi[2] + phi[3] * P_phi[3];
    float32_t denom = rls->lambda + phi_P_phi;
    
    /* Guard against numerical issues */
    if (denom < 1e-6f) {
        rls->num_rejected++;
        return;  // Skip update
    }
    
    float32_t denom_inv = 1.0f / denom;
    
    /* Kalman gain: K = (P * φ) / denom */
    float32_t K[4];
    K[0] = P_phi[0] * denom_inv;
    K[1] = P_phi[1] * denom_inv;
    K[2] = P_phi[2] * denom_inv;
    K[3] = P_phi[3] * denom_inv;
    
    /* Update parameters: θ = θ + K * error */
    rls->theta[0] += K[0] * error;  // Rs
    rls->theta[1] += K[1] * error;  // Ld
    rls->theta[2] += K[2] * error;  // Vbias
    rls->theta[3] += K[3] * error;  // Vdt_sign
    
    /* Apply parameter bounds (prevent divergence) */
    rls->theta[0] = clampf(rls->theta[0], RS_MIN_OHM, RS_MAX_OHM);
    rls->theta[1] = clampf(rls->theta[1], LD_MIN_H, LD_MAX_H);
    rls->theta[2] = clampf(rls->theta[2], -VBIAS_MAX_V, VBIAS_MAX_V);
    rls->theta[3] = clampf(rls->theta[3], -VDT_SIGN_MAX_V, VDT_SIGN_MAX_V);
    
    /* Update covariance: P = (P - K * φᵀ * P) / λ
     * Expanded: P_new[i][j] = (P[i][j] - K[i] * phi[0] * P[0][j] 
     *                                    - K[i] * phi[1] * P[1][j]
     *                                    - K[i] * phi[2] * P[2][j]
     *                                    - K[i] * phi[3] * P[3][j]) / λ
     * 
     * Optimized: Only update upper triangle (P is symmetric)
     */
    float32_t lambda_inv = 1.0f / rls->lambda;
    
    /* Row 0 */
    rls->P[0][0] = (rls->P[0][0] - K[0] * P_phi[0]) * lambda_inv;
    rls->P[0][1] = (rls->P[0][1] - K[0] * P_phi[1]) * lambda_inv;
    rls->P[0][2] = (rls->P[0][2] - K[0] * P_phi[2]) * lambda_inv;
    rls->P[0][3] = (rls->P[0][3] - K[0] * P_phi[3]) * lambda_inv;
    
    /* Row 1 (skip [1][0], use symmetry) */
    rls->P[1][1] = (rls->P[1][1] - K[1] * P_phi[1]) * lambda_inv;
    rls->P[1][2] = (rls->P[1][2] - K[1] * P_phi[2]) * lambda_inv;
    rls->P[1][3] = (rls->P[1][3] - K[1] * P_phi[3]) * lambda_inv;
    
    /* Row 2 */
    rls->P[2][2] = (rls->P[2][2] - K[2] * P_phi[2]) * lambda_inv;
    rls->P[2][3] = (rls->P[2][3] - K[2] * P_phi[3]) * lambda_inv;
    
    /* Row 3 */
    rls->P[3][3] = (rls->P[3][3] - K[3] * P_phi[3]) * lambda_inv;
    
    /* Enforce minimum covariance (prevent over-confidence) */
    for (int i = 0; i < 4; i++) {
        if (rls->P[i][i] < P_MIN) {
            rls->P[i][i] = P_MIN;
        }
    }
    
    /* Update statistics */
    rls->num_updates++;
    rls->residual_sum_sq += error * error;  // For RMS calculation
    rls->residual = error;  // Store latest residual
    
    /* Convergence detection: Check if covariance trace is below threshold */
    float32_t trace_P = rls->P[0][0] + rls->P[1][1] + rls->P[2][2] + rls->P[3][3];
    
    if (!rls->converged) {
        /* Check convergence: trace(P) < threshold */
        if (trace_P < rls->convergence_threshold) {
            rls->converged = true;
            rls->convergence_count = rls->num_updates;
        }
    } else {
        /* Check divergence: trace(P) significantly increased (with hysteresis) */
        if (trace_P > rls->convergence_threshold * 2.0f) {
            rls->converged = false;
        }
    }
}
```

**Key Implementation Notes:**

1. **Symmetry exploitation**: P is symmetric, only store/update upper triangle
   - Saves ~50% memory (10 floats instead of 16)
   - Saves ~50% computation in covariance update

2. **Explicit operations**: All matrix operations written out
   - No CMSIS-DSP function call overhead (~10-20 cycles per call)
   - Better compiler optimization (can inline, reorder, vectorize)
   - Easier to inspect in debugger

3. **Guard conditions**:
   - Division by zero check before computing gain
   - Minimum covariance floor prevents over-confidence
   - Parameter bounds prevent divergence

4. **Computational cost**: ~500-800 cycles for 4-parameter RLS
   - 4 multiplies + 4 adds: prediction (8 ops)
   - 4×4 matrix-vector multiply: P*φ (16 multiplies, 12 adds)
   - 4 dot products + division: gain (20 ops)
   - 4 parameter updates (8 ops)
   - 10 covariance updates (20 ops)
   - Total: ~74 operations + overhead ≈ 500-800 cycles

**Alternative: CMSIS-DSP Implementation** (for reference, not recommended for 4-param):

```c
/* Using CMSIS-DSP matrix functions (more code, slower for small matrices) */
void rls_motor_est_update_cmsis(struct rls_motor_est *rls, ...) {
    arm_matrix_instance_f32 mat_P = {4, 4, (float32_t *)rls->P};
    arm_matrix_instance_f32 mat_phi = {4, 1, phi};
    arm_matrix_instance_f32 mat_P_phi = {4, 1, P_phi};
    
    /* P * φ */
    arm_mat_mult_f32(&mat_P, &mat_phi, &mat_P_phi);
    
    /* φᵀ * P * φ (dot product) */
    float32_t phi_P_phi;
    arm_dot_prod_f32(phi, P_phi, 4, &phi_P_phi);
    
    /* ... rest of implementation */
}
```

**Recommendation**: Use explicit implementation for 4-parameter system. CMSIS-DSP adds overhead without benefit at this scale.

### 4-Parameter Implementation (High Saliency Stepper Motor)
**Hybrid stepper motors have significant saliency** - both inductances must be estimated:

**Parameter vector:**
```
θ = [Rs, Ld, Lq, Vbias]ᵀ
```

**Use both voltage equations simultaneously:**

**D-axis:**
```
Vd[k] = Rs*Id[k] + Ld*(Id[k] - Id[k-1])/Ts - ω[k]*Lq*Iq[k] + Vbias
```

**Q-axis:**
```
Vq[k] = Rs*Iq[k] + Lq*(Iq[k] - Iq[k-1])/Ts + ω[k]*Ld*Id[k] + Vbias
```

**Regression vectors:**
```
φ_d[k] = [Id[k], (Id[k] - Id[k-1])/Ts, -ω[k]*Iq[k], 1]ᵀ
φ_q[k] = [Iq[k], (Iq[k] - Iq[k-1])/Ts,  ω[k]*Id[k], 1]ᵀ
```

**RLS Update** (process both equations each cycle for faster convergence):
```c
// D-axis update
e_d[k] = Vd_measured[k] - φ_d[k]ᵀ * θ[k-1]
K_d[k] = P[k-1] * φ_d[k] / (λ + φ_d[k]ᵀ * P[k-1] * φ_d[k])
θ_temp = θ[k-1] + K_d[k] * e_d[k]
P_temp = (I - K_d[k] * φ_d[k]ᵀ) * P[k-1] / λ

// Q-axis update (using temp values)
e_q[k] = Vq_measured[k] - φ_q[k]ᵀ * θ_temp
K_q[k] = P_temp * φ_q[k] / (λ + φ_q[k]ᵀ * P_temp * φ_q[k])
θ[k] = θ_temp + K_q[k] * e_q[k]
P[k] = (I - K_q[k] * φ_q[k]ᵀ) * P_temp / λ
```

This dual-equation approach improves convergence speed by ~2x.

## RLS Residual Analysis

The RLS residual `e[k] = Vd_measured - Vd_predicted` contains rich diagnostic information beyond just parameter estimation error.

### 1. PI Saturation Detection

**Mechanism**: When PI controllers saturate, commanded voltage ≠ achievable voltage
- Residual shows persistent bias (not zero-mean)
- Magnitude correlates with saturation level

**Detection:**
```c
// Low-pass filter residual using fo_filter
// Initialize once: filter_fo_init(&filter_residual_dc);
// Set coefficients for ~1Hz cutoff at 20kHz: a1 = exp(-2π*1/20000) ≈ 0.9997
filter_fo_set_num_coeffs(&filter_residual_dc, 0.0003f, 0.0f);
filter_fo_set_den_coeffs(&filter_residual_dc, 0.9997f);

// Each cycle
filter_fo_step(&filter_residual_dc, fabsf(e[k]));
float residual_dc = filter_fo_get_y1(&filter_residual_dc);

if (residual_dc > SATURATION_THRESHOLD) {
    // PI controller saturated
    // Possible causes:
    // - Current demand too high
    // - Speed too high (back-EMF limiting)
    // - Vbus too low
}
```

**Action**: Reduce current demand, increase Vbus limit, or flag warning

### 2. ADC Offset Drift

**Mechanism**: Temperature-dependent ADC offset causes DC bias in current measurements
- Appears as slowly-varying DC component in residual
- Affects both d-axis and q-axis equations similarly

**Detection:**
```c
// Very slow filter (time constant ~10s) using fo_filter
// Initialize: filter_fo_init(&filter_offset_drift);
// For 10s time constant at 20kHz: fc = 1/(2π*10) ≈ 0.016 Hz
// a1 = exp(-2π*0.016/20000) ≈ 0.99999
filter_fo_set_num_coeffs(&filter_offset_drift, 0.00001f, 0.0f);
filter_fo_set_den_coeffs(&filter_offset_drift, 0.99999f);

// Each cycle
filter_fo_step(&filter_offset_drift, e[k]);
float offset_estimate = filter_fo_get_y1(&filter_offset_drift);

if (fabsf(offset_estimate) > OFFSET_DRIFT_THRESHOLD) {
    // ADC offset has drifted
    // Re-run offset calibration or apply correction
}
```

**Action**: 
- Online offset correction: `Id_measured_corrected = Id_adc - offset_estimate`
- Or flag for re-calibration during next stop

### 3. Dead-time Mis-compensation

**Mechanism**: Dead-time compensation model error causes voltage distortion
- Residual shows current-sign-dependent error
- Harmonic content at switching frequency

**Initial Dead-time Value**: Use gate driver IC specification
- DRV8328: 200ns typical (from datasheet)
- This is the hardware dead-time inserted by the gate driver
- Start with this value for compensation model

**Detection of Dead-time Error:**
```c
// Correlate residual with sign of current using fo_filter
// Initialize: filter_fo_init(&filter_corr_pos); filter_fo_init(&filter_corr_neg);
// Time constant ~1s: fc ≈ 0.16 Hz, a1 ≈ 0.99995
filter_fo_set_num_coeffs(&filter_corr_pos, 0.00005f, 0.0f);
filter_fo_set_den_coeffs(&filter_corr_pos, 0.99995f);
filter_fo_set_num_coeffs(&filter_corr_neg, 0.00005f, 0.0f);
filter_fo_set_den_coeffs(&filter_corr_neg, 0.99995f);

// Each cycle
filter_fo_step(&filter_corr_pos, e[k] * (Id > 0.0f ? 1.0f : 0.0f));
filter_fo_step(&filter_corr_neg, e[k] * (Id < 0.0f ? 1.0f : 0.0f));

float correlation_pos = filter_fo_get_y1(&filter_corr_pos);
float correlation_neg = filter_fo_get_y1(&filter_corr_neg);
float dead_time_error = correlation_pos - correlation_neg;

if (fabsf(dead_time_error) > DEADTIME_THRESHOLD) {
    // Dead-time compensation needs adjustment
    // Refine dead_time constant from initial IC datasheet value
}
```

**Action**: 
- Start with gate driver IC dead-time specification
- Use RLS residual analysis to refine estimate if needed
- Typical adjustment: ±20-50ns from nominal value

### 4. Phase Lag Detection

**Mechanism**: Delays in current sensing, computation, or PWM update cause phase lag between command and measurement
- Residual correlates with rate of change of current
- Increases with current bandwidth

**Detection:**
```c
// Correlate residual with current derivative using fo_filter
// Initialize: filter_fo_init(&filter_phase_lag);
// Very slow integration: time constant ~10s
filter_fo_set_num_coeffs(&filter_phase_lag, 0.00001f, 0.0f);
filter_fo_set_den_coeffs(&filter_phase_lag, 0.99999f);

// Each cycle
float dId_dt = (Id[k] - Id[k-1]) / Ts;
filter_fo_step(&filter_phase_lag, e[k] * dId_dt);
float phase_lag_indicator = filter_fo_get_y1(&filter_phase_lag);

if (fabsf(phase_lag_indicator) > PHASE_LAG_THRESHOLD) {
    // Significant phase lag detected
    // Possible causes:
    // - ADC sampling delay
    // - Computational delay
    // - PWM update delay
}
```

**Action**: Adjust sampling trigger point, optimize ISR timing, or add predictive correction

### 5. Residual Monitoring for System Health

**Overall residual magnitude** indicates model quality:
```c
// RMS calculation using fo_filter for mean-square averaging
// Initialize: filter_fo_init(&filter_residual_ms);
// Time constant ~5s for RMS: fc ≈ 0.032 Hz
filter_fo_set_num_coeffs(&filter_residual_ms, 0.00002f, 0.0f);
filter_fo_set_den_coeffs(&filter_residual_ms, 0.99998f);

// Each cycle
float e_squared = e[k] * e[k];
filter_fo_step(&filter_residual_ms, e_squared);
float residual_rms = sqrtf(filter_fo_get_y1(&filter_residual_ms));

if (residual_rms > HEALTH_THRESHOLD) {
    // Model mismatch detected
    // Possible causes:
    // - Thermal drift (Rs increased)
    // - Magnetic saturation (Ld decreased)
    // - Hardware fault (loose connection, partial short)
}
```

## Additional Online Measurements

### 1. Saliency Ratio (Ld/Lq)

**Method**: 3-parameter RLS estimates Ld and Lq independently

**Use cases:**
- Interior PMSMs (IPMSMs) have significant saliency
- Enables reluctance torque utilization
- MTPA (Maximum Torque Per Ampere) control

**Detection:**
```c
saliency_ratio = Ld_est / Lq_est

if (saliency_ratio > 1.2) {
    // Significant saliency - enable MTPA
}
```

### 2. Cross-Coupling (Id → Iq) - PRBS-Based

**Mechanism**: Non-ideal decoupling or magnetic saturation causes Id changes to affect Iq
- PRBS injection on Id naturally excites cross-coupling
- Monitor Iq controller error correlated with PRBS signal
- Residual in q-axis equation

**Detection:**
```c
// PRBS injection on Id automatically provides excitation for cross-coupling measurement
// Initialize: filter_fo_init(&filter_coupling);
// Time constant ~5s: fc ≈ 0.032 Hz
filter_fo_set_num_coeffs(&filter_coupling, 0.00002f, 0.0f);
filter_fo_set_den_coeffs(&filter_coupling, 0.99998f);

// Each cycle
filter_fo_step(&filter_coupling, Id_prbs[k] * Iq_error[k]);
float correlation_Id_Iq = filter_fo_get_y1(&filter_coupling);

if (fabsf(correlation_Id_Iq) > COUPLING_THRESHOLD) {
    // Significant cross-coupling
    // Improve decoupling compensation
    // Or adjust current controller structure
}
```

**Action**: 
- Improve decoupling terms in PI controllers
- Add feedforward compensation
- Use multi-variable control (MPC, H-infinity)

### 3. Reluctance Torque Measurement

**Mechanism**: For IPMSMs, torque has two components:
```
T = (3/2) * P * [λ_pm * Iq + (Ld - Lq) * Id * Iq]
         [Electromagnetic]  [Reluctance]
```

**Limitation**: Flux linkage (λ_pm) **cannot be measured with PRBS/RLS** due to lack of direct phase voltage measurement. Must use:
- Datasheet value, or
- Separate offline measurement (e.g., open-circuit back-EMF test)

**Measurement** (assuming flux linkage is known):
```c
// With known Ld, Lq from RLS, and flux linkage from datasheet/offline test
T_electromagnetic = (3.0/2.0) * POLE_PAIRS * flux_linkage * Iq
T_reluctance = (3.0/2.0) * POLE_PAIRS * (Ld - Lq) * Id * Iq
T_total = T_electromagnetic + T_reluctance

reluctance_fraction = T_reluctance / T_total
```

**Use case**: MTPA algorithms need accurate saliency to maximize efficiency. RLS provides Ld and Lq, but flux linkage must come from external source.

### 4. Encoder Lag Detection (PRBS-Based)

**Mechanism**: Encoder reading lags actual position due to:
- Processing delay (quadrature decoding, filtering)
- Transmission delay (SPI, SSI)
- Mechanical coupling compliance

**Back-EMF Method Not Available**: Cannot use back-EMF comparison since phase voltages are not directly measured.

**PRBS-Based Detection** (uses existing PRBS injection on Id):

**Principle**: The PRBS signal on Id creates torque disturbances that cause small position perturbations. If the encoder lags, there will be phase shift between the PRBS signal and the resulting current response.

```c
// Cross-correlate PRBS signal with current error at various delays using fo_filter
// Initialize filters for different delay taps
static struct filter_fo delay_filters[NUM_DELAY_TAPS];

// For each delay tap (e.g., 0, 1, 2, 3, 4 samples = 0-0.2ms at 20kHz)
for (int delay = 0; delay < NUM_DELAY_TAPS; delay++) {
    // Correlate PRBS with Id error at this delay
    // Time constant ~2s for stable correlation
    filter_fo_step(&delay_filters[delay], 
                   prbs_signal[k-delay] * (Id_ref[k] - Id_measured[k]));
}

// Find delay with maximum correlation
float max_corr = 0.0f;
int best_delay = 0;
for (int delay = 0; delay < NUM_DELAY_TAPS; delay++) {
    float corr = fabsf(filter_fo_get_y1(&delay_filters[delay]));
    if (corr > max_corr) {
        max_corr = corr;
        best_delay = delay;
    }
}

// Convert delay to phase lag
float encoder_lag_samples = (float)best_delay;
float encoder_lag_ms = encoder_lag_samples * SAMPLE_TIME_MS;
```

**Advantages of PRBS Method**:
- No additional excitation needed
- Works during normal operation
- Broad frequency content excites multiple dynamics
- Can run continuously for monitoring

**Action**: 
- Apply predictive correction: `angle_corrected = angle_encoder + (speed * lag_time)`
- Or flag encoder for replacement if lag excessive

### 5. Cross-Correlation Diagnostics (PRBS-Based)

PRBS is close to white noise, making cross-correlation extremely powerful for system identification and diagnostics without heavy math.

#### 5.1 Lock-in Style Extraction
**Extract PRBS-driven component of any signal:**
```c
// Correlate PRBS bit (±1) with measured signal
// Initialize: filter_fo_init(&filter_lockin); time constant ~2s

// Each cycle
filter_fo_step(&filter_lockin, prbs_bit[k] * signal[k]);
float correlation = filter_fo_get_y1(&filter_lockin);

// Correlation ≈ gain from PRBS → signal
// Use to verify PRBS is actually affecting the system
```

**Applications:**
- **Observability check**: Correlate PRBS with `dId/dt` - if near zero, PRBS too small or gating too strict
- **Current loop gain**: Correlate `id_prbs` with `id_measured` - should be close to 1.0
- **Bias detection**: Separate correlation for PRBS = +1 vs -1, difference indicates DC bias

#### 5.2 Impulse Response Estimation
**For LTI systems, cross-correlation approximates impulse response:**
```c
// Compute cross-correlation at multiple delays
float r_uy[MAX_DELAY];
for (int tau = 0; tau < MAX_DELAY; tau++) {
    // Correlate input[k] with output[k+tau]
    filter_fo_step(&filters[tau], input[k] * output[k+tau]);
    r_uy[tau] = filter_fo_get_y1(&filters[tau]);
}

// r_uy[τ] ≈ impulse response h[τ] (scaled)
```

**Applications:**
- **Current loop dynamics**: `id_ref → id` impulse response reveals bandwidth, damping, delays
- **Plant identification**: `vd → id` transfer function
- **PI tuning validation**: Compare measured response to expected from PI gains

#### 5.3 Loop Delay Estimation (Very Practical)
```c
// Find delay that maximizes correlation
float max_corr = 0.0f;
int best_tau = 0;

for (int tau = 0; tau < 10; tau++) {  // Check 0-0.5ms at 20kHz
    float corr = fabsf(correlation_at_delay(id_prbs, id_measured, tau));
    if (corr > max_corr) {
        max_corr = corr;
        best_tau = tau;
    }
}

float loop_delay_ms = best_tau * SAMPLE_TIME_MS;
```

**Use to validate:**
- PWM update timing
- ADC sampling synchronization
- "Predict angle for next cycle" compensation
- Current PI phase margin

#### 5.4 Deadtime / Sign-Dependent Effects
```c
// Correlate residual with sign of current
filter_fo_step(&filter_deadtime_pos, residual[k] * (Id > 0 ? 1.0f : 0.0f));
filter_fo_step(&filter_deadtime_neg, residual[k] * (Id < 0 ? 1.0f : 0.0f));

float corr_pos = filter_fo_get_y1(&filter_deadtime_pos);
float corr_neg = filter_fo_get_y1(&filter_deadtime_neg);
float deadtime_indicator = corr_pos - corr_neg;

if (fabsf(deadtime_indicator) > THRESHOLD) {
    // Sign-dependent voltage drop detected
    // Add Vdt_sign * sign(Id) to regression vector
}
```

#### 5.5 Continuous Excitation Monitoring
**Runtime check that estimator is observable:**
```c
// Correlate PRBS with dId/dt (should be strong if PRBS excites system)
float excitation_strength = correlation(prbs_bit, dId_dt);

if (excitation_strength < MIN_EXCITATION) {
    // Warning: PRBS amplitude too small or current loop too slow
    // Estimator will have poor convergence
}
```

**Summary: Cross-correlation provides:**
- Simple system ID without parametric models
- Confidence metrics (is PRBS working?)
- Diagnostic tools (delay, deadtime, bandwidth)
- Can run continuously with minimal overhead (just low-pass filters)

### 6. Magnetic Saturation Effects

**Mechanism**: As current increases, Ld and Lq decrease due to magnetic saturation
- RLS tracks Ld(Id, Iq) and Lq(Id, Iq)
- Build lookup table online

**Tracking:**
```c
// Store Ld, Lq estimates indexed by current magnitude using fo_filter per bin
// Initialize once per bin: filter_fo_init(&Ld_filters[i]); filter_fo_init(&Lq_filters[i]);
// Time constant ~5s per bin: a1 ≈ 0.99998

float I_mag = sqrtf(Id*Id + Iq*Iq);
int index = (int)(I_mag / CURRENT_BIN_SIZE);
if (index < NUM_CURRENT_BINS) {
    filter_fo_step(&Ld_filters[index], Ld_est);
    filter_fo_step(&Lq_filters[index], Lq_est);
    
    Ld_table[index] = filter_fo_get_y1(&Ld_filters[index]);
    Lq_table[index] = filter_fo_get_y1(&Lq_filters[index]);
}
```

**Use case**: 
- Improve current controller performance at high current
- Adjust PI gains based on operating point
- Torque estimation accuracy

### 7. Temperature Drift Tracking (PRBS-Based)

**Mechanism**: Rs increases ~0.4%/°C for copper windings
- RLS tracks Rs continuously using PRBS excitation
- **No back-EMF required** - Rs estimated directly from voltage equation
- **Requires Rs0@T0 anchor** - one-time offline calibration at known temperature

**RLS-based temperature estimate:**
```c
// Rs(T) = Rs0 * (1 + α * (T - T0))
// Solve for T:
T_winding_rls = T0 + (Rs_est - Rs0) / (COPPER_TEMP_COEFF * Rs0)

// Copper: α ≈ 0.00393 /°C
// Rs0, T0: from offline anchor measurement (stored in NVM)
```

**Thermal Model Integration** (dual-method monitoring):
```c
// In ISR, after RLS update:
if ((count & RLS_DECIMATION_MASK) == 0) {
    // RLS temperature (calculated internally from Rs drift)
    bool rls_temp_valid = rls_motor_est_get_temperature(
        &params->rls_est, &T_rls);
    // Returns false if temperature outside normal range (-40 to 180°C)
}

// Thermal model update (slower, 1-10 Hz)
if ((count & THERMAL_MODEL_DECIMATION_MASK) == 0) {
    // Update physics-based model
    T_thermal = thermal_model_update(&params->thermal_model,
                                     Id_A, Iq_A,           // For I²R
                                     params->Rs_est_ohm,   // From RLS
                                     ambient_temp_C);       // Optional
    
    // Cross-validate (returns worst-case and warning flag)
    bool thermal_warning = thermal_model_validate(
        &params->thermal_model,
        T_rls, T_thermal, &T_max);
    // Returns true if discrepancy > threshold (diagnostic issue)
    // T_max contains worst-case for protection
    
    // Overheating protection
    if (T_max > TEMP_WARNING_THRESHOLD) {
        motor_api_reduce_current_limit();
    }
}
```

**API Functions Handle**:
- `rls_motor_est_get_temperature()`: Rs drift → temperature, bounds checking, validation
- `thermal_model_update()`: I²R thermal dynamics, first-order ODE integration
- `thermal_model_validate()`: Compare estimates, detect anomalies, return worst-case

**Benefits of Dual-Method Approach**:
- **RLS temperature**: Fast response (updated every RLS cycle), no thermal model needed
- **Thermal model**: Independent validation, can detect RLS divergence
- **Redundancy**: If one method fails, other provides backup
- **Diagnostics**: Discrepancy indicates problems (cooling, connections, model error)
- **Protection**: Use worst-case ensures conservative thermal management

**Diagnostic: Inverter Ron Estimation**
```c
// Compare Rs measurements at different current levels
// Rs_measured = Rs_winding + Ron_inverter
// Measure at two current levels to separate:
float Rs_at_low_current = /* RLS estimate at I1 */;
float Rs_at_high_current = /* RLS estimate at I2 */;

// If Ron is significant, Rs_measured will appear to decrease with current
// (because voltage drop from Ron is attributed to winding resistance)
// This is for diagnostics only - not used in real-time compensation
```

**Use cases:**
- Dual-method winding temperature monitoring (RLS + thermal model)
- Overheating protection with redundancy
- Thermal model validation
- Performance degradation detection
- Diagnostic: Separate winding losses from inverter losses

### 8. Mechanical Resonance Detection (PRBS-Based)

**Mechanism**: PRBS injection excites mechanical resonances
- Broadband excitation (DC to ~1kHz with 5-10 cycle bit duration)
- Resonances show up in RLS residual frequency content
- FFT or bandpass filters on residual

**Detection:**
```c
// Bandpass filter RLS residual at suspected resonance frequencies
// PRBS excites all frequencies, resonances amplified in residual
// Common ranges: 50-200 Hz (shaft torsion), 500-2000 Hz (bearing)

for (int i = 0; i < NUM_RESONANCE_BINS; i++) {
    resonance_power[i] = bandpass_filter(e[k], f_center[i], f_bandwidth)
    
    if (resonance_power[i] > RESONANCE_THRESHOLD) {
        // Resonance detected at f_center[i]
        // Apply notch filter to speed/torque command
    }
}
```

**Advantages**:
- PRBS provides continuous excitation across frequency range
- No dedicated frequency sweep needed
- Can monitor continuously during operation

**Action**: 
- Notch filter in trajectory generator
- Reduce current bandwidth near resonance
- Mechanical damping (add inertia, dampers)

### 9. Cogging Torque Characterization

**Mechanism**: Cogging torque creates position-dependent load torque
- At constant speed, Iq oscillates with electrical position
- Build cogging torque map online

**Measurement:**
```c
// During constant speed operation using fo_filter per position bin
// Initialize once per bin: for (i=0; i<NUM_BINS; i++) filter_fo_init(&cogging_filters[i]);
// Time constant ~2s: a1 ≈ 0.99997

int position_bin = (int)(elec_angle_deg / ANGLE_BIN_SIZE) % NUM_BINS;

// Low-pass filter Iq at each position
filter_fo_step(&cogging_filters[position_bin], Iq[k]);
cogging_map[position_bin] = filter_fo_get_y1(&cogging_filters[position_bin]);
```

**Compensation:**
```c
// During operation, apply feedforward
int bin = (int)(elec_angle_deg / ANGLE_BIN_SIZE) % NUM_BINS
Iq_ref = Iq_command + cogging_map[bin]  // Pre-compensate
```

### 10. Friction, Load, and Inertia Estimation (Out of Scope)

**Note**: These mechanical parameters will be estimated by the **velocity controller's DOB/ESO** (Disturbance Observer / Extended State Observer), not by RLS.

**DOB/ESO Advantages for Mechanical Estimation:**
- Direct measurement from velocity loop error
- No additional excitation needed
- Natural integration with velocity controller
- Handles non-linear and time-varying disturbances
- Better suited for trajectory tracking applications

**RLS Focus:**
- Electromagnetic parameters only (Rs, Ld, Lq)
- Provides accurate motor model for DOB/ESO
- Complementary approaches: RLS for motor, DOB for load

**Brief Overview (For Reference):**

**Friction + Load via DOB:**
```c
// Velocity controller with disturbance observer
// Plant model: ω_dot = (T_motor - T_dist) / J
// DOB estimates: T_dist = T_friction + T_load + unmodeled dynamics

float T_motor = (3.0f/2.0f) * POLE_PAIRS * flux_linkage * Iq;
float omega_accel = (omega[k] - omega[k-1]) / Ts;
float T_dist_est = T_motor - J_nominal * omega_accel;

// Low-pass filtered disturbance = friction + load
```

**Inertia via ESO:**
```c
// Extended state observer adds inertia as state
// States: [position, velocity, inertia, disturbance]
// Estimates J online during acceleration/deceleration
```

This architecture separates electromagnetic estimation (RLS) from mechanical estimation (DOB/ESO), improving both accuracy and convergence.

## Summary: What Can Be Measured Online with PRBS Injection?

### Motor Parameters (Primary - Direct from RLS)
1. ✅ **Stator resistance (Rs)** - Temperature drift, connection quality
2. ✅ **D-axis inductance (Ld)** - Magnetic saturation effects
3. ✅ **Q-axis inductance (Lq)** - Saliency ratio, saturation
4. ❌ **Flux linkage (λ_pm)** - **Cannot measure without phase voltage sensing or back-EMF test** - use datasheet or offline measurement

### System Diagnostics (Residual Analysis)
5. ✅ **PI controller saturation** - Voltage/current limits reached
6. ✅ **ADC offset drift** - Temperature effects on sensing
7. ✅ **Dead-time compensation error** - Inverter non-linearity
8. ✅ **Phase lag** - Sampling/computational delays
9. ✅ **Overall system health** - Model mismatch indicator

### Magnetic Effects (PRBS-Based)
10. ✅ **Saliency ratio (Ld/Lq)** - Direct from RLS estimates
11. ✅ **Cross-coupling (Id→Iq)** - PRBS excites coupling, measure Iq disturbance
12. ✅ **Magnetic saturation** - Inductance vs current curves from continuous RLS
13. ⚠️ **Reluctance torque** - Requires flux linkage (datasheet or offline test)

### Position/Speed Sensing (PRBS-Based)
14. ✅ **Encoder lag** - Cross-correlation of PRBS with current response
15. ✅ **Position error** - Alignment quality (from d-axis regulation)

### Mechanical System (PRBS-Based)
16. ✅ **Mechanical resonances** - PRBS excites resonances, detect in residual FFT
17. ✅ **Cogging torque map** - Position-dependent torque ripple (hybrid steppers have significant cogging)
18. 🔄 **Friction (Coulomb + viscous)** - Handled by velocity controller DOB
19. 🔄 **Load torque** - Handled by velocity controller DOB
20. 🔄 **Inertia** - Handled by velocity controller ESO

### Thermal
21. ✅ **Winding temperature** - From Rs drift (0.4%/°C for copper)
22. 🔄 **Inverter Ron** - Diagnostic comparison at different currents (optional)

## System Integration

### State Machine Changes

**Remove ROVERL_MEAS State:**
With continuous online RLS estimation, the dedicated R/L measurement state becomes unnecessary:
- **Old sequence**: `OFFSET_MEAS → RS_EST → ROVERL_MEAS → ALIGN → IDLE → ONLINE`
- **New sequence**: `OFFSET_MEAS → RS_EST → ALIGN → IDLE → ONLINE`
- **Benefit**: Faster startup (~1-3s instead of ~5-8s)
- **RS_EST state**: Keep optional for quick initial Rs check, or rely entirely on RLS

**ROVERL_MEAS can be retained as optional** for initial Ld/Lq seeds if faster convergence desired, but RLS will refine these within 10-15s anyway.

### ISR Integration (motor_isr.c)

RLS estimator runs **only in ONLINE state** during normal FOC operation.

**Key Timing Optimization**: PRBS injection occurs before PI controllers (critical path), but RLS parameter update happens **after PWM output** (non-critical).

```c
void adc_callback(const struct device *dev, const q31_t *values,
                  uint8_t count, void *user_data)
{
    struct motor_parameters *params = (struct motor_parameters *)user_data;
    const struct smf_state *state = params->state_for_isr;
    
    /* ... existing ADC conversion, encoder read, Park transform ... */
    
    /* PRBS injection - BEFORE PI controllers (but as feedforward, not reference) */
    float Vd_prbs = 0.0f;  /* PRBS voltage injection */
    if (state == &motor_states[MOTOR_STATE_ONLINE]) {
        /* Check decimation and gating */
        bool needs_prbs_update = (params->control_loop_count & RLS_PRBS_DECIMATION_MASK) == 0;
        bool pi_saturated = params->pi_Id.saturated || params->pi_Iq.saturated;
        bool vbus_braking = (Vbus_V > VBUS_REGEN_LIMIT_V);
        bool observable = (fabsf(Id_A) > ID_MIN_A) && 
                         (fabsf(Id_A - params->Id_prev) > DID_DT_MIN * TS_DECIMATED);
        
        /* Store gating state for RLS update later */
        params->rls_gating_ok = !pi_saturated && !vbus_braking && observable;
        
        if (params->rls_gating_ok) {
            /* Get PRBS voltage (not current) for direct injection */
            /* Typical: 0.1A * 10Ω = 1V excitation amplitude */
            Vd_prbs = prbs_get_output(&params->prbs_gen) * RLS_PRBS_VOLTAGE_AMPLITUDE_V;
        }
    }
    
    /* Run PI controllers for all states */
    /* PRBS injected as feedforward voltage (4th parameter) - bypasses PI dynamics */
    pi_run_series(&params->pi_Id, Id_ref_A, Id_A, Vd_prbs, &Vd_V);
    pi_run_series(&params->pi_Iq, Iq_ref_A, Iq_A, 0.0f, &Vq_V);
    
    /* ... existing inverse Park, SVPWM, PWM generation ... */
    
    /* Output PWM - CRITICAL TIMING DEADLINE */
    mcpwm_stm32_set_duty_cycle_2phase_f32(pwm1, Da_hb1_pu, Da_hb2_pu);
    mcpwm_stm32_set_duty_cycle_2phase_f32(pwm8, Db_hb1_pu, Db_hb2_pu);
    
    /* ========== AFTER PWM OUTPUT - Non-critical ========== */
    
    /* RLS estimation (only in ONLINE state with decimation) */
    if (state == &motor_states[MOTOR_STATE_ONLINE]) {
        if ((params->control_loop_count & RLS_DECIMATION_MASK) == 0) {
            if (params->rls_gating_ok) {
                /* RLS update using THIS cycle's measurements */
                /* Derivative calculated internally using control_loop_freq from init */
                rls_motor_est_update(&params->rls_est, 
                                    Vd_V,         /* PI output from this cycle */
                                    Id_A,         /* Measured current this cycle */
                                    params->Id_A, /* Previous cycle current (before writeback) */
                                    params->observer.omega_e_rad_s);  /* Electrical speed */
                
                /* Extract parameter estimates and calculate temperature */
                params->Rs_est_ohm = rls_motor_est_get_Rs(&params->rls_est);
                params->Ld_est_H = rls_motor_est_get_Ld(&params->rls_est);
                params->Vbias_est_V = rls_motor_est_get_Vbias(&params->rls_est);
                params->Vdt_sign_est_V = rls_motor_est_get_Vdt_sign(&params->rls_est);
                
                /* RLS-based temperature (uses Rs0, T0 from init, returns status) */
                params->rls_temp_warning = !rls_motor_est_get_temperature(
                    &params->rls_est, &params->T_winding_rls_C);
            }
        }
        
        /* Thermal model update (slower decimation, e.g., 1-10 Hz) */
        if ((params->control_loop_count & THERMAL_MODEL_DECIMATION_MASK) == 0) {
            /* Update physics-based thermal model (returns temperature) */
            params->T_winding_thermal_C = thermal_model_update(
                &params->thermal_model, Id_A, Iq_A, 
                params->Rs_est_ohm, params->ambient_temp_C);
            
            /* Cross-validate and check for thermal protection (returns warning flags) */
            params->thermal_validation_warning = thermal_model_validate(
                &params->thermal_model,
                params->T_winding_rls_C,
                params->T_winding_thermal_C,
                &params->T_winding_max_C);  /* Worst-case for protection */
            
            /* Thermal protection if either estimate exceeds threshold */
            if (params->T_winding_max_C > TEMP_WARNING_THRESHOLD) {
                motor_api_post_warning(WARNING_OVERTEMPERATURE);
            }
        }
        
        /* Update PRBS generator for NEXT cycle (after RLS uses current state) */
        if ((params->control_loop_count & RLS_PRBS_DECIMATION_MASK) == 0) {
            if (params->rls_gating_ok) {
                prbs_run(&params->prbs_gen);
            }
        }
    }
    
    /* ... existing telemetry, timing measurement ... */
}
```

**Key Implementation Details:**

1. **Timing optimization**: PRBS injection before PI controllers, RLS update after PWM output
   - **PRBS injection**: Critical path - must happen before PI controllers
   - **RLS computation**: Non-critical - happens after PWM deadline
   - **Benefit**: RLS computation doesn't affect PWM latency or jitter
   - **One-cycle PRBS latency**: Generator advances after PWM, negligible for broadband signal

2. **Computational load**: STM32H753 @ 480 MHz, ISR @ 20 kHz
   - Available cycles per ISR: 24,000 cycles
   - RLS update (decimated 16x): ~500-1000 cycles when it runs
   - Average per ISR: ~62 cycles (< 0.3% of ISR budget)
   - Moving RLS after PWM: doesn't block critical control loop

3. **Separate decimation counters**: RLS update and PRBS update can have different rates
   - Typical: RLS_DECIMATION = 16 (1.25kHz), RLS_PRBS_DECIMATION = 16 (1.25kHz)
   - Can make PRBS faster if needed: RLS_PRBS_DECIMATION = 8 (2.5kHz)

4. **Voltage source**: Use `Vd_V, Vq_V` directly from PI controller outputs (after saturation limiter)
   - These are already computed each cycle
   - No additional PWM reconstruction needed
   - Inverter non-linearities captured by Vbias parameter

5. **Gating checks**: Performed once, used for both PRBS injection and RLS update
   - PI saturation: `params->pi_Id.saturated` flag (add to PI controller struct)
   - Vbus braking: Active when `Vbus_V > VBUS_REGEN_LIMIT_V`
   - Observability: PRBS amplitude sufficient and visible in current
   - Gating state stored in `params->rls_gating_ok` for use after PWM

6. **PRBS injection**: Use as **feedforward voltage** (4th parameter to PI controller)
   - Uses `prbs_get_output()` - returns current ±1 state without advancing
   - Scaled to voltage: `Vd_prbs = prbs_output * amplitude` (1V typical)
   - **Feedforward advantages**:
     - Bypasses PI dynamics (no bandwidth limitation)
     - Full PRBS frequency content reaches motor
     - Cleaner system ID (direct V → I relationship)
     - PI still maintains reference tracking via feedback
   - Small amplitude (1V → ~0.1A) minimizes torque disturbance
   - Injection on d-axis only (doesn't directly produce torque)
   - Generator advances after PWM output for next cycle

7. **State transitions**: RLS freezes automatically when leaving ONLINE state
   - Parameters hold last good estimates
   - Resume estimation when returning to ONLINE

### Data Flow Summary

```
ISR Cycle (20 kHz)
├── ADC Read → Ia, Ib, Vbus
├── Encoder Read → θ (via RTIO)
├── Park Transform → Id, Iq
├── [If ONLINE state]
│   └── [If gating passes]
│       └── PRBS Injection → Id_ref += id_prbs (uses previous PRBS state)
├── PI Controllers → Vd_V, Vq_V (post-limiter = RLS voltage source)
├── Inverse Park → Va_V, Vb_V
├── SVPWM → Duty cycles
├── PWM Output ← CRITICAL TIMING DEADLINE
│
└── ========== AFTER PWM OUTPUT (Non-critical) ==========
    └── [If ONLINE state & decimation match & gating ok]
        ├── RLS Update (Vd_V, Id, Id_prev, ω)
        ├── PRBS Generator Update → Next cycle's PRBS state
        ├── Extract Parameters (Rs, Ld, Vbias)
        ├── Temperature Calculation from Rs
        └── Store Id_prev for next cycle
```

**Key Benefits of This Architecture:**
- **Deterministic PWM timing**: RLS computation cannot affect control latency
- **Reduced jitter**: Critical path is minimal and predictable
- **Scalable**: Could move RLS to lower-priority context if needed
- **One-cycle PRBS delay**: Negligible for broadband excitation (~50μs at 20kHz)

### Startup Flow with RLS

**One-time setup (factory calibration):**
1. Measure Rs0 at known temperature T0 (e.g., 25°C using thermometer)
2. Store (Rs0, T0) in NVM

**Each power-on:**
1. **OFFSET_MEAS** (1s): Current sensor offset calibration
2. **RS_EST** (optional, 2s): Quick Rs verification
   - Compare against Rs0 scaled for ambient temperature
   - Can skip if ambient temp unknown - RLS will converge anyway
3. **ALIGN** (1s): Rotor alignment to known position
4. **IDLE**: Ready for commands
5. **ONLINE**: FOC with continuous RLS running in background
   - RLS converges to accurate Rs, Ld within ~10-15 seconds
   - Thermal tracking continues indefinitely

**Total startup: 1-3 seconds** (vs 5-8s with ROVERL_MEAS)

### Adding 4-Parameter RLS (High Saliency)

For stepper motors with significant Ld ≠ Lq, estimate both inductances:

```c
/* After PWM output - Non-critical section */
if (state == &motor_states[MOTOR_STATE_ONLINE]) {
    if ((params->control_loop_count & RLS_DECIMATION_MASK) == 0) {
        if (params->rls_gating_ok) {
            /* RLS update using BOTH voltage equations (dual-equation update) */
            /* For saliency estimation: separate Ld from Lq */
            
            /* D-axis update (pass currents, derivative calculated internally) */
            rls_motor_est_update_d(&params->rls_est, 
                                   Vd_V, Id_A, params->Id_A,
                                   params->observer.omega_e_rad_s, Iq_A);
            
            /* Q-axis update (same cycle for faster convergence) */
            rls_motor_est_update_q(&params->rls_est, 
                                   Vq_V, Iq_A, params->Iq_A,
                                   params->observer.omega_e_rad_s, Id_A);
            
            /* Extract 4 parameters: Rs, Ld, Lq, Vbias */
            params->Rs_est_ohm = rls_motor_est_get_Rs(&params->rls_est);
            params->Ld_est_H = rls_motor_est_get_Ld(&params->rls_est);
            params->Lq_est_H = rls_motor_est_get_Lq(&params->rls_est);
            params->Vbias_est_V = rls_motor_est_get_Vbias(&params->rls_est);
        }
        
        params->Id_prev = Id_A;
        params->Iq_prev = Iq_A;  /* Need both for 4-parameter */
    }
}
```

## Critical Implementation Warnings

### 1. Current Derivative Timing
**Key Insight**: `params->Id_A` contains the **previous cycle's value** until the writeback at end of ISR.

**Implementation** (derivative calculated inside RLS estimator):
```c
// Early in ISR: Id_A computed from this cycle's ADC
arm_park_f32(Ia_A, Ib_A, &Id_A, &Iq_A, sin_theta, cos_theta);

// RLS section (after PWM output):
if (decimation_match) {
    // Pass current and previous current - derivative calculated internally
    rls_motor_est_update(&params->rls_est, 
                        Vd_V, Id_A, params->Id_A, omega);
    // RLS estimator calculates: dId_dt = (Id - Id_prev) * control_freq
    // where control_freq was provided during rls_motor_est_init()
}

// End of ISR: writeback updates params->Id_A for next cycle
params->Id_A = Id_A;
```

**Benefits**:
- **Cleaner separation**: ISR doesn't need to know about derivative calculation
- **No separate variable**: ISR structure naturally provides previous value
- **Encapsulation**: Control frequency stored in RLS estimator struct
- **Testability**: RLS estimator can be unit tested independently

### 2. Numerical Stability
**Covariance matrix conditioning**:
- Add minimum eigenvalue floor: `P[i,i] = MAX(P[i,i], P_MIN)` after each update
- Monitor condition number: `max(diag(P)) / min(diag(P))`
- Consider periodic reset (every 1000 updates) or when converged

**Division by zero prevention**:
```c
float denominator = lambda + phi_T_P_phi;
if (denominator < 1e-6f) {
    // Skip update - numerical issue
    return;
}
```

### 3. NVM Storage Requirements
**Must store persistently**:
- `Rs0_ohm`: Resistance at calibration temperature
- `T0_C`: Calibration temperature
- `calibration_valid`: Boolean flag (reject corrupted data)
- `calibration_crc`: Data integrity check

**Recommended location**: Dedicated flash sector with wear leveling if > 10k calibrations expected.

### 4. Observability During Acceleration
Current gating check:
```c
bool observable = (fabsf(dId_dt) > threshold);
```

**Issue**: During rapid acceleration, dId_dt dominated by trajectory, not PRBS.

**Better approach - Correlation-based observability**:
```c
// Instantaneous correlation between PRBS and current derivative
float correlation = dId_dt * prbs_get_output(&params->prbs_gen);

// Low-pass filter to get average correlation strength
filter_fo_step(&params->filter_prbs_observability, fabsf(correlation));
float excitation_strength = filter_fo_get_y1(&params->filter_prbs_observability);

bool observable = (excitation_strength > OBSERVABILITY_THRESHOLD);
```

**Advantages**:
- Rejects trajectory-induced dId/dt (uncorrelated with PRBS)
- Directly measures PRBS effectiveness
- Simple: one multiply, one filter per cycle
- **Recommended implementation**: Better than total dId/dt or bandpass filtering

### 5. Vbias and Dead-time Compensation
Inverter voltage errors have multiple sources:
- Dead-time: `Vdt = f(sign(Id))` - **current-sign-dependent** (dominant)
- Diode drops: `Vd = f(|Id|)` - current-magnitude-dependent
- PWM artifacts: `Vpwm = f(ω)` - speed-dependent

**Recommended 4-parameter approach** (for stepper motors):
```c
θ = [Rs, Ld, Vbias, Vdt_sign]ᵀ
φ = [Id, dId/dt, 1, sign(Id)]ᵀ
```

**Rationale**:
- `Vbias`: Captures constant offsets (ADC bias, average diode drop)
- `Vdt_sign`: Captures current-sign-dependent dead-time effects (typically 1-3V)
- Dead-time is significant in motor drives and warrants dedicated parameter
- Minimal computational cost (one extra multiply per update)

**Alternative**: Start with 3-parameter, monitor residuals. Add `Vdt_sign` if residual correlates with `sign(Id)`.

### 6. Forgetting Factor Time Constant
**Claimed**: λ = 0.9995 for thermal tracking

**Actual time constant**:
```
τ = Ts / (1 - λ) = (1/20000) / 0.0005 = 0.1s
With 16x decimation: τ_eff = 0.1s × 16 = 1.6s
```

**Problem**: Thermal time constants are **minutes**, not seconds. RLS may track noise instead of temperature.

**Recommended**: λ = 0.9999 → τ = 8s at 16x decimation, still fast for thermal drift.

### 7. Initial Parameter Values
RLS requires starting estimates. Options:

1. **Devicetree defaults** (primary source):
   ```c
   // From devicetree (motor-specific, per board overlay)
   params->rls_est.theta[0] = DT_PROP(MOTOR, rs_nominal_milliohms) / 1000.0f;
   params->rls_est.theta[1] = DT_PROP(MOTOR, ld_nominal_microhenry) / 1e6f;
   params->rls_est.theta[2] = 0.0f;  // Vbias unknown initially
   params->rls_est.theta[3] = 0.0f;  // Vdt_sign unknown initially
   ```

2. **NVM override** (if available, using Settings API):
   ```c
   // Load from NVM if valid (Settings API)
   if (settings_load_subsys("rls") == 0) {
       // Validate loaded values (bounds check)
       if (rls_params_valid(&nvm_params)) {
           memcpy(params->rls_est.theta, nvm_params.theta, sizeof(theta));
           // Faster convergence with previous estimates
       }
   }
   ```

3. **Initial covariance** (high uncertainty):
   ```c
   // Large initial covariance regardless of starting estimates
   P[0,0] = RLS_INITIAL_COVARIANCE;  // From devicetree, e.g., 1000.0
   P[1,1] = RLS_INITIAL_COVARIANCE;
   P[2,2] = RLS_INITIAL_COVARIANCE;
   P[3,3] = RLS_INITIAL_COVARIANCE;  // If using Vdt_sign
   ```

**Implementation sequence**:
1. Load devicetree defaults (always available)
2. Attempt NVM load via Settings API (may not exist on first boot)
3. Validate any loaded values (sanity bounds)
4. Initialize with large covariance for fast adaptation
5. Store converged estimates to NVM periodically (Settings API)

**Best practice**: Devicetree provides safe defaults, NVM accelerates convergence on subsequent boots.

### 8. Shell Command Safety
**Dangerous** (step-change excitation):
```c
shell> motor rls prbs 0.5  // Suddenly change from 0.1A to 0.5A
```

**Safe** (ramped change):
```c
// In ISR or background thread
float prbs_amplitude_target = 0.5f;
float ramp_rate = 0.01f;  // A/s

if (prbs_amplitude_current < prbs_amplitude_target) {
    prbs_amplitude_current += ramp_rate * Ts;
} else {
    prbs_amplitude_current = prbs_amplitude_target;
}
```

Also enforce:
- Only allow changes in ONLINE state
- Bounds checking: `clampf(amplitude, 0.0f, MAX_SAFE_AMPLITUDE)`
- Disable during critical operations (homing, calibration)

### 9. Dual-Equation RLS: When is it Needed?

**For 3-parameter RLS** [Rs, Ld, Vbias] or [Rs, Ld, Vbias, Vdt_sign]:
- **Use d-axis equation only**
- Ld appears in dId/dt term, directly observable
- Single equation sufficient
- Computation: ~500-1000 cycles

**For saliency estimation** (Ld ≠ Lq):
- **Requires both equations** to separate Ld from Lq:
  - Vd equation: has Ld in dId/dt, Lq in -ω*Lq*Iq (cross-term)
  - Vq equation: has Lq in dIq/dt, Ld in ω*Ld*Id (cross-term)
- Without both equations, cannot uniquely identify Ld vs Lq
- Computation: 2× matrix operations → ~1000-2000 cycles
- Still acceptable: < 10% of 24,000-cycle ISR budget

**For hybrid stepper motors**:
- Significant saliency (Ld ≠ Lq) is common
- Dual-equation recommended for accurate parameter estimates
- Start with 3-parameter (d-axis only) during development
- Add q-axis equation for production (better convergence)

**If timing critical**: Process equations alternately (d/q on odd/even cycles) - halves convergence but reduces load.

### 10. Temperature Edge Cases
**Equation**:
```c
T = T0 + (Rs_est - Rs0) / (α * Rs0)
```

**Edge cases**:
1. **Rs_est < Rs0**: Implies T < T0 (valid if ambient < T0, e.g., cold start)
2. **Rs_est < 0.5*Rs0**: Physically implausible → RLS not converged or hardware fault
3. **Rs_est > 3*Rs0**: Would imply T > 500°C → thermal runaway or open circuit

**Required bounds**:
```c
// Always calculate temperature (allow T < T0 for cold starts)
T_est = T0 + (Rs_est - Rs0) / (ALPHA * Rs0);

// Clamp to physically possible range (absolute zero to material limits)
T_est = clampf(T_est, -273.0f, 250.0f);

// Warn if outside normal operating range (but don't reject)
if (T_est < -40.0f || T_est > 180.0f) {
    flag_temperature_out_of_range_warning();
}

// Only reject if Rs estimate is clearly invalid
if (Rs_est < RS_MIN || Rs_est > RS_MAX) {
    flag_rls_divergence_error();
    // Use thermal model temperature as fallback
}
```

## Implementation Priority

### Phase 1: Core RLS (Week 1)
1. **Offline anchor calibration**: Rs0@T0 measurement procedure and NVM storage
2. **Remove/skip ROVERL_MEAS state**: Simplify state machine sequence
3. **PRBS generator module**: `prbs.h/c` with LFSR (complete implementation in Data Structures section)
   - **12-bit maximal-length LFSR**: 4095-sample sequence (~4s period at 1kHz)
   - **Polynomial**: x^12 + x^11 + x^10 + x^4 + 1 → Generator 0x0C09
   - **Implementation pattern**: Based on liquid-dsp msequence.c
     - Binary dot product for feedback calculation
     - Store generator polynomial, not individual taps
     - Standard API: `prbs_init()`, `prbs_advance()`, `prbs_get_output()`, `prbs_reset()`
   - **Why 12-bit**: Sufficient frequency resolution without excessive memory
     - 10-bit (1023): Only ~1s period, coarse frequency resolution
     - 12-bit (4095): ~4s period, good resolution for parameter estimation
     - 15-bit (32767): ~33s period, unnecessary for this application
   - **Frequency content**: Flat spectrum from DC to Nyquist (500Hz at 1kHz update)
   - **LFSR advantages**: Zero memory (just shift register), deterministic, repeatable
4. **4-parameter RLS module**: `rls_motor_est.h/c` with [Rs, Ld, Vbias, Vdt_sign]
   ```c
   // Initialization (control freq, RLS params, temperature calibration)
   // Initial P matrix reflects uncertainty in parameters:
   float P0[4][4] = {
       {1.0f, 0, 0, 0},      // Rs: ±1 Ω initial uncertainty
       {0, 0.01f, 0, 0},     // L: ±0.1 H (0.01 = 0.1²) initial uncertainty
       {0, 0, 4.0f, 0},      // Vbias: ±2V (4.0 = 2²) initial uncertainty
       {0, 0, 0, 1.0f}       // Vdt_sign: ±1V initial uncertainty
   };
   float theta_init[4] = {Rs_nominal, L_nominal, 0.0f, 0.0f};
   
   rls_motor_est_init(&params->rls_est, 
                      CONTROL_LOOP_FREQUENCY_HZ,  // e.g., 20000 Hz
                      0.9999f,                    // lambda (forgetting factor)
                      P0,                         // Initial covariance
                      theta_init,                 // Initial parameter guess
                      Rs0_ohm, T0_C, 0.00393f);   // Temperature calibration (copper)
   
   // Update (pass current, previous current, omega, cross-coupling)
   rls_motor_est_update(&params->rls_d, Vd, Id, Id_prev, omega, Lq_est, Iq);
   rls_motor_est_update(&params->rls_q, Vq, Iq, Iq_prev, omega, Ld_est, Id);
   // Derivative calculated internally: dI/dt = (I - I_prev) * freq
   // Cross-term compensated internally: V_compensated = V + omega*L_cross*I_cross
   
   // Get temperature (calculated internally from Rs drift)
   bool valid = rls_motor_est_get_temperature(&params->rls_est, &T_winding_C);
   // Returns false if temperature outside reasonable range (warning)
   ```
5. **Thermal model module**: `thermal_model.h/c` for independent temperature monitoring
   ```c
   // Initialization (set thermal parameters and initial conditions)
   thermal_model_init(&params->thermal_model,
                      CONTROL_LOOP_FREQUENCY_HZ,
                      tau_thermal_s, T_ambient_C, Rs0, T0, alpha,
                      T_validation_threshold_C);  // For cross-validation
   
   // Update (uses RLS Rs estimate for accurate I²R calculation)
   T_thermal = thermal_model_update(&params->thermal_model,
                                    Id, Iq, Rs_est, T_ambient);
   
   // Validate (compare RLS vs thermal, return worst-case)
   bool warning = thermal_model_validate(&params->thermal_model,
                                         T_rls, T_thermal, &T_max);
   // Returns true if discrepancy exceeds threshold (cooling issue, etc.)
   ```
6. **ISR integration**: Add RLS update call in ONLINE state with decimation check
7. **Use Vd_V/Vq_V directly** from PI outputs (no PWM reconstruction)
8. **Gating implementation**: Freeze during braking, saturation, low excitation
9. **Observability checks**: Correlation-based (PRBS × dId/dt filtered)
10. **Parameter bounds**: Clamp Rs, Ld, Vbias, Vdt_sign to sane ranges
11. **Dual temperature monitoring**: RLS (from Rs) + thermal model (from I²R dynamics)
12. **Thermal validation**: Compare RLS vs thermal model, use worst-case for protection
13. **Convergence detection**: 
    - Monitor trace(P) = P[0,0] + P[1,1] + P[2,2] + P[3,3]
    - Converged when trace(P) < threshold (e.g., 0.1 for well-tuned system)
    - Hysteresis: Diverged when trace(P) > 2*threshold (prevents flapping)
    - Log convergence time: Record num_updates at first convergence
    - Typical: ~15-30 seconds after PRBS start (depends on excitation, speed)
13. **Convergence detection**: 
    - Monitor trace(P) = P[0,0] + P[1,1] + P[2,2] + P[3,3]
    - Converged when trace(P) < threshold (e.g., 0.1 for well-tuned system)
    - Hysteresis: Diverged when trace(P) > 2*threshold (prevents flapping)
    - Log convergence time: Record num_updates at first convergence
    - Typical: ~15-30 seconds after PRBS start (depends on excitation, speed)

### Phase 2: Diagnostics (Week 2)
1. **Residual RMS monitoring**: Detect model mismatch and system health
2. **Basic cross-correlation**: Verify PRBS observability
3. **PI saturation detection**: Flag gating conditions in telemetry
4. **Convergence detection**: Monitor covariance diagonal
5. **Quality metrics logging**: Residual, acceptance rate, parameter confidence
6. **Shell commands**: 
   ```
   motor rls status           # Show Rs, Ld, Vbias, Vdt_sign, temperature, convergence
   motor rls reset            # Reset covariance matrix
   motor rls prbs <amp>       # Adjust PRBS amplitude (ramped)
   motor rls lambda <value>   # Adjust forgetting factor (0.999-0.9999)
   motor rls calibrate <temp> # Set Rs0@T0 anchor
   motor rls gate <threshold> # Adjust observability threshold
   ```

### Phase 3: Advanced Features (Week 3+)
1. **4-parameter RLS**: Add Lq estimation using dual-equation update (stepper motors)
2. **Optional Vdt_sign**: Sign-dependent deadtime if Vbias insufficient
3. **Cross-correlation diagnostics**: Loop delay, bandwidth validation
4. **Encoder lag detection**: PRBS-based correlation method
5. **Thermal model comparison**: RLS temperature vs thermal model
6. **Adaptive PI gains**: Adjust current controller based on Rs, Ld estimates

### Phase 4: Mechanical Characterization (Week 4+)
1. **Cogging torque mapping**: Position-dependent Iq (stepper-specific)
2. **Resonance detection**: FFT of RLS residuals
3. **Impulse response estimation**: System ID from cross-correlation
4. **DOB/ESO integration**: Use accurate Rs, Ld, Lq from RLS in disturbance observer
5. **Magnetic saturation tables**: Ld(I), Lq(I) lookup tables

## Configuration Parameters

### Devicetree Parameters
These parameters are configured in devicetree (e.g., `rubus,user-parameters` binding):

```c
/* PRBS Configuration (from devicetree) */
#define RLS_PRBS_VOLTAGE_AMPLITUDE_V DT_PROP(USER_PARAMS, rls_prbs_voltage_amplitude_millivolts) / 1000.0f  // Default: 1000 → 1.0V
#define RLS_PRBS_DECIMATION         DT_PROP(USER_PARAMS, rls_prbs_decimation)                     // Default: 16 (power of 2)
#define RLS_PRBS_DECIMATION_MASK    (RLS_PRBS_DECIMATION - 1)                                     // Bit mask for efficient modulo
// Update rate = CONTROL_LOOP_FREQUENCY_HZ / RLS_PRBS_DECIMATION (e.g., 20kHz / 16 = 1.25kHz)
// Voltage amplitude chosen to give ~0.1-0.2A excitation (depends on Rs + Ld*ω)

/* RLS Configuration (from devicetree) */
#define RLS_DECIMATION              DT_PROP(USER_PARAMS, rls_decimation)                          // Default: 16 (power of 2)
#define RLS_DECIMATION_MASK         (RLS_DECIMATION - 1)                                          // Bit mask for efficient modulo
// Update rate = CONTROL_LOOP_FREQUENCY_HZ / RLS_DECIMATION (e.g., 20kHz / 16 = 1.25kHz)
#define RLS_FORGETTING_FACTOR       DT_PROP(USER_PARAMS, rls_forgetting_factor) / 10000.0f       // Default: 9995 → 0.9995
#define RLS_INITIAL_COVARIANCE      DT_PROP(USER_PARAMS, rls_initial_covariance)       // Default: 1000.0
#define RLS_MIN_COVARIANCE          DT_PROP(USER_PARAMS, rls_min_covariance) / 1000.0f // Default: 1 → 0.001
#define RLS_CONVERGENCE_THRESHOLD   DT_PROP(USER_PARAMS, rls_convergence_threshold) / 100.0f  // Default: 1 → 0.01

/* Offline Anchor (stored in NVM, read at runtime) */
// Rs0 and T0 stored in non-volatile memory from one-time calibration
// COPPER_TEMP_COEFF from devicetree:
#define COPPER_TEMP_COEFF           DT_PROP(USER_PARAMS, copper_temp_coeff) / 100000.0f  // Default: 393 → 0.00393

/* Thermal Model Configuration (from devicetree) */
#define THERMAL_MODEL_DECIMATION    DT_PROP(USER_PARAMS, thermal_model_decimation)       // Default: 2000 (1 Hz at 20kHz)
#define THERMAL_MODEL_DECIMATION_MASK (THERMAL_MODEL_DECIMATION - 1)                     // Bit mask for efficient modulo
#define THERMAL_TIME_CONSTANT_S     DT_PROP(USER_PARAMS, thermal_time_constant_s)        // Default: 180 (3 min)
#define AMBIENT_TEMP_DEFAULT_C      DT_PROP(USER_PARAMS, ambient_temp_default_degC)      // Default: 25
#define THERMAL_VALIDATION_THRESHOLD DT_PROP(USER_PARAMS, thermal_validation_threshold_degC)  // Default: 15°C
#define TEMP_WARNING_THRESHOLD_C    DT_PROP(USER_PARAMS, temp_warning_threshold_degC)    // Default: 120
#define TEMP_ERROR_THRESHOLD_C      DT_PROP(USER_PARAMS, temp_error_threshold_degC)      // Default: 150

/* Gating Thresholds (from devicetree) */
#define ID_MIN_A                    DT_PROP(USER_PARAMS, rls_id_min_milliamps) / 1000.0f     // Default: 50 → 0.05A
#define DID_DT_MIN                  DT_PROP(USER_PARAMS, rls_did_dt_min)                     // Default: 50.0 A/s
#define OMEGA_MIN_RAD_S             DT_PROP(USER_PARAMS, rls_omega_min_millirad_s) / 1000.0f // Default: 1000 → 1.0 rad/s
#define Y_MAX_SCALE                 DT_PROP(USER_PARAMS, rls_y_max_scale) / 100.0f          // Default: 80 → 0.8

/* Parameter Bounds (from devicetree) */
#define RS_MIN_OHM                  DT_PROP(USER_PARAMS, rls_rs_min_milliohms) / 1000.0f    // Default: 500 → 0.5Ω
#define RS_MAX_OHM                  DT_PROP(USER_PARAMS, rls_rs_max_ohms)                   // Default: 50Ω
#define LD_MIN_H                    DT_PROP(USER_PARAMS, rls_ld_min_microhenry) / 1000000.0f // Default: 100 → 0.0001H
#define LD_MAX_H                    DT_PROP(USER_PARAMS, rls_ld_max_millihenry) / 1000.0f   // Default: 100 → 0.1H
#define VBIAS_MIN_V                 -DT_PROP(USER_PARAMS, rls_vbias_max_volts)              // Default: ±5V
#define VBIAS_MAX_V                 DT_PROP(USER_PARAMS, rls_vbias_max_volts)

/* Residual Diagnostics (from devicetree) */
#define RESIDUAL_HEALTH_THRESHOLD   DT_PROP(USER_PARAMS, rls_residual_health_mv) / 1000.0f       // Default: 1000 → 1.0V
#define SATURATION_THRESHOLD        DT_PROP(USER_PARAMS, rls_saturation_threshold_mv) / 1000.0f  // Default: 500 → 0.5V
#define OFFSET_DRIFT_THRESHOLD      DT_PROP(USER_PARAMS, rls_offset_drift_ma) / 1000.0f         // Default: 100 → 0.1A
#define DEADTIME_ERROR_THRESHOLD    DT_PROP(USER_PARAMS, rls_deadtime_error_mv_per_a) / 1000.0f // Default: 200 → 0.2V/A
#define PHASE_LAG_THRESHOLD         DT_PROP(USER_PARAMS, rls_phase_lag_threshold_deg)           // Default: 5.0°

/* Cross-Correlation Diagnostics (from devicetree) */
#define CORR_TIME_CONSTANT_S        DT_PROP(USER_PARAMS, rls_corr_time_constant_ms) / 1000.0f   // Default: 2000 → 2.0s
#define MIN_EXCITATION_THRESHOLD    DT_PROP(USER_PARAMS, rls_min_excitation_threshold) / 100.0f // Default: 10 → 0.1
#define ENCODER_LAG_NUM_TAPS        DT_PROP(USER_PARAMS, rls_encoder_lag_num_taps)              // Default: 5
```

### KConfig Options
These features are enabled/disabled via KConfig (e.g., `prj.conf` or menuconfig):

```kconfig
# Enable/Disable RLS Features
CONFIG_RLS_ENABLE_VBIAS=y              # Estimate voltage bias (recommended)
CONFIG_RLS_ENABLE_VDT_SIGN=n           # Estimate sign-dependent deadtime (if Vbias insufficient)
CONFIG_RLS_ENABLE_SALIENCY=y           # Must be y for hybrid stepper (estimate Lq separately)
CONFIG_RLS_ENABLE_DIAGNOSTICS=y        # Residual analysis
CONFIG_RLS_ENABLE_TEMP_ESTIMATION=y    # Winding temp from Rs drift
CONFIG_RLS_ENABLE_CROSS_CORR=y         # Cross-correlation diagnostics
CONFIG_RLS_ENABLE_ENCODER_LAG=y        # PRBS-based cross-correlation method
CONFIG_RLS_ENABLE_RON_DIAGNOSTIC=n     # Estimate inverter Ron (diagnostic only)
CONFIG_RLS_DUAL_EQUATION_UPDATE=y      # Process both Vd and Vq each cycle (faster convergence)
```

**Usage in code:**
```c
#if IS_ENABLED(CONFIG_RLS_ENABLE_VBIAS)
    // Include Vbias in parameter vector
    θ = [Rs, Ld, Vbias]ᵀ
#else
    // 2-parameter only
    θ = [Rs, Ld]ᵀ
#endif
```

## Expected Performance

### Convergence Time
- **Rs**: ~5 seconds to 1% accuracy
- **Ld**: ~10 seconds to 2% accuracy  
- **Lq**: ~10 seconds to 2% accuracy (dual-equation update improves convergence)
- **Total**: ~15 seconds for all parameters to converge
- **Acceptable for stepper motor applications** (no impact on startup)

### Tracking Bandwidth
- **Temperature drift**: <1 minute time constant
- **Magnetic saturation**: Real-time (per control cycle)
- **Offset drift**: ~10 second time constant

### Accuracy
- **Rs**: ±2% typical (limited by voltage compensation accuracy)
- **Ld/Lq**: ±5% typical (limited by measurement noise)
- **Temperature**: ±5°C typical (from Rs)

## Advantages Over Offline Calibration

1. **Continuous tracking** - Parameters update during operation
2. **Temperature adaptation** - Rs tracks thermal state
3. **Load-dependent effects** - Captures saturation at actual operating points
4. **No downtime** - No calibration sequence required
5. **Diagnostic capability** - Rich fault detection from residuals
6. **Adaptive control** - PI gains can adjust to changing parameters

## Limitations

1. **Voltage reconstruction accuracy** - No direct Vd/Vq measurement; computed from duty cycles + Vbus
   - Dead-time compensation optional but recommended
   - Inverter non-linearities (Ron, Vf) create small bias in estimates
   - Acceptable for most applications, improves with dead-time compensation
2. **Excitation during operation** - PRBS adds small torque ripple (minimized by 5% amplitude)
   - Acceptable for stepper motors in positioning applications
3. **Flux linkage not measurable** - Cannot measure without direct phase voltage sensing
   - Must use datasheet value or separate offline back-EMF test
   - Limits reluctance torque calculation (requires known flux)
4. **Computation load** - 3-parameter RLS with dual equations adds ~100-150 cycles to 20kHz ISR
   - STM32H753 @ 480MHz can handle this comfortably
5. **Convergence time** - 10-15s acceptable for stepper applications
6. **PRBS frequency range** - Limited to ~DC-1kHz by bit duration
   - Sufficient for most motor and mechanical dynamics

## Conclusion

Online RLS parameter estimation provides far more than just Rs and Ld tracking. The residual analysis enables a comprehensive diagnostic and monitoring system that can detect faults, optimize performance, and characterize the mechanical system - all without interrupting operation.

**Recommended approach for hybrid stepper motor:**

**Setup (one-time factory calibration):**
1. Measure Rs0 at known temperature T0 (e.g., 25°C with thermometer)
2. Store (Rs0, T0) in NVM (absolute temperature reference for all future operation)

**State machine changes:**
3. Remove or make ROVERL_MEAS optional (RLS estimates Ld/Lq online)
4. Startup sequence: OFFSET_MEAS → RS_EST (optional) → ALIGN → ONLINE
5. Total startup time: 1-3 seconds (vs 5-8s with full TI calibration)

**Core implementation:**
6. Use Vd_V/Vq_V directly from PI controller outputs (post-limiter, no PWM reconstruction)
7. Implement PRBS generator: decimation-based update, 0.1A amplitude on Id
8. Start with 3-parameter RLS: [Rs, Ld, Vbias] with power-of-2 decimation
9. Integrate into ISR: RLS runs only in ONLINE state, freezes automatically in other states
10. Implement gating: freeze during braking, saturation, low excitation
6. Add observability checks: |Id|, |dId/dt|, optionally |ωe|
7. Clamp parameters to physically plausible ranges
8. Temperature from Rs using Rs0@T0 anchor

**Expansion:**
9. If high saliency: Add Lq (4-parameter: [Rs, Ld, Lq, Vbias])
10. If residuals show current-sign dependence: Add Vdt_sign parameter
11. Cross-correlation diagnostics: loop delay, observability, bandwidth
12. Implement thermal model for validation (runs at 1-10 Hz)
13. Use DOB/ESO in velocity controller for mechanical disturbances

**Optional refinements:**
14. Cogging torque mapping (stepper-specific)
15. Resonance detection via residual FFT
16. Ron diagnostic (separate winding from inverter losses)

**System Architecture:**
```
┌─────────────────────────────────────────────────────────┐
│              Motor Control ISR (20 kHz)                 │
│                                                         │
│  Measurements: Ia, Ib, Vbus, θe                        │
│       ↓                                                 │
│  Clarke/Park Transform → Id, Iq                        │
│       ↓                                                 │
│  PRBS Injection (1 kHz): id_ref += id_prbs            │
│       ↓                                                 │
│  PI Controllers → Vd_cmd, Vq_cmd (post-limiter)        │
│       ↓                                                 │
│  ┌───────────────────────────────────────────────┐    │
│  │ RLS Update (1 kHz decimated)                  │    │
│  │  - Inputs: Vd_cmd, Id, dId/dt, ωe            │    │
│  │  - Estimate: [Rs, Ld, Vbias] or [Rs,Ld,Lq,V] │    │
│  │  - Gating: freeze if saturated/braking/low-Id │    │
│  │  - Output: Rs → Temperature via Rs0@T0       │    │
│  └───────────────────────────────────────────────┘    │
│       ↓                                                 │
│  Inverse Park/Clarke → PWM outputs                     │
└─────────────────────────────────────────────────────────┘
           ↕ (disturbance estimation)
┌─────────────────────────────────────────────────────────┐
│         Velocity Controller with DOB/ESO                │
│  - Estimates load torque, friction                      │
│  - Estimates inertia (ESO)                              │
│  - Uses accurate Rs, Ld, Lq from RLS                    │
└─────────────────────────────────────────────────────────┘
           ↕ (monitoring & validation)
┌─────────────────────────────────────────────────────────┐
│            Thermal Model (lower rate: 1-10Hz)           │
│  - Estimates winding temperature from I²R + cooling     │
│  - Validates RLS temperature estimate                   │
│  - Overheating protection (redundant with RLS)          │
│  - NOT used for voltage compensation                    │
└─────────────────────────────────────────────────────────┘
```

This creates a comprehensive motor parameter estimation system:
- **RLS**: Tracks electromagnetic parameters (Rs, Ld, Lq) + temperature
- **Thermal Model**: Independent temperature monitoring and validation
- **DOB/ESO**: Tracks mechanical disturbances
- **Residuals**: Detect faults and calibration drift

All parameters continuously adapt during operation, providing robust control and diagnostic capability.

---

## Staged Implementation Plan

This section provides a practical roadmap for implementing the RLS parameter estimation system. Each phase is designed to be testable and provides value independently.

### Phase 1: PRBS Generator & Voltage Injection
**Goal**: Validate signal injection without estimation

**Implementation:**
1. Create `prbs.h/c` from document lines 295-378
   - Copy `struct prbs_gen` and `PRBS_GENPOLY_M12` definition
   - Copy all API functions: `prbs_init()`, `prbs_advance()`, `prbs_get_output()`, `prbs_reset()`
   - Add `#include <stdint.h>`
   - Note: PRBS returns `uint32_t` (0 or 1), ISR converts to ±voltage

2. Add devicetree property (`rubus,user-parameters`):
   ```dts
   prbs-amplitude-millivolts = <1000>;  // 1V default
   ```

3. Add to `motor_parameters` struct:
   ```c
   struct prbs_gen prbs_gen;
   uint32_t rls_decimation;        // From devicetree (default: 16)
   float32_t prbs_amplitude_V;     // From devicetree (millivolts → volts)
   ```

4. Initialize in `CTRL_INIT` state:
   ```c
   prbs_init(&params->prbs_gen);
   params->rls_decimation = 16;  // From devicetree
   params->prbs_amplitude_V = 1.0f;  // Convert from devicetree millivolts
   ```

5. Inject in ISR (d-axis only, decimated as feedforward to PI controller):
   ```c
   const uint32_t rls_mask = params->rls_decimation - 1;  // 0xF for decimation=16
   
   // Generate PRBS voltage feedforward (before PI controller)
   float32_t V_prbs_d = 0.0f;
   if ((params->control_loop_count & rls_mask) == 0) {  // Every 16th cycle
       uint32_t prbs_bit = prbs_advance(&params->prbs_gen);  // Returns 0 or 1
       V_prbs_d = (2.0f * (float32_t)prbs_bit - 1.0f) * params->prbs_amplitude_V;
   }
   
   // Run d-axis PI controller with PRBS feedforward
   pi_run_series(&params->pi_Id, Id_ref_A, Id_A, V_prbs_d, &Vd_V);
   ```

**Testing:**
- ✅ Verify PRBS visible in Id telemetry (should see ±1V / Rs current ripple at standstill)
- ✅ Check sequence period: 4095 samples × 16 cycles × 50μs = ~3.3s
- ✅ No motor torque ripple (d-axis injection orthogonal to torque)
- ✅ Verify branchless conversion: `2*bit - 1` produces ±1 without conditionals
- ✅ Shell command: `motor prbs <millivolts>` to adjust amplitude

**Success Criteria:**
- PRBS amplitude in Id matches expected: `I_prbs ≈ V_prbs / Rs` (at standstill)
- No audible noise or vibration from PRBS injection
- Stable FOC operation with PRBS running
- PRBS output alternates randomly between +1V and -1V

---

### Phase 2: D-axis 4-Parameter RLS (Rs, Ld, Vbias, Vdt_sign)
**Goal**: Complete parameter estimation with convergence

**Implementation:**
1. Create `rls_motor_est.h/c` from document lines 381-656
   - Copy `struct rls_motor_est` (lines 381-418)
   - Copy `rls_motor_est_update()` function (lines 524-656)
   - Implement full 4-parameter version: `θ = [Rs, Ld, Vbias, Vdt_sign]ᵀ`
   - Add initialization and temperature helper functions

2. Add to `motor_parameters`:
   ```c
   struct rls_motor_est rls_d;
   float32_t Rs_est, Ld_est, Vbias_est, Vdt_sign_est;
   float32_t Id_A_prev;  // For storing previous cycle current
   ```

3. Create initialization helper:
   ```c
   void rls_motor_est_init(struct rls_motor_est *rls,
                           float32_t control_freq,
                           float32_t lambda,
                           float32_t Rs_initial, float32_t Ld_initial,
                           float32_t Rs0_ohm, float32_t T0_C, float32_t alpha);
   ```

4. Initialize in `CTRL_INIT`:
   ```c
   rls_motor_est_init(&params->rls_d, 
                      CONTROL_LOOP_FREQUENCY_HZ,
                      0.9999f,  // Lambda (forgetting factor)
                      Rs_initial, Ld_initial,
                      Rs0_ohm, T0_C, 0.00393f);  // Temperature calibration
   params->Id_A_prev = 0.0f;
   ```

5. Update in ISR (same decimation as PRBS):
   ```c
   const uint32_t rls_mask = params->rls_decimation - 1;
   
   // Generate PRBS voltage feedforward
   float32_t V_prbs_d = 0.0f;
   if ((params->control_loop_count & rls_mask) == 0) {
       uint32_t prbs_bit = prbs_advance(&params->prbs_gen);
       V_prbs_d = (2.0f * (float32_t)prbs_bit - 1.0f) * params->prbs_amplitude_V;
   }
   
   // Run d-axis PI controller with PRBS feedforward
   pi_run_series(&params->pi_Id, Id_ref_A, Id_A, V_prbs_d, &Vd_V);
   
   // RLS update (use Vd_V which includes PRBS via feedforward)
   if ((params->control_loop_count & rls_mask) == 0) {
       float32_t omega_elec = angle_observer_get_elec_speed_rad_s(&params->observer);
       
       // D-axis RLS (no cross-coupling yet, set Lq=0, Iq=0 for Phase 2)
       rls_motor_est_update(&params->rls_d, Vd_V, Id_A, params->Id_A_prev,
                            omega_elec, 0.0f, 0.0f);
       
       // Store current for next cycle derivative calculation
       params->Id_A_prev = Id_A;
       
       // Update global estimates
       params->Rs_est = params->rls_d.theta[0];
       params->Ld_est = params->rls_d.theta[1];
       params->Vbias_est = params->rls_d.theta[2];
       params->Vdt_sign_est = params->rls_d.theta[3];
   }
   ```

**Testing:**
- ✅ Monitor convergence: `trace(P) = P[0][0] + P[1][1] + P[2][2] + P[3][3]`
  - Should drop from ~10 to <0.1 within 15-30s
- ✅ Rs estimate matches offline measurement (±5%)
- ✅ Ld estimate reasonable (within 20% of expected value)
- ✅ Vbias small (<2V typical, captures dead-time + ADC offsets)
- ✅ Vdt_sign small (<1V typical, captures sign-dependent drops)
- ✅ Temperature from Rs0@T0 calibration reasonable
- ✅ Plot residuals: should be white noise after convergence

**Shell Commands:**
```
motor rls status           # Show θ, trace(P), convergence, temperature
motor rls reset            # Reset P matrix to initial values
motor rls lambda <value>   # Adjust forgetting factor (0.999-0.9999)
motor rls calibrate <T>    # Set Rs0@T0 anchor at current Rs and temp
```

**Success Criteria:**
- Rs converges to within 5% of offline calibration value
- All covariance diagonal elements decrease monotonically
- Residuals become small and random (RMS < 1V after convergence)
- Temperature estimate reasonable (±10°C if Rs0@T0 calibrated)

---

### Phase 3: Dual-Axis RLS with Staggered Execution
**Goal**: Estimate both Ld and Lq for hybrid stepper with cross-coupling compensation

**Implementation:**
1. Add q-axis RLS to `motor_parameters`:
   ```c
   struct rls_motor_est rls_q;
   float32_t Lq_est;
   float32_t Iq_A_prev;  // For q-axis derivative calculation
   ```

2. Add devicetree properties (`rubus,user-parameters`):
   ```dts
   rls-decimation = <16>;           // Power-of-2 for efficient bit masking
   rls-stagger-offset = <8>;        // Half of decimation for load spreading
   ```

3. Initialize both estimators in `CTRL_INIT`:
   ```c
   // D-axis estimator
   rls_motor_est_init(&params->rls_d, 
                      CONTROL_LOOP_FREQUENCY_HZ,
                      0.9999f, Rs_initial, Ld_initial,
                      Rs0_ohm, T0_C, 0.00393f);
   
   // Q-axis estimator (same lambda, different inductance)
   rls_motor_est_init(&params->rls_q,
                      CONTROL_LOOP_FREQUENCY_HZ,
                      0.9999f, Rs_initial, Lq_initial,
                      Rs0_ohm, T0_C, 0.00393f);
   
   params->Id_A_prev = 0.0f;
   params->Iq_A_prev = 0.0f;
   
   // Initialize cross-estimates (will converge)
   params->Ld_est = Ld_initial;
   params->Lq_est = Lq_initial;
   ```

4. Load config from devicetree:
   ```c
   params->rls_decimation = 16;  // From DT
   params->rls_stagger_offset = 8;  // From DT
   ```

5. Staggered updates in ISR (full cross-coupling compensation):
   ```c
   const uint32_t rls_mask = params->rls_decimation - 1;  // 0xF for 16
   const uint32_t rls_offset = params->rls_stagger_offset;  // 8
   
   // Generate PRBS voltage feedforward
   float32_t V_prbs_d = 0.0f;
   if ((params->control_loop_count & rls_mask) == 0) {
       uint32_t prbs_bit = prbs_advance(&params->prbs_gen);
       V_prbs_d = (2.0f * (float32_t)prbs_bit - 1.0f) * params->prbs_amplitude_V;
   }
   
   // Run d-axis PI controller with PRBS feedforward
   pi_run_series(&params->pi_Id, Id_ref_A, Id_A, V_prbs_d, &Vd_V);
   
   // Run q-axis PI controller (no PRBS injection)
   pi_run_series(&params->pi_Iq, Iq_ref_A, Iq_A, 0.0f, &Vq_V);
   
   // D-axis RLS: counts 0, 16, 32, 48, ...
   if ((params->control_loop_count & rls_mask) == 0) {
       float32_t omega_elec = angle_observer_get_elec_speed_rad_s(&params->observer);
       
       // D-axis equation: Vd = Rs*Id + Ld*dId/dt + Vbias + Vdt*sign(Id) - ω*Lq*Iq
       // Pass Lq and Iq for cross-coupling compensation
       // Vd_V already includes PRBS via feedforward
       rls_motor_est_update(&params->rls_d, Vd_V, Id_A, params->Id_A_prev,
                            omega_elec, params->Lq_est, Iq_A);
       
       params->Id_A_prev = Id_A;
   }
   
   // Q-axis RLS: counts 8, 24, 40, 56, ... (staggered by offset)
   if ((params->control_loop_count & rls_mask) == rls_offset) {
       float32_t omega_elec = angle_observer_get_elec_speed_rad_s(&params->observer);
       
       // Q-axis equation: Vq = Rs*Iq + Lq*dIq/dt + Vbias + Vdt*sign(Iq) + ω*Ld*Id
       // Pass Ld and Id for cross-coupling compensation
       rls_motor_est_update(&params->rls_q, Vq_V, Iq_A, params->Iq_A_prev,
                            omega_elec, params->Ld_est, Id_A);
       
       params->Iq_A_prev = Iq_A;
       
       // Parameter synthesis (use most recent estimates from both axes)
       params->Rs_est = (params->rls_d.theta[0] + params->rls_q.theta[0]) / 2.0f;
       params->Ld_est = params->rls_d.theta[1];
       params->Lq_est = params->rls_q.theta[1];
       params->Vbias_est = (params->rls_d.theta[2] + params->rls_q.theta[2]) / 2.0f;
       params->Vdt_sign_est = (params->rls_d.theta[3] + params->rls_q.theta[3]) / 2.0f;
   }
   ```

**Testing:**
- ✅ Verify Ld ≠ Lq for hybrid stepper (should differ by 20-50%)
- ✅ Check both converge independently:
  - `trace_d(P) = P_d[0][0] + P_d[1][1] + P_d[2][2] + P_d[3][3]`
  - `trace_q(P) = P_q[0][0] + P_q[1][1] + P_q[2][2] + P_q[3][3]`
- ✅ ISR timing: measure peak at count 8 (d-axis update + q-axis update within 8 cycles)
- ✅ Rs estimates from d and q axes agree (±5%)
- ✅ Cross-coupling visible: omega_elec * Lq_est * Iq_A ≠ 0 during motion
- ✅ Both axes contribute to parameter synthesis

**Diagnostic Commands:**
```
motor rls status d         # D-axis: θ_d, trace(P_d), T_d
motor rls status q         # Q-axis: θ_q, trace(P_q), T_q
motor rls status all       # Combined: Rs, Ld, Lq, Vbias, Vdt_sign, temp
```

**Success Criteria:**
- Ld and Lq converge to distinct, stable values (Ld ≠ Lq)
- Rs estimates from both axes agree within 5%
- Peak ISR time <10μs (includes both RLS updates within stagger window)
- Cross-coupling terms properly compensated (visible in residuals)

---

### Phase 4: Thermal Model & Temperature Validation
**Goal**: Independent temperature monitoring and RLS validation

**Implementation:**
1. Create `thermal_model.h/c` from document lines 421-449
   - Copy `struct thermal_model` definition
   - Implement `thermal_model_init()` and `thermal_model_update()`
   - Add temperature validation logic

2. Add devicetree property (`rubus,user-parameters`):
   ```dts
   thermal-decimation = <2048>;  // ~10Hz at 20kHz ISR
   ```

3. Add to `motor_parameters`:
   ```c
   struct thermal_model thermal_model;
   float32_t T_winding_rls_C, T_winding_thermal_C;
   uint32_t thermal_decimation;  // From devicetree
   ```

4. Initialize in `CTRL_INIT`:
   ```c
   thermal_model_init(&params->thermal_model,
                      CONTROL_LOOP_FREQUENCY_HZ,
                      180.0f,    // tau_s (thermal time constant from DT or default)
                      25.0f,     // T_ambient_C (from DT or default)
                      params->rls_d.Rs0_ohm,  // Use same Rs0 as RLS
                      params->rls_d.T0_C,     // Use same T0 as RLS
                      0.00393f); // Copper temp coefficient
   
   params->thermal_decimation = 2048;  // From devicetree
   ```

5. Implement temperature helper for RLS:
   ```c
   float32_t rls_motor_est_get_temperature(const struct rls_motor_est *rls)
   {
       // Rs(T) = Rs0 * (1 + α*(T - T0))
       // Solve for T: T = T0 + (Rs/Rs0 - 1) / α
       float32_t Rs_ratio = rls->theta[0] / rls->Rs0_ohm;
       return rls->T0_C + (Rs_ratio - 1.0f) / rls->alpha;
   }
   ```

6. Update in ISR (low rate ~10Hz):
   ```c
   const uint32_t thermal_mask = params->thermal_decimation - 1;  // 0x7FF for 2048
   
   if ((params->control_loop_count & thermal_mask) == 0) {
       // Update thermal model with current I²R losses
       thermal_model_update(&params->thermal_model, Id_A, Iq_A, 
                            params->Rs_est, 25.0f);  // T_ambient from config
       
       // Calculate RLS temperature from d-axis Rs estimate
       params->T_winding_rls_C = rls_motor_est_get_temperature(&params->rls_d);
       params->T_winding_thermal_C = params->thermal_model.T_winding_C;
       
       // Validation: flag large discrepancies (>10°C default)
       float32_t temp_diff = fabsf(params->T_winding_rls_C - params->T_winding_thermal_C);
       if (temp_diff > params->thermal_model.validation_threshold_C) {
           params->thermal_model.validation_warning = true;
           // Log warning (not fault yet, could indicate Rs0@T0 miscalibration)
       } else {
           params->thermal_model.validation_warning = false;
       }
   }
   ```

**Testing:**
- ✅ Both temperatures start near ambient (T0)
- ✅ Temperatures track together during steady-state (±5°C typical)
- ✅ Thermal model lags RLS during transients (has thermal inertia τ)
- ✅ Temperature rises during sustained load (I²R heating visible)
- ✅ Cooling follows exponential decay (time constant ~180s typical)
- ✅ Validation warning triggers if discrepancy >10°C (check Rs0@T0 calibration)
- ✅ RLS temperature responds immediately to Rs changes (no thermal lag)

**Diagnostic Commands:**
```
motor temp                 # Show RLS vs thermal temps, discrepancy, warning
motor temp calibrate <T>   # Set Rs0@T0 anchor (updates both RLS and thermal)
motor thermal tau <s>      # Adjust thermal time constant (180-300s typical)
motor thermal ambient <T>  # Update ambient temperature reference
```

**Success Criteria:**
- Temperature tracking within ±5°C during steady-state operation
- Thermal model shows realistic heating/cooling curves (exponential)
- RLS temperature responds faster than thermal model (instantaneous vs τ lag)
- Validation warning only triggers during gross miscalibration (not normal drift)
- Both temperatures stay within safe limits (<120°C for typical motors)

---

### Phase 5: Gating & Robustness
**Goal**: Production-ready estimator with fault handling

**Implementation:**
1. Add gating checks to `motor_parameters`:
   ```c
   bool rls_enabled;  // Master enable/disable
   uint32_t rls_num_gated;  // Count of gated updates
   ```

2. Implement comprehensive gating logic before RLS updates:
   ```c
   // Master enable check
   bool rls_valid = params->rls_enabled;
   
   if (rls_valid) {
       // 1. Check motor state - only run during ONLINE operation
       if (params->state_for_isr != &motor_states[MOTOR_STATE_ONLINE]) {
           rls_valid = false;  // Freeze during ALIGN, RS_EST, ROVERL_MEAS, etc.
       }
       
       // 2. Check for PI saturation (commanded voltage ≠ applied voltage)
       if (params->pi_Id.saturated || params->pi_Iq.saturated) {
           rls_valid = false;
       }
       
       // 3. Check braking mode (Vbus-regulated short-circuit changes PWM)
       if (params->dc_bus_voltage_V > VBUS_REGEN_LIMIT_V) {
           float32_t speed_hz = angle_observer_get_mech_speed_hz(&params->observer);
           bool is_braking = (params->Iq_ref_A * speed_hz) < 0.0f;
           if (is_braking && fabsf(speed_hz) > 0.1f) {
               rls_valid = false;
           }
       }
       
       // 4. Observability check (sufficient PRBS excitation visible)
       if (fabsf(Id_A) < 0.05f) {  // 50mA minimum (below PRBS amplitude)
           rls_valid = false;  // Current too small, derivative unreliable
       }
       
       // 5. Voltage sanity check (prevent unrealistic measurements)
       // Vd_V already includes PRBS via feedforward, so check directly
       if (fabsf(Vd_V) > 0.95f * params->dc_bus_voltage_V) {
           rls_valid = false;  // Voltage command exceeds bus capability
       }
       
       // Track gating statistics
       if (!rls_valid) {
           params->rls_num_gated++;
       }
   }
   
   // D-axis RLS update (only if gating passes)
   // Vd_V already includes PRBS via feedforward
   if (rls_valid) {
       rls_motor_est_update(&params->rls_d, Vd_V, Id_A, params->Id_A_prev,
                            omega_elec, params->Lq_est, Iq_A);
   }
   ```

3. Add parameter bounds in `rls_motor_est_update()` (lines 623-626):
   ```c
   // Apply after theta update to prevent divergence
   rls->theta[0] = clampf(rls->theta[0], 0.5f, 50.0f);       // Rs: 0.5-50Ω
   rls->theta[1] = clampf(rls->theta[1], 0.0001f, 0.1f);     // L: 0.1-100mH
   rls->theta[2] = clampf(rls->theta[2], -5.0f, 5.0f);       // Vbias: ±5V
   rls->theta[3] = clampf(rls->theta[3], -2.0f, 2.0f);       // Vdt_sign: ±2V
   ```

4. Statistics tracking already in `struct rls_motor_est`:
   ```c
   uint32_t num_updates;        // Total RLS updates performed
   uint32_t num_rejected;       // Updates rejected (numerical issues)
   float32_t residual_sum_sq;   // Sum of squared residuals (for RMS)
   ```
   Add in ISR update:
   ```c
   // Increment update counter inside rls_motor_est_update()
   rls->num_updates++;
   
   // Track residual for diagnostics
   rls->residual_sum_sq += error * error;
   ```

**Testing:**
- ✅ RLS freezes during ALIGN, RS_EST, ROVERL_MEAS states
- ✅ RLS freezes during regen braking (Vbus > VBUS_REGEN_LIMIT_V)
- ✅ RLS freezes when current PI saturates
- ✅ RLS skips updates when Id < 50mA (below excitation threshold)
- ✅ Parameters stay within bounds during transients and faults
- ✅ Acceptance rate >90% during normal ONLINE operation
- ✅ Statistics counters increment correctly

**Diagnostic Commands:**
```
motor rls enable           # Enable RLS updates
motor rls disable          # Disable RLS updates (freeze parameters)
motor rls stats            # Show num_updates, num_rejected, num_gated
motor rls stats d          # D-axis statistics
motor rls stats q          # Q-axis statistics
motor rls residuals        # Show residual RMS for both axes
motor rls reset_stats      # Clear all counters
motor rls bounds           # Show current parameter bounds
```

**Success Criteria:**
- No parameter divergence during any operating mode (alignment, braking, saturation)
- High update acceptance rate (>90%) during normal ONLINE/FOC operation
- Gating triggers correctly during special modes
- Residuals remain small (<1V RMS) after convergence
- Parameters respect bounds at all times (no overflow/NaN)

---

### Phase 6: Shell Interface & Production Tuning
**Goal**: User-friendly diagnostics and runtime adjustment

**Implementation:**
1. Add comprehensive shell commands (`motor_shell.c`):
   ```c
   // RLS control
   motor rls enable                      # Enable RLS updates
   motor rls disable                     # Disable RLS updates
   motor rls reset                       # Reset both P matrices to initial values
   motor rls reset d                     # Reset d-axis P matrix only
   motor rls reset q                     # Reset q-axis P matrix only
   
   // RLS status and diagnostics
   motor rls status                      # Combined view: Rs, Ld, Lq, Vbias, Vdt_sign, temp
   motor rls status d                    # D-axis: θ_d[4], trace(P_d), converged, T_d
   motor rls status q                    # Q-axis: θ_q[4], trace(P_q), converged, T_q
   motor rls stats                       # Update/reject/gate counts (both axes)
   motor rls stats d                     # D-axis statistics only
   motor rls stats q                     # Q-axis statistics only
   motor rls residuals                   # Residual RMS for both axes
   motor rls reset_stats                 # Clear all statistics counters
   
   // Runtime tuning
   motor rls lambda <value>              # Adjust forgetting factor (0.999-0.9999)
   motor rls prbs <millivolts>           # Adjust PRBS amplitude (500-2000mV)
   motor rls decimation <value>          # Adjust RLS decimation (8/16/32/64)
   motor rls offset <value>              # Adjust stagger offset (typically decimation/2)
   
   // Temperature calibration
   motor temp                            # Show RLS vs thermal temps, discrepancy
   motor temp calibrate <T_C>            # Set Rs0@T0 anchor at current Rs
   motor thermal tau <seconds>           # Adjust thermal time constant (120-300s)
   motor thermal ambient <T_C>           # Update ambient temperature reference
   motor thermal decimation <value>      # Adjust thermal update rate (1024/2048/4096)
   ```

2. Add telemetry logging (optional, for development):
   ```c
   // Log every second during convergence testing
   if ((params->control_loop_count % 20000) == 0) {
       // Calculate trace(P) for both axes
       float32_t trace_d = params->rls_d.P[0][0] + params->rls_d.P[1][1] + 
                           params->rls_d.P[2][2] + params->rls_d.P[3][3];
       float32_t trace_q = params->rls_q.P[0][0] + params->rls_q.P[1][1] + 
                           params->rls_q.P[2][2] + params->rls_q.P[3][3];
       
       LOG_INF("RLS: Rs=%.3fΩ Ld=%.1fmH Lq=%.1fmH T=%.1f°C trace_d=%.6f trace_q=%.6f",
               params->Rs_est, 
               params->Ld_est * 1000.0f, 
               params->Lq_est * 1000.0f,
               params->T_winding_rls_C,
               trace_d, trace_q);
       
       // Log statistics
       uint32_t total_updates = params->rls_d.num_updates + params->rls_q.num_updates;
       uint32_t total_rejected = params->rls_d.num_rejected + params->rls_q.num_rejected;
       float32_t acceptance_rate = 100.0f * (float32_t)(total_updates - total_rejected) / 
                                   (float32_t)total_updates;
       
       LOG_INF("RLS stats: updates=%u rejected=%u gated=%u acceptance=%.1f%%",
               total_updates, total_rejected, params->rls_num_gated, acceptance_rate);
   }
   ```

3. Production tuning checklist:
   - [ ] **PRBS amplitude**: Start at 1000mV, adjust for 0.1-0.2A Id ripple visible
   - [ ] **Forgetting factor**: Start at 0.9999, decrease to 0.999 if parameters drift
   - [ ] **Initial P matrix**: Diagonal [1.0, 1.0, 1.0, 1.0] for 4-param, larger if uncertain
   - [ ] **Decimation rate**: 16 recommended (1.25kHz), increase to 32 if ISR overloaded
   - [ ] **Stagger offset**: decimation/2 for optimal load spreading
   - [ ] **Parameter bounds**: 
     - Rs: [0.5Ω, 50Ω] (adjust based on motor size)
     - Ld/Lq: [0.1mH, 100mH] (adjust based on motor type)
     - Vbias: [-5V, +5V] (captures dead-time + offsets)
     - Vdt_sign: [-2V, +2V] (current-dependent drops)
   - [ ] **Gating thresholds**:
     - Id_min: 50mA (below PRBS amplitude)
     - Voltage margin: 0.95 × Vbus (safety factor)
   - [ ] **Thermal time constant**: Measure via cooldown test (heat motor, log Rs decay)
   - [ ] **Convergence threshold**: trace(P) < 0.1 for "converged" flag
   - [ ] **Rs0@T0 calibration**: Use DC injection at known ambient temperature

4. Implementation example for shell command handler:
   ```c
   static int cmd_motor_rls_status(const struct shell *sh, size_t argc, char **argv)
   {
       if (argc == 1) {
           // Combined view
           shell_print(sh, "RLS Combined Status:");
           shell_print(sh, "  Rs:      %.3f Ω", params->Rs_est);
           shell_print(sh, "  Ld:      %.2f mH", params->Ld_est * 1000.0f);
           shell_print(sh, "  Lq:      %.2f mH", params->Lq_est * 1000.0f);
           shell_print(sh, "  Vbias:   %.2f V", params->Vbias_est);
           shell_print(sh, "  Vdt_sign:%.2f V", params->Vdt_sign_est);
           shell_print(sh, "  Temp:    %.1f °C", params->T_winding_rls_C);
           shell_print(sh, "  Converged: %s", 
                       (params->rls_d.converged && params->rls_q.converged) ? "Yes" : "No");
       } else if (strcmp(argv[1], "d") == 0) {
           // D-axis details
           float32_t trace_d = params->rls_d.P[0][0] + params->rls_d.P[1][1] + 
                               params->rls_d.P[2][2] + params->rls_d.P[3][3];
           shell_print(sh, "D-axis RLS:");
           shell_print(sh, "  θ = [%.3f, %.5f, %.2f, %.2f]", 
                       params->rls_d.theta[0], params->rls_d.theta[1],
                       params->rls_d.theta[2], params->rls_d.theta[3]);
           shell_print(sh, "  trace(P): %.6f", trace_d);
           shell_print(sh, "  Converged: %s (at update %u)", 
                       params->rls_d.converged ? "Yes" : "No",
                       params->rls_d.convergence_count);
       }
       // Similar for q-axis...
       return 0;
   }
   ```

**Testing:**
- ✅ All shell commands work and display formatted output
- ✅ Runtime adjustments take effect immediately (next ISR cycle)
- ✅ Logging captures convergence behavior (trace(P) decreases)
- ✅ Parameter estimates stable over hours of operation (±2% drift max)
- ✅ Temperature tracking accurate vs external thermocouple (±5°C)
- ✅ Can adjust PRBS amplitude and see Id ripple change in real-time
- ✅ Lambda adjustment affects convergence speed as expected
- ✅ Statistics counters accurate and reset properly

**Success Criteria:**
- Complete shell interface functional and user-friendly
- Can tune all RLS parameters at runtime without recompilation
- Parameter estimates stable over long-term operation (8+ hours continuous)
- Acceptance rate >90% during normal operation
- Temperature estimates match external measurement within ±5°C
- Ready for field deployment and production use

---

### Validation Checklist (All Phases)

**Convergence:**
- [ ] Rs converges within 15-30s to ±5% of offline measurement
- [ ] Ld/Lq converge within 15-30s to reasonable values
- [ ] trace(P) decreases monotonically, reaches <0.1
- [ ] Convergence detected automatically via threshold

**Accuracy:**
- [ ] Rs matches offline DC injection test (±5%)
- [ ] Temperature from Rs matches thermal model (±5°C)
- [ ] Ld and Lq distinct for hybrid stepper (20-50% difference)
- [ ] Vbias small and stable (<2V typical)

**Robustness:**
- [ ] No divergence during transients (step commands)
- [ ] No divergence during braking or saturation
- [ ] Parameters bounded and physically plausible
- [ ] High update acceptance rate (>90%)

**Performance:**
- [ ] ISR timing <10μs (peak with both RLS updates)
- [ ] No audible noise or torque ripple from PRBS
- [ ] FOC control quality unchanged with RLS running
- [ ] Convergence time <30s from cold start

**Diagnostics:**
- [ ] Shell commands provide useful real-time data
- [ ] Temperature warning system functional
- [ ] Residuals and statistics logged
- [ ] Can tune parameters at runtime

---

### Timeline Estimate

| Phase | Effort | Elapsed Time |
|-------|--------|--------------|
| Phase 1: PRBS | 4-8 hours | 1-2 days |
| Phase 2: D-axis RLS | 8-16 hours | 2-3 days |
| Phase 3: Dual-axis | 4-8 hours | 1-2 days |
| Phase 4: Thermal | 4-8 hours | 1 day |
| Phase 5: Gating | 8-16 hours | 2-3 days |
| Phase 6: Polish | 4-8 hours | 1-2 days |
| **Total** | **32-64 hours** | **1-2 weeks** |

**Note**: Timeline assumes familiarity with codebase and one developer working part-time. Add buffer for integration issues and hardware testing.
