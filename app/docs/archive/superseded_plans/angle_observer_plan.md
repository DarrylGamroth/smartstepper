# Angle Tracking Observer Plan

## Overview

The motor controller uses an absolute encoder (e.g. AEAT9955) to measure rotor position. The encoder driver outputs angles in Q31 fixed-point format, where the range `[-1, 0.999999]` corresponds to `[-180°, 179°]` for use with trigonometric functions. We want an angle tracking observer that consumes these raw encoder readings and produces:

- Mechanical angle (deg)
- Electrical angle (deg)
- Mechanical angular velocity (deg/s)

All internal observer math will be done in **degrees**, which matches the trigonometric conventions used elsewhere (e.g. CMSIS `arm_sin_cos_f32`).

The observer will evolve in two stages:

1. **Stage 1**: Lightweight α–β (angle/velocity) tracking observer driven directly by encoder angle.
2. **Stage 2**: Higher-order, PLL‑based type‑3 angle tracking observer using motor model information for ultra slow speed performance.

---

## Inputs and Outputs

### Inputs

- Encoder mechanical angle in `float32_t` degrees from the encoder library:
	- Range: approximately `[-180°, 179°]`
	- Source: Q31 → float conversion done in the encoder path (e.g. AEAT9955 decoder)
- Configuration parameters from devicetree:
	- `pole_pairs`: motor pole pairs from `rubus,motor-parameters`
	- `sample_period_s`: control loop period derived from `control-loop-frequency-hz` in `user_parameters`
	- `observer_bandwidth_hz`: desired angle observer bandwidth from a dedicated observer node

### Outputs (directly usable with `arm_sin_cos_f32`)

All angle outputs are in **degrees**. `arm_sin_cos_f32` internally normalizes its degree input, so these values can be passed to it directly with no additional wrapping required.

- `mech_angle_deg`        — filtered mechanical angle for the current cycle
- `elec_angle_deg`        — filtered electrical angle for the current cycle
- `mech_speed_dps`        — mechanical angular velocity in degrees per second (deg/s)
- `mech_angle_pred_deg`   — predicted mechanical angle for the *next* control cycle
- `elec_angle_pred_deg`   — predicted electrical angle for the *next* control cycle

Electromechanical conversion for both current and predicted angles:

- `elec_angle_deg      = wrap_360(mech_angle_deg * pole_pairs)`
- `elec_angle_pred_deg = wrap_360(mech_angle_pred_deg * pole_pairs)`

---

## Stage 1: α–β Tracking Observer

### Rationale

The first implementation should be:

- Simple and numerically robust
- Able to smooth velocity estimates compared to raw differentiation
- Straightforward to tune via a single bandwidth parameter

A classic α–β (position/velocity) observer fits these requirements. It treats encoder angle as a noisy position measurement and estimates both position and velocity.

### State and Configuration

Proposed structures in a new module (e.g. `angle_observer.h/.c`):

```c
struct angle_observer_state {
	/* Current-cycle outputs (directly usable with arm_sin_cos_f32) */
	float32_t mech_angle_deg;        /* [0, 360) */
	float32_t elec_angle_deg;        /* [0, 360) */
	float32_t mech_speed_dps;        /* deg/s */

	/* One-step prediction outputs (for next control cycle) */
	float32_t mech_angle_pred_deg;   /* [0, 360) */
	float32_t elec_angle_pred_deg;   /* [0, 360) */

	/* Internal */
	float32_t angle_est_deg;         /* internal unwrapped estimate */
	float32_t speed_est_dps;         /* internal speed estimate */
	float32_t sample_period_s;       /* cached Ts from configuration */
	float32_t bandwidth_hz;          /* cached observer bandwidth from devicetree */
	uint32_t  pole_pairs;            /* cached from motor_parameters */
	bool      initialized;
};
```

**API (single-context, non-reentrant):**

```c
void angle_observer_init(struct angle_observer_state *obs,
			float32_t sample_period_s,
			float32_t bandwidth_hz,
			uint32_t pole_pairs);

void angle_observer_update(struct angle_observer_state *obs,
			float32_t encoder_angle_deg);

/* Accessors for angles and speeds */
static inline float32_t angle_observer_get_mech_angle_deg(
	const struct angle_observer_state *obs)
{
	return obs->mech_angle_deg;
}

static inline float32_t angle_observer_get_mech_angle_pred_deg(
	const struct angle_observer_state *obs)
{
	return obs->mech_angle_pred_deg;
}

static inline float32_t angle_observer_get_elec_angle_deg(
	const struct angle_observer_state *obs)
{
	return obs->elec_angle_deg;
}

static inline float32_t angle_observer_get_elec_angle_pred_deg(
	const struct angle_observer_state *obs)
{
	return obs->elec_angle_pred_deg;
}

/* Mechanical speed for outer velocity controller (mechanical Hz, rev/s) */
static inline float32_t angle_observer_get_mech_speed_hz(
	const struct angle_observer_state *obs)
{
	return obs->mech_speed_dps / 360.0f;
}
```

The observer is intended to be called from a single control-loop context
(e.g. ISR or dedicated control task) and is **not** thread-safe or
re-entrant. Callers must ensure exclusive access to
`struct angle_observer_state`. Higher-level code should prefer these
accessors over directly touching the state fields.

### Encoder Angle Input

The encoder library is responsible for converting the raw Q31 angle to `float32_t` degrees. The observer receives:

- `encoder_angle_deg` in approximately `[-180°, 179°]`

This value is treated as the **mechanical angle measurement**. The observer handles wrap and filtering; no further scaling is required before using the outputs with `arm_sin_cos_f32`.

### Observer Equations

We define the continuous-time observer conceptually as:

- States: angle $\hat{\theta}$ (deg), speed $\hat{\omega}$ (deg/s)
- Measured angle: $\theta_\text{meas}$ (deg)
- Sampling period: $T_s$
- Observer bandwidth: $f_o$ (Hz), $\omega_o = 2\pi f_o$ (rad/s)

A standard 2nd-order tracking observer can be written as:

$$
\begin{aligned}
\dot{\hat{\theta}} &= \hat{\omega} + L_1 e \\
\dot{\hat{\omega}} &= L_2 e
\end{aligned}
$$

with error:

$$
 e = \text{wrap}(\theta_\text{meas} - \hat{\theta})
$$

and gains chosen from the desired bandwidth (critically damped case):

$$
 L_1 = 2\omega_o, \quad L_2 = \omega_o^2.
$$

We discretize with forward Euler:

$$
\begin{aligned}
\hat{\theta}_{k+1} &= \hat{\theta}_k + T_s \hat{\omega}_k + T_s L_1 e_k \\
\hat{\omega}_{k+1} &= \hat{\omega}_k + T_s L_2 e_k
\end{aligned}
$$

Implementation outline:

```c
static float32_t wrap_deg_180(float32_t angle)
{
	/* Map angle to (-180, 180] */
	while (angle > 180.0f) {
		angle -= 360.0f;
	}
	while (angle <= -180.0f) {
		angle += 360.0f;
	}
	return angle;
}

static float32_t wrap_deg_360(float32_t angle)
{
	/* Map angle to [0, 360) */
	while (angle >= 360.0f) {
		angle -= 360.0f;
	}
	while (angle < 0.0f) {
		angle += 360.0f;
	}
	return angle;
}

void angle_observer_update(struct angle_observer_state *obs,
			float32_t encoder_angle_deg)
{
	const float32_t Ts = obs->sample_period_s;
	const float32_t wo = 2.0f * PI_F32 * obs->bandwidth_hz;
	const float32_t L1 = 2.0f * wo;
	const float32_t L2 = wo * wo;

	/* Compute wrapped error in (-180, 180] */
	float32_t err_deg = wrap_deg_180(encoder_angle_deg - obs->angle_est_deg);

	/* Observer update */
	obs->angle_est_deg += Ts * obs->speed_est_dps + Ts * L1 * err_deg;
	obs->speed_est_dps += Ts * L2 * err_deg;

	/* Current-cycle outputs */
	obs->mech_angle_deg = wrap_deg_360(obs->angle_est_deg);
	obs->mech_speed_dps = obs->speed_est_dps;
	obs->elec_angle_deg = wrap_deg_360(obs->mech_angle_deg * obs->pole_pairs);

	/* One-step prediction for next control cycle */
	float32_t mech_angle_pred = obs->angle_est_deg + Ts * obs->speed_est_dps;
	obs->mech_angle_pred_deg = wrap_deg_360(mech_angle_pred);
	obs->elec_angle_pred_deg = wrap_deg_360(obs->mech_angle_pred_deg * obs->pole_pairs);
}
```

Initialization:

- `angle_observer_init` must be called before `angle_observer_update`.
- `angle_observer_init` sets `angle_est_deg = 0`, `speed_est_dps = 0`, all
	public outputs (`mech_angle_deg`, `elec_angle_deg`, `mech_speed_dps`,
	`mech_angle_pred_deg`, `elec_angle_pred_deg`) to 0, caches
	`sample_period_s`, `bandwidth_hz`, and `pole_pairs` into the state, and
	marks `initialized = true`.
- After init, each call to `angle_observer_update` only requires the
	current state and `encoder_angle_deg`.

---

## Stage 2: Type‑3 PLL‑Based Angle Observer (Future Work)

For ultra-low-speed, low-noise angle estimation, a more sophisticated type‑3 tracking observer with a PLL structure can be implemented later.

### High-Level Idea

- Use the motor’s electrical model (stator voltages, currents, and known inductance/resistance/flux linkage) to build a virtual back‑EMF vector.
- Run a PLL that forces the estimated electrical angle to align with this vector.
- Combine encoder information as a reference/correction path to reduce drift and improve startup/transient behavior.

### Benefits

- Much smoother electrical angle and speed, especially at ultra low speeds where raw encoder quantization/phase jitter can be significant.
- Potential robustness against short encoder glitches or missing counts.

### Design Sketch (to be refined later)

- States:
  - $\hat{\theta}_e$ (electrical angle, deg)
  - $\hat{\omega}_e$ (electrical speed, deg/s)
  - Additional integrator state(s) for type‑3 compensation
- Inputs:
  - Measured currents (Id/Iq or Ia/Ib)
  - Applied voltages (Vd/Vq or Va/Vb)
  - Encoder angle as a reference
- Output:
  - Same as stage 1, but with improved filtering and robustness

This observer would likely live in its own module and share the same public interface as the stage‑1 observer, so the rest of the control code can switch between them via configuration.

---

## Integration Plan

1. **Create module** `angle_observer.h/.c` in the application:
   - Implement the stage‑1 α–β observer as above.
   - Use degrees throughout to match `arm_sin_cos_f32`.
2. **Hook into ADC/control loop** later:
   - Convert the encoder’s Q31 output to degrees and feed it into `angle_observer_update()` each control tick.
   - Use `elec_angle_deg` for Park/Clarke transforms and `mech_speed_dps` for any speed loops or diagnostics.
3. **Configuration (devicetree-only)**:
	- All observer configuration is provided via devicetree; there is no separate Kconfig.
	- Observer bandwidth and related settings come from a dedicated observer node.
	- Pole pairs come from the existing `rubus,motor-parameters` node.
	- The choice of `bandwidth-hz` is entirely application-defined; for a
	  control-loop frequency of, for example, 20 kHz (`sample_period_s = 50 µs`),
	  a bandwidth in the range of 50–300 Hz is typically reasonable, with
	  higher values giving tighter tracking but more encoder-noise following.
4. **Future upgrade**:
   - Add a compile‑time or runtime switch between **simple α–β** and **PLL‑based type‑3** observers.
   - Keep the same public API so the rest of the code remains unchanged.

---

## Example Devicetree Node

An example observer configuration node in the application overlay might look like this:

```dts
	angle_observer: angle_observer {
		compatible = "rubus,angle-observer";
		/* Observer bandwidth in Hz (position tracking) */
		bandwidth-hz = <300>;
	};
```

Usage in C (via `config.h`):

```c
#define ANGLE_OBSERVER_NODE DT_PATH(angle_observer)
#define ANGLE_OBSERVER_BANDWIDTH_HZ \
	((float32_t)DT_PROP(ANGLE_OBSERVER_NODE, bandwidth_hz))

struct angle_observer_state obs;

angle_observer_init(&obs,
		      1.0f / CONTROL_LOOP_FREQUENCY_HZ,
		      ANGLE_OBSERVER_BANDWIDTH_HZ,
		      MOTOR_POLE_PAIRS);

/* In the control loop: */
angle_observer_update(&obs, encoder_angle_deg);
```

This keeps all runtime-tunable parameters in devicetree and ensures the observer can be reconfigured per board or application variant without changing code.
