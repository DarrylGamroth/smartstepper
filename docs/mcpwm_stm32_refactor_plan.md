# MCPWM STM32 Refactor Plan

## Background
Our `mcpwm_stm32` fork predates the large clean-up that recently landed in Zephyr's upstream `pwm_stm32` driver. The upstream work modernised timer initialisation, centralised channel helpers, and simplified breakpoint handling. The MCPWM driver still carries the older scaffolding, which makes future rebases painful and hides real functional deltas (break input chaining, high-rate duty-cycle helpers, etc.).

## Objectives
- **Match upstream structure:** bring the shared scaffolding (device data/config layout, helper tables, IRQ glue, logging style) in line with the refreshed `pwm_stm32` driver while keeping MCPWM-only behaviour.
- **Preserve MCPWM extras:** maintain complementary-channel handling, master/slave chaining, FOC helper APIs, and break callbacks exactly as they exist today.
- **Ease future diffs:** keep our local files self-contained so that future comparisons against upstream Zephyr remain as simple as `git diff` between the two directories.

## Proposed Change Set

### 1. Sync the shared timer plumbing
- Copy the upstream helper arrays (`ch2ll`, complementary maps, compare setters) and counter-mode utilities verbatim into `mcpwm_stm32.c`, adjusting names only when the MCPWM driver already exports a symbol (e.g. keep the existing `mcpwm_stm32_set_timer_compare` for the inline helpers).
- Align log messages, error paths, and clock-control sequencing with upstream so behavioural diffs stand out.
- **Local copy note:** we will paste the upstream snippets directly into our MCPWM sources rather than sharing a header. That keeps our driver independent while still making a future diff against upstream straightforward.

### 2. Harmonise configuration and data structs
- Reorder the members of `struct mcpwm_stm32_config` and `struct mcpwm_stm32_data` to match upstream where fields overlap (`timer`, `prescaler`, `countermode`, `pclken`, `pcfg`, `reset`, etc.).
- Preserve the MCPWM-only fields (break/dead-time knobs, trigger settings, repetition counter, cached period) exactly as they are today.
- **"Unused legacy fields" clarification:** this phrase refers to members that trace back to the pre-refactor layout but stop feeding any logic after we adopt the upstream initialisation flow. Think of items such as duplicate cached period values or DT flags whose semantics moved upstream. We have not identified removals yet; each candidate will be called out explicitly with justification before it is dropped. Anything that still powers inline fast paths (`period_cycles_x2`), break handling, or DT-exposed behaviour will remain in place.

### 3. Adopt upstream initialisation flow
- Mirror the upstream order of operations: enable clocks, toggle reset, apply pinctrl, compute the period, initialise the timer, then configure master/slave and BDTR features.
- Integrate MCPWM-specific paths (complementary outputs, trigger polarity, break2) into the same flow so that diffs to upstream stay tight.

### 4. Share helpers with inline fast paths safely
- Move the compare-setter table and any other helper required by `include/drivers/pwm/mcpwm_stm32.h` behind a single non-static declaration so the inline fast paths can reuse the upstream-aligned code without duplicating logic.
- Document this linkage in both the C file and the header so future diffs know exactly why those symbols remain exported.

### 5. Record upstream baseline for diffs
- Drop a short note (and optional commit hash) in this document that identifies the upstream Zephyr revision we mirrored. When we refresh again, we can diff against that recorded hash to see only the new churn.
- Keep our forked files in-place (`drivers/pwm/mcpwm_stm32.c` and `include/drivers/pwm/mcpwm_stm32.h`) rather than symlinking or including upstream sources. This honours the request to rely on local copies and keeps future diffs trivial.

## Implementation Notes
- Every functional change will ship with unit or integration validation where possible (e.g. exercising complementary-channel enable/disable or break ISR callbacks).
- We will only retire struct members after confirming they are completely unused, and each removal will call out the rationale in the commit message to avoid confusion.
- No Zephyr tree files will be modified in-place; all updates land in the `chopper/` fork so west updates remain straightforward.

## Next Steps
1. Mirror the upstream helper sections and reconcile naming with our exported inline APIs.
2. Align the init/config path while proving that MCPWM-only behaviours still compile and run (serial shell build via `West build Serial Shell (app)`).
3. Audit remaining struct members, documenting any removals directly in this file for posterity.
4. Capture the upstream commit hash used for the sync in this document as soon as the code lands.
