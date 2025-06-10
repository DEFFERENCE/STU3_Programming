/**
 * @file emergency_switch.c
 * @brief Emergency Switch Digital Signal Processing Implementation
 * @details Noise filtering implementation to prevent false emergency triggers
 *
 * @author Generated for STM32 Applications
 * @date 2025
 * @version 1.0
 */

#include "emergency_switch.h"
#include <math.h>

/* Private Constants */
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* Private Function Prototypes */
static float calculate_filter_coefficient(float sample_rate_hz, float cutoff_hz);
static float clamp_float(float value, float min_val, float max_val);

/* Public Functions */

void emergency_switch_init(emergency_switch_t *filter,
                          float sample_rate_hz,
                          float cutoff_hz,
                          uint32_t debounce_ms)
{
    if (filter == NULL || sample_rate_hz <= 0.0f || cutoff_hz <= 0.0f) {
        return;
    }

    /* Calculate low-pass filter coefficient */
    filter->alpha = calculate_filter_coefficient(sample_rate_hz, cutoff_hz);

    /* Initialize state */
    filter->filtered_value = 0.0f;
    filter->stable_state = 0;
    filter->debounce_count = 0;

    /* Calculate debounce limit in samples */
    filter->debounce_limit = (uint32_t)((debounce_ms * sample_rate_hz) / 1000.0f);
    if (filter->debounce_limit < 5) {
        filter->debounce_limit = 5;  /* Minimum 5 samples */
    }

    filter->initialized = 1;
}

uint8_t emergency_switch_update(emergency_switch_t *filter, uint8_t raw_switch_state)
{
    if (filter == NULL || !filter->initialized) {
        return 0;
    }

    /* Convert binary input to float */
    float raw_signal = (raw_switch_state > 0) ? 1.0f : 0.0f;

    /* Apply low-pass filter (RC filter equation) */
    filter->filtered_value = filter->alpha * raw_signal +
                             (1.0f - filter->alpha) * filter->filtered_value;

    /* Clamp filtered value to valid range */
    filter->filtered_value = clamp_float(filter->filtered_value, 0.0f, 1.0f);

    /* Determine desired state using hysteresis */
    uint8_t desired_state;
    if (filter->stable_state == 0) {
        /* Currently OFF - need strong signal to turn ON */
        desired_state = (filter->filtered_value > EMERGENCY_ON_THRESHOLD) ? 1 : 0;
    } else {
        /* Currently ON - need weak signal to turn OFF */
        desired_state = (filter->filtered_value > EMERGENCY_OFF_THRESHOLD) ? 1 : 0;
    }

    /* Debouncing logic */
    if (desired_state == filter->stable_state) {
        /* State is stable - reset counter */
        filter->debounce_count = 0;
        return 0;  /* No state change */
    } else {
        /* State wants to change - count debounce samples */
        filter->debounce_count++;

        if (filter->debounce_count >= filter->debounce_limit) {
            /* Debounce time elapsed - confirm state change */
            filter->stable_state = desired_state;
            filter->debounce_count = 0;
            return 1;  /* State changed */
        }

        return 0;  /* Still debouncing */
    }
}

uint8_t emergency_switch_get_state(const emergency_switch_t *filter)
{
    if (filter == NULL || !filter->initialized) {
        return 0;
    }
    return filter->stable_state;
}

bool emergency_switch_is_pressed(const emergency_switch_t *filter)
{
    if (filter == NULL || !filter->initialized) {
        return false;
    }
    return (filter->stable_state == 1);
}

float emergency_switch_get_signal_level(const emergency_switch_t *filter)
{
    if (filter == NULL || !filter->initialized) {
        return 0.0f;
    }
    return filter->filtered_value;
}

void emergency_switch_reset(emergency_switch_t *filter)
{
    if (filter == NULL) {
        return;
    }

    /* Reset state but keep configuration */
    filter->filtered_value = 0.0f;
    filter->stable_state = 0;
    filter->debounce_count = 0;
}

/* Private Functions */

static float calculate_filter_coefficient(float sample_rate_hz, float cutoff_hz)
{
    /* Calculate RC low-pass filter coefficient */
    /* alpha = dt / (tau + dt), where tau = 1/(2*pi*fc) */

    float tau = 1.0f / (2.0f * M_PI * cutoff_hz);
    float dt = 1.0f / sample_rate_hz;
    float alpha = dt / (tau + dt);

    /* Clamp to reasonable range */
    return clamp_float(alpha, 0.001f, 0.5f);
}

static float clamp_float(float value, float min_val, float max_val)
{
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}
