/**
 * @file emergency_switch.h
 * @brief Emergency Switch Digital Signal Processing Header
 * @details Reliable noise filtering for emergency switch to prevent false triggering
 *
 * Problem: Emergency switch triggers by itself due to electrical noise
 * Solution: Digital low-pass filter + debouncing + hysteresis
 *
 * @author Generated for STM32 Applications
 * @date 2025
 * @version 1.0
 */

#ifndef EMERGENCY_SWITCH_H
#define EMERGENCY_SWITCH_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes */
#include <stdint.h>
#include <stdbool.h>

/* Configuration Constants */
#define EMERGENCY_SAMPLE_RATE_HZ    1000.0f     /**< Sampling frequency (1kHz) */
#define EMERGENCY_FILTER_CUTOFF_HZ  10.0f       /**< Low-pass filter cutoff (10Hz) */
#define EMERGENCY_DEBOUNCE_TIME_MS  50          /**< Debounce time (50ms) */
#define EMERGENCY_ON_THRESHOLD      0.7f        /**< Turn ON threshold */
#define EMERGENCY_OFF_THRESHOLD     0.3f        /**< Turn OFF threshold */

/**
 * @brief Emergency Switch Filter Structure
 */
typedef struct {
    float alpha;                /**< Filter coefficient */
    float filtered_value;       /**< Current filtered signal value */
    uint8_t stable_state;       /**< Current stable switch state (0 or 1) */
    uint32_t debounce_count;    /**< Debounce sample counter */
    uint32_t debounce_limit;    /**< Debounce limit in samples */
    uint8_t initialized;        /**< Initialization flag */
} emergency_switch_t;

/* Function Prototypes */

/**
 * @brief Initialize emergency switch filter
 * @param filter Pointer to emergency switch structure
 * @param sample_rate_hz Sampling frequency in Hz
 * @param cutoff_hz Filter cutoff frequency in Hz
 * @param debounce_ms Debounce time in milliseconds
 */
void emergency_switch_init(emergency_switch_t *filter,
                          float sample_rate_hz,
                          float cutoff_hz,
                          uint32_t debounce_ms);

/**
 * @brief Process emergency switch sample
 * @param filter Pointer to emergency switch structure
 * @param raw_switch_state Raw switch reading (0 or 1)
 * @return 1 if state changed, 0 if no change
 */
uint8_t emergency_switch_update(emergency_switch_t *filter, uint8_t raw_switch_state);

/**
 * @brief Get current switch state
 * @param filter Pointer to emergency switch structure
 * @return Current switch state (0 = released, 1 = pressed)
 */
uint8_t emergency_switch_get_state(const emergency_switch_t *filter);

/**
 * @brief Check if switch is currently pressed
 * @param filter Pointer to emergency switch structure
 * @return true if pressed, false if released
 */
bool emergency_switch_is_pressed(const emergency_switch_t *filter);

/**
 * @brief Get filtered signal level (for diagnostics)
 * @param filter Pointer to emergency switch structure
 * @return Filtered signal value (0.0 to 1.0)
 */
float emergency_switch_get_signal_level(const emergency_switch_t *filter);

/**
 * @brief Reset switch filter to initial state
 * @param filter Pointer to emergency switch structure
 */
void emergency_switch_reset(emergency_switch_t *filter);

/* Convenience Macros */

/**
 * @brief Standard initialization macro (1kHz, 10Hz cutoff, 50ms debounce)
 */
#define EMERGENCY_SWITCH_INIT_STANDARD(filter) \
    emergency_switch_init(filter, EMERGENCY_SAMPLE_RATE_HZ, \
                         EMERGENCY_FILTER_CUTOFF_HZ, EMERGENCY_DEBOUNCE_TIME_MS)

/**
 * @brief Fast response initialization (1kHz, 20Hz cutoff, 20ms debounce)
 */
#define EMERGENCY_SWITCH_INIT_FAST(filter) \
    emergency_switch_init(filter, EMERGENCY_SAMPLE_RATE_HZ, 20.0f, 20)

/**
 * @brief Robust initialization (1kHz, 5Hz cutoff, 100ms debounce)
 */
#define EMERGENCY_SWITCH_INIT_ROBUST(filter) \
    emergency_switch_init(filter, EMERGENCY_SAMPLE_RATE_HZ, 5.0f, 100)

#ifdef __cplusplus
}
#endif

#endif /* EMERGENCY_SWITCH_H */
