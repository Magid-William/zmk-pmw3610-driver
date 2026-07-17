#pragma once

/**
 * @file pixart.h
 *
 * @brief Common header file for all optical motion sensor by PIXART
 */

#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>

#ifdef __cplusplus
extern "C" {
#endif

/* device data structure */
struct pixart_data {
    const struct device          *dev;
    int64_t                      dx;
    int64_t                      dy;
#if CONFIG_PMW3610_ALT_REPORT_INTERVAL_MIN > 0
    int64_t                      last_smp_time;
    int64_t                      last_rpt_time;
#endif
    bool                         sw_smart_flag; // for pmw3610 smart algorithm

#if CONFIG_PMW3610_ALT_LAYER_TOGGLE
    bool                         layer_toggle_layer_enabled;
    int64_t                      layer_toggle_last_motion_time;
    struct k_work_delayable      layer_toggle_deactivation_work;
#endif

    struct gpio_callback         irq_gpio_cb; // motion pin irq callback
    struct k_work                trigger_work; // realtrigger job

    struct k_work_delayable      init_work; // the work structure for delayable init steps
    int                          async_init_step;

    bool                         ready; // whether init is finished successfully
    int                          err; // error code during async init

    // acceleration state — smooths &mmv-like ±1 events over BLE
    struct k_work_delayable      accel_work;
    int64_t                      accel_start_ms;
    int16_t                      accel_target_x;
    int16_t                      accel_target_y;
    uint8_t                      accel_magnitude;
    bool                         accel_active;
};

// device config data structure
struct pixart_config {
	struct spi_dt_spec spi;
    struct gpio_dt_spec irq_gpio;
    uint16_t cpi;
    bool swap_xy;
    bool inv_x;
    bool inv_y;
    uint8_t evt_type;
    uint8_t x_input_code;
    uint8_t y_input_code;
    bool force_awake;
    bool force_awake_4ms_mode;
    int8_t layer_toggle;
    int32_t layer_toggle_timeout_ms;
};

#ifdef __cplusplus
}
#endif

/**
 * @}
 */
