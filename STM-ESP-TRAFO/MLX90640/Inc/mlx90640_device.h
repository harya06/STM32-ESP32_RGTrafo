/**
 ******************************************************************************
 * @file    mlx90640_device.h
 * @brief   MLX90640 Device Instance Structure
 * @author  Refactored for Industrial Multi-Instance Architecture
 * @date    2024
 ******************************************************************************
 * @attention
 *
 * This structure encapsulates ALL state for a single MLX90640 sensor.
 * Multiple instances can coexist independently on different I2C buses.
 *
 * DESIGN PRINCIPLES:
 * - No global state
 * - Explicit ownership
 * - Thread-safe oriented (stateless functions)
 * - Scalable architecture
 *
 ******************************************************************************
 */

#ifndef MLX90640_DEVICE_H
#define MLX90640_DEVICE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "mlx90640_types.h"
#include "stm32f4xx_hal.h"

/* ============================================================================ */
/*                          DEVICE INSTANCE STRUCTURE                           */
/* ============================================================================ */

/**
 * @brief MLX90640 Device Instance
 * 
 * This structure contains ALL state and data for one MLX90640 thermal camera.
 * 
 * Memory Layout:
 * - Hardware abstraction: I2C handle (injected, NOT owned)
 * - Device state: Initialization flags, configuration
 * - Calibration data: EEPROM + extracted parameters
 * - Frame buffers: Raw data + computed temperatures
 * - Statistics: Min/max/avg computed automatically
 * - Error tracking: Runtime diagnostics
 * 
 * Usage:
 * @code
 * MLX90640_Device_t camera1;  // Stack allocation
 * MLX90640_Init(&camera1, &hi2c1, 0x33);
 * @endcode
 */
typedef struct {
    
    /* ===== HARDWARE LAYER ===== */
    I2C_HandleTypeDef *hi2c;        /**< Injected I2C handle (NOT global) */
    uint8_t i2c_address;             /**< Device I2C address (typically 0x33) */
    
    /* ===== STATE FLAGS ===== */
    uint8_t is_initialized;          /**< 1 if Init() called successfully */
    uint8_t last_subpage;            /**< Last read subpage (0 or 1) */
    uint8_t current_mode;            /**< Current measurement mode */
    uint8_t current_refresh_rate;    /**< Current refresh rate setting */
    uint8_t current_resolution;      /**< Current ADC resolution */
    
    /* ===== CALIBRATION DATA ===== */
    uint16_t eeprom_data[MLX90640_EEPROM_SIZE];  /**< Raw EEPROM dump */
    MLX90640_Params_t calib_params;              /**< Extracted parameters */
    
    /* ===== FRAME BUFFERS ===== */
    uint16_t frame_data[MLX90640_FRAME_SIZE];         /**< Raw frame (768+64+2) */
    float temperature_image[MLX90640_PIXEL_COUNT];    /**< Temperatures in °C */
    
    /* ===== COMPUTED VALUES ===== */
    float ambient_temp;              /**< Ambient temperature (Ta) in °C */
    float vdd;                       /**< Supply voltage in V */
    float min_temp;                  /**< Minimum temperature in current frame */
    float max_temp;                  /**< Maximum temperature in current frame */
    float avg_temp;                  /**< Average temperature in current frame */
    float center_temp;               /**< Center pixel temperature (ROI) */
    
    /* ===== ERROR TRACKING ===== */
    uint32_t error_count;            /**< Total error counter */
    int32_t last_error;              /**< Last error code */
    uint32_t frame_count;            /**< Successfully acquired frames */
    uint32_t init_timestamp;         /**< Timestamp of last Init() call */
    
} MLX90640_Device_t;

#ifdef __cplusplus
}
#endif

#endif /* MLX90640_DEVICE_H */