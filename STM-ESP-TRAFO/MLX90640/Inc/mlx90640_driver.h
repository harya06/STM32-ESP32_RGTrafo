/**
 ******************************************************************************
 * @file    mlx90640_driver.h
 * @brief   MLX90640 High-Level Instance-Based Driver
 * @author  Industrial Multi-Instance Architecture
 * @date    2024
 ******************************************************************************
 * @attention
 *
 * This is the main API for application code.
 * All functions operate on device instances (no globals).
 *
 * USAGE EXAMPLE:
 * @code
 * MLX90640_Device_t camera1, camera2;
 * 
 * MLX90640_Init(&camera1, &hi2c1, 0x33);
 * MLX90640_Init(&camera2, &hi2c2, 0x33);
 * 
 * MLX90640_SetRefreshRate(&camera1, MLX90640_REFRESH_4_HZ);
 * MLX90640_SetMode(&camera1, MLX90640_MODE_CHESS);
 * 
 * while(1) {
 *     MLX90640_GetFrameData(&camera1);
 *     MLX90640_CalculateTemperatures(&camera1, 0.95f, camera1.ambient_temp);
 *     printf("Cam1 Max: %.2f°C\n", camera1.max_temp);
 * }
 * @endcode
 *
 ******************************************************************************
 */

#ifndef MLX90640_DRIVER_H
#define MLX90640_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "mlx90640_device.h"
#include "mlx90640_types.h"
#include "mlx90640_core.h"
#include "mlx90640_i2c.h"

/* ============================================================================ */
/*                          LIFECYCLE MANAGEMENT                                */
/* ============================================================================ */

/**
 * @brief Initialize MLX90640 device instance
 * 
 * This function performs full initialization:
 * - Assigns I2C handle and address
 * - Reads EEPROM calibration data
 * - Extracts calibration parameters
 * - Validates device functionality
 * - Marks device as initialized
 * 
 * @param dev Pointer to device instance
 * @param hi2c Pointer to I2C handle (injected, NOT global)
 * @param address I2C slave address (typically 0x33)
 * 
 * @return MLX90640_OK on success, error code otherwise
 * 
 * @note Must be called before any other operations on this device
 * 
 * Example:
 * @code
 * MLX90640_Device_t camera1;
 * if (MLX90640_Init(&camera1, &hi2c1, 0x33) != MLX90640_OK) {
 *     // Handle error
 * }
 * @endcode
 */
MLX90640_Status_t MLX90640_Init(MLX90640_Device_t *dev,
                                I2C_HandleTypeDef *hi2c,
                                uint8_t address);

/**
 * @brief De-initialize device (cleanup)
 * 
 * @param dev Pointer to device instance
 * @return MLX90640_OK on success
 */
MLX90640_Status_t MLX90640_DeInit(MLX90640_Device_t *dev);

/* ============================================================================ */
/*                          CONFIGURATION                                       */
/* ============================================================================ */

/**
 * @brief Set refresh rate
 * 
 * @param dev Pointer to device instance
 * @param rate Refresh rate (use MLX90640_RefreshRate_t enum)
 * @return MLX90640_OK on success
 * 
 * Available rates: 0.5Hz, 1Hz, 2Hz, 4Hz, 8Hz, 16Hz, 32Hz, 64Hz
 */
MLX90640_Status_t MLX90640_SetRefreshRate(MLX90640_Device_t *dev,
                                          MLX90640_RefreshRate_t rate);

/**
 * @brief Get current refresh rate
 * 
 * @param dev Pointer to device instance
 * @return Refresh rate code (0-7), or negative on error
 */
int MLX90640_GetRefreshRate(MLX90640_Device_t *dev);

/**
 * @brief Set ADC resolution
 * 
 * @param dev Pointer to device instance
 * @param resolution Resolution (use MLX90640_Resolution_t enum)
 * @return MLX90640_OK on success
 * 
 * Available: 16-bit, 17-bit, 18-bit, 19-bit
 */
MLX90640_Status_t MLX90640_SetResolution(MLX90640_Device_t *dev,
                                         MLX90640_Resolution_t resolution);

/**
 * @brief Get current ADC resolution
 * 
 * @param dev Pointer to device instance
 * @return Resolution code (0-3), or negative on error
 */
int MLX90640_GetResolution(MLX90640_Device_t *dev);

/**
 * @brief Set measurement mode
 * 
 * @param dev Pointer to device instance
 * @param mode Mode (MLX90640_MODE_INTERLEAVED or MLX90640_MODE_CHESS)
 * @return MLX90640_OK on success
 */
MLX90640_Status_t MLX90640_SetMode(MLX90640_Device_t *dev,
                                   MLX90640_Mode_t mode);

/**
 * @brief Get current measurement mode
 * 
 * @param dev Pointer to device instance
 * @return Mode (0=interleaved, 1=chess), or negative on error
 */
int MLX90640_GetMode(MLX90640_Device_t *dev);

/* ============================================================================ */
/*                          DATA ACQUISITION                                    */
/* ============================================================================ */

/**
 * @brief Trigger single measurement (for triggered mode)
 * 
 * @param dev Pointer to device instance
 * @return MLX90640_OK on success
 */
MLX90640_Status_t MLX90640_TriggerMeasurement(MLX90640_Device_t *dev);

/**
 * @brief Synchronize to next frame (blocking)
 * 
 * Waits until data ready flag is set.
 * 
 * @param dev Pointer to device instance
 * @return MLX90640_OK on success
 */
MLX90640_Status_t MLX90640_SynchFrame(MLX90640_Device_t *dev);

/**
 * @brief Get raw frame data from sensor
 * 
 * This function:
 * - Waits for data ready flag
 * - Reads 768 pixel values
 * - Reads 64 auxiliary values
 * - Reads control/status registers
 * - Validates frame integrity
 * - Stores in device->frame_data
 * 
 * @param dev Pointer to device instance
 * @return Subpage number (0 or 1) on success, negative on error
 * 
 * @note Does NOT calculate temperatures - call MLX90640_CalculateTemperatures()
 */
int MLX90640_GetFrameData(MLX90640_Device_t *dev);

/**
 * @brief Calculate temperatures from current frame data
 * 
 * This function:
 * - Computes pixel temperatures using calibration
 * - Applies emissivity correction
 * - Calculates min/max/avg/center temperatures
 * - Stores results in device structure
 * 
 * @param dev Pointer to device instance
 * @param emissivity Object emissivity (0.1 - 1.0, typically 0.95)
 * @param tr Reflected temperature in °C (typically use device->ambient_temp)
 * @return MLX90640_OK on success
 * 
 * @note Must call MLX90640_GetFrameData() first
 * 
 * Example:
 * @code
 * MLX90640_GetFrameData(&camera1);
 * MLX90640_CalculateTemperatures(&camera1, 0.95f, camera1.ambient_temp);
 * printf("Max: %.2f°C, Avg: %.2f°C\n", camera1.max_temp, camera1.avg_temp);
 * @endcode
 */
MLX90640_Status_t MLX90640_CalculateTemperatures(MLX90640_Device_t *dev,
                                                 float emissivity,
                                                 float tr);

/* ============================================================================ */
/*                          UTILITY FUNCTIONS                                   */
/* ============================================================================ */

/**
 * @brief Get ambient temperature (Ta) from current frame
 * 
 * @param dev Pointer to device instance
 * @return Ambient temperature in °C
 */
float MLX90640_GetAmbientTemperature(const MLX90640_Device_t *dev);

/**
 * @brief Get supply voltage (Vdd) from current frame
 * 
 * @param dev Pointer to device instance
 * @return Supply voltage in V
 */
float MLX90640_GetVddVoltage(const MLX90640_Device_t *dev);

/**
 * @brief Get center pixel temperature
 * 
 * @param dev Pointer to device instance
 * @return Center temperature in °C
 */
float MLX90640_GetCenterTemperature(const MLX90640_Device_t *dev);

/**
 * @brief Get minimum temperature in frame
 * 
 * @param dev Pointer to device instance
 * @return Minimum temperature in °C
 */
float MLX90640_GetMinTemperature(const MLX90640_Device_t *dev);

/**
 * @brief Get maximum temperature in frame
 * 
 * @param dev Pointer to device instance
 * @return Maximum temperature in °C
 */
float MLX90640_GetMaxTemperature(const MLX90640_Device_t *dev);

/**
 * @brief Get average temperature in frame
 * 
 * @param dev Pointer to device instance
 * @return Average temperature in °C
 */
float MLX90640_GetAverageTemperature(const MLX90640_Device_t *dev);

/**
 * @brief Get pointer to full temperature image array (read-only)
 * 
 * @param dev Pointer to device instance
 * @return Pointer to 768-element float array (24 rows × 32 columns)
 * 
 * @note Array is organized row-major: index = row * 32 + column
 */
const float* MLX90640_GetTemperatureImage(const MLX90640_Device_t *dev);

/**
 * @brief Get human-readable error string
 * 
 * @param status Error code
 * @return Constant string describing error
 */
const char* MLX90640_GetErrorString(MLX90640_Status_t status);

#ifdef __cplusplus
}
#endif

#endif /* MLX90640_DRIVER_H */