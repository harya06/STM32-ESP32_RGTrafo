/**
 ******************************************************************************
 * @file    mlx90640_i2c.h
 * @brief   MLX90640 I2C Transport Abstraction Layer
 * @author  Refactored - NO GLOBAL I2C HANDLE
 * @date    2024
 ******************************************************************************
 * @attention
 *
 * This layer abstracts I2C communication with injected handle.
 * 
 * KEY CHANGES FROM ORIGINAL:
 * - NO global "extern I2C_HandleTypeDef hi2c1"
 * - I2C handle is passed as function parameter
 * - Multi-bus support (hi2c1, hi2c2, hi2c3, ...)
 * - Platform-agnostic design
 *
 ******************************************************************************
 */

#ifndef MLX90640_I2C_H
#define MLX90640_I2C_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "mlx90640_types.h"
#include "stm32f4xx_hal.h"

/* ============================================================================ */
/*                          I2C TRANSPORT FUNCTIONS                             */
/* ============================================================================ */

/**
 * @brief Read multiple 16-bit words from MLX90640 via I2C
 * 
 * @param hi2c Pointer to I2C handle (INJECTED, not global)
 * @param slave_addr I2C slave address (7-bit)
 * @param start_addr Register start address (16-bit)
 * @param word_count Number of 16-bit words to read
 * @param data Pointer to buffer (must be word_count * 2 bytes)
 * 
 * @return MLX90640_OK on success, error code otherwise
 * 
 * @note Handles byte swapping (MLX90640 is big-endian)
 */
MLX90640_Status_t MLX90640_I2C_Read(I2C_HandleTypeDef *hi2c,
                                    uint8_t slave_addr,
                                    uint16_t start_addr,
                                    uint16_t word_count,
                                    uint16_t *data);

/**
 * @brief Write single 16-bit word to MLX90640 via I2C
 * 
 * @param hi2c Pointer to I2C handle (INJECTED, not global)
 * @param slave_addr I2C slave address (7-bit)
 * @param reg_addr Register address (16-bit)
 * @param data Data to write (16-bit)
 * 
 * @return MLX90640_OK on success, error code otherwise
 * 
 * @note Includes read-back verification
 */
MLX90640_Status_t MLX90640_I2C_Write(I2C_HandleTypeDef *hi2c,
                                     uint8_t slave_addr,
                                     uint16_t reg_addr,
                                     uint16_t data);

/**
 * @brief I2C General Reset (broadcast)
 * 
 * @param hi2c Pointer to I2C handle
 * @return MLX90640_OK on success
 * 
 * @note This affects ALL devices on the I2C bus
 */
MLX90640_Status_t MLX90640_I2C_GeneralReset(I2C_HandleTypeDef *hi2c);

#ifdef __cplusplus
}
#endif

#endif /* MLX90640_I2C_H */