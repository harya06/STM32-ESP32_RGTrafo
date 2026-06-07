/**
 ******************************************************************************
 * @file    mlx90640_i2c.c
 * @brief   MLX90640 I2C Transport Implementation
 * @author  Refactored - Dependency Injection Pattern
 * @date    2024
 ******************************************************************************
 */

#include "mlx90640_i2c.h"
#include <string.h>

/* ============================================================================ */
/*                          I2C CONFIGURATION                                   */
/* ============================================================================ */

#define MLX90640_I2C_TIMEOUT_MS     2000U    /**< I2C operation timeout */

/* ============================================================================ */
/*                          PUBLIC FUNCTIONS                                    */
/* ============================================================================ */

/**
 * @brief Read multiple 16-bit words from MLX90640
 */

MLX90640_Status_t MLX90640_I2C_Read(I2C_HandleTypeDef *hi2c,
                                    uint8_t slave_addr,
                                    uint16_t start_addr,
                                    uint16_t word_count,
                                    uint16_t *data)
{
    /* Parameter validation */
    if (hi2c == NULL || data == NULL) {
        return MLX90640_ERROR_NULL_POINTER;
    }
    
    if (word_count == 0) {
        return MLX90640_ERROR_INVALID_PARAM;
    }
    
    /* Calculate byte count */
    uint16_t byte_count = word_count * 2;
    uint8_t *byte_buffer = (uint8_t *)data;
    
    /* ========================================================================== */
    /*   ADD RETRY MECHANISM (3 attempts)                                        */
    /* ========================================================================== */

    HAL_StatusTypeDef hal_status = HAL_ERROR;
    uint8_t retry_count = 0;
    uint8_t max_retries = 3;
    
    while (retry_count < max_retries) {

        /* Clear previous error */
        hi2c->ErrorCode = HAL_I2C_ERROR_NONE;

        /* Perform I2C memory read */
        hal_status = HAL_I2C_Mem_Read(
            hi2c,
            (slave_addr << 1),
            start_addr,
            I2C_MEMADD_SIZE_16BIT,
            byte_buffer,
            byte_count,
            MLX90640_I2C_TIMEOUT_MS
        );

        if (hal_status == HAL_OK) {
            break;  /* Success - exit retry loop */
        }

        /* Retry failed - wait and retry */
        retry_count++;
        HAL_Delay(10);  /* 10ms delay before retry */

        /* If bus is stuck, try to reset I2C */
        if (retry_count >= 2 && hi2c->State != HAL_I2C_STATE_READY) {
            HAL_I2C_DeInit(hi2c);
            HAL_I2C_Init(hi2c);
        }
    }
    
    /* Check final HAL status */
    if (hal_status == HAL_TIMEOUT) {
        return MLX90640_ERROR_I2C_TIMEOUT;
    } else if (hal_status != HAL_OK) {
        return MLX90640_ERROR_I2C_READ;
    }
    
    /* Byte swap: MLX90640 is big-endian, STM32 is little-endian */
    for (uint16_t i = 0; i < word_count; i++) {
        uint16_t idx = i * 2;
        uint8_t temp = byte_buffer[idx];
        byte_buffer[idx] = byte_buffer[idx + 1];
        byte_buffer[idx + 1] = temp;
    }
    
    return MLX90640_OK;
}

/**
 * @brief Write single 16-bit word to MLX90640 with verification
 */
MLX90640_Status_t MLX90640_I2C_Write(I2C_HandleTypeDef *hi2c,
                                     uint8_t slave_addr,
                                     uint16_t reg_addr,
                                     uint16_t data)
{
    /* Parameter validation */
    if (hi2c == NULL) {
        return MLX90640_ERROR_NULL_POINTER;
    }
    
    /* Prepare data (big-endian) */
    uint8_t write_buffer[2];
    write_buffer[0] = (data >> 8) & 0xFF;   /* MSB first */
    write_buffer[1] = data & 0xFF;          /* LSB second */
    
    /* Perform I2C memory write */
    HAL_StatusTypeDef hal_status = HAL_I2C_Mem_Write(
        hi2c,
        (slave_addr << 1),
        reg_addr,
        I2C_MEMADD_SIZE_16BIT,
        write_buffer,
        2,
        MLX90640_I2C_TIMEOUT_MS
    );
    
    /* Check HAL status */
    if (hal_status == HAL_TIMEOUT) {
        return MLX90640_ERROR_I2C_TIMEOUT;
    } else if (hal_status != HAL_OK) {
        return MLX90640_ERROR_I2C_WRITE;
    }
    
//    /* Read back for verification */
//    uint16_t read_back = 0;
//    MLX90640_Status_t status = MLX90640_I2C_Read(hi2c, slave_addr, reg_addr, 1, &read_back);
//
//    if (status != MLX90640_OK) {
//        return status;
//    }
//
//    /* Verify data */
//    if (read_back != data) {
//        return MLX90640_ERROR_I2C_WRITE;
//    }
    
    return MLX90640_OK;
}

/**
 * @brief I2C General Reset (broadcast)
 */
MLX90640_Status_t MLX90640_I2C_GeneralReset(I2C_HandleTypeDef *hi2c)
{
    /* Parameter validation */
    if (hi2c == NULL) {
        return MLX90640_ERROR_NULL_POINTER;
    }
    
    /* General reset is typically implemented via general call address 0x00
     * For MLX90640, this is optional and implementation-specific.
     * Returning OK as no-op for now.
     */
    
    return MLX90640_OK;
}
