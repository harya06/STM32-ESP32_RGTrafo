/**
 ******************************************************************************
 * @file    mlx90640_driver.c
 * @brief   MLX90640 High-Level Driver Implementation
 * @author  Industrial Multi-Instance Architecture
 * @date    2024
 ******************************************************************************
 */

#include "mlx90640_driver.h"
#include <string.h>
#include <math.h>

/* ============================================================================ */
/*                          PRIVATE DEFINITIONS                                 */
/* ============================================================================ */

#define MLX90640_STATUS_DATA_READY      (1U << 3)
#define MLX90640_STATUS_INIT_VALUE      0x0030

/* Timeout for waiting data ready (in loop iterations) */
#define MLX90640_DATA_READY_TIMEOUT     10000U

/* ============================================================================ */
/*                          LIFECYCLE FUNCTIONS                                 */
/* ============================================================================ */

/**
 * @brief Initialize MLX90640 device
 */
MLX90640_Status_t MLX90640_Init(MLX90640_Device_t *dev,
                                I2C_HandleTypeDef *hi2c,
                                uint8_t address)
{
    /* Parameter validation */
    if (dev == NULL || hi2c == NULL) {
        return MLX90640_ERROR_NULL_POINTER;
    }
    
    /* Clear device structure */
    memset(dev, 0, sizeof(MLX90640_Device_t));
    
    /* Assign hardware */
    dev->hi2c = hi2c;
    dev->i2c_address = address;
    dev->init_timestamp = HAL_GetTick();
    
    /* Read EEPROM calibration data */
    MLX90640_Status_t status = MLX90640_I2C_Read(
        dev->hi2c,
        dev->i2c_address,
        MLX90640_EEPROM_START_ADDRESS,
        MLX90640_EEPROM_SIZE,
        dev->eeprom_data
    );
    
    if (status != MLX90640_OK) {
        dev->last_error = status;
        dev->error_count++;
        return MLX90640_ERROR_EEPROM;
    }
    
    /* Extract calibration parameters */
    status = MLX90640_ExtractParameters(dev->eeprom_data, &dev->calib_params);
    
    if (status != MLX90640_OK) {
        dev->last_error = status;
        dev->error_count++;
        return MLX90640_ERROR_PARAM_EXTRACT;
    }
    
    /* Set default configuration */
    dev->current_mode = MLX90640_MODE_CHESS;
    dev->current_refresh_rate = MLX90640_REFRESH_4_HZ;
    dev->current_resolution = MLX90640_RESOLUTION_18_BIT;
    
    /* Mark as initialized */
    dev->is_initialized = 1;
    
    return MLX90640_OK;
}

/**
 * @brief De-initialize device
 */
MLX90640_Status_t MLX90640_DeInit(MLX90640_Device_t *dev)
{
    if (dev == NULL) {
        return MLX90640_ERROR_NULL_POINTER;
    }
    
    /* Clear all data */
    memset(dev, 0, sizeof(MLX90640_Device_t));
    
    return MLX90640_OK;
}

/* ============================================================================ */
/*                          CONFIGURATION FUNCTIONS                             */
/* ============================================================================ */

/**
 * @brief Set refresh rate
 */
MLX90640_Status_t MLX90640_SetRefreshRate(MLX90640_Device_t *dev,
                                          MLX90640_RefreshRate_t rate)
{
    if (dev == NULL) {
        return MLX90640_ERROR_NULL_POINTER;
    }
    
    if (!dev->is_initialized) {
        return MLX90640_ERROR_NOT_INITIALIZED;
    }
    
    if (rate > MLX90640_REFRESH_64_HZ) {
        return MLX90640_ERROR_INVALID_PARAM;
    }
    
    /* Read current control register */
    uint16_t ctrl_reg;
    MLX90640_Status_t status = MLX90640_I2C_Read(
        dev->hi2c,
        dev->i2c_address,
        MLX90640_CONTROL_REGISTER,
        1,
        &ctrl_reg
    );
    
    if (status != MLX90640_OK) {
        dev->last_error = status;
        dev->error_count++;
        return status;
    }
    
    /* Modify refresh rate bits */
    ctrl_reg &= ~MLX90640_CTRL_REFRESH_MASK;
    ctrl_reg |= ((uint16_t)rate << MLX90640_CTRL_REFRESH_SHIFT);
    
    /* Write back */
    status = MLX90640_I2C_Write(
        dev->hi2c,
        dev->i2c_address,
        MLX90640_CONTROL_REGISTER,
        ctrl_reg
    );
    
    if (status != MLX90640_OK) {
        dev->last_error = status;
        dev->error_count++;
        return status;
    }
    
    dev->current_refresh_rate = rate;
    
    return MLX90640_OK;
}

/**
 * @brief Get refresh rate
 */
int MLX90640_GetRefreshRate(MLX90640_Device_t *dev)
{
    if (dev == NULL) {
        return (int)MLX90640_ERROR_NULL_POINTER;
    }
    
    if (!dev->is_initialized) {
        return (int)MLX90640_ERROR_NOT_INITIALIZED;
    }
    
    uint16_t ctrl_reg;
    MLX90640_Status_t status = MLX90640_I2C_Read(
        dev->hi2c,
        dev->i2c_address,
        MLX90640_CONTROL_REGISTER,
        1,
        &ctrl_reg
    );
    
    if (status != MLX90640_OK) {
        dev->last_error = status;
        dev->error_count++;
        return (int)status;
    }
    
    uint8_t rate = (uint8_t)((ctrl_reg & MLX90640_CTRL_REFRESH_MASK) >> MLX90640_CTRL_REFRESH_SHIFT);
    
    return (int)rate;
}

/**
 * @brief Set ADC resolution
 */
MLX90640_Status_t MLX90640_SetResolution(MLX90640_Device_t *dev,
                                         MLX90640_Resolution_t resolution)
{
    if (dev == NULL) {
        return MLX90640_ERROR_NULL_POINTER;
    }
    
    if (!dev->is_initialized) {
        return MLX90640_ERROR_NOT_INITIALIZED;
    }
    
    if (resolution > MLX90640_RESOLUTION_19_BIT) {
        return MLX90640_ERROR_INVALID_PARAM;
    }
    
    /* Read current control register */
    uint16_t ctrl_reg;
    MLX90640_Status_t status = MLX90640_I2C_Read(
        dev->hi2c,
        dev->i2c_address,
        MLX90640_CONTROL_REGISTER,
        1,
        &ctrl_reg
    );
    
    if (status != MLX90640_OK) {
        dev->last_error = status;
        dev->error_count++;
        return status;
    }
    
    /* Modify resolution bits */
    ctrl_reg &= ~MLX90640_CTRL_RESOLUTION_MASK;
    ctrl_reg |= ((uint16_t)resolution << MLX90640_CTRL_RESOLUTION_SHIFT);
    
    /* Write back */
    status = MLX90640_I2C_Write(
        dev->hi2c,
        dev->i2c_address,
        MLX90640_CONTROL_REGISTER,
        ctrl_reg
    );
    
    if (status != MLX90640_OK) {
        dev->last_error = status;
        dev->error_count++;
        return status;
    }
    
    dev->current_resolution = resolution;
    
    return MLX90640_OK;
}

/**
 * @brief Get ADC resolution
 */
int MLX90640_GetResolution(MLX90640_Device_t *dev)
{
    if (dev == NULL) {
        return (int)MLX90640_ERROR_NULL_POINTER;
    }
    
    if (!dev->is_initialized) {
        return (int)MLX90640_ERROR_NOT_INITIALIZED;
    }
    
    uint16_t ctrl_reg;
    MLX90640_Status_t status = MLX90640_I2C_Read(
        dev->hi2c,
        dev->i2c_address,
        MLX90640_CONTROL_REGISTER,
        1,
        &ctrl_reg
    );
    
    if (status != MLX90640_OK) {
        dev->last_error = status;
        dev->error_count++;
        return (int)status;
    }
    
    uint8_t resolution = (uint8_t)((ctrl_reg & MLX90640_CTRL_RESOLUTION_MASK) >> 
                                   MLX90640_CTRL_RESOLUTION_SHIFT);
    
    return (int)resolution;
}

/**
 * @brief Set measurement mode
 */
MLX90640_Status_t MLX90640_SetMode(MLX90640_Device_t *dev,
                                   MLX90640_Mode_t mode)
{
    if (dev == NULL) {
        return MLX90640_ERROR_NULL_POINTER;
    }
    
    if (!dev->is_initialized) {
        return MLX90640_ERROR_NOT_INITIALIZED;
    }
    
    /* Read current control register */
    uint16_t ctrl_reg;
    MLX90640_Status_t status = MLX90640_I2C_Read(
        dev->hi2c,
        dev->i2c_address,
        MLX90640_CONTROL_REGISTER,
        1,
        &ctrl_reg
    );
    
    if (status != MLX90640_OK) {
        dev->last_error = status;
        dev->error_count++;
        return status;
    }
    
    /* Modify mode bit */
    if (mode == MLX90640_MODE_CHESS) {
        ctrl_reg |= MLX90640_CTRL_MODE_MASK;
    } else {
        ctrl_reg &= ~MLX90640_CTRL_MODE_MASK;
    }
    
    /* Write back */
    status = MLX90640_I2C_Write(
        dev->hi2c,
        dev->i2c_address,
        MLX90640_CONTROL_REGISTER,
        ctrl_reg
    );
    
    if (status != MLX90640_OK) {
        dev->last_error = status;
        dev->error_count++;
        return status;
    }
    
    dev->current_mode = mode;
    
    return MLX90640_OK;
}

/**
 * @brief Get measurement mode
 */
int MLX90640_GetMode(MLX90640_Device_t *dev)
{
    if (dev == NULL) {
        return (int)MLX90640_ERROR_NULL_POINTER;
    }
    
    if (!dev->is_initialized) {
        return (int)MLX90640_ERROR_NOT_INITIALIZED;
    }
    
    uint16_t ctrl_reg;
    MLX90640_Status_t status = MLX90640_I2C_Read(
        dev->hi2c,
        dev->i2c_address,
        MLX90640_CONTROL_REGISTER,
        1,
        &ctrl_reg
    );
    
    if (status != MLX90640_OK) {
        dev->last_error = status;
        dev->error_count++;
        return (int)status;
    }
    
    uint8_t mode = (uint8_t)((ctrl_reg & MLX90640_CTRL_MODE_MASK) >> MLX90640_CTRL_MODE_SHIFT);
    
    return (int)mode;
}

/* ============================================================================ */
/*                          DATA ACQUISITION FUNCTIONS                          */
/* ============================================================================ */

/**
 * @brief Trigger single measurement
 */
MLX90640_Status_t MLX90640_TriggerMeasurement(MLX90640_Device_t *dev)
{
    if (dev == NULL) {
        return MLX90640_ERROR_NULL_POINTER;
    }
    
    if (!dev->is_initialized) {
        return MLX90640_ERROR_NOT_INITIALIZED;
    }
    
    /* Read control register */
    uint16_t ctrl_reg;
    MLX90640_Status_t status = MLX90640_I2C_Read(
        dev->hi2c,
        dev->i2c_address,
        MLX90640_CONTROL_REGISTER,
        1,
        &ctrl_reg
    );
    
    if (status != MLX90640_OK) {
        dev->last_error = status;
        dev->error_count++;
        return status;
    }
    
    /* Set trigger bit */
    ctrl_reg |= MLX90640_CTRL_TRIGGER_BIT;
    
    /* Write trigger */
    status = MLX90640_I2C_Write(
        dev->hi2c,
        dev->i2c_address,
        MLX90640_CONTROL_REGISTER,
        ctrl_reg
    );
    
    if (status != MLX90640_OK) {
        dev->last_error = status;
        dev->error_count++;
        return status;
    }
    
    /* General reset (optional) */
    MLX90640_I2C_GeneralReset(dev->hi2c);
    
    /* Verify trigger bit cleared */
    status = MLX90640_I2C_Read(
        dev->hi2c,
        dev->i2c_address,
        MLX90640_CONTROL_REGISTER,
        1,
        &ctrl_reg
    );
    
    if (status != MLX90640_OK) {
        dev->last_error = status;
        dev->error_count++;
        return status;
    }
    
    if ((ctrl_reg & MLX90640_CTRL_TRIGGER_BIT) != 0) {
        dev->last_error = MLX90640_ERROR;
        dev->error_count++;
        return MLX90640_ERROR;
    }
    
    return MLX90640_OK;
}

/**
 * @brief Synchronize to next frame (blocking)
 */
MLX90640_Status_t MLX90640_SynchFrame(MLX90640_Device_t *dev)
{
    if (dev == NULL) {
        return MLX90640_ERROR_NULL_POINTER;
    }
    
    if (!dev->is_initialized) {
        return MLX90640_ERROR_NOT_INITIALIZED;
    }
    
    /* Clear status register */
    MLX90640_Status_t status = MLX90640_I2C_Write(
        dev->hi2c,
        dev->i2c_address,
        MLX90640_STATUS_REGISTER,
        MLX90640_STATUS_INIT_VALUE
    );
    
    if (status != MLX90640_OK) {
        dev->last_error = status;
        dev->error_count++;
        return status;
    }
    
    /* Wait for data ready */
    uint16_t status_reg = 0;
    uint32_t timeout = MLX90640_DATA_READY_TIMEOUT;
    
    while ((status_reg & MLX90640_STATUS_DATA_READY) == 0) {
        
        status = MLX90640_I2C_Read(
            dev->hi2c,
            dev->i2c_address,
            MLX90640_STATUS_REGISTER,
            1,
            &status_reg
        );
        
        if (status != MLX90640_OK) {
            dev->last_error = status;
            dev->error_count++;
            return status;
        }
        
        if (--timeout == 0) {
            dev->last_error = MLX90640_ERROR_TIMEOUT;
            dev->error_count++;
            return MLX90640_ERROR_TIMEOUT;
        }
    }
    
    return MLX90640_OK;
}

/**
 * @brief Get frame data from sensor
 */
int MLX90640_GetFrameData(MLX90640_Device_t *dev)
{
    if (dev == NULL) {
        return (int)MLX90640_ERROR_NULL_POINTER;
    }
    
    if (!dev->is_initialized) {
        return (int)MLX90640_ERROR_NOT_INITIALIZED;
    }
    
    /* ========================================================================== */
    /*   FIX: CLEAR I2C ERROR STATE BEFORE OPERATION                             */
    /* ========================================================================== */

    if (dev->hi2c->ErrorCode != HAL_I2C_ERROR_NONE) {
        /* Clear previous error */
        dev->hi2c->ErrorCode = HAL_I2C_ERROR_NONE;

        /* Reset I2C if needed */
        if (dev->hi2c->State != HAL_I2C_STATE_READY) {
            HAL_I2C_DeInit(dev->hi2c);
            HAL_I2C_Init(dev->hi2c);
        }
    }

    /* ========================================================================== */
    /*   NOW WAIT FOR DATA READY                                                 */
    /* ========================================================================== */

    uint16_t status_reg = 0;
    uint32_t timeout_count = 0;
    uint32_t max_timeout = 1000;
    /* Wait for data ready with delay */
    while ((status_reg & MLX90640_STATUS_DATA_READY) == 0) {
        
        MLX90640_Status_t status = MLX90640_I2C_Read(
            dev->hi2c,
            dev->i2c_address,
            MLX90640_STATUS_REGISTER,
            1,
            &status_reg
        );
        
        if (status != MLX90640_OK) {
            dev->last_error = status;
            dev->error_count++;
            return (int)status;
        }
        
        HAL_Delay(10);

        if (++timeout_count > max_timeout) {
            dev->last_error = MLX90640_ERROR_TIMEOUT;
            dev->error_count++;
            return (int)MLX90640_ERROR_TIMEOUT;
        }
    }
    
    /* ========================================================================== */
    /*   DATA READY - CLEAR STATUS AGAIN                                         */
    /* ========================================================================== */

    MLX90640_Status_t status = MLX90640_I2C_Write(
        dev->hi2c,
        dev->i2c_address,
        MLX90640_STATUS_REGISTER,
        MLX90640_STATUS_INIT_VALUE
    );
    
    if (status != MLX90640_OK) {
        dev->last_error = status;
        dev->error_count++;
        return (int)status;
    }
    
    /* ========================================================================== */
    /*   READ PIXEL DATA                                                         */
    /* ========================================================================== */

    status = MLX90640_I2C_Read(
        dev->hi2c,
        dev->i2c_address,
        MLX90640_PIXEL_DATA_ADDRESS,
        MLX90640_PIXEL_COUNT,
        dev->frame_data
    );
    
    if (status != MLX90640_OK) {
        dev->last_error = status;
        dev->error_count++;
        return (int)status;
    }
    
    /* Read auxiliary data (64 words) */
    uint16_t aux_data[MLX90640_AUX_COUNT];
    status = MLX90640_I2C_Read(
        dev->hi2c,
        dev->i2c_address,
        MLX90640_AUX_DATA_ADDRESS,
        MLX90640_AUX_COUNT,
        aux_data
    );
    
    if (status != MLX90640_OK) {
        dev->last_error = status;
        dev->error_count++;
        return (int)status;
    }
    
    /* Read control register */
    uint16_t ctrl_reg;
    status = MLX90640_I2C_Read(
        dev->hi2c,
        dev->i2c_address,
        MLX90640_CONTROL_REGISTER,
        1,
        &ctrl_reg
    );
    
    if (status != MLX90640_OK) {
        dev->last_error = status;
        dev->error_count++;
        return (int)status;
    }
    
    /* Store control register and subpage */
    dev->frame_data[832] = ctrl_reg;
    dev->frame_data[833] = status_reg & 0x0001;
    dev->last_subpage = dev->frame_data[833];
    
    /* Validate auxiliary data */
    status = MLX90640_ValidateAuxData(aux_data);
    
    if (status == MLX90640_OK) {
        /* Copy aux data to frame buffer */
        memcpy(&dev->frame_data[MLX90640_PIXEL_COUNT], aux_data, 
               MLX90640_AUX_COUNT * sizeof(uint16_t));
    } else {
        dev->last_error = status;
        dev->error_count++;
        return (int)status;
    }
    
    /* Validate frame data */
    status = MLX90640_ValidateFrameData(dev->frame_data);
    
    if (status != MLX90640_OK) {
        dev->last_error = status;
        dev->error_count++;
        return (int)status;
    }
    
    /* Update ambient temp and Vdd */
    dev->ambient_temp = MLX90640_GetTa(dev->frame_data, &dev->calib_params);
    dev->vdd = MLX90640_GetVdd(dev->frame_data, &dev->calib_params);
    
    /* Increment frame counter */
    dev->frame_count++;
    
    /* Return subpage number */
    return (int)dev->last_subpage;
}

/**
 * @brief Calculate temperatures from frame data
 */
MLX90640_Status_t MLX90640_CalculateTemperatures(MLX90640_Device_t *dev,
                                                 float emissivity,
                                                 float tr)
{
    if (dev == NULL) {
        return MLX90640_ERROR_NULL_POINTER;
    }
    
    if (!dev->is_initialized) {
        return MLX90640_ERROR_NOT_INITIALIZED;
    }
    
    /* Validate emissivity */
    if (emissivity < 0.1f || emissivity > 1.0f) {
        return MLX90640_ERROR_INVALID_PARAM;
    }
    
    /* Calculate temperatures using core engine */
    MLX90640_CalculateTo(
        dev->frame_data,
        &dev->calib_params,
        emissivity,
        tr,
        dev->temperature_image
    );
    
    /* Apply bad pixel correction */
    MLX90640_BadPixelsCorrection(
        dev->calib_params.brokenPixels,
        dev->temperature_image,
        dev->current_mode,
        &dev->calib_params
    );
    
    MLX90640_BadPixelsCorrection(
        dev->calib_params.outlierPixels,
        dev->temperature_image,
        dev->current_mode,
        &dev->calib_params
    );
    
    /* Compute statistics (min, max, avg, center) */
    float min_temp = 1000.0f;
    float max_temp = -1000.0f;
    float sum_temp = 0.0f;
    uint16_t valid_pixel_count = 0;
    
    for (int i = 0; i < MLX90640_PIXEL_COUNT; i++) {
        float temp = dev->temperature_image[i];
        
        /* Skip invalid temperatures (could be from non-processed subpage) */
        if (temp > -273.0f && temp < 500.0f) {
            sum_temp += temp;
            valid_pixel_count++;
            
            if (temp < min_temp) min_temp = temp;
            if (temp > max_temp) max_temp = temp;
        }
    }
    
    dev->min_temp = min_temp;
    dev->max_temp = max_temp;
    dev->avg_temp = (valid_pixel_count > 0) ? (sum_temp / valid_pixel_count) : 0.0f;
    
    /* Center pixel (index 384 = row 12, col 16) */
    dev->center_temp = dev->temperature_image[384];
    
    return MLX90640_OK;
}

/* ============================================================================ */
/*                          UTILITY FUNCTIONS                                   */
/* ============================================================================ */

/**
 * @brief Get ambient temperature
 */
float MLX90640_GetAmbientTemperature(const MLX90640_Device_t *dev)
{
    if (dev == NULL || !dev->is_initialized) {
        return 0.0f;
    }
    return dev->ambient_temp;
}

/**
 * @brief Get VDD voltage
 */
float MLX90640_GetVddVoltage(const MLX90640_Device_t *dev)
{
    if (dev == NULL || !dev->is_initialized) {
        return 0.0f;
    }
    return dev->vdd;
}

/**
 * @brief Get center temperature
 */
float MLX90640_GetCenterTemperature(const MLX90640_Device_t *dev)
{
    if (dev == NULL || !dev->is_initialized) {
        return 0.0f;
    }
    return dev->center_temp;
}

/**
 * @brief Get minimum temperature
 */
float MLX90640_GetMinTemperature(const MLX90640_Device_t *dev)
{
    if (dev == NULL || !dev->is_initialized) {
        return 0.0f;
    }
    return dev->min_temp;
}

/**
 * @brief Get maximum temperature
 */
float MLX90640_GetMaxTemperature(const MLX90640_Device_t *dev)
{
    if (dev == NULL || !dev->is_initialized) {
        return 0.0f;
    }
    return dev->max_temp;
}

/**
 * @brief Get average temperature
 */
float MLX90640_GetAverageTemperature(const MLX90640_Device_t *dev)
{
    if (dev == NULL || !dev->is_initialized) {
        return 0.0f;
    }
    return dev->avg_temp;
}

/**
 * @brief Get temperature image array
 */
const float* MLX90640_GetTemperatureImage(const MLX90640_Device_t *dev)
{
    if (dev == NULL || !dev->is_initialized) {
        return NULL;
    }
    return dev->temperature_image;
}

/**
 * @brief Get error string
 */
const char* MLX90640_GetErrorString(MLX90640_Status_t status)
{
    switch (status) {
        case MLX90640_OK:
            return "OK";
        case MLX90640_ERROR:
            return "Generic error";
        case MLX90640_ERROR_I2C_NACK:
            return "I2C NACK error";
        case MLX90640_ERROR_I2C_WRITE:
            return "I2C write error";
        case MLX90640_ERROR_I2C_READ:
            return "I2C read error";
        case MLX90640_ERROR_I2C_TIMEOUT:
            return "I2C timeout";
        case MLX90640_ERROR_EEPROM:
            return "EEPROM read error";
        case MLX90640_ERROR_PARAM_EXTRACT:
            return "Parameter extraction failed";
        case MLX90640_ERROR_FRAME_DATA:
            return "Invalid frame data";
        case MLX90640_ERROR_BROKEN_PIXELS:
            return "Too many broken pixels";
        case MLX90640_ERROR_OUTLIER_PIXELS:
            return "Too many outlier pixels";
        case MLX90640_ERROR_BAD_PIXELS:
            return "Too many bad pixels";
        case MLX90640_ERROR_ADJACENT_PIXELS:
            return "Adjacent bad pixels detected";
        case MLX90640_ERROR_TIMEOUT:
            return "Timeout waiting for data";
        case MLX90640_ERROR_NOT_INITIALIZED:
            return "Device not initialized";
        case MLX90640_ERROR_NULL_POINTER:
            return "NULL pointer passed";
        case MLX90640_ERROR_INVALID_PARAM:
            return "Invalid parameter";
        default:
            return "Unknown error";
    }
}
