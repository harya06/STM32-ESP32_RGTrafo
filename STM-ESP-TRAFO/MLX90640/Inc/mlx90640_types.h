/**
 ******************************************************************************
 * @file    mlx90640_types.h
 * @brief   MLX90640 Common Types and Constants
 * @author  Refactored for Industrial Multi-Instance Architecture
 * @date    2024
 ******************************************************************************
 * @attention
 *
 * This file contains all type definitions and constants used across the
 * MLX90640 driver stack. NO global state is defined here.
 *
 ******************************************************************************
 */

#ifndef MLX90640_TYPES_H
#define MLX90640_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stddef.h>

/* ============================================================================ */
/*                          HARDWARE CONSTANTS                                  */
/* ============================================================================ */

#define MLX90640_I2C_DEFAULT_ADDRESS    0x33    /**< Default 7-bit I2C address */

#define MLX90640_EEPROM_START_ADDRESS   0x2400  /**< EEPROM start register */
#define MLX90640_EEPROM_SIZE            832     /**< EEPROM size in words */

#define MLX90640_PIXEL_DATA_ADDRESS     0x0400  /**< Pixel data start register */
#define MLX90640_PIXEL_COUNT            768     /**< Total pixels (24x32) */
#define MLX90640_LINE_COUNT             24      /**< Image height */
#define MLX90640_COLUMN_COUNT           32      /**< Image width */

#define MLX90640_AUX_DATA_ADDRESS       0x0700  /**< Auxiliary data start */
#define MLX90640_AUX_COUNT              64      /**< Aux data size in words */

#define MLX90640_STATUS_REGISTER        0x8000  /**< Status register address */
#define MLX90640_CONTROL_REGISTER       0x800D  /**< Control register address */

#define MLX90640_FRAME_SIZE             834     /**< Frame data size (768 + 64 + 2) */

/* ============================================================================ */
/*                          BIT MASKS & SHIFTS                                  */
/* ============================================================================ */

/* Status Register Bits */
#define MLX90640_STATUS_DATA_READY_BIT  3
#define MLX90640_STATUS_SUBPAGE_BIT     0

/* Control Register Bits */
#define MLX90640_CTRL_REFRESH_SHIFT     7
#define MLX90640_CTRL_REFRESH_MASK      0x0380  /**< Bits [9:7] */

#define MLX90640_CTRL_RESOLUTION_SHIFT  10
#define MLX90640_CTRL_RESOLUTION_MASK   0x0C00  /**< Bits [11:10] */

#define MLX90640_CTRL_MODE_SHIFT        12
#define MLX90640_CTRL_MODE_MASK         0x1000  /**< Bit [12] */

#define MLX90640_CTRL_TRIGGER_BIT       15

/* ============================================================================ */
/*                          CALIBRATION PARAMETERS                              */
/* ============================================================================ */

/**
 * @brief MLX90640 Calibration Parameters
 * 
 * Extracted from EEPROM, contains all factory calibration data
 * needed for temperature computation.
 */
typedef struct {
    /* VDD Parameters */
    int16_t kVdd;
    int16_t vdd25;
    
    /* PTAT Parameters */
    float KvPTAT;
    float KtPTAT;
    uint16_t vPTAT25;
    float alphaPTAT;
    
    /* Gain & TGC */
    int16_t gainEE;
    float tgc;
    
    /* Compensation Pixel Parameters */
    float cpKv;
    float cpKta;
    float cpAlpha[2];
    int16_t cpOffset[2];
    
    /* Resolution & Mode */
    uint8_t resolutionEE;
    uint8_t calibrationModeEE;
    
    /* Temperature Sensitivity */
    float KsTa;
    float ksTo[5];
    int16_t ct[5];
    
    /* Pixel-specific Sensitivity (768 pixels) */
    uint16_t alpha[MLX90640_PIXEL_COUNT];
    uint8_t alphaScale;
    
    /* Pixel Offset (768 pixels) */
    int16_t offset[MLX90640_PIXEL_COUNT];
    
    /* Pixel Kta (768 pixels) */
    int8_t kta[MLX90640_PIXEL_COUNT];
    uint8_t ktaScale;
    
    /* Pixel Kv (768 pixels) */
    int8_t kv[MLX90640_PIXEL_COUNT];
    uint8_t kvScale;
    
    /* Chess Pattern Compensation */
    float ilChessC[3];
    
    /* Defective Pixels */
    uint16_t brokenPixels[5];
    uint16_t outlierPixels[5];
    
} MLX90640_Params_t;

/* ============================================================================ */
/*                          ENUMERATIONS                                        */
/* ============================================================================ */

/**
 * @brief MLX90640 Status Codes
 */
typedef enum {
    MLX90640_OK                     = 0,    /**< Operation successful */
    MLX90640_ERROR                  = -1,   /**< Generic error */
    MLX90640_ERROR_I2C_NACK         = -2,   /**< I2C NACK received */
    MLX90640_ERROR_I2C_WRITE        = -3,   /**< I2C write failed */
    MLX90640_ERROR_I2C_READ         = -4,   /**< I2C read failed */
    MLX90640_ERROR_I2C_TIMEOUT      = -5,   /**< I2C timeout */
    MLX90640_ERROR_EEPROM           = -6,   /**< EEPROM read error */
    MLX90640_ERROR_PARAM_EXTRACT    = -7,   /**< Parameter extraction failed */
    MLX90640_ERROR_FRAME_DATA       = -8,   /**< Frame data invalid */
    MLX90640_ERROR_BROKEN_PIXELS    = -9,   /**< Too many broken pixels */
    MLX90640_ERROR_OUTLIER_PIXELS   = -10,  /**< Too many outlier pixels */
    MLX90640_ERROR_BAD_PIXELS       = -11,  /**< Too many bad pixels total */
    MLX90640_ERROR_ADJACENT_PIXELS  = -12,  /**< Adjacent bad pixels detected */
    MLX90640_ERROR_TIMEOUT          = -13,  /**< General timeout */
    MLX90640_ERROR_NOT_INITIALIZED  = -14,  /**< Device not initialized */
    MLX90640_ERROR_NULL_POINTER     = -15,  /**< NULL pointer passed */
    MLX90640_ERROR_INVALID_PARAM    = -16,  /**< Invalid parameter */
} MLX90640_Status_t;

/**
 * @brief Refresh Rate Options
 */
typedef enum {
    MLX90640_REFRESH_0_5_HZ = 0x00,  /**< 0.5 Hz */
    MLX90640_REFRESH_1_HZ   = 0x01,  /**< 1 Hz */
    MLX90640_REFRESH_2_HZ   = 0x02,  /**< 2 Hz */
    MLX90640_REFRESH_4_HZ   = 0x03,  /**< 4 Hz */
    MLX90640_REFRESH_8_HZ   = 0x04,  /**< 8 Hz */
    MLX90640_REFRESH_16_HZ  = 0x05,  /**< 16 Hz */
    MLX90640_REFRESH_32_HZ  = 0x06,  /**< 32 Hz */
    MLX90640_REFRESH_64_HZ  = 0x07,  /**< 64 Hz */
} MLX90640_RefreshRate_t;

/**
 * @brief ADC Resolution Options
 */
typedef enum {
    MLX90640_RESOLUTION_16_BIT = 0x00,  /**< 16-bit ADC */
    MLX90640_RESOLUTION_17_BIT = 0x01,  /**< 17-bit ADC */
    MLX90640_RESOLUTION_18_BIT = 0x02,  /**< 18-bit ADC */
    MLX90640_RESOLUTION_19_BIT = 0x03,  /**< 19-bit ADC */
} MLX90640_Resolution_t;

/**
 * @brief Measurement Mode
 */
typedef enum {
    MLX90640_MODE_INTERLEAVED = 0x00,  /**< Interleaved mode */
    MLX90640_MODE_CHESS       = 0x01,  /**< Chess pattern mode */
} MLX90640_Mode_t;

/* ============================================================================ */
/*                          HELPER MACROS                                       */
/* ============================================================================ */

#define MLX90640_BIT(x)                 (1UL << (x))
#define MLX90640_MS_BYTE(x)             (((x) >> 8) & 0xFF)
#define MLX90640_LS_BYTE(x)             ((x) & 0xFF)

#define MLX90640_NIBBLE1(x)             ((x) & 0x000F)
#define MLX90640_NIBBLE2(x)             (((x) >> 4) & 0x000F)
#define MLX90640_NIBBLE3(x)             (((x) >> 8) & 0x000F)
#define MLX90640_NIBBLE4(x)             (((x) >> 12) & 0x000F)

#define MLX90640_POW2(x)                (1UL << (x))

#define MLX90640_SCALE_ALPHA            0.000001f

#ifdef __cplusplus
}
#endif

#endif /* MLX90640_TYPES_H */