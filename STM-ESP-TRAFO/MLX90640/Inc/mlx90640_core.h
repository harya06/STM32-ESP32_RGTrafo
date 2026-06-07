/**
 ******************************************************************************
 * @file    mlx90640_core.h
 * @brief   MLX90640 Core Calculation Engine (Reentrant, Stateless)
 * @author  Refactored from Melexis MLX90640_API.c
 * @date    2024
 ******************************************************************************
 * @attention
 *
 * This module contains the core temperature calculation algorithms.
 * All functions are REENTRANT and STATELESS - they operate only on
 * passed parameters and buffers.
 *
 * KEY DESIGN PRINCIPLES:
 * - No global state
 * - No static variables
 * - Thread-safe by design
 * - Pure functions (input → output)
 *
 ******************************************************************************
 */

#ifndef MLX90640_CORE_H
#define MLX90640_CORE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "mlx90640_types.h"
#include "mlx90640_device.h"

/* ============================================================================ */
/*                          PARAMETER EXTRACTION                                */
/* ============================================================================ */

/**
 * @brief Extract calibration parameters from EEPROM data
 * 
 * @param eeprom_data Pointer to 832-word EEPROM buffer
 * @param params Pointer to parameters structure (output)
 * 
 * @return MLX90640_OK on success, error code on failure
 * 
 * @note This function validates defective pixels and returns error if
 *       too many bad pixels are detected
 */
MLX90640_Status_t MLX90640_ExtractParameters(const uint16_t *eeprom_data,
                                              MLX90640_Params_t *params);

/* ============================================================================ */
/*                          FRAME DATA PROCESSING                               */
/* ============================================================================ */

/**
 * @brief Calculate VDD (supply voltage) from frame data
 * 
 * @param frame_data Pointer to frame data buffer
 * @param params Pointer to calibration parameters
 * 
 * @return VDD in volts
 */
float MLX90640_GetVdd(const uint16_t *frame_data,
                      const MLX90640_Params_t *params);

/**
 * @brief Calculate ambient temperature (Ta) from frame data
 * 
 * @param frame_data Pointer to frame data buffer
 * @param params Pointer to calibration parameters
 * 
 * @return Ambient temperature in °C
 */
float MLX90640_GetTa(const uint16_t *frame_data,
                     const MLX90640_Params_t *params);

/**
 * @brief Get subpage number from frame data
 * 
 * @param frame_data Pointer to frame data buffer
 * 
 * @return Subpage number (0 or 1)
 */
uint8_t MLX90640_GetSubPage(const uint16_t *frame_data);

/**
 * @brief Calculate object temperatures for all pixels
 * 
 * This is the main temperature calculation function. It computes absolute
 * temperature in °C for each of the 768 pixels.
 * 
 * @param frame_data Pointer to frame data buffer
 * @param params Pointer to calibration parameters
 * @param emissivity Object emissivity (0.1 - 1.0, typically 0.95)
 * @param tr Reflected temperature (typically Ta)
 * @param result Pointer to 768-element output array
 * 
 * @note Only pixels matching the current subpage are calculated
 */
void MLX90640_CalculateTo(const uint16_t *frame_data,
                          const MLX90640_Params_t *params,
                          float emissivity,
                          float tr,
                          float *result);

/**
 * @brief Calculate IR image values (raw, without temperature conversion)
 * 
 * @param frame_data Pointer to frame data buffer
 * @param params Pointer to calibration parameters
 * @param result Pointer to 768-element output array
 */
void MLX90640_GetImage(const uint16_t *frame_data,
                       const MLX90640_Params_t *params,
                       float *result);

/* ============================================================================ */
/*                          BAD PIXEL CORRECTION                                */
/* ============================================================================ */

/**
 * @brief Correct defective pixels using interpolation
 * 
 * @param pixels Array of defective pixel indices (terminated by 0xFFFF)
 * @param to Temperature array to correct (modified in-place)
 * @param mode Measurement mode (0 = interleaved, 1 = chess)
 * @param params Pointer to calibration parameters
 */
void MLX90640_BadPixelsCorrection(const uint16_t *pixels,
                                  float *to,
                                  uint8_t mode,
                                  const MLX90640_Params_t *params);

/* ============================================================================ */
/*                          FRAME VALIDATION                                    */
/* ============================================================================ */

/**
 * @brief Validate frame data integrity
 * 
 * @param frame_data Pointer to frame data buffer
 * 
 * @return MLX90640_OK if valid, error code otherwise
 */
MLX90640_Status_t MLX90640_ValidateFrameData(const uint16_t *frame_data);

/**
 * @brief Validate auxiliary data integrity
 * 
 * @param aux_data Pointer to auxiliary data buffer (64 words)
 * 
 * @return MLX90640_OK if valid, error code otherwise
 */
MLX90640_Status_t MLX90640_ValidateAuxData(const uint16_t *aux_data);

#ifdef __cplusplus
}
#endif

#endif /* MLX90640_CORE_H */