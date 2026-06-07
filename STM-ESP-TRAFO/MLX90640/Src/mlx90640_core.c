/**
 ******************************************************************************
 * @file    mlx90640_core.c
 * @brief   MLX90640 Core Engine Implementation
 * @author  Refactored from Melexis MLX90640_API.c
 * @date    2024
 ******************************************************************************
 */

#include "mlx90640_core.h"
#include <math.h>
#include <string.h>

/* ============================================================================ */
/*                          PRIVATE FUNCTION PROTOTYPES                         */
/* ============================================================================ */

static void ExtractVDDParameters(const uint16_t *eeData, MLX90640_Params_t *params);
static void ExtractPTATParameters(const uint16_t *eeData, MLX90640_Params_t *params);
static void ExtractGainParameters(const uint16_t *eeData, MLX90640_Params_t *params);
static void ExtractTgcParameters(const uint16_t *eeData, MLX90640_Params_t *params);
static void ExtractResolutionParameters(const uint16_t *eeData, MLX90640_Params_t *params);
static void ExtractKsTaParameters(const uint16_t *eeData, MLX90640_Params_t *params);
static void ExtractKsToParameters(const uint16_t *eeData, MLX90640_Params_t *params);
static void ExtractAlphaParameters(const uint16_t *eeData, MLX90640_Params_t *params);
static void ExtractOffsetParameters(const uint16_t *eeData, MLX90640_Params_t *params);
static void ExtractKtaPixelParameters(const uint16_t *eeData, MLX90640_Params_t *params);
static void ExtractKvPixelParameters(const uint16_t *eeData, MLX90640_Params_t *params);
static void ExtractCPParameters(const uint16_t *eeData, MLX90640_Params_t *params);
static void ExtractCILCParameters(const uint16_t *eeData, MLX90640_Params_t *params);
static MLX90640_Status_t ExtractDeviatingPixels(const uint16_t *eeData, MLX90640_Params_t *params);

static int CheckAdjacentPixels(uint16_t pix1, uint16_t pix2);
static float GetMedian(float *values, int n);
static int IsPixelBad(uint16_t pixel, const MLX90640_Params_t *params);

/* ============================================================================ */
/*                          PUBLIC FUNCTIONS                                    */
/* ============================================================================ */

/**
 * @brief Extract all calibration parameters from EEPROM
 */
MLX90640_Status_t MLX90640_ExtractParameters(const uint16_t *eeprom_data,
                                              MLX90640_Params_t *params)
{
    /* Parameter validation */
    if (eeprom_data == NULL || params == NULL) {
        return MLX90640_ERROR_NULL_POINTER;
    }
    
    /* Clear parameter structure */
    memset(params, 0, sizeof(MLX90640_Params_t));
    
    /* Extract all parameter groups */
    ExtractVDDParameters(eeprom_data, params);
    ExtractPTATParameters(eeprom_data, params);
    ExtractGainParameters(eeprom_data, params);
    ExtractTgcParameters(eeprom_data, params);
    ExtractResolutionParameters(eeprom_data, params);
    ExtractKsTaParameters(eeprom_data, params);
    ExtractKsToParameters(eeprom_data, params);
    ExtractCPParameters(eeprom_data, params);
    ExtractAlphaParameters(eeprom_data, params);
    ExtractOffsetParameters(eeprom_data, params);
    ExtractKtaPixelParameters(eeprom_data, params);
    ExtractKvPixelParameters(eeprom_data, params);
    ExtractCILCParameters(eeprom_data, params);
    
    /* Extract and validate defective pixels */
    MLX90640_Status_t status = ExtractDeviatingPixels(eeprom_data, params);
    
    if (status != MLX90640_OK) {
        return status;
    }
    
    return MLX90640_OK;
}

/**
 * @brief Get VDD from frame data
 */
float MLX90640_GetVdd(const uint16_t *frame_data,
                      const MLX90640_Params_t *params)
{
    if (frame_data == NULL || params == NULL) {
        return 0.0f;
    }
    
    /* Extract resolution from control register (frame_data[832]) */
    uint16_t resolution_ram = (frame_data[832] >> MLX90640_CTRL_RESOLUTION_SHIFT) & 0x03;
    
    /* Resolution correction factor */
    float resolution_correction = (float)MLX90640_POW2(params->resolutionEE) / 
                                  (float)MLX90640_POW2(resolution_ram);
    
    /* Calculate VDD */
    int16_t vdd_pixel = (int16_t)frame_data[810];
    float vdd = (resolution_correction * vdd_pixel - params->vdd25) / 
                (float)params->kVdd + 3.3f;
    
    return vdd;
}

/**
 * @brief Get ambient temperature from frame data
 */
float MLX90640_GetTa(const uint16_t *frame_data,
                     const MLX90640_Params_t *params)
{
    if (frame_data == NULL || params == NULL) {
        return 0.0f;
    }
    
    /* Get VDD first */
    float vdd = MLX90640_GetVdd(frame_data, params);
    
    /* PTAT calculation */
    int16_t ptat = (int16_t)frame_data[800];
    float ptat_art = ((float)ptat / ((float)ptat * params->alphaPTAT + (float)(int16_t)frame_data[768])) * 
                     (float)MLX90640_POW2(18);
    
    /* Calculate Ta */
    float ta = (ptat_art / (1.0f + params->KvPTAT * (vdd - 3.3f)) - params->vPTAT25);
    ta = ta / params->KtPTAT + 25.0f;
    
    return ta;
}

/**
 * @brief Get subpage number
 */
uint8_t MLX90640_GetSubPage(const uint16_t *frame_data)
{
    if (frame_data == NULL) {
        return 0;
    }
    
    return (uint8_t)(frame_data[833] & 0x0001);
}

/**
 * @brief Validate frame data
 */
MLX90640_Status_t MLX90640_ValidateFrameData(const uint16_t *frame_data)
{
    if (frame_data == NULL) {
        return MLX90640_ERROR_NULL_POINTER;
    }
    
    uint8_t subpage = frame_data[833] & 0x0001;
    
    /* Check each line for invalid data (0x7FFF) */
    for (int i = 0; i < MLX90640_PIXEL_COUNT; i += MLX90640_COLUMN_COUNT) {
        uint8_t line = i / MLX90640_COLUMN_COUNT;
        
        if ((frame_data[i] == 0x7FFF) && ((line % 2) == subpage)) {
            return MLX90640_ERROR_FRAME_DATA;
        }
    }
    
    return MLX90640_OK;
}

/**
 * @brief Validate auxiliary data
 */
MLX90640_Status_t MLX90640_ValidateAuxData(const uint16_t *aux_data)
{
    if (aux_data == NULL) {
        return MLX90640_ERROR_NULL_POINTER;
    }
    
    /* Check critical aux data elements for 0x7FFF (invalid marker) */
    if (aux_data[0] == 0x7FFF) return MLX90640_ERROR_FRAME_DATA;
    
    for (int i = 8; i < 19; i++) {
        if (aux_data[i] == 0x7FFF) return MLX90640_ERROR_FRAME_DATA;
    }
    
    for (int i = 20; i < 23; i++) {
        if (aux_data[i] == 0x7FFF) return MLX90640_ERROR_FRAME_DATA;
    }
    
    for (int i = 24; i < 33; i++) {
        if (aux_data[i] == 0x7FFF) return MLX90640_ERROR_FRAME_DATA;
    }
    
    for (int i = 40; i < 51; i++) {
        if (aux_data[i] == 0x7FFF) return MLX90640_ERROR_FRAME_DATA;
    }
    
    for (int i = 52; i < 55; i++) {
        if (aux_data[i] == 0x7FFF) return MLX90640_ERROR_FRAME_DATA;
    }
    
    for (int i = 56; i < 64; i++) {
        if (aux_data[i] == 0x7FFF) return MLX90640_ERROR_FRAME_DATA;
    }
    
    return MLX90640_OK;
}

/* ============================================================================ */
/*                          PRIVATE PARAMETER EXTRACTION                        */
/* ============================================================================ */

static void ExtractVDDParameters(const uint16_t *eeData, MLX90640_Params_t *params)
{
    int8_t kVdd = (int8_t)MLX90640_MS_BYTE(eeData[51]);
    int16_t vdd25 = (int16_t)MLX90640_LS_BYTE(eeData[51]);
    vdd25 = ((vdd25 - 256) << 5) - 8192;
    
    params->kVdd = 32 * kVdd;
    params->vdd25 = vdd25;
}

static void ExtractPTATParameters(const uint16_t *eeData, MLX90640_Params_t *params)
{
    float KvPTAT = (float)((eeData[50] >> 10) & 0x3F);
    if (KvPTAT > 31.0f) {
        KvPTAT = KvPTAT - 64.0f;
    }
    KvPTAT = KvPTAT / 4096.0f;
    
    float KtPTAT = (float)(eeData[50] & 0x03FF);
    if (KtPTAT > 511.0f) {
        KtPTAT = KtPTAT - 1024.0f;
    }
    KtPTAT = KtPTAT / 8.0f;
    
    uint16_t vPTAT25 = eeData[49];
    
    float alphaPTAT = (float)MLX90640_NIBBLE4(eeData[16]) / powf(2.0f, 14.0f) + 8.0f;
    
    params->KvPTAT = KvPTAT;
    params->KtPTAT = KtPTAT;
    params->vPTAT25 = vPTAT25;
    params->alphaPTAT = alphaPTAT;
}

static void ExtractGainParameters(const uint16_t *eeData, MLX90640_Params_t *params)
{
    params->gainEE = (int16_t)eeData[48];
}

static void ExtractTgcParameters(const uint16_t *eeData, MLX90640_Params_t *params)
{
    params->tgc = (float)((int8_t)MLX90640_LS_BYTE(eeData[60])) / 32.0f;
}

static void ExtractResolutionParameters(const uint16_t *eeData, MLX90640_Params_t *params)
{
    params->resolutionEE = (uint8_t)((eeData[56] >> 12) & 0x03);
}

static void ExtractKsTaParameters(const uint16_t *eeData, MLX90640_Params_t *params)
{
    params->KsTa = (float)((int8_t)MLX90640_MS_BYTE(eeData[60])) / 8192.0f;
}

static void ExtractKsToParameters(const uint16_t *eeData, MLX90640_Params_t *params)
{
    int8_t step = (int8_t)(((eeData[63] >> 12) & 0x03) * 10);
    
    params->ct[0] = -40;
    params->ct[1] = 0;
    params->ct[2] = (int16_t)MLX90640_NIBBLE2(eeData[63]) * step;
    params->ct[3] = params->ct[2] + (int16_t)MLX90640_NIBBLE3(eeData[63]) * step;
    params->ct[4] = 400;
    
    uint32_t KsToScale = MLX90640_NIBBLE1(eeData[63]) + 8;
    KsToScale = 1UL << KsToScale;
    
    params->ksTo[0] = (float)((int8_t)MLX90640_LS_BYTE(eeData[61])) / (float)KsToScale;
    params->ksTo[1] = (float)((int8_t)MLX90640_MS_BYTE(eeData[61])) / (float)KsToScale;
    params->ksTo[2] = (float)((int8_t)MLX90640_LS_BYTE(eeData[62])) / (float)KsToScale;
    params->ksTo[3] = (float)((int8_t)MLX90640_MS_BYTE(eeData[62])) / (float)KsToScale;
    params->ksTo[4] = -0.0002f;
}

static void ExtractAlphaParameters(const uint16_t *eeData, MLX90640_Params_t *params)
{
    int accRow[24];
    int accColumn[32];
    float alphaTemp[768];
    
    uint8_t accRemScale = MLX90640_NIBBLE1(eeData[32]);
    uint8_t accColumnScale = MLX90640_NIBBLE2(eeData[32]);
    uint8_t accRowScale = MLX90640_NIBBLE3(eeData[32]);
    uint8_t alphaScale = MLX90640_NIBBLE4(eeData[32]) + 30;
    uint16_t alphaRef = eeData[33];
    
    /* Extract row accumulation */
    for (int i = 0; i < 6; i++) {
        int p = i * 4;
        accRow[p + 0] = MLX90640_NIBBLE1(eeData[34 + i]);
        accRow[p + 1] = MLX90640_NIBBLE2(eeData[34 + i]);
        accRow[p + 2] = MLX90640_NIBBLE3(eeData[34 + i]);
        accRow[p + 3] = MLX90640_NIBBLE4(eeData[34 + i]);
    }
    
    for (int i = 0; i < 24; i++) {
        if (accRow[i] > 7) {
            accRow[i] = accRow[i] - 16;
        }
    }
    
    /* Extract column accumulation */
    for (int i = 0; i < 8; i++) {
        int p = i * 4;
        accColumn[p + 0] = MLX90640_NIBBLE1(eeData[40 + i]);
        accColumn[p + 1] = MLX90640_NIBBLE2(eeData[40 + i]);
        accColumn[p + 2] = MLX90640_NIBBLE3(eeData[40 + i]);
        accColumn[p + 3] = MLX90640_NIBBLE4(eeData[40 + i]);
    }
    
    for (int i = 0; i < 32; i++) {
        if (accColumn[i] > 7) {
            accColumn[i] = accColumn[i] - 16;
        }
    }
    
    /* Calculate alpha for each pixel */
    for (int i = 0; i < 24; i++) {
        for (int j = 0; j < 32; j++) {
            int p = 32 * i + j;
            alphaTemp[p] = (float)((eeData[64 + p] >> 4) & 0x3F);
            if (alphaTemp[p] > 31.0f) {
                alphaTemp[p] = alphaTemp[p] - 64.0f;
            }
            alphaTemp[p] = alphaTemp[p] * (1 << accRemScale);
            alphaTemp[p] = (alphaRef + (accRow[i] << accRowScale) + 
                           (accColumn[j] << accColumnScale) + alphaTemp[p]);
            alphaTemp[p] = alphaTemp[p] / powf(2.0f, (float)alphaScale);
            alphaTemp[p] = alphaTemp[p] - params->tgc * 
                          (params->cpAlpha[0] + params->cpAlpha[1]) / 2.0f;
            alphaTemp[p] = MLX90640_SCALE_ALPHA / alphaTemp[p];
        }
    }
    
    /* Find max alpha for scaling */
    float temp = alphaTemp[0];
    for (int i = 1; i < 768; i++) {
        if (alphaTemp[i] > temp) {
            temp = alphaTemp[i];
        }
    }
    
    /* Scale alpha to fit in uint16_t */
    uint8_t alpha_scale = 0;
    while (temp < 32767.4f) {
        temp = temp * 2.0f;
        alpha_scale = alpha_scale + 1;
    }
    
    for (int i = 0; i < 768; i++) {
        temp = alphaTemp[i] * powf(2.0f, (float)alpha_scale);
        params->alpha[i] = (uint16_t)(temp + 0.5f);
    }
    
    params->alphaScale = alpha_scale;
}

static void ExtractOffsetParameters(const uint16_t *eeData, MLX90640_Params_t *params)
{
    int occRow[24];
    int occColumn[32];
    
    uint8_t occRemScale = MLX90640_NIBBLE1(eeData[16]);
    uint8_t occColumnScale = MLX90640_NIBBLE2(eeData[16]);
    uint8_t occRowScale = MLX90640_NIBBLE3(eeData[16]);
    int16_t offsetRef = (int16_t)eeData[17];
    
    /* Extract row offset */
    for (int i = 0; i < 6; i++) {
        int p = i * 4;
        occRow[p + 0] = MLX90640_NIBBLE1(eeData[18 + i]);
        occRow[p + 1] = MLX90640_NIBBLE2(eeData[18 + i]);
        occRow[p + 2] = MLX90640_NIBBLE3(eeData[18 + i]);
        occRow[p + 3] = MLX90640_NIBBLE4(eeData[18 + i]);
    }
    
    for (int i = 0; i < 24; i++) {
        if (occRow[i] > 7) {
            occRow[i] = occRow[i] - 16;
        }
    }
    
    /* Extract column offset */
    for (int i = 0; i < 8; i++) {
        int p = i * 4;
        occColumn[p + 0] = MLX90640_NIBBLE1(eeData[24 + i]);
        occColumn[p + 1] = MLX90640_NIBBLE2(eeData[24 + i]);
        occColumn[p + 2] = MLX90640_NIBBLE3(eeData[24 + i]);
        occColumn[p + 3] = MLX90640_NIBBLE4(eeData[24 + i]);
    }
    
    for (int i = 0; i < 32; i++) {
        if (occColumn[i] > 7) {
            occColumn[i] = occColumn[i] - 16;
        }
    }
    
    /* Calculate offset for each pixel */
    for (int i = 0; i < 24; i++) {
        for (int j = 0; j < 32; j++) {
            int p = 32 * i + j;
            params->offset[p] = (int16_t)((eeData[64 + p] >> 10) & 0x3F);
            if (params->offset[p] > 31) {
                params->offset[p] = params->offset[p] - 64;
            }
            params->offset[p] = params->offset[p] * (1 << occRemScale);
            params->offset[p] = (offsetRef + (occRow[i] << occRowScale) + 
                               (occColumn[j] << occColumnScale) + params->offset[p]);
        }
    }
}

static void ExtractKtaPixelParameters(const uint16_t *eeData, MLX90640_Params_t *params)
{
    int8_t KtaRC[4];
    float ktaTemp[768];
    
    KtaRC[0] = (int8_t)MLX90640_MS_BYTE(eeData[54]);
    KtaRC[2] = (int8_t)MLX90640_LS_BYTE(eeData[54]);
    KtaRC[1] = (int8_t)MLX90640_MS_BYTE(eeData[55]);
    KtaRC[3] = (int8_t)MLX90640_LS_BYTE(eeData[55]);
    
    uint8_t ktaScale1 = MLX90640_NIBBLE2(eeData[56]) + 8;
    uint8_t ktaScale2 = MLX90640_NIBBLE1(eeData[56]);
    
    for (int i = 0; i < 24; i++) {
        for (int j = 0; j < 32; j++) {
            int p = 32 * i + j;
            int split = 2 * (p / 32 - (p / 64) * 2) + p % 2;
            ktaTemp[p] = (float)((eeData[64 + p] >> 1) & 0x07);
            if (ktaTemp[p] > 3.0f) {
                ktaTemp[p] = ktaTemp[p] - 8.0f;
            }
            ktaTemp[p] = ktaTemp[p] * (1 << ktaScale2);
            ktaTemp[p] = KtaRC[split] + ktaTemp[p];
            ktaTemp[p] = ktaTemp[p] / powf(2.0f, (float)ktaScale1);
        }
    }
    
    /* Find max kta for scaling */
    float temp = fabsf(ktaTemp[0]);
    for (int i = 1; i < 768; i++) {
        if (fabsf(ktaTemp[i]) > temp) {
            temp = fabsf(ktaTemp[i]);
        }
    }
    
    uint8_t kta_scale = 0;
    while (temp < 63.4f) {
        temp = temp * 2.0f;
        kta_scale = kta_scale + 1;
    }
    
    for (int i = 0; i < 768; i++) {
        temp = ktaTemp[i] * powf(2.0f, (float)kta_scale);
        if (temp < 0.0f) {
            params->kta[i] = (int8_t)(temp - 0.5f);
        } else {
            params->kta[i] = (int8_t)(temp + 0.5f);
        }
    }
    
    params->ktaScale = kta_scale;
}

static void ExtractKvPixelParameters(const uint16_t *eeData, MLX90640_Params_t *params)
{
    int8_t KvT[4];
    float kvTemp[768];
    
    KvT[0] = (int8_t)MLX90640_NIBBLE4(eeData[52]);
    if (KvT[0] > 7) KvT[0] = KvT[0] - 16;
    
    KvT[2] = (int8_t)MLX90640_NIBBLE3(eeData[52]);
    if (KvT[2] > 7) KvT[2] = KvT[2] - 16;
    
    KvT[1] = (int8_t)MLX90640_NIBBLE2(eeData[52]);
    if (KvT[1] > 7) KvT[1] = KvT[1] - 16;
    
    KvT[3] = (int8_t)MLX90640_NIBBLE1(eeData[52]);
    if (KvT[3] > 7) KvT[3] = KvT[3] - 16;
    
    uint8_t kvScale = MLX90640_NIBBLE3(eeData[56]);
    
    for (int i = 0; i < 24; i++) {
        for (int j = 0; j < 32; j++) {
            int p = 32 * i + j;
            int split = 2 * (p / 32 - (p / 64) * 2) + p % 2;
            kvTemp[p] = (float)KvT[split];
            kvTemp[p] = kvTemp[p] / powf(2.0f, (float)kvScale);
        }
    }
    
    /* Find max kv for scaling */
    float temp = fabsf(kvTemp[0]);
    for (int i = 1; i < 768; i++) {
        if (fabsf(kvTemp[i]) > temp) {
            temp = fabsf(kvTemp[i]);
        }
    }
    
    uint8_t kv_scale = 0;
    while (temp < 63.4f) {
        temp = temp * 2.0f;
        kv_scale = kv_scale + 1;
    }
    
    for (int i = 0; i < 768; i++) {
        temp = kvTemp[i] * powf(2.0f, (float)kv_scale);
        if (temp < 0.0f) {
            params->kv[i] = (int8_t)(temp - 0.5f);
        } else {
            params->kv[i] = (int8_t)(temp + 0.5f);
        }
    }
    
    params->kvScale = kv_scale;
}

static void ExtractCPParameters(const uint16_t *eeData, MLX90640_Params_t *params)
{
    float alphaSP[2];
    int16_t offsetSP[2];
    
    uint8_t alphaScale = MLX90640_NIBBLE4(eeData[32]) + 27;
    
    offsetSP[0] = (int16_t)(eeData[58] & 0x03FF);
    if (offsetSP[0] > 511) {
        offsetSP[0] = offsetSP[0] - 1024;
    }
    
    offsetSP[1] = (int16_t)((eeData[58] >> 10) & 0x3F);
    if (offsetSP[1] > 31) {
        offsetSP[1] = offsetSP[1] - 64;
    }
    offsetSP[1] = offsetSP[1] + offsetSP[0];
    
    alphaSP[0] = (float)(eeData[57] & 0x03FF);
    if (alphaSP[0] > 511.0f) {
        alphaSP[0] = alphaSP[0] - 1024.0f;
    }
    alphaSP[0] = alphaSP[0] / powf(2.0f, (float)alphaScale);
    
    alphaSP[1] = (float)((eeData[57] >> 10) & 0x3F);
    if (alphaSP[1] > 31.0f) {
        alphaSP[1] = alphaSP[1] - 64.0f;
    }
    alphaSP[1] = (1.0f + alphaSP[1] / 128.0f) * alphaSP[0];
    
    float cpKta = (float)((int8_t)MLX90640_LS_BYTE(eeData[59]));
    uint8_t ktaScale1 = MLX90640_NIBBLE2(eeData[56]) + 8;
    params->cpKta = cpKta / powf(2.0f, (float)ktaScale1);
    
    float cpKv = (float)((int8_t)MLX90640_MS_BYTE(eeData[59]));
    uint8_t kvScale = MLX90640_NIBBLE3(eeData[56]);
    params->cpKv = cpKv / powf(2.0f, (float)kvScale);
    
    params->cpAlpha[0] = alphaSP[0];
    params->cpAlpha[1] = alphaSP[1];
    params->cpOffset[0] = offsetSP[0];
    params->cpOffset[1] = offsetSP[1];
}

static void ExtractCILCParameters(const uint16_t *eeData, MLX90640_Params_t *params)
{
    float ilChessC[3];
    
    uint8_t calibrationModeEE = (uint8_t)(((eeData[10] & 0x0800) >> 4) ^ 0x80);
    
    ilChessC[0] = (float)(eeData[53] & 0x003F);
    if (ilChessC[0] > 31.0f) {
        ilChessC[0] = ilChessC[0] - 64.0f;
    }
    ilChessC[0] = ilChessC[0] / 16.0f;
    
    ilChessC[1] = (float)((eeData[53] >> 6) & 0x1F);
    if (ilChessC[1] > 15.0f) {
        ilChessC[1] = ilChessC[1] - 32.0f;
    }
    ilChessC[1] = ilChessC[1] / 2.0f;
    
    ilChessC[2] = (float)((eeData[53] >> 11) & 0x1F);
    if (ilChessC[2] > 15.0f) {
        ilChessC[2] = ilChessC[2] - 32.0f;
    }
    ilChessC[2] = ilChessC[2] / 8.0f;
    
    params->calibrationModeEE = calibrationModeEE;
    params->ilChessC[0] = ilChessC[0];
    params->ilChessC[1] = ilChessC[1];
    params->ilChessC[2] = ilChessC[2];
}

static MLX90640_Status_t ExtractDeviatingPixels(const uint16_t *eeData, MLX90640_Params_t *params)
{
    uint16_t pixCnt = 0;
    uint16_t brokenPixCnt = 0;
    uint16_t outlierPixCnt = 0;
    
    /* Initialize defective pixel lists */
    for (int i = 0; i < 5; i++) {
        params->brokenPixels[i] = 0xFFFF;
        params->outlierPixels[i] = 0xFFFF;
    }
    
    /* Scan all pixels */
    pixCnt = 0;
    while (pixCnt < 768 && brokenPixCnt < 5 && outlierPixCnt < 5) {
        if (eeData[pixCnt + 64] == 0) {
            params->brokenPixels[brokenPixCnt] = pixCnt;
            brokenPixCnt++;
        } else if ((eeData[pixCnt + 64] & 0x0001) != 0) {
            params->outlierPixels[outlierPixCnt] = pixCnt;
            outlierPixCnt++;
        }
        pixCnt++;
    }
    
    /* Check pixel count limits */
    if (brokenPixCnt > 4) {
        return MLX90640_ERROR_BROKEN_PIXELS;
    }
    
    if (outlierPixCnt > 4) {
        return MLX90640_ERROR_OUTLIER_PIXELS;
    }
    
    if ((brokenPixCnt + outlierPixCnt) > 4) {
        return MLX90640_ERROR_BAD_PIXELS;
    }
    
    /* Check for adjacent broken pixels */
    for (uint16_t i = 0; i < brokenPixCnt; i++) {
        for (uint16_t j = i + 1; j < brokenPixCnt; j++) {
            if (CheckAdjacentPixels(params->brokenPixels[i], params->brokenPixels[j]) != 0) {
                return MLX90640_ERROR_ADJACENT_PIXELS;
            }
        }
    }
    
    /* Check for adjacent outlier pixels */
    for (uint16_t i = 0; i < outlierPixCnt; i++) {
        for (uint16_t j = i + 1; j < outlierPixCnt; j++) {
            if (CheckAdjacentPixels(params->outlierPixels[i], params->outlierPixels[j]) != 0) {
                return MLX90640_ERROR_ADJACENT_PIXELS;
            }
        }
    }
    
    /* Check for adjacent broken and outlier pixels */
    for (uint16_t i = 0; i < brokenPixCnt; i++) {
        for (uint16_t j = 0; j < outlierPixCnt; j++) {
            if (CheckAdjacentPixels(params->brokenPixels[i], params->outlierPixels[j]) != 0) {
                return MLX90640_ERROR_ADJACENT_PIXELS;
            }
        }
    }
    
    return MLX90640_OK;
}

static int CheckAdjacentPixels(uint16_t pix1, uint16_t pix2)
{
    uint16_t lp1 = pix1 >> 5;
    uint16_t lp2 = pix2 >> 5;
    uint16_t cp1 = pix1 - (lp1 << 5);
    uint16_t cp2 = pix2 - (lp2 << 5);
    
    int pixPosDif = (int)lp1 - (int)lp2;
    if (pixPosDif > -2 && pixPosDif < 2) {
        pixPosDif = (int)cp1 - (int)cp2;
        if (pixPosDif > -2 && pixPosDif < 2) {
            return -6;
        }
    }
    
    return 0;
}

/* ============================================================================ */
/*                          TEMPERATURE CALCULATION                             */
/* ============================================================================ */

/**
 * @brief Calculate object temperatures for all pixels
 */
void MLX90640_CalculateTo(const uint16_t *frame_data,
                          const MLX90640_Params_t *params,
                          float emissivity,
                          float tr,
                          float *result)
{
    if (frame_data == NULL || params == NULL || result == NULL) {
        return;
    }
    
    /* Get basic parameters */
    uint8_t subPage = frame_data[833] & 0x0001;
    float vdd = MLX90640_GetVdd(frame_data, params);
    float ta = MLX90640_GetTa(frame_data, params);
    
    /* Precompute temperature factors */
    float ta4 = (ta + 273.15f);
    ta4 = ta4 * ta4;
    ta4 = ta4 * ta4;
    
    float tr4 = (tr + 273.15f);
    tr4 = tr4 * tr4;
    tr4 = tr4 * tr4;
    
    float taTr = tr4 - (tr4 - ta4) / emissivity;
    
    /* Precompute scale factors */
    float ktaScale = powf(2.0f, (float)params->ktaScale);
    float kvScale = powf(2.0f, (float)params->kvScale);
    float alphaScale = powf(2.0f, (float)params->alphaScale);
    
    /* Alpha correction factors for temperature ranges */
    float alphaCorrR[4];
    alphaCorrR[0] = 1.0f / (1.0f + params->ksTo[0] * 40.0f);
    alphaCorrR[1] = 1.0f;
    alphaCorrR[2] = (1.0f + params->ksTo[1] * (float)params->ct[2]);
    alphaCorrR[3] = alphaCorrR[2] * (1.0f + params->ksTo[2] * 
                    ((float)params->ct[3] - (float)params->ct[2]));
    
    /* Gain calculation */
    float gain = (float)params->gainEE / (float)(int16_t)frame_data[778];
    
    /* Get measurement mode */
    uint8_t mode = (uint8_t)((frame_data[832] >> 5) & 0x01);
    
    /* Compensation pixel data */
    float irDataCP[2];
    irDataCP[0] = (float)(int16_t)frame_data[776] * gain;
    irDataCP[1] = (float)(int16_t)frame_data[808] * gain;
    
    irDataCP[0] = irDataCP[0] - params->cpOffset[0] * 
                  (1.0f + params->cpKta * (ta - 25.0f)) * 
                  (1.0f + params->cpKv * (vdd - 3.3f));
    
    if (mode == params->calibrationModeEE) {
        irDataCP[1] = irDataCP[1] - params->cpOffset[1] * 
                      (1.0f + params->cpKta * (ta - 25.0f)) * 
                      (1.0f + params->cpKv * (vdd - 3.3f));
    } else {
        irDataCP[1] = irDataCP[1] - (params->cpOffset[1] + params->ilChessC[0]) * 
                      (1.0f + params->cpKta * (ta - 25.0f)) * 
                      (1.0f + params->cpKv * (vdd - 3.3f));
    }
    
    /* Process each pixel */
    for (int pixelNumber = 0; pixelNumber < 768; pixelNumber++) {
        
        /* Pattern calculation */
        int8_t ilPattern = pixelNumber / 32 - (pixelNumber / 64) * 2;
        int8_t chessPattern = ilPattern ^ (pixelNumber - (pixelNumber / 2) * 2);
        int8_t conversionPattern = ((pixelNumber + 2) / 4 - (pixelNumber + 3) / 4 + 
                                    (pixelNumber + 1) / 4 - pixelNumber / 4) * 
                                   (1 - 2 * ilPattern);
        
        int8_t pattern;
        if (mode == 0) {
            pattern = ilPattern;
        } else {
            pattern = chessPattern;
        }
        
        /* Only process pixels matching current subpage */
        if (pattern == (int8_t)subPage) {
            
            /* Get IR data with gain */
            float irData = (float)(int16_t)frame_data[pixelNumber] * gain;
            
            /* Get pixel sensitivity */
            float kta = (float)params->kta[pixelNumber] / ktaScale;
            float kv = (float)params->kv[pixelNumber] / kvScale;
            
            /* Compensate for offset */
            irData = irData - (float)params->offset[pixelNumber] * 
                     (1.0f + kta * (ta - 25.0f)) * 
                     (1.0f + kv * (vdd - 3.3f));
            
            /* Chess pattern compensation */
            if (mode != params->calibrationModeEE) {
                irData = irData + params->ilChessC[2] * (2.0f * ilPattern - 1.0f) - 
                         params->ilChessC[1] * conversionPattern;
            }
            
            /* TGC compensation */
            irData = irData - params->tgc * irDataCP[subPage];
            
            /* Emissivity compensation */
            irData = irData / emissivity;
            
            /* Alpha compensation */
            float alphaCompensated = (MLX90640_SCALE_ALPHA * alphaScale) / 
                                    (float)params->alpha[pixelNumber];
            alphaCompensated = alphaCompensated * (1.0f + params->KsTa * (ta - 25.0f));
            
            /* First iteration temperature calculation */
            float Sx = alphaCompensated * alphaCompensated * alphaCompensated * 
                      (irData + alphaCompensated * taTr);
            Sx = sqrtf(sqrtf(Sx)) * params->ksTo[1];
            
            float To = sqrtf(sqrtf(irData / (alphaCompensated * 
                      (1.0f - params->ksTo[1] * 273.15f) + Sx) + taTr)) - 273.15f;
            
            /* Determine temperature range */
            int8_t range;
            if (To < (float)params->ct[1]) {
                range = 0;
            } else if (To < (float)params->ct[2]) {
                range = 1;
            } else if (To < (float)params->ct[3]) {
                range = 2;
            } else {
                range = 3;
            }
            
            /* Second iteration with range correction */
            To = sqrtf(sqrtf(irData / (alphaCompensated * alphaCorrR[range] * 
                (1.0f + params->ksTo[range] * (To - (float)params->ct[range]))) + 
                taTr)) - 273.15f;
            
            result[pixelNumber] = To;
        }
    }
}

/**
 * @brief Get IR image (raw, without temperature conversion)
 */
void MLX90640_GetImage(const uint16_t *frame_data,
                       const MLX90640_Params_t *params,
                       float *result)
{
    if (frame_data == NULL || params == NULL || result == NULL) {
        return;
    }
    
    uint8_t subPage = frame_data[833] & 0x0001;
    float vdd = MLX90640_GetVdd(frame_data, params);
    float ta = MLX90640_GetTa(frame_data, params);
    
    float ktaScale = powf(2.0f, (float)params->ktaScale);
    float kvScale = powf(2.0f, (float)params->kvScale);
    
    float gain = (float)params->gainEE / (float)(int16_t)frame_data[778];
    
    uint8_t mode = (uint8_t)((frame_data[832] >> 5) & 0x01);
    
    float irDataCP[2];
    irDataCP[0] = (float)(int16_t)frame_data[776] * gain;
    irDataCP[1] = (float)(int16_t)frame_data[808] * gain;
    
    irDataCP[0] = irDataCP[0] - params->cpOffset[0] * 
                  (1.0f + params->cpKta * (ta - 25.0f)) * 
                  (1.0f + params->cpKv * (vdd - 3.3f));
    
    if (mode == params->calibrationModeEE) {
        irDataCP[1] = irDataCP[1] - params->cpOffset[1] * 
                      (1.0f + params->cpKta * (ta - 25.0f)) * 
                      (1.0f + params->cpKv * (vdd - 3.3f));
    } else {
        irDataCP[1] = irDataCP[1] - (params->cpOffset[1] + params->ilChessC[0]) * 
                      (1.0f + params->cpKta * (ta - 25.0f)) * 
                      (1.0f + params->cpKv * (vdd - 3.3f));
    }
    
    for (int pixelNumber = 0; pixelNumber < 768; pixelNumber++) {
        
        int8_t ilPattern = pixelNumber / 32 - (pixelNumber / 64) * 2;
        int8_t chessPattern = ilPattern ^ (pixelNumber - (pixelNumber / 2) * 2);
        int8_t conversionPattern = ((pixelNumber + 2) / 4 - (pixelNumber + 3) / 4 + 
                                    (pixelNumber + 1) / 4 - pixelNumber / 4) * 
                                   (1 - 2 * ilPattern);
        
        int8_t pattern;
        if (mode == 0) {
            pattern = ilPattern;
        } else {
            pattern = chessPattern;
        }
        
        if (pattern == (int8_t)subPage) {
            
            float irData = (float)(int16_t)frame_data[pixelNumber] * gain;
            
            float kta = (float)params->kta[pixelNumber] / ktaScale;
            float kv = (float)params->kv[pixelNumber] / kvScale;
            
            irData = irData - (float)params->offset[pixelNumber] * 
                     (1.0f + kta * (ta - 25.0f)) * 
                     (1.0f + kv * (vdd - 3.3f));
            
            if (mode != params->calibrationModeEE) {
                irData = irData + params->ilChessC[2] * (2.0f * ilPattern - 1.0f) - 
                         params->ilChessC[1] * conversionPattern;
            }
            
            irData = irData - params->tgc * irDataCP[subPage];
            
            float alphaCompensated = (float)params->alpha[pixelNumber];
            float image = irData * alphaCompensated;
            
            result[pixelNumber] = image;
        }
    }
}

/* ============================================================================ */
/*                          BAD PIXEL CORRECTION                                */
/* ============================================================================ */

/**
 * @brief Correct defective pixels using neighbor interpolation
 */
void MLX90640_BadPixelsCorrection(const uint16_t *pixels,
                                  float *to,
                                  uint8_t mode,
                                  const MLX90640_Params_t *params)
{
    if (pixels == NULL || to == NULL || params == NULL) {
        return;
    }
    
    float ap[4];
    uint16_t pix = 0;
    
    /* Process each defective pixel */
    while (pixels[pix] != 0xFFFF && pix < 5) {
        
        uint8_t line = pixels[pix] >> 5;
        uint8_t column = pixels[pix] - (line << 5);
        
        if (mode == 1) {  /* Chess mode */
            
            /* Corner and edge cases */
            if (line == 0) {
                if (column == 0) {
                    to[pixels[pix]] = to[33];
                } else if (column == 31) {
                    to[pixels[pix]] = to[62];
                } else {
                    to[pixels[pix]] = (to[pixels[pix] + 31] + to[pixels[pix] + 33]) / 2.0f;
                }
            } else if (line == 23) {
                if (column == 0) {
                    to[pixels[pix]] = to[705];
                } else if (column == 31) {
                    to[pixels[pix]] = to[734];
                } else {
                    to[pixels[pix]] = (to[pixels[pix] - 33] + to[pixels[pix] - 31]) / 2.0f;
                }
            } else if (column == 0) {
                to[pixels[pix]] = (to[pixels[pix] - 31] + to[pixels[pix] + 33]) / 2.0f;
            } else if (column == 31) {
                to[pixels[pix]] = (to[pixels[pix] - 33] + to[pixels[pix] + 31]) / 2.0f;
            } else {
                /* Interior pixel - use 4 diagonal neighbors */
                ap[0] = to[pixels[pix] - 33];
                ap[1] = to[pixels[pix] - 31];
                ap[2] = to[pixels[pix] + 31];
                ap[3] = to[pixels[pix] + 33];
                to[pixels[pix]] = GetMedian(ap, 4);
            }
            
        } else {  /* Interleaved mode */
            
            if (column == 0) {
                to[pixels[pix]] = to[pixels[pix] + 1];
            } else if (column == 1 || column == 30) {
                to[pixels[pix]] = (to[pixels[pix] - 1] + to[pixels[pix] + 1]) / 2.0f;
            } else if (column == 31) {
                to[pixels[pix]] = to[pixels[pix] - 1];
            } else {
                /* Check if neighbors are also bad */
                if (IsPixelBad(pixels[pix] - 2, params) == 0 && 
                    IsPixelBad(pixels[pix] + 2, params) == 0) {
                    
                    ap[0] = to[pixels[pix] + 1] - to[pixels[pix] + 2];
                    ap[1] = to[pixels[pix] - 1] - to[pixels[pix] - 2];
                    
                    if (fabsf(ap[0]) > fabsf(ap[1])) {
                        to[pixels[pix]] = to[pixels[pix] - 1] + ap[1];
                    } else {
                        to[pixels[pix]] = to[pixels[pix] + 1] + ap[0];
                    }
                } else {
                    to[pixels[pix]] = (to[pixels[pix] - 1] + to[pixels[pix] + 1]) / 2.0f;
                }
            }
        }
        
        pix++;
    }
}

/* ============================================================================ */
/*                          PRIVATE HELPER FUNCTIONS                            */
/* ============================================================================ */

/**
 * @brief Get median of float array (with sorting)
 */
static float GetMedian(float *values, int n)
{
    /* Simple bubble sort */
    for (int i = 0; i < n - 1; i++) {
        for (int j = i + 1; j < n; j++) {
            if (values[j] < values[i]) {
                float temp = values[i];
                values[i] = values[j];
                values[j] = temp;
            }
        }
    }
    
    /* Return median */
    if (n % 2 == 0) {
        return (values[n / 2] + values[n / 2 - 1]) / 2.0f;
    } else {
        return values[n / 2];
    }
}

/**
 * @brief Check if pixel is in defective list
 */
static int IsPixelBad(uint16_t pixel, const MLX90640_Params_t *params)
{
    for (int i = 0; i < 5; i++) {
        if (pixel == params->outlierPixels[i] || pixel == params->brokenPixels[i]) {
            return 1;
        }
    }
    return 0;
}