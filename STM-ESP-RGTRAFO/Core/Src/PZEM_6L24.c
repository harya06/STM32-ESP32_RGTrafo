/*
 * PZEM_6L24.c
 *
 *  Created on: May 9, 2026
 *      Author:
 *
 *  Mode: Interrupt (IT)
 *
 *  Alur IT:
 *    1. PZEM_IT_RequestAll()
 *         - BuildReadFrame() bangun 8 byte (40 register, 0x0000–0x0027)
 *         - HAL_UART_Transmit_IT()    state = TX
 *
 *    2. HAL_UART_TxCpltCallback()  (ISR)
 *         - PZEM_IT_TxCpltCallback()
 *         - arm HAL_UART_Receive_IT 1 byte   state = RX
 *
 *    3. HAL_UART_RxCpltCallback()  (ISR, dipanggil tiap 1 byte)
 *         - PZEM_IT_RxCpltCallback()
 *         - kumpulkan byte ke rxBuf
 *         - setelah byte ke-3 (byteCount) diketahui, hitung rxExpected
 *         - bila rxIndex == rxExpected: verif CRC -> COMPLETE / ERROR
 *         - bila belum penuh: arm ulang Receive_IT 1 byte
 *
 *    4. Main loop panggil PZEM_IT_Tick() tiap 1 ms
 *         - TX timeout -> FAULT (AbortTransmit)
 *         - RX timeout -> FAULT (AbortReceive)
 *         - ERROR      -> FAULT (AbortReceive)
 *
 *    5. Main loop cek state == PZEM_STATE_COMPLETE
 *         - PZEM_IT_ProcessData() -> parse rxBuf ke struct
 *
 *
 *   Reg    | Desimal | Isi
 *  --------|---------|--------------------------------------------
 *   0x0000 |  0      | Voltage A         (uint16, 0.1 V)
 *   0x0001 |  1      | Voltage B
 *   0x0002 |  2      | Voltage C
 *   0x0003 |  3      | Current A         (uint16, 0.01 A)
 *   0x0004 |  4      | Current B
 *   0x0005 |  5      | Current C
 *   0x0006 |  6      | Frequency A       (uint16, 0.01 Hz)
 *   0x0007 |  7      | Frequency B
 *   0x0008 |  8      | Frequency C
 *   0x0009 |  9      | Voltage Phase S-R (uint16, 0.01 deg, terhadap tegangan phase R)
 *   0x000A | 10      | Voltage Phase T-R
 *   0x000B | 11      | Current Phase A   (uint16, 0.01 deg, terhadap tegangan phase R)
 *   0x000C | 12      | Current Phase B
 *   0x000D | 13      | Current Phase C
 *   0x000E | 14,15   | Active Power A    (int32, low dulu, 0.1 W)
 *   0x0010 | 16,17   | Active Power B
 *   0x0012 | 18,19   | Active Power C
 *   0x0014 | 20,21   | Reactive Power A  (int32, lo dulu, 0.1 VAR)
 *   0x0016 | 22,23   | Reactive Power B
 *   0x0018 | 24,25   | Reactive Power C
 *   0x001A | 26,27   | Apparent Power A  (int32, lo dulu, 0.1 VA)
 *   0x001C | 28,29   | Apparent Power B
 *   0x001E | 30,31   | Apparent Power C
 *   0x0026 | 38      | PF A=hi byte, PF B=lo byte  (uint8, 0.01)
 *   0x0027 | 39      | PF C=hi byte                (uint8, 0.01)
 */

#include "PZEM_6L24.h"
#include <math.h>

/* ============================================================
 * Internal: Modbus CRC16
 * ============================================================ */
static uint16_t Checksum(const uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFFU;
    for (uint16_t pos = 0; pos < len; pos++) {
        crc ^= (uint16_t)buf[pos];
        for (int i = 8; i != 0; i--) {
            if ((crc & 0x0001U) != 0U) { 
                crc >>= 1; crc ^= 0xA001U; 
            }
            else { 
                crc >>= 1; 
            }
        }
    }
    return crc;
}

/* ============================================================
 * Internal: build frame Modbus FC04
 * ============================================================ */
static void BuildReadFrame(PZEM6L24_t *dev, uint16_t reg, uint16_t count)
{
    dev->txBuf[0] = dev->addr;
    dev->txBuf[1] = 0x04U;
    dev->txBuf[2] = (uint8_t)(reg >> 8);
    dev->txBuf[3] = (uint8_t)(reg & 0xFFU);
    dev->txBuf[4] = (uint8_t)(count >> 8);
    dev->txBuf[5] = (uint8_t)(count & 0xFFU);
    uint16_t crc  = Checksum(dev->txBuf, 6U);
    dev->txBuf[6] = (uint8_t)(crc & 0xFFU);
    dev->txBuf[7] = (uint8_t)(crc >> 8);
    dev->expectedCount = count;
}

/* ============================================================
 * Internal: read register 16-bit dari rxBuf
 * ============================================================ */
static inline uint16_t RxReg(const PZEM6L24_t *dev, uint16_t regAddr)
{
    uint16_t offset = (uint16_t)(3U + (regAddr - PZEM_REQUEST_REG_START) * 2U);
    /* LITTLE-ENDIAN: byte pertama = low, byte kedua = high */
    return (uint16_t)(dev->rxBuf[offset] | ((uint16_t)dev->rxBuf[offset + 1U] << 8));
}

/* ============================================================
 * Internal: read register 32-bit signed dari rxBuf
 *   PZEM-6L24: low word di register N (L-endian), high word di N+1 (L-endian)
 * ============================================================ */
static inline int32_t RxReg32(const PZEM6L24_t *dev, uint16_t regAddr)
{
    uint16_t lo = RxReg(dev, regAddr);
    uint16_t hi = RxReg(dev, regAddr + 1U);
    return (int32_t)(((uint32_t)hi << 16) | (uint32_t)lo);
}

/* ============================================================
 * Internal: parse power factor dari register 16-bit
 *   Reg 0x0026 (little-endian): byte[0] = L = fase B, byte[1] = H = fase A
 *   Reg 0x0027 (little-endian): byte[0] = L = combined, byte[1] = H = fase C
 * ============================================================ */
static inline float ParsePF(uint16_t regVal, bool hiByteIsTarget)
{
    uint8_t raw = hiByteIsTarget ? (uint8_t)((regVal >> 8) & 0xFFU) : (uint8_t)( regVal       & 0xFFU);
    return raw * PZEM_POWER_FACTOR_RESOLUTION;
}


/* ============================================================
 * PZEM_IT_DumpRaw  — debug helper
 *   Print raw rxBuf ke UART/ITM supaya bisa konfirmasi endian.
 *   Panggil sementara waktu dari main loop saat state COMPLETE,
 *   sebelum ProcessData(). Hapus setelah nilai sudah benar.
 *
 *   Contoh pemakaian di main.c:
 *     if (pzem.state == PZEM_STATE_COMPLETE) {
 *         PZEM_IT_DumpRaw(&pzem);
 *         PZEM_IT_ProcessData(&pzem);
 *     }
 *
 *   Output format (via printf / ITM):
 *     RAW[00]: 01 04 50 08 98 ...
 *     Volt A raw bytes: [08][98] → little-endian=0x0898=2200 → 220.0V
 *                                  big-endian  =0x9808=38920 → 3892.0V
 * ============================================================ */
void PZEM_IT_DumpRaw(const PZEM6L24_t *dev)
{
    /* Byte mentah voltage A: offset 3 + (0x0000-0x0000)*2 = 3 */
    uint8_t vA_L = dev->rxBuf[3];   /* byte pertama di wire */
    uint8_t vA_H = dev->rxBuf[4];   /* byte kedua di wire   */
    uint16_t vA_LE = (uint16_t)(vA_L | ((uint16_t)vA_H << 8));   /* little-endian */
    uint16_t vA_BE = (uint16_t)((vA_L << 8) | vA_H);             /* big-endian    */
    (void)vA_LE; (void)vA_BE;
    /*
     * Pasang breakpoint di sini atau printf:
     *   printf("VoltA wire: [%02X][%02X]  LE=%u(%.1fV)  BE=%u(%.1fV)\n",
     *          vA_L, vA_H,
     *          vA_LE, vA_LE * 0.1f,
     *          vA_BE, vA_BE * 0.1f);
     *
     * Mana yang mendekati 220V = endian yang benar.
     */

    /* Total bytes diterima untuk verifikasi */
    (void)dev->rxExpected;  /* tambahkan ke watch: dev->rxExpected harus 85 untuk 40 reg */
}
/* ============================================================
 * PZEM_Init
 * ============================================================ */
void PZEM_Init(PZEM6L24_t *dev, UART_HandleTypeDef *huart, uint8_t addr)
{
    dev->huart         = huart;
    dev->addr          = addr;
    dev->state         = PZEM_STATE_IDLE;
    dev->faultCode     = PZEM_FAULT_NONE;
    dev->tickMs        = 0U;
    dev->rxIndex       = 0U;
    dev->rxExpected    = 0U;
    dev->expectedCount = 0U;
    dev->rxByte        = 0U;
    memset(dev->txBuf, 0, PZEM_TX_BUF_SIZE);
    memset(dev->rxBuf, 0, PZEM_RX_BUF_SIZE);
}

/* ============================================================
 * PZEM_IT_Tick  — panggil tiap 1 ms dari SysTick / TIM callback
 *
 *   Mendeteksi kondisi stuck dan membebaskan state machine:
 *     TX timeout : UART tidak selesai kirim dalam PZEM_TX_TIMEOUT_MS
 *     RX timeout : Response tidak datang dalam PZEM_RX_TIMEOUT_MS
 *     ERROR      : CRC fail / overflow -> angkat ke FAULT
 * ============================================================ */
void PZEM_IT_Tick(PZEM6L24_t *dev)
{
    switch (dev->state) {
        case PZEM_STATE_TX:
            if (++dev->tickMs > PZEM_TX_TIMEOUT_MS) {
                HAL_UART_AbortTransmit(dev->huart);
                dev->faultCode = PZEM_FAULT_TX_TIMEOUT;
                dev->state     = PZEM_STATE_FAULT;
            }
            break;

        case PZEM_STATE_RX:
            if (++dev->tickMs > PZEM_RX_TIMEOUT_MS) {
                HAL_UART_AbortReceive(dev->huart);
                dev->faultCode = PZEM_FAULT_RX_TIMEOUT;
                dev->state     = PZEM_STATE_FAULT;
            }
            break;

        case PZEM_STATE_ERROR:
            HAL_UART_AbortReceive(dev->huart);
            /* faultCode sudah di-set di RxCpltCallback */
            dev->state = PZEM_STATE_FAULT;
            break;

        default:
            dev->tickMs = 0U;
            break;
    }
}

/* ============================================================
 * PZEM_IT_RequestAll  (non-blocking)
 * ============================================================ */
HAL_StatusTypeDef PZEM_IT_RequestAll(PZEM6L24_t *dev)
{
    if (dev->state == PZEM_STATE_TX || dev->state == PZEM_STATE_RX)
        return HAL_BUSY;

    memset(dev->rxBuf, 0, PZEM_RX_BUF_SIZE);
    dev->rxIndex    = 0U;
    dev->rxExpected = 0U;
    dev->tickMs     = 0U;
    dev->faultCode  = PZEM_FAULT_NONE;

    /* Request 40 register: 0x0000 – 0x0027 */
    BuildReadFrame(dev, PZEM_REQUEST_REG_START, PZEM_REQUEST_REG_COUNT);
    dev->state = PZEM_STATE_TX;

    HAL_StatusTypeDef st = HAL_UART_Transmit_IT(dev->huart, dev->txBuf, PZEM_TX_BUF_SIZE);
    if (st != HAL_OK) {
        dev->faultCode = PZEM_FAULT_TX_TIMEOUT;
        dev->state     = PZEM_STATE_FAULT;
    }
    return st;
}

/* ============================================================
 * PZEM_IT_TxCpltCallback
 * ============================================================ */
void PZEM_IT_TxCpltCallback(PZEM6L24_t *dev)
{
    if (dev->state != PZEM_STATE_TX) return;

    dev->state  = PZEM_STATE_RX;
    dev->tickMs = 0U;   /* reset timer, mulai hitung RX timeout */

    if (HAL_UART_Receive_IT(dev->huart, &dev->rxByte, 1U) != HAL_OK) {
        dev->faultCode = PZEM_FAULT_RX_TIMEOUT;
        dev->state     = PZEM_STATE_FAULT;
    }
}

/* ============================================================
 * PZEM_IT_RxCpltCallback
 *
 *   rxBuf layout response FC04:
 *     [0]      = slave address
 *     [1]      = function code (0x04)
 *     [2]      = byte count (= count*2 = 80 untuk 40 register)
 *     [3..N-3] = data register (big-endian per register)
 *     [N-2]    = CRC low
 *     [N-1]    = CRC high
 * ============================================================ */
void PZEM_IT_RxCpltCallback(PZEM6L24_t *dev)
{
    if (dev->state != PZEM_STATE_RX) return;

    /* Simpan byte */
    if (dev->rxIndex < PZEM_RX_BUF_SIZE) {
        dev->rxBuf[dev->rxIndex++] = dev->rxByte;
    } else {
        dev->faultCode = PZEM_FAULT_CRC;
        dev->state     = PZEM_STATE_ERROR;
        return;
    }

    /* Setelah byte ke-3 (index 2 = byteCount), hitung total panjang frame */
    if (dev->rxIndex == 3U) {
        uint8_t byteCount = dev->rxBuf[2];
        dev->rxExpected   = (uint16_t)(3U + byteCount + 2U);

        if (dev->rxExpected > PZEM_RX_BUF_SIZE) {
            dev->faultCode = PZEM_FAULT_CRC;
            dev->state     = PZEM_STATE_ERROR;
            return;
        }
    }

    /* Cek kelengkapan frame */
    if (dev->rxExpected > 0U && dev->rxIndex >= dev->rxExpected) {
        uint16_t len     = dev->rxExpected;
        uint16_t calcCRC = Checksum(dev->rxBuf, len - 2U);
        uint16_t recvCRC = ((uint16_t)dev->rxBuf[len - 1U] << 8) | (uint16_t)dev->rxBuf[len - 2U];

        if (calcCRC == recvCRC) {
            dev->state = PZEM_STATE_COMPLETE;
        } else {
            dev->faultCode = PZEM_FAULT_CRC;
            dev->state     = PZEM_STATE_ERROR;
        }
        return;
    }

    /* Belum lengkap — arm ulang 1 byte */
    if (HAL_UART_Receive_IT(dev->huart, &dev->rxByte, 1U) != HAL_OK) {
        dev->faultCode = PZEM_FAULT_RX_TIMEOUT;
        dev->state     = PZEM_STATE_FAULT;
    }
}

/* ============================================================
 * PZEM_IT_ProcessData
 *   Panggil hanya saat state == PZEM_STATE_COMPLETE.
 * ============================================================ */
HAL_StatusTypeDef PZEM_IT_ProcessData(PZEM6L24_t *dev)
{
    if (dev->state != PZEM_STATE_COMPLETE) return HAL_ERROR;

    /* ---- Voltage 0x0000 - 0x0002, 0.1 V ---- */
    for (uint8_t i = 0U; i < 3U; i++)
        dev->voltage[i] = RxReg(dev, PZEM_VOLTAGE_REG + i) * PZEM_VOLTAGE_RESOLUTION;

    /* ---- Current 0x0003 - 0x0005, 0.01 A ---- */
    for (uint8_t i = 0U; i < 3U; i++)
        dev->current[i] = RxReg(dev, PZEM_CURRENT_REG + i) * PZEM_CURRENT_RESOLUTION;

    /* ---- Frequency 0x0006 - 0x0008, 0.01 Hz ---- */
    for (uint8_t i = 0U; i < 3U; i++)
        dev->frequency[i] = RxReg(dev, PZEM_FREQUENCY_REG + i) * PZEM_FREQUENCY_RESOLUTION;

    /* ---- Voltage Phase Angle V-V (0x0009 = Phase S, 0x000A = Phase T), R = 0 deg fixed ---- */
    dev->theta[0] = 0.0f;
    dev->theta[1] = RxReg(dev, PZEM_VOLTAGE_PHASE_REG + 0U) * PZEM_PHASE_RESOLUTION;
    dev->theta[2] = RxReg(dev, PZEM_VOLTAGE_PHASE_REG + 1U) * PZEM_PHASE_RESOLUTION;

    /* ---- Current Phase Angle 0x000B - 0x000D ---- */
    /* phi[0] (A/R): PZEM mengukur relatif terhadap Voltage Phase R → konversi 360-angle */
    {
        float cpa0 = RxReg(dev, PZEM_CURRENT_PHASE_REG + 0U) * PZEM_PHASE_RESOLUTION;
        dev->phi[0] = (cpa0 != 0.0f) ? (360.0f - cpa0) : 0.0f;
    }
    for (uint8_t i = 1U; i < 3U; i++) {
        float cpa = RxReg(dev, PZEM_CURRENT_PHASE_REG + i) * PZEM_PHASE_RESOLUTION;
        dev->phi[i] = dev->theta[i] - cpa;
    }

    /* ---- Active Power 32-bit signed (0x000E/10/12), 0.1 W ---- */
    for (uint8_t i = 0U; i < 3U; i++) {
        float raw = RxReg32(dev, PZEM_ACTIVE_POWER_REG + (i * 2U)) * PZEM_POWER_RESOLUTION;
        dev->power[i] = (fabsf(raw) < 2.0f) ? 0.0f : raw;
    }

    /* ---- Reactive Power 32-bit signed (0x0014/16/18), 0.1 VAR ---- */
    for (uint8_t i = 0U; i < 3U; i++) {
        float raw = RxReg32(dev, PZEM_REACTIVE_POWER_REG + (i * 2U)) * PZEM_POWER_RESOLUTION;
        dev->var[i] = (fabsf(raw) < 2.0f) ? 0.0f : raw;
    }

    /* ---- Apparent Power 32-bit signed (0x001A/1C/1E), 0.1 VA ---- */
    for (uint8_t i = 0U; i < 3U; i++)
        dev->va[i] = RxReg32(dev, PZEM_APPARENT_POWER_REG + (i * 2U)) * PZEM_POWER_RESOLUTION;

    /* ---- Power Factor ---- */
    /*
     * Wire format reg 0x0026: [A_byte][B_byte]
     * Setelah RxReg() little-endian swap: uint16 = (B_byte<<8) | A_byte
     *   pf[0] A = low byte  hasil RxReg (hiByteIsTarget=false)
     *   pf[1] B = high byte hasil RxReg (hiByteIsTarget=true)
     *
     * Wire format reg 0x0027: [C_byte][x_byte]
     * Setelah swap: uint16 = (x_byte<<8) | C_byte
     *   pf[2] C = low byte hasil RxReg (hiByteIsTarget=false)
     */
    {
        uint16_t pfAB = RxReg(dev, PZEM_POWER_FACTOR_A_B_REG);
        uint16_t pfC  = RxReg(dev, PZEM_POWER_FACTOR_C_COMBINED_REG);
        dev->pf[0] = roundf(ParsePF(pfAB, false) * 100.0f) / 100.0f;  /* A = lo byte */
        dev->pf[1] = roundf(ParsePF(pfAB, true)  * 100.0f) / 100.0f;  /* B = hi byte */
        dev->pf[2] = roundf(ParsePF(pfC,  false) * 100.0f) / 100.0f;  /* C = lo byte */
    }

    dev->state = PZEM_STATE_IDLE;
    return HAL_OK;
}