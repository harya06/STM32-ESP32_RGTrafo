/*
 * PZEM_6L24.h
 *
 *  Created on: May 22, 2026
 *      Author: Harya Susanta
 *
 *  Mode: IT (Interrupt)
 */

#ifndef INC_PZEM_6L24_H_
#define INC_PZEM_6L24_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* --------------------------------------------------------------------------
 * Input register addresses (absolute, dari datasheet PZEM-6L24)
 * -------------------------------------------------------------------------- */
#define PZEM_VOLTAGE_REG                  0x0000U   /* 3 reg: A,B,C           */
#define PZEM_CURRENT_REG                  0x0003U   /* 3 reg: A,B,C           */
#define PZEM_FREQUENCY_REG                0x0006U   /* 3 reg: A,B,C           */
#define PZEM_VOLTAGE_PHASE_REG            0x0009U   /* 2 reg: S-R, T-R angle  */
#define PZEM_CURRENT_PHASE_REG            0x000BU   /* 3 reg: A,B,C           */
#define PZEM_ACTIVE_POWER_REG             0x000EU   /* 6 reg: A(2),B(2),C(2)  */
#define PZEM_REACTIVE_POWER_REG           0x0014U   /* 6 reg: A(2),B(2),C(2)  */
#define PZEM_APPARENT_POWER_REG           0x001AU   /* 6 reg: A(2),B(2),C(2)  */
#define PZEM_POWER_FACTOR_A_B_REG         0x0026U   /* 1 reg: A=hi, B=lo      */
#define PZEM_POWER_FACTOR_C_COMBINED_REG  0x0027U   /* 1 reg: C=hi            */

/* Request span: 0x0000 – 0x0027 = 40 register */
#define PZEM_REQUEST_REG_START  0x0000U
#define PZEM_REQUEST_REG_COUNT  40U

/* --------------------------------------------------------------------------
 * Resolution factors
 * -------------------------------------------------------------------------- */
#define PZEM_VOLTAGE_RESOLUTION      0.1f
#define PZEM_CURRENT_RESOLUTION      0.01f
#define PZEM_FREQUENCY_RESOLUTION    0.01f
#define PZEM_POWER_RESOLUTION        0.1f
#define PZEM_POWER_FACTOR_RESOLUTION 0.01f
#define PZEM_PHASE_RESOLUTION        0.01f

/* --------------------------------------------------------------------------
 * Buffer sizes
 * Response 40 reg = 1+1+1 + 80 + 2 = 85 byte  →  pakai 90 untuk safety
 * -------------------------------------------------------------------------- */
#define PZEM_TX_BUF_SIZE   8U
#define PZEM_RX_BUF_SIZE   90U

/* --------------------------------------------------------------------------
 * Timeout (ms) - digunakan oleh PZEM_IT_Tick()
 * -------------------------------------------------------------------------- */
#define PZEM_TX_TIMEOUT_MS   50U
#define PZEM_RX_TIMEOUT_MS  300U

/* --------------------------------------------------------------------------
 * State machine
 * -------------------------------------------------------------------------- */
typedef enum {
    PZEM_STATE_IDLE     = 0,
    PZEM_STATE_TX,          /* Sedang kirim request via IT              */
    PZEM_STATE_RX,          /* Sedang terima byte via IT                */
    PZEM_STATE_COMPLETE,    /* Frame lengkap & CRC valid                */
    PZEM_STATE_ERROR,       /* CRC gagal / overflow — handled by Tick() */
    PZEM_STATE_FAULT,       /* Terminal: UART tidak respond / timeout   */
} PZEM_State_t;

/* --------------------------------------------------------------------------
 * Fault code (untuk logging / alarm)
 * -------------------------------------------------------------------------- */
typedef enum {
    PZEM_FAULT_NONE       = 0,
    PZEM_FAULT_TX_TIMEOUT,   /* TX tidak selesai dalam PZEM_TX_TIMEOUT_MS */
    PZEM_FAULT_RX_TIMEOUT,   /* Response tidak datang dalam PZEM_RX_TIMEOUT_MS */
    PZEM_FAULT_CRC,          /* CRC mismatch atau buffer overflow */
} PZEM_FaultCode_t;

/* --------------------------------------------------------------------------
 * Device struct
 * -------------------------------------------------------------------------- */
typedef struct {
    UART_HandleTypeDef *huart;
    uint8_t             addr;

    /* Buffers */
    uint8_t  txBuf[PZEM_TX_BUF_SIZE];
    uint8_t  rxBuf[PZEM_RX_BUF_SIZE];

    /* IT RX state */
    uint8_t  rxByte;        /* staging 1 byte untuk HAL_UART_Receive_IT */
    uint16_t rxIndex;       /* posisi tulis berikutnya di rxBuf          */
    uint16_t rxExpected;    /* total byte yang dinantikan                */
    uint16_t expectedCount; /* jumlah register yang di-request           */

    /* State & fault */
    volatile PZEM_State_t     state;
    volatile PZEM_FaultCode_t faultCode;
    uint32_t                  tickMs;   /* counter ms untuk timeout detection */

    /* Hasil pengukuran */
    float voltage[3];    /* V   */
    float current[3];    /* A   */
    float frequency[3];  /* Hz  */
    float theta[3];      /* deg     - sudut fase V-V (R=0, S, T) */
    float phi[3];        /* deg     - sudut fase V-I per fasa     */
    float power[3];      /* W       - active power                */
    float var[3];        /* VAR     - reactive power              */
    float va[3];         /* VA      - apparent power              */
    float pf[3];         /* cos phi - power factor                */
} PZEM6L24_t;

/* --------------------------------------------------------------------------
 * API — IT mode
 * -------------------------------------------------------------------------- */

/* Inisialisasi device */
void              PZEM_Init              (PZEM6L24_t *dev, UART_HandleTypeDef *huart, uint8_t addr);

/* Panggil tiap 1 ms (dari SysTick callback atau timer ISR) */
void              PZEM_IT_Tick           (PZEM6L24_t *dev);

/* Kirim request, non-blocking */
HAL_StatusTypeDef PZEM_IT_RequestAll     (PZEM6L24_t *dev);

/* Dipanggil dari HAL_UART_TxCpltCallback() di main.c */
void              PZEM_IT_TxCpltCallback (PZEM6L24_t *dev);

/* Dipanggil dari HAL_UART_RxCpltCallback() di main.c */
void              PZEM_IT_RxCpltCallback (PZEM6L24_t *dev);

/* Parse rxBuf -> struct (panggil hanya saat state == PZEM_STATE_COMPLETE) */
HAL_StatusTypeDef PZEM_IT_ProcessData    (PZEM6L24_t *dev);

/* Debug: dump raw bytes — hapus setelah endian dikonfirmasi */
void              PZEM_IT_DumpRaw        (const PZEM6L24_t *dev);

HAL_StatusTypeDef PZEM_IT_Ping(PZEM6L24_t *dev);

HAL_StatusTypeDef PZEM_IT_CheckStatus(PZEM6L24_t *dev);

#endif /* INC_PZEM_6L24_H_ */
