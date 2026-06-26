/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "PZEM_6L24.h"
#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* Struktur data 1 fasa */
typedef struct {
    float voltage;
    float current;
    float power;
    float frequency;
    float pf;
    float va;
    float var;
    float phi;
    float theta;
} PhaseData_t;

/* Struktur data listrik 3 fasa */
typedef struct {
    PhaseData_t phase[3];   /* indeks: 0=A, 1=B, 2=C */
    uint16_t    faultCode;
} Electrical_t;

/* Struktur data thermal MLX90640 */
typedef struct {
    float    Ta;            /* suhu ambient/sensor */
    float    minTemp;
    float    maxTemp;
    float    avgTemp;
    float    centerTemp;    /* pixel tengah */
    uint16_t txBuffer[768];
    uint8_t  initialized;
    uint8_t  valid;
} ThermalData_t;

/* Struktur data RTC DS3231 */
typedef struct {
    uint8_t  second;
    uint8_t  minute;
    uint8_t  hour;
    uint8_t  day;           /* 1-7 */
    uint8_t  date;          /* 1-31 */
    uint8_t  month;         /* 1-12 */
    uint16_t year;          /* contoh: 2026 */
} RTC_DS3231_t;

/* SD Card Object */
typedef struct
{
    FATFS   fs;
    FIL     file;
    FRESULT result;
    UINT    bw;
    uint8_t mounted;

    uint8_t (*mount)     (void *self);
    uint8_t (*createFile)(void *self, const char *filename);
    uint8_t (*append)    (void *self, const char *filename, const char *text);
} SDCard_Class;

/* MLX90640 State Machine */
typedef enum {
    MLX_STATE_IDLE = 0,
    MLX_STATE_READING,
    MLX_STATE_ERROR,
    MLX_STATE_RECOVERING
} MLX_State_t;

/* MLX Debug + State Management */
typedef struct {
    uint32_t    frame_success_count;
    uint32_t    frame_fail_count;
    uint32_t    consecutive_errors;    /* Counter error berturut-turut */
    uint32_t    last_success_tick;     /* Kapan terakhir berhasil baca */
    uint32_t    recovery_start_tick;   /* Waktu mulai recovery */
    uint32_t    next_retry_tick;       /* Waktu next retry saat error */
    MLX_State_t state;                 /* State machine */
    uint8_t     initialized;           /* 1 = pernah sukses init */
} MLX_Debug_t;

typedef struct {
    float    rms;
    float    peak;
    float    band;
    float	 dB;
    uint32_t last_update_tick;   /* HAL_GetTick() saat terakhir data valid diterima */
    uint8_t  valid;              /* 1 = data segar (< MIC_TIMEOUT_MS), 0 = stale/no data */
} MicData_t;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* ── Protokol TLV+CRC16 untuk frame gateway STM32 -> ESP32-TFT ──
 * Menggantikan CSV ASCII (Build_ESP1_Message) dan frame thermal
 * biner lama (THERMAL_SYNC0/SYNC1 + checksum XOR 1-byte).
 * SYNC1=0xAA, SYNC2=0x99 — unik, beda dari mic (0xAA/0x55) dan
 * thermal lama (0xAA/0xBB) yang sudah tidak dipakai lagi di huart1.
 */
#define GW_TLV_SYNC1            0xAAU
#define GW_TLV_SYNC2            0x99U

#define GW_TLV_TYPE_TELEMETRY   0x20U
#define GW_TLV_TYPE_THERMAL1    0x30U   /* MLX1 */
#define GW_TLV_TYPE_THERMAL2    0x31U   /* MLX2 */

/* TAG telemetry */
#define GW_TAG_DATETIME   0x1F   /* 7 byte raw: date,month,yearLo,yearHi,hour,min,sec */

#define GW_TAG_VA   0x01
#define GW_TAG_IA   0x02
#define GW_TAG_PA   0x03
#define GW_TAG_FA   0x04
#define GW_TAG_PFA  0x05
#define GW_TAG_VAA  0x06
#define GW_TAG_VARA 0x07
#define GW_TAG_PHIA 0x08
#define GW_TAG_THTA 0x09

#define GW_TAG_VB   0x0A
#define GW_TAG_IB   0x0B
#define GW_TAG_PB   0x0C
#define GW_TAG_FB   0x0D
#define GW_TAG_PFB  0x0E
#define GW_TAG_VAB  0x0F
#define GW_TAG_VARB 0x10
#define GW_TAG_PHIB 0x11
#define GW_TAG_THTB 0x12

#define GW_TAG_VC   0x13
#define GW_TAG_IC   0x14
#define GW_TAG_PC   0x15
#define GW_TAG_FC   0x16
#define GW_TAG_PFC  0x17
#define GW_TAG_VAC  0x18
#define GW_TAG_VARC 0x19
#define GW_TAG_PHIC 0x1A
#define GW_TAG_THTC 0x1B

#define GW_TAG_TMAX   0x1C
#define GW_TAG_TMAX2  0x1D

#define GW_TAG_MIC_RMS    0x40
#define GW_TAG_MIC_BAND   0x41
#define GW_TAG_MIC_DB     0x42
#define GW_TAG_MIC_VALID  0x43


#define GW_TAG_THERMAL_GRID  0x50 /* TAG thermal (TYPE 0x30/0x31) */
#define GW_THERMAL_TLV_VALLEN  (768U * 2U)  /* 1536 byte, 2-byte LEN karena >255 */
#define GW_TX_BUF_LEN   1600U

#define MIC_TLV_SYNC1            0xAAU
#define MIC_TLV_SYNC2            0x55U
#define MIC_TLV_TYPE_DATA        0x10U

#define MIC_TLV_TAG_RMS          0x01U
#define MIC_TLV_TAG_PEAK         0x02U
#define MIC_TLV_TAG_BAND         0x03U
#define MIC_TLV_TAG_DB           0x04U

#define MIC_TLV_PAYLOAD_LEN      24U   /* 4 TLV x (TAG+LEN+4 byte float) */
#define MIC_TLV_FRAME_LEN        (2U + 1U + 2U + MIC_TLV_PAYLOAD_LEN + 2U)  /* = 31 (SYNC2+TYPE1+LEN2+PAYLOAD24+CRC2) */

#define VOLT_NOMINAL     220.0f
#define LTOL             0.10f
#define HTOL             0.05f
#define PZEM_CONN_RETRY  3U

#define DS3231_ADDRESS   (0x68 << 1)

#define MLX90640_I2C_ADDR            0x33   /* MLX1: BAB → I2C1 */
#define MLX90640_I2C_ADDR2           0x33   /* MLX2: BAA → I2C2 (addr sama, bus beda) */
#define MLX_MAX_CONSECUTIVE_ERRORS   5
#define MLX_RECOVERY_TIMEOUT_MS      10000UL
#define MLX_RETRY_INTERVAL_MS        2000UL
#define MLX_BLINK_ERROR_COUNT        3

#define PZEM_SLAVE_ADDRESS           0x01
#define PZEM_REQUEST_INTERVAL        500UL	  /* Request tiap 500ms */
#define PZEM_TIMEOUT_MS				 2000UL	  /* Timeout 2 detik */

#define MIC_UART                     huart6
#define MIC_RX_BUF_LEN               64
#define MIC_TIMEOUT_MS               3000UL   /* Data stale setelah 3 detik */
#define MIC_RESTART_INTERVAL_MS      2000UL   /* Restart UART IT tiap 2 detik jika mati */

#define SD_LOG_INTERVAL_MS           5000UL   /* Log ke SD tiap 5 detik */
#define ESP1_TELEMETRY_INTERVAL_MS   1000UL   /* Kirim data ke ESP1 tiap 1 detik */
#define ESP1_THERMAL_INTERVAL_MS     10000UL  /* Kirim frame thermal biner tiap 10 detik */

/* ── Protokol frame biner thermal (terpisah dari CSV telemetry) ──
 * Dipisah dari Build_ESP1_Message supaya tidak overflow txBuffer
 * dan tidak membebani siklus 1 Hz. Dikirim di kanal UART yang sama
 * (huart1/RS485_1), dibedakan lewat 2 byte sync yang mustahil muncul
 * di stream CSV (CSV selalu printable ASCII 0x20-0x7E).
 */
#define THERMAL_SYNC0                0xAAU
#define THERMAL_SYNC1                0xBBU
#define THERMAL_FRAME_TYPE           0x01U   /* MLX1 (BAB, I2C1) */
#define THERMAL_FRAME_TYPE2          0x02U   /* MLX2 (BAA, I2C2) */
#define THERMAL_PAYLOAD_LEN          (768U * 2U)   /* 768 piksel x 2 byte (uint16) = 1536 byte */

/* Fault codes */
#define FAULT_NORMAL                 0
#define FAULT_UNDERVOLT_A            1
#define FAULT_UNDERVOLT_B            2
#define FAULT_UNDERVOLT_C            3
#define FAULT_OVERVOLT_A             4
#define FAULT_OVERVOLT_B             5
#define FAULT_OVERVOLT_C             6
#define FAULT_PZEM_LOST              404
#define FAULT_MLX_CRITICAL           500

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* State machine penerima TLV mic
 *
 * PENTING: ESP32-INMP441 (lihat buildTLVPacket() di firmware mic)
 * menulis LENGTH sebagai 2 byte little-endian:
 *     out[idx++] = (uint8_t)(TLV_PAYLOAD_LEN & 0xFF);         // LEN_L
 *     out[idx++] = (uint8_t)((TLV_PAYLOAD_LEN >> 8) & 0xFF);  // LEN_H
 * dan CRC dihitung atas TYPE + LEN(2 byte) + PAYLOAD(24 byte) = 27 byte.
 * Total frame = 2(sync)+1(type)+2(len)+24(payload)+2(crc) = 31 byte.
 *
 * Versi sebelumnya di sini sempat disederhanakan jadi SATU state
 * MIC_WAIT_LEN (1 byte) -- ini SALAH dan menyebabkan seluruh payload
 * + CRC bergeser 1 byte (byte LEN_H=0x00 ikut terbaca sbg payload),
 * sehingga CRC selalu mismatch walau sync & data byte-nya benar.
 * Dikembalikan ke 2 state terpisah (LEN_L lalu LEN_H) agar cocok
 * dengan apa yang benar2 dikirim ESP-INMP. */
typedef enum {
    MIC_WAIT_SYNC1 = 0,
    MIC_WAIT_SYNC2,
    MIC_WAIT_TYPE,
    MIC_WAIT_LEN_L,
    MIC_WAIT_LEN_H,
    MIC_WAIT_PAYLOAD,
    MIC_WAIT_CRC_L,
    MIC_WAIT_CRC_H
} MicTLVState_t;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

IWDG_HandleTypeDef hiwdg;

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

/* ----- MLX90640 (sensor 1: BAB, I2C1) ----- */
paramsMLX90640 mlx90640;
uint16_t       eeMLX90640[832];
uint16_t       mlx90640Frame[834];
float          mlx90640To[768];

/* ----- MLX90640 (sensor 2: BAA, I2C2) ----- */
paramsMLX90640 mlx90640_2;
uint16_t       eeMLX90640_2[832];
uint16_t       mlx90640Frame2[834];
float          mlx90640To2[768];

/* ----- Data terstruktur ----- */
Electrical_t  elecData;
ThermalData_t thermData;
ThermalData_t thermData2;
MLX_Debug_t   mlx_debug;
MLX_Debug_t   mlx_debug2;

/* ----- PZEM ----- */
PZEM6L24_t pzem;
static uint8_t connFailCount = 0U;

/* ----- RTC DS3231 ----- */
volatile RTC_DS3231_t rtcData;

/* ----- SD Card ----- */
SDCard_Class sd;

static volatile uint32_t mlx_consecutive_errors = 0;
static volatile uint32_t pzem_last_request = 0;
uint32_t last_pzem_update_tick = 0;

static uint8_t gw_tx_buf[GW_TX_BUF_LEN];

/* ── Counter debug GW frame (RS485_1 / gateway -> TFT) ──
 * Sengaja GLOBAL (bukan static) dan volatile, supaya bisa ditambahkan
 * ke "Live Expressions" / Watch window STM32CubeIDE saat debugging
 * tanpa perlu UART debug tambahan (huart1/2/6 semua sudah terpakai).
 * Naik terus selama HAL_UART_Transmit di GW_SendFrame() benar2
 * dipanggil & return -- kalau nilainya diam di 0 padahal program
 * sudah lama jalan, berarti GW_SendTelemetry/GW_SendThermalFrame
 * tidak pernah tercapai (macet sebelum sampai situ), BUKAN soal
 * transceiver/wiring.
 */
volatile uint32_t dbg_gwFrameSentCount = 0;
volatile uint32_t dbg_gwLastFrameLen   = 0;
volatile uint8_t  dbg_gwLastFrameType  = 0;

/* Buffer raw 1-byte untuk HAL_UARTEx_ReceiveToIdle_IT — tetap dipakai
 * karena metode penerimaan UART tidak diubah, hanya isi/interpretasinya. */
static volatile uint8_t  mic_rx_raw[MIC_RX_BUF_LEN];
static volatile uint8_t  mic_event_buf[MIC_RX_BUF_LEN];
static volatile uint16_t mic_event_len = 0;
static volatile uint8_t  mic_frame_ready = 0;    /* flag: ada event baru dari ISR, perlu di-feed ke parser */
static volatile uint8_t  mic_rx_active = 0;
static uint32_t mic_check_last = 0;
static volatile uint32_t mic_rx_event_count = 0;

/* DEBUG: pantau via Live Expressions bersama mic_rx_event_count.
 * Membedakan dua skenario:
 *  - mic_rx_event_count = 0          -> tidak ada byte fisik masuk sama
 *                                       sekali (wiring/baud/DIR pin).
 *  - mic_rx_event_count > 0 tapi
 *    dbg_micFrameOkCount = 0         -> byte masuk, tapi gagal sync atau
 *                                       CRC mismatch terus (cek SYNC byte
 *                                       & representasi float ESP-INMP vs
 *                                       STM32, atau noise bus).
 */
volatile uint32_t dbg_micFrameOkCount  = 0;
volatile uint32_t dbg_micCrcFailCount  = 0;
volatile uint16_t dbg_micLastEventSize = 0;   /* DEBUG: nilai Size terakhir dari ISR RxEvent */
volatile uint32_t dbg_micEventRejected = 0;   /* DEBUG: berapa kali Size di luar rentang valid (event diabaikan) */
volatile uint32_t dbg_micProcessCount  = 0;   /* DEBUG: berapa kali MIC_TLV_ProcessEventBuffer benar2 dipanggil dari main loop */
volatile uint8_t  dbg_micRawByte0 = 0;        /* DEBUG: byte pertama mentah event terakhir, mestinya 0xAA */
volatile uint8_t  dbg_micRawByte1 = 0;        /* DEBUG: byte kedua mentah event terakhir, mestinya 0x55 */
volatile uint8_t  dbg_micRawByte2 = 0;
volatile uint8_t  dbg_micRawByte3 = 0;

/* DEBUG tambahan: membedakan "HAL pikir receive aktif" vs "ISR benar2
 * jalan". Kalau mic_rx_event_count tetap 0 TAPI dbg_micHalStartOkCount
 * naik (artinya HAL_UARTEx_ReceiveToIdle_IT berulang kali return HAL_OK),
 * itu mengarah ke NVIC interrupt USART6 TIDAK ENABLE di level hardware
 * (HAL_NVIC_EnableIRQ belum dipanggil utk USART6_IRQn, biasanya di
 * stm32f4xx_hal_msp.c -- file yg tidak diupload), BUKAN soal protokol/HAL
 * state. Kalau dbg_micHalStartFailCount yang naik, artinya HAL_UART
 * sendiri menolak start receive (state UART tidak READY, dsb). */
volatile uint32_t dbg_micHalStartOkCount   = 0;
volatile uint32_t dbg_micHalStartFailCount = 0;
volatile uint8_t  dbg_micRxActiveNow       = 0;  /* snapshot mic_rx_active terakhir */

/* State machine TLV mic (dijalankan di main loop, bukan di ISR) */
static MicTLVState_t mic_tlv_state = MIC_WAIT_SYNC1;
static uint8_t        mic_tlv_payload[MIC_TLV_PAYLOAD_LEN];
static uint16_t       mic_tlv_payload_idx = 0;
static uint16_t       mic_tlv_len_field = 0;
static uint8_t        mic_tlv_type = 0;
static uint16_t       mic_tlv_crc_calc = 0;
static uint16_t       mic_tlv_crc_recv = 0;

MicData_t micData = {
    .rms              = 0.0f,
    .peak             = 0.0f,
    .band             = 0.0f,
	.dB				  = 0.0f,
    .last_update_tick = 0,
    .valid            = 0
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */

static void MLX_Init(void);
static void MLX_Init2(void);
static void MLX_Process(void);
static void MLX_Process2(void);
static void MLX_HandleError(void);
static void MLX_HandleError2(void);
static void MLX_RecoveryAttempt(void);
static void MLX_RecoveryAttempt2(void);

static uint16_t CheckVoltageFault(float vA, float vB, float vC);
static void     UpdateFaultCode(uint16_t code);
static uint8_t BCD_To_Dec(uint8_t val);
static uint8_t Dec_To_BCD(uint8_t val);
static HAL_StatusTypeDef DS3231_GetTime(volatile RTC_DS3231_t *rtc);
static HAL_StatusTypeDef DS3231_SetTime(RTC_DS3231_t *rtc);

static uint8_t SD_mount(void *self);
static uint8_t SD_createFile(void *self, const char *filename);
static uint8_t SD_append(void *self, const char *filename, const char *text);

void ESP1_Send(char *msg);
static uint16_t GW_TLV_WriteFloat(uint8_t *buf, uint8_t tag, float val);
static uint16_t GW_TLV_WriteRaw(uint8_t *buf, uint8_t tag, const uint8_t *data, uint8_t len);
static uint16_t GW_BuildFrame(uint8_t *out, uint8_t type, const uint8_t *payload, uint16_t payloadLen);
static void     GW_SendFrame(uint8_t *frame, uint16_t frameLen);

void GW_SendTelemetry(void);
void GW_SendThermalFrame(void);

/* ─── Fungsi mikrofon ─── */
static void MIC_StartReceive(void);
static void MIC_UpdateValidity(MicData_t *mic);
static uint16_t crc16_ccitt(const uint8_t *data, uint16_t len);
static void     MIC_TLV_Reset(void);
static uint8_t  MIC_TLV_FeedByte(uint8_t b, MicData_t *out);
static void     MIC_TLV_ProcessEventBuffer(MicData_t *out);

static void PZEM_HardReset(PZEM6L24_t *dev);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void RS485_1_TX(void)
{
    HAL_GPIO_WritePin(RS485_DIR_ESP1_GPIO_Port,
                      RS485_DIR_ESP1_Pin,
                      GPIO_PIN_SET);
    for (volatile int i = 0; i < 50; i++);
}

void RS485_1_RX(void)
{
    HAL_GPIO_WritePin(RS485_DIR_ESP1_GPIO_Port,
                      RS485_DIR_ESP1_Pin,
                      GPIO_PIN_RESET);
}

void RS485_2_TX(void)
{
    HAL_GPIO_WritePin(RS485_DIR_ESP2_GPIO_Port,
                      RS485_DIR_ESP2_Pin,
                      GPIO_PIN_SET);
}

void RS485_2_RX(void)
{
    HAL_GPIO_WritePin(RS485_DIR_ESP2_GPIO_Port,
                      RS485_DIR_ESP2_Pin,
                      GPIO_PIN_RESET);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_SPI2_Init();
  MX_FATFS_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */

  /* Inisialisasi PZEM */
  PZEM_Init(&pzem, &huart2, PZEM_SLAVE_ADDRESS);

  /* ═══════════════════════════════════════════════
   *  INISIALISASI MLX90640 - NON-BLOCKING
   * ═══════════════════════════════════════════════ */
  memset(&mlx_debug, 0, sizeof(mlx_debug));
  memset(&thermData, 0, sizeof(thermData));
  memset(&mlx_debug2, 0, sizeof(mlx_debug2));
  memset(&thermData2, 0, sizeof(thermData2));

  HAL_Delay(100);

  MLX_Init();   /* MLX1: BAB, I2C1 */
  MLX_Init2();  /* MLX2: BAA, I2C2 */

  /* Inisialisasi struktur data */
  memset(&elecData, 0, sizeof(elecData));

  /* DS3231 RTC */
  rtcData.second = 00;
  rtcData.minute = 36;
  rtcData.hour   = 19;
  rtcData.day    = 6; /*Minggu*/
  rtcData.date   = 26;
  rtcData.month  = 6;
  rtcData.year   = 2026;
//  DS3231_SetTime((RTC_DS3231_t*)&rtcData);  /* Uncomment untuk set waktu */

  /* SD Card */
  sd.mount = SD_mount;
  sd.createFile = SD_createFile;
  sd.append = SD_append;
  if (sd.mount(&sd)) {
      sd.createFile(&sd, "LOG.CSV");
  }

  memset(mic_rx_raw, 0, MIC_RX_BUF_LEN);
  __HAL_UART_FLUSH_DRREGISTER(&MIC_UART);
  RS485_2_RX();
  MIC_StartReceive();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      /* WATCHDOG REFRESH */
      HAL_IWDG_Refresh(&hiwdg);

//      DS3231_GetTime(&rtcData);
      if (HAL_I2C_GetState(&hi2c3) != HAL_I2C_STATE_READY) {
    	  HAL_I2C_DeInit(&hi2c3);
          MX_I2C3_Init(); // Re-inisialisasi
          DS3231_GetTime((RTC_DS3231_t*)&rtcData);
      } else {
          DS3231_GetTime((RTC_DS3231_t*)&rtcData);
      }

      if (HAL_GetTick() - mic_check_last > MIC_RESTART_INTERVAL_MS) {
          mic_check_last = HAL_GetTick();
          if (!mic_rx_active || (HAL_GetTick() - micData.last_update_tick > MIC_TIMEOUT_MS * 2)) {
              __HAL_UART_FLUSH_DRREGISTER(&MIC_UART);
              memset(mic_rx_raw, 0, MIC_RX_BUF_LEN);
              MIC_StartReceive();
          }
      }

      /* Update validitas microphone */
      MIC_UpdateValidity(&micData);

      /* Proses event mic jika ada (TLV+CRC16) */
      if (mic_frame_ready)
      {
           dbg_micProcessCount++;   /* DEBUG: konfirmasi baris ini tereksekusi */
           MIC_TLV_ProcessEventBuffer(&micData);
           mic_frame_ready = 0;
           MIC_StartReceive();  /* Restart untuk frame berikutnya */
      }

      PZEM_IT_Tick(&pzem);

      if (HAL_GetTick() - pzem_last_request > 3000U) {
              if (pzem.state != PZEM_STATE_IDLE) {
                  PZEM_HardReset(&pzem);
              }

              memset(elecData.phase, 0, sizeof(elecData.phase));

              if (HAL_GetTick() - pzem_last_request > 5000U) {
                   PZEM_IT_RequestAll(&pzem);
                   pzem_last_request = HAL_GetTick();
              }
          }
          else {
              if (pzem.state == PZEM_STATE_IDLE && (HAL_GetTick() - pzem_last_request >= PZEM_REQUEST_INTERVAL)) {
                  PZEM_IT_RequestAll(&pzem);
                  pzem_last_request = HAL_GetTick();
              }
          }

      if (pzem.state == PZEM_STATE_FAULT || pzem.state == PZEM_STATE_ERROR) {
    	  for (int i = 0; i < 3; i++) {
    		  elecData.phase[i].voltage   = 0;
    		  elecData.phase[i].current   = 0;
    		  elecData.phase[i].power     = 0;
    		  elecData.phase[i].frequency = 0;
    		  elecData.phase[i].pf        = 0;
    		  elecData.phase[i].va        = 0;
    		  elecData.phase[i].var       = 0;
    		  elecData.phase[i].phi       = 0;
    		  elecData.phase[i].theta     = 0;
    	  }

    	  HAL_UART_Abort(pzem.huart);
    	  pzem.state = PZEM_STATE_IDLE;
      }

      if (pzem.state == PZEM_STATE_COMPLETE) {
    	  if (PZEM_IT_ProcessData(&pzem) == HAL_OK) {
    		  for (int i = 0; i < 3; i++) {
    			  elecData.phase[i].voltage   = pzem.voltage[i];
    			  elecData.phase[i].current   = pzem.current[i];
    			  elecData.phase[i].power     = pzem.power[i];
    			  elecData.phase[i].frequency = pzem.frequency[i];
    			  elecData.phase[i].pf        = pzem.pf[i];
    			  elecData.phase[i].va        = pzem.va[i];
    			  elecData.phase[i].var       = pzem.var[i];
    			  elecData.phase[i].phi       = pzem.phi[i];
    			  elecData.phase[i].theta     = pzem.theta[i];
    		  }
    		  last_pzem_update_tick = HAL_GetTick(); // Catat waktu terakhir data sukses
    	  }
    	  pzem.state = PZEM_STATE_IDLE; // Balik ke IDLE untuk siklus berikutnya
      }

      if (HAL_GetTick() - last_pzem_update_tick > PZEM_TIMEOUT_MS) {
    	  memset(elecData.phase, 0, sizeof(elecData.phase));
    	  if (pzem.state == PZEM_STATE_ERROR || pzem.state == PZEM_STATE_FAULT) {
    		  pzem.state = PZEM_STATE_IDLE;
    	  }
      }

      MLX_Process();
      MLX_Process2();

      static uint32_t lastLog = 0;
      if (HAL_GetTick() - lastLog > SD_LOG_INTERVAL_MS)
      {
          lastLog = HAL_GetTick();

          char line[768];
          sprintf(line,
              "%02d/%02d/%04d,%02d:%02d:%02d,"
              "%.2f,%.3f,%.2f,%.2f,%.3f,%.2f,%.2f,%.2f,%.2f,"
              "%.2f,%.3f,%.2f,%.2f,%.3f,%.2f,%.2f,%.2f,%.2f,"
              "%.2f,%.3f,%.2f,%.2f,%.3f,%.2f,%.2f,%.2f,%.2f,"
              "%.2f,%.2f,%.2f,%.2f,%.2f,"
              "%.2f,%.2f,%.2f,%.2f,%.2f,"
              "%u,%lu,%lu,%lu,%lu,"
              "%.4f,%.4f,%.4f,%u\r\n",

              rtcData.date, rtcData.month, rtcData.year,
              rtcData.hour, rtcData.minute, rtcData.second,

              elecData.phase[0].voltage, elecData.phase[0].current,
              elecData.phase[0].power, elecData.phase[0].frequency,
              elecData.phase[0].pf, elecData.phase[0].va,
              elecData.phase[0].var, elecData.phase[0].phi,
              elecData.phase[0].theta,

              elecData.phase[1].voltage, elecData.phase[1].current,
              elecData.phase[1].power, elecData.phase[1].frequency,
              elecData.phase[1].pf, elecData.phase[1].va,
              elecData.phase[1].var, elecData.phase[1].phi,
              elecData.phase[1].theta,

              elecData.phase[2].voltage, elecData.phase[2].current,
              elecData.phase[2].power, elecData.phase[2].frequency,
              elecData.phase[2].pf, elecData.phase[2].va,
              elecData.phase[2].var, elecData.phase[2].phi,
              elecData.phase[2].theta,

              /* Thermal MLX1 */
              thermData.initialized ? thermData.avgTemp    : 0.0f,
              thermData.initialized ? thermData.maxTemp    : 0.0f,
              thermData.initialized ? thermData.minTemp    : 0.0f,
              thermData.initialized ? thermData.Ta         : 0.0f,
              thermData.initialized ? thermData.centerTemp : 0.0f,

              /* Thermal MLX2 */
              thermData2.initialized ? thermData2.avgTemp    : 0.0f,
              thermData2.initialized ? thermData2.maxTemp    : 0.0f,
              thermData2.initialized ? thermData2.minTemp    : 0.0f,
              thermData2.initialized ? thermData2.Ta         : 0.0f,
              thermData2.initialized ? thermData2.centerTemp : 0.0f,

              (unsigned int)elecData.faultCode,
              (unsigned long)mlx_debug.frame_success_count,
              (unsigned long)mlx_debug.frame_fail_count,
              (unsigned long)mlx_debug2.frame_success_count,
              (unsigned long)mlx_debug2.frame_fail_count,

              micData.rms, micData.peak, micData.band,
              (unsigned int)micData.valid
          );

          sd.append(&sd, "LOG.CSV", line);
      }

      GW_SendTelemetry();
      GW_SendThermalFrame();

      /* Small delay untuk stabilitas */
      HAL_Delay(1);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RS485_DIR_ESP1_Pin|RS485_DIR_ESP2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RS485_DIR_ESP1_Pin SPI2_CS_Pin RS485_DIR_ESP2_Pin */
  GPIO_InitStruct.Pin = RS485_DIR_ESP1_Pin|SPI2_CS_Pin|RS485_DIR_ESP2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


/**
 * @brief Inisialisasi MLX90640 dengan error handling non-blocking
 */
static void MLX_Init(void)
{
    mlx_debug.state = MLX_STATE_IDLE;
    mlx_debug.initialized = 0;
    mlx_debug.consecutive_errors = 0;
    mlx_debug.frame_success_count = 0;
    mlx_debug.frame_fail_count = 0;
    mlx_debug.last_success_tick = 0;
    mlx_debug.next_retry_tick = 0;

    thermData.initialized = 0;
    thermData.valid = 0;  /* ← INIT INVALID */

    MLX90640_I2CSelectBus(&hi2c1);  /* Sensor 1: I2C1 */

    if (MLX90640_DumpEE(MLX90640_I2C_ADDR, eeMLX90640) == 0)
    {
        if (MLX90640_ExtractParameters(eeMLX90640, &mlx90640) >= 0)
        {
            MLX90640_SetChessMode(MLX90640_I2C_ADDR);
            MLX90640_SetRefreshRate(MLX90640_I2C_ADDR, 0x03);

            mlx_debug.state = MLX_STATE_IDLE;
            mlx_debug.initialized = 1;
            mlx_debug.last_success_tick = HAL_GetTick();
            thermData.initialized = 1;
            thermData.valid = 1;  /* ← SUKSES INIT → VALID */

            for (int i = 0; i < 2; i++) {
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
                HAL_Delay(50);
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
                HAL_Delay(50);
            }
            return;
        }
    }

    /* Init gagal */
    mlx_debug.state = MLX_STATE_ERROR;
    mlx_debug.next_retry_tick = HAL_GetTick() + MLX_RETRY_INTERVAL_MS;
    thermData.initialized = 0;
    thermData.valid = 0;  /* ← GAGAL INIT → INVALID */

    for (int i = 0; i < 5; i++) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
        HAL_Delay(20);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
        HAL_Delay(20);
    }
}

/**
 * @brief Proses pembacaan MLX90640
 *        Dipanggil dari main loop
 */
static void MLX_Process(void)
{
    uint32_t now = HAL_GetTick();
    MLX90640_I2CSelectBus(&hi2c1);  /* Sensor 1: I2C1 */

    switch (mlx_debug.state)
    {
        case MLX_STATE_IDLE:
            mlx_debug.state = MLX_STATE_READING;
            break;

        case MLX_STATE_READING:
            if (MLX90640_GetFrameData(MLX90640_I2C_ADDR, mlx90640Frame) >= 0)
            {
                /* ── SUKSES ── */
                mlx_debug.frame_success_count++;
                mlx_debug.consecutive_errors = 0;
                mlx_debug.last_success_tick = now;

                float Ta = MLX90640_GetTa(mlx90640Frame, &mlx90640);
                MLX90640_CalculateTo(mlx90640Frame, &mlx90640, 0.95f, Ta, mlx90640To);

                float minT = 1000.0f, maxT = -1000.0f, sumT = 0.0f;
                for (int i = 0; i < 768; i++) {
                    float t = mlx90640To[i];
                    thermData.txBuffer[i] = (uint16_t)(mlx90640To[i] * 100);
                    sumT += t;
                    if (t < minT) minT = t;
                    if (t > maxT) maxT = t;
                }

                thermData.Ta         = Ta;
                thermData.minTemp    = minT;
                thermData.maxTemp    = maxT;
                thermData.avgTemp    = sumT / 768.0f;
                thermData.centerTemp = mlx90640To[384];
                thermData.initialized = 1;

                /* ── SET VALID FLAG ── */
                thermData.valid = 1;

                mlx_debug.state = MLX_STATE_IDLE;
            }
            else
            {
                /* ── GAGAL BACA ── */
                mlx_debug.frame_fail_count++;
                mlx_debug.consecutive_errors++;

                /* Jika gagal berturut-turut > threshold, tandai invalid */
                if (mlx_debug.consecutive_errors >= MLX_MAX_CONSECUTIVE_ERRORS)
                {
                    thermData.valid = 0;  /* ← SET INVALID */
                    mlx_debug.state = MLX_STATE_ERROR;
                    mlx_debug.next_retry_tick = now + MLX_RETRY_INTERVAL_MS;
                }
                else
                {
                    mlx_debug.state = MLX_STATE_IDLE;
                }
            }
            break;

        case MLX_STATE_ERROR:
            thermData.valid = 0;
            if (now >= mlx_debug.next_retry_tick)
            {
                MLX_HandleError();
            }
            break;

        case MLX_STATE_RECOVERING:
            /* Masih invalid selama recovery */
            thermData.valid = 0;

            if ((now - mlx_debug.recovery_start_tick) > MLX_RECOVERY_TIMEOUT_MS)
            {
                /* Recovery timeout */
                elecData.faultCode = FAULT_MLX_CRITICAL;
                mlx_debug.state = MLX_STATE_ERROR;
                mlx_debug.next_retry_tick = now + MLX_RETRY_INTERVAL_MS;

                for (int i = 0; i < 10; i++) {
                    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
                    HAL_Delay(10);
                }
            }
            else
            {
                if (now >= mlx_debug.next_retry_tick)
                {
                    MLX_RecoveryAttempt();
                }
            }
            break;
    }
}
/**
 * @brief Handle error MLX - coba reset I2C
 */
static void MLX_HandleError(void)
{
    /* Kedip LED tanda masuk recovery */
    for (int i = 0; i < MLX_BLINK_ERROR_COUNT; i++) {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        HAL_Delay(50);
    }

    /* Reset I2C */
    HAL_I2C_DeInit(&hi2c1);
    HAL_Delay(100);
    MX_I2C1_Init();
    MLX90640_I2CSelectBus(&hi2c1);
    MLX90640_I2CInit();

    mlx_debug.state = MLX_STATE_RECOVERING;
    mlx_debug.recovery_start_tick = HAL_GetTick();
    mlx_debug.next_retry_tick = HAL_GetTick() + MLX_RETRY_INTERVAL_MS;
}

/**
 * @brief Coba recovery MLX90640
 */
static void MLX_RecoveryAttempt(void)
{
    MLX90640_I2CSelectBus(&hi2c1);

    if (MLX90640_DumpEE(MLX90640_I2C_ADDR, eeMLX90640) == 0)
    {
        if (MLX90640_ExtractParameters(eeMLX90640, &mlx90640) >= 0)
        {
            MLX90640_SetChessMode(MLX90640_I2C_ADDR);
            MLX90640_SetRefreshRate(MLX90640_I2C_ADDR, 0x03);

            /* RECOVERY SUKSES */
            mlx_debug.state = MLX_STATE_IDLE;
            mlx_debug.consecutive_errors = 0;
            mlx_debug.initialized = 1;
            thermData.initialized = 1;
            thermData.valid = 1;  /* ← RESTORE VALID */

            if (elecData.faultCode == FAULT_MLX_CRITICAL)
                elecData.faultCode = FAULT_NORMAL;

            for (int i = 0; i < 3; i++) {
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
                HAL_Delay(100);
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
                HAL_Delay(100);
            }
            return;
        }
    }

    /* Gagal, tetap invalid */
    thermData.valid = 0;
    mlx_debug.next_retry_tick = HAL_GetTick() + MLX_RETRY_INTERVAL_MS;
}

/* ═══════════════════════════════════════════════
 *  MLX90640 SENSOR 2 (BAA, I2C2) — clone dari sensor 1
 * ═══════════════════════════════════════════════ */
static void MLX_Init2(void)
{
    mlx_debug2.state = MLX_STATE_IDLE;
    mlx_debug2.initialized = 0;
    mlx_debug2.consecutive_errors = 0;
    mlx_debug2.frame_success_count = 0;
    mlx_debug2.frame_fail_count = 0;
    mlx_debug2.last_success_tick = 0;
    mlx_debug2.next_retry_tick = 0;

    thermData2.initialized = 0;
    thermData2.valid = 0;

    MLX90640_I2CSelectBus(&hi2c2);  /* Pilih I2C2 */

    int dumpResult = MLX90640_DumpEE(MLX90640_I2C_ADDR2, eeMLX90640_2);
    if (dumpResult == 0)
    {
        int extractResult = MLX90640_ExtractParameters(eeMLX90640_2, &mlx90640_2);
        if (extractResult >= 0)
        {
            MLX90640_SetChessMode(MLX90640_I2C_ADDR2);
            MLX90640_SetRefreshRate(MLX90640_I2C_ADDR2, 0x03);

            mlx_debug2.state = MLX_STATE_IDLE;
            mlx_debug2.initialized = 1;
            mlx_debug2.last_success_tick = HAL_GetTick();
            thermData2.initialized = 1;
            thermData2.valid = 1;
            /* LED: 4 kedip cepat = MLX2 init OK */
            for (int i = 0; i < 4; i++) {
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); HAL_Delay(80);
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); HAL_Delay(80);
            }
            return;
        }
        /* Extract gagal: 1 kedip panjang */
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); HAL_Delay(500);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    }
    else
    {
        /* DumpEE gagal (I2C error): 3 kedip panjang — paling umum = I2C2 pin salah */
        for (int i = 0; i < 3; i++) {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); HAL_Delay(300);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); HAL_Delay(150);
        }
    }

    mlx_debug2.state = MLX_STATE_ERROR;
    mlx_debug2.next_retry_tick = HAL_GetTick() + MLX_RETRY_INTERVAL_MS;
    thermData2.initialized = 0;
    thermData2.valid = 0;
}

static void MLX_Process2(void)
{
    uint32_t now = HAL_GetTick();
    MLX90640_I2CSelectBus(&hi2c2);  /* Pastikan bus benar sebelum operasi apapun */

    switch (mlx_debug2.state)
    {
        case MLX_STATE_IDLE:
            mlx_debug2.state = MLX_STATE_READING;
            break;

        case MLX_STATE_READING:
            if (MLX90640_GetFrameData(MLX90640_I2C_ADDR2, mlx90640Frame2) >= 0)
            {
                mlx_debug2.frame_success_count++;
                mlx_debug2.consecutive_errors = 0;
                mlx_debug2.last_success_tick = now;

                float Ta = MLX90640_GetTa(mlx90640Frame2, &mlx90640_2);
                MLX90640_CalculateTo(mlx90640Frame2, &mlx90640_2, 0.95f, Ta, mlx90640To2);

                float minT = 1000.0f, maxT = -1000.0f, sumT = 0.0f;
                for (int i = 0; i < 768; i++) {
                    float t = mlx90640To2[i];
                    thermData2.txBuffer[i] = (uint16_t)(t * 100);
                    sumT += t;
                    if (t < minT) minT = t;
                    if (t > maxT) maxT = t;
                }

                thermData2.Ta         = Ta;
                thermData2.minTemp    = minT;
                thermData2.maxTemp    = maxT;
                thermData2.avgTemp    = sumT / 768.0f;
                thermData2.centerTemp = mlx90640To2[384];
                thermData2.initialized = 1;
                thermData2.valid = 1;

                mlx_debug2.state = MLX_STATE_IDLE;
            }
            else
            {
                mlx_debug2.frame_fail_count++;
                mlx_debug2.consecutive_errors++;

                if (mlx_debug2.consecutive_errors >= MLX_MAX_CONSECUTIVE_ERRORS)
                {
                    thermData2.valid = 0;
                    mlx_debug2.state = MLX_STATE_ERROR;
                    mlx_debug2.next_retry_tick = now + MLX_RETRY_INTERVAL_MS;
                }
                else
                {
                    mlx_debug2.state = MLX_STATE_IDLE;
                }
            }
            break;

        case MLX_STATE_ERROR:
            thermData2.valid = 0;
            if (now >= mlx_debug2.next_retry_tick)
                MLX_HandleError2();
            break;

        case MLX_STATE_RECOVERING:
            thermData2.valid = 0;
            if ((now - mlx_debug2.recovery_start_tick) > MLX_RECOVERY_TIMEOUT_MS)
            {
                mlx_debug2.state = MLX_STATE_ERROR;
                mlx_debug2.next_retry_tick = now + MLX_RETRY_INTERVAL_MS;
            }
            else if (now >= mlx_debug2.next_retry_tick)
            {
                MLX_RecoveryAttempt2();
            }
            break;
    }

    /* Kembalikan bus ke I2C1 untuk sensor 1 */
    MLX90640_I2CSelectBus(&hi2c1);
}

static void MLX_HandleError2(void)
{
    HAL_I2C_DeInit(&hi2c2);
    HAL_Delay(100);
    MX_I2C2_Init();
    MLX90640_I2CSelectBus(&hi2c2);
    MLX90640_I2CInit();

    mlx_debug2.state = MLX_STATE_RECOVERING;
    mlx_debug2.recovery_start_tick = HAL_GetTick();
    mlx_debug2.next_retry_tick = HAL_GetTick() + MLX_RETRY_INTERVAL_MS;
}

static void MLX_RecoveryAttempt2(void)
{
    MLX90640_I2CSelectBus(&hi2c2);

    if (MLX90640_DumpEE(MLX90640_I2C_ADDR2, eeMLX90640_2) == 0)
    {
        if (MLX90640_ExtractParameters(eeMLX90640_2, &mlx90640_2) >= 0)
        {
            MLX90640_SetChessMode(MLX90640_I2C_ADDR2);
            MLX90640_SetRefreshRate(MLX90640_I2C_ADDR2, 0x03);

            mlx_debug2.state = MLX_STATE_IDLE;
            mlx_debug2.consecutive_errors = 0;
            mlx_debug2.initialized = 1;
            thermData2.initialized = 1;
            thermData2.valid = 1;
            return;
        }
    }

    thermData2.valid = 0;
    mlx_debug2.next_retry_tick = HAL_GetTick() + MLX_RETRY_INTERVAL_MS;
}

/* PZEM UART callbacks */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
        PZEM_IT_TxCpltCallback(&pzem);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
        PZEM_IT_RxCpltCallback(&pzem);
}

/* ─────────────────── DS3231 ─────────────────── */
static uint8_t BCD_To_Dec(uint8_t val)
{
    return (val >> 4) * 10 + (val & 0x0F);
}

static uint8_t Dec_To_BCD(uint8_t val)
{
    return ((val / 10) << 4) | (val % 10);
}

static HAL_StatusTypeDef DS3231_GetTime(volatile RTC_DS3231_t *rtc)
{
    uint8_t buf[7];
    if (HAL_I2C_Mem_Read(&hi2c3, DS3231_ADDRESS,
                          0x00, I2C_MEMADD_SIZE_8BIT,
                          buf, 7, 50) != HAL_OK)
        return HAL_ERROR;

    rtc->second = BCD_To_Dec(buf[0]);
    rtc->minute = BCD_To_Dec(buf[1]);
    rtc->hour   = BCD_To_Dec(buf[2] & 0x3F);
    rtc->day    = BCD_To_Dec(buf[3]);
    rtc->date   = BCD_To_Dec(buf[4]);
    rtc->month  = BCD_To_Dec(buf[5] & 0x1F);
    rtc->year   = 2000 + BCD_To_Dec(buf[6]);
    return HAL_OK;
}

static HAL_StatusTypeDef DS3231_SetTime(RTC_DS3231_t *rtc)
{
    uint8_t buf[7];
    buf[0] = Dec_To_BCD(rtc->second);
    buf[1] = Dec_To_BCD(rtc->minute);
    buf[2] = Dec_To_BCD(rtc->hour);
    buf[3] = Dec_To_BCD(rtc->day);
    buf[4] = Dec_To_BCD(rtc->date);
    buf[5] = Dec_To_BCD(rtc->month);
    buf[6] = Dec_To_BCD(rtc->year - 2000);
    return HAL_I2C_Mem_Write(&hi2c3, DS3231_ADDRESS,
                              0x00, I2C_MEMADD_SIZE_8BIT,
                              buf, 7, HAL_MAX_DELAY);
}

/* ─────────────────── SD Card ─────────────────── */
static uint8_t SD_mount(void *self)
{
    SDCard_Class *s = (SDCard_Class*)self;
    if (f_mount(&s->fs, "", 1) == FR_OK) { s->mounted = 1; return 1; }
    s->mounted = 0;
    return 0;
}

static uint8_t SD_createFile(void *self, const char *filename)
{
    SDCard_Class *s = (SDCard_Class*)self;
    if (!s->mounted) return 0;

    s->result = f_open(&s->file, filename, FA_OPEN_ALWAYS | FA_WRITE);
    if (s->result == FR_OK)
    {
        f_lseek(&s->file, f_size(&s->file));
        if (f_size(&s->file) == 0)
        {
            /* ── Header CSV — ditambah kolom MIC di akhir ── */
            const char header[] =
                "Date,Time,"
                "VA_Volt,VA_Curr,VA_Pow,VA_Freq,VA_PF,VA_VA,VA_VAR,VA_Phi,VA_Theta,"
                "VB_Volt,VB_Curr,VB_Pow,VB_Freq,VB_PF,VB_VA,VB_VAR,VB_Phi,VB_Theta,"
                "VC_Volt,VC_Curr,VC_Pow,VC_Freq,VC_PF,VC_VA,VC_VAR,VC_Phi,VC_Theta,"
                "Therm_Avg,Therm_Max,Therm_Min,Therm_Ta,Therm_Center,"
                "FaultCode,MLX_OK,MLX_Fail,"
                /* BARU */
                "MIC_RMS,MIC_PEAK,MIC_BAND,MIC_Valid"
                "\r\n";
            f_write(&s->file, header, strlen(header), &s->bw);
        }
        f_close(&s->file);
        return 1;
    }
    return 0;
}

static uint8_t SD_append(void *self, const char *filename, const char *text)
{
    SDCard_Class *s = (SDCard_Class*)self;
    if (!s->mounted) return 0;
    if (f_open(&s->file, filename, FA_OPEN_APPEND | FA_WRITE) == FR_OK) {
        f_write(&s->file, text, strlen(text), &s->bw);
        f_close(&s->file);
        return 1;
    }
    return 0;
}

static void MIC_StartReceive(void)
{
    // Batalkan dulu jika ada yang sedang berjalan
    HAL_UART_AbortReceive(&MIC_UART);
    RS485_2_RX();  // Pastikan DIR selalu LOW sebelum receive

    HAL_StatusTypeDef res = HAL_UARTEx_ReceiveToIdle_IT(
        &MIC_UART,
        (uint8_t*)mic_rx_raw,
        MIC_RX_BUF_LEN - 1
    );
    mic_rx_active = (res == HAL_OK) ? 1 : 0;

    /* DEBUG: lihat catatan dbg_micHalStartOkCount di atas. */
    if (res == HAL_OK) {
        dbg_micHalStartOkCount++;
    } else {
        dbg_micHalStartFailCount++;
    }
    dbg_micRxActiveNow = mic_rx_active;
}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART6)
    {
        mic_rx_event_count++;

        dbg_micLastEventSize = Size;   /* DEBUG */

        if (Size > 0 && Size < MIC_RX_BUF_LEN)
        {
            /* Copy raw byte (biner, bukan string) ke buffer event.
             * Tidak ada null-terminator karena payload TLV bisa
             * mengandung byte 0x00 yang sah. */
            memcpy((void*)mic_event_buf, (void*)mic_rx_raw, Size);
            mic_event_len = Size;
            mic_frame_ready = 1;

            dbg_micRawByte0 = mic_event_buf[0];
            dbg_micRawByte1 = (Size > 1) ? mic_event_buf[1] : 0xFF;
            dbg_micRawByte2 = (Size > 2) ? mic_event_buf[2] : 0xFF;
            dbg_micRawByte3 = (Size > 3) ? mic_event_buf[3] : 0xFF;
        }
        else
        {
            dbg_micEventRejected++;
        }

        RS485_2_RX();
        HAL_StatusTypeDef res = HAL_UARTEx_ReceiveToIdle_IT(
            &MIC_UART,
            (uint8_t*)mic_rx_raw,
            MIC_RX_BUF_LEN - 1
        );
        mic_rx_active = (res == HAL_OK) ? 1 : 0;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART6) {
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_PEFLAG(huart);

        mic_rx_active = 0;   // biar watchdog di main loop restart otomatis
    }
}

static uint16_t crc16_ccitt(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= ((uint16_t)data[i] << 8);
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc = (crc << 1);
            }
        }
    }
    return crc;
}

static void MIC_TLV_Reset(void)
{
    mic_tlv_state        = MIC_WAIT_SYNC1;
    mic_tlv_payload_idx  = 0;
    mic_tlv_len_field     = 0;
    mic_tlv_type          = 0;
    mic_tlv_crc_calc      = 0;
    mic_tlv_crc_recv       = 0;
}

static uint8_t MIC_TLV_FeedByte(uint8_t b, MicData_t *out)
{
    switch (mic_tlv_state)
    {
        case MIC_WAIT_SYNC1:
            if (b == MIC_TLV_SYNC1) {
                mic_tlv_state = MIC_WAIT_SYNC2;
            }
            /* selain itu, tetap di WAIT_SYNC1 (cari sync berikutnya) */
            break;

        case MIC_WAIT_SYNC2:
            if (b == MIC_TLV_SYNC2) {
                mic_tlv_state = MIC_WAIT_TYPE;
            } else if (b == MIC_TLV_SYNC1) {
                /* tetap di WAIT_SYNC2, mungkin SYNC1 dobel sebelum SYNC2 */
                mic_tlv_state = MIC_WAIT_SYNC2;
            } else {
                MIC_TLV_Reset();
            }
            break;

        case MIC_WAIT_TYPE:
            mic_tlv_type = b;
            mic_tlv_state = MIC_WAIT_LEN_L;
            break;

        case MIC_WAIT_LEN_L:
            mic_tlv_len_field = b;            /* LEN low byte */
            mic_tlv_state = MIC_WAIT_LEN_H;
            break;

        case MIC_WAIT_LEN_H:
            mic_tlv_len_field |= ((uint16_t)b << 8);   /* LEN high byte */
            if (mic_tlv_len_field != MIC_TLV_PAYLOAD_LEN) {
                MIC_TLV_Reset();     /* panjang tidak sesuai -> noise, resync */
            } else {
                mic_tlv_payload_idx = 0;
                mic_tlv_state = MIC_WAIT_PAYLOAD;
            }
            break;


        case MIC_WAIT_PAYLOAD:
            mic_tlv_payload[mic_tlv_payload_idx++] = b;
            if (mic_tlv_payload_idx >= MIC_TLV_PAYLOAD_LEN) {
                mic_tlv_state = MIC_WAIT_CRC_L;
            }
            break;

        case MIC_WAIT_CRC_L:
            mic_tlv_crc_recv = b;   /* low byte */
            mic_tlv_state = MIC_WAIT_CRC_H;
            break;

        case MIC_WAIT_CRC_H:
        {
            mic_tlv_crc_recv |= ((uint16_t)b << 8);   /* high byte */

            /* Hitung ulang CRC atas TYPE + LENGTH(2 byte LE) + PAYLOAD,
             * PERSIS sama dengan crcLen di buildTLVPacket() ESP-INMP
             * (1 byte TYPE + 2 byte LEN + 24 byte PAYLOAD = 27 byte). */
            uint8_t crcBuf[1 + 2 + MIC_TLV_PAYLOAD_LEN];
            crcBuf[0] = mic_tlv_type;
            crcBuf[1] = (uint8_t)(mic_tlv_len_field & 0xFFU);
            crcBuf[2] = (uint8_t)((mic_tlv_len_field >> 8) & 0xFFU);
            memcpy(&crcBuf[3], mic_tlv_payload, MIC_TLV_PAYLOAD_LEN);

            mic_tlv_crc_calc = crc16_ccitt(crcBuf, sizeof(crcBuf));

            uint8_t frameOk = (mic_tlv_crc_calc == mic_tlv_crc_recv) &&
                               (mic_tlv_type == MIC_TLV_TYPE_DATA);

            if (frameOk)
            {
                dbg_micFrameOkCount++;   /* DEBUG */

                /* ── Decode TLV: 4x [TAG][LEN][VALUE float32 LE] ── */
                uint16_t p = 0;
                while (p < MIC_TLV_PAYLOAD_LEN)
                {
                    uint8_t tag = mic_tlv_payload[p];
                    uint8_t len = mic_tlv_payload[p + 1];

                    if (len != 4U || (p + 2U + len) > MIC_TLV_PAYLOAD_LEN) {
                        /* TLV korup di tengah payload -> batalkan frame ini */
                        MIC_TLV_Reset();
                        return 0;
                    }

                    float val;
                    memcpy(&val, &mic_tlv_payload[p + 2], sizeof(float));

                    switch (tag)
                    {
                        case MIC_TLV_TAG_RMS:  out->rms  = val; break;
                        case MIC_TLV_TAG_PEAK: out->peak = val; break;
                        case MIC_TLV_TAG_BAND: out->band = val; break;
                        case MIC_TLV_TAG_DB:   out->dB   = val; break;
                        default: /* TAG tidak dikenal, lewati saja */ break;
                    }

                    p += (2U + len);
                }

                MIC_TLV_Reset();
                return 1;
            }

            /* CRC mismatch -> buang frame, resync */
            dbg_micCrcFailCount++;   /* DEBUG */
            MIC_TLV_Reset();
            return 0;
        }

        default:
            MIC_TLV_Reset();
            break;
    }

    return 0;
}

/**
 * MIC_TLV_ProcessEventBuffer()
 *
 * Dipanggil dari main loop saat mic_frame_ready==1. Mem-feed seluruh
 * byte yang diterima dari satu event ReceiveToIdle_IT ke state
 * machine TLV satu per satu. Karena RS485 mengirim frame 30-byte
 * fixed dan ESP32-INMP441 mengirim per-frame (settle DIR sebelum &
 * setelah TX — lihat Tahap 1), satu event IDLE pada umumnya berisi
 * tepat satu frame utuh. Namun parser tetap byte-oriented dan robust
 * walau event datang terpotong/menyatu, karena state machine akan
 * otomatis resync lewat SYNC1/SYNC2.
 */
static void MIC_TLV_ProcessEventBuffer(MicData_t *out)
{
    for (uint16_t i = 0; i < mic_event_len; i++)
    {
        if (MIC_TLV_FeedByte(mic_event_buf[i], out))
        {
            out->last_update_tick = HAL_GetTick();
            out->valid = 1;
        }
    }
}

/**
 * MIC_UpdateValidity()
 *
 * Menandai micData.valid = 0 jika tidak ada data baru
 * selama lebih dari MIC_TIMEOUT_MS.
 *
 * Ini penting agar sistem tidak menggunakan data lama
 * saat ESP32 mikrofon mati atau disconnect.
 */
static void MIC_UpdateValidity(MicData_t *mic)
{
    if (mic->valid &&
        (HAL_GetTick() - mic->last_update_tick) > MIC_TIMEOUT_MS)
    {
        mic->valid = 0;
    }
}

/**
 * GW_TLV_WriteFloat()
 *
 * Menulis satu TLV float32 (TAG+LEN=4+VALUE) ke buffer 'buf'.
 * Return: jumlah byte yang ditulis (selalu 6).
 */
static uint16_t GW_TLV_WriteFloat(uint8_t *buf, uint8_t tag, float val)
{
    buf[0] = tag;
    buf[1] = 4U;
    memcpy(&buf[2], &val, sizeof(float));
    return 6U;
}

/**
 * GW_TLV_WriteRaw()
 *
 * Menulis satu TLV raw byte (TAG+LEN+data mentah), dipakai untuk
 * field DateTime (7 byte) yang bukan float.
 * Return: jumlah byte yang ditulis (2 + len).
 */
static uint16_t GW_TLV_WriteRaw(uint8_t *buf, uint8_t tag, const uint8_t *data, uint8_t len)
{
    buf[0] = tag;
    buf[1] = len;
    memcpy(&buf[2], data, len);
    return (uint16_t)(2U + len);
}

/**
 * GW_BuildFrame()
 *
 * Menyusun frame lengkap:
 *   [SYNC1][SYNC2][TYPE][LEN_L][LEN_H][PAYLOAD][CRC_L][CRC_H]
 * LENGTH 2-byte little-endian (beda dari frame mic Tahap 1 yang
 * 1-byte) — diperlukan karena payload thermal (1536 byte) tidak
 * muat di 1 byte.
 *
 * Return: panjang total frame dalam byte.
 */
static uint16_t GW_BuildFrame(uint8_t *out, uint8_t type, const uint8_t *payload, uint16_t payloadLen)
{
    uint16_t idx = 0;

    dbg_gwLastFrameType = type;   /* DEBUG: lihat catatan dbg_gwFrameSentCount di atas */

    out[idx++] = GW_TLV_SYNC1;
    out[idx++] = GW_TLV_SYNC2;

    uint16_t crcStartIdx = idx;

    out[idx++] = type;
    out[idx++] = (uint8_t)(payloadLen & 0xFFU);
    out[idx++] = (uint8_t)((payloadLen >> 8) & 0xFFU);

    memcpy(&out[idx], payload, payloadLen);
    idx += payloadLen;

    uint16_t crcLen = idx - crcStartIdx;   /* TYPE + LEN(2) + PAYLOAD */
    uint16_t crc = crc16_ccitt(&out[crcStartIdx], crcLen);

    out[idx++] = (uint8_t)(crc & 0xFFU);
    out[idx++] = (uint8_t)((crc >> 8) & 0xFFU);

    return idx;
}

/**
 * GW_SendFrame()
 *
 * Wrapper transmit RS485_1 — kontrol DIR identik dengan ESP1_Send()
 * lama (TX HIGH, transmit, RX LOW).
 */
static void GW_SendFrame(uint8_t *frame, uint16_t frameLen)
{
    RS485_1_TX();
    HAL_UART_Transmit(&huart1, frame, frameLen, 500);
    RS485_1_RX();

    /* DEBUG: bukti GW_SendFrame benar2 dipanggil & HAL_UART_Transmit
     * sudah return. Pantau via Live Expressions (lihat deklarasi di
     * atas). */
    dbg_gwFrameSentCount++;
    dbg_gwLastFrameLen = frameLen;
}

/**
 * GW_SendTelemetry()
 *
 * Menggantikan Build_ESP1_Message() + ESP1_SendTelemetry().
 * Menyusun payload dari TLV-TLV kecil (DateTime + 27 float listrik
 * + TMAX/TMAX2 + 4 field mic), bungkus jadi frame TYPE_TELEMETRY,
 * kirim tiap 1 detik — interval & sumber data identik dengan versi
 * CSV lama, hanya representasi byte yang berubah.
 */
void GW_SendTelemetry(void)
{
    static uint32_t lastSend = 0;

    if (HAL_GetTick() - lastSend < ESP1_TELEMETRY_INTERVAL_MS) return;
    lastSend = HAL_GetTick();

    uint8_t payload[300];
    uint16_t p = 0;

    /* DateTime: date, month, yearLo, yearHi, hour, min, sec (7 byte) */
    uint8_t dt[7];
    dt[0] = rtcData.date;
    dt[1] = rtcData.month;
    dt[2] = (uint8_t)(rtcData.year & 0xFFU);
    dt[3] = (uint8_t)((rtcData.year >> 8) & 0xFFU);
    dt[4] = rtcData.hour;
    dt[5] = rtcData.minute;
    dt[6] = rtcData.second;
    p += GW_TLV_WriteRaw(&payload[p], GW_TAG_DATETIME, dt, sizeof(dt));

    /* Phase A */
    p += GW_TLV_WriteFloat(&payload[p], GW_TAG_VA,   elecData.phase[0].voltage);
    p += GW_TLV_WriteFloat(&payload[p], GW_TAG_IA,   elecData.phase[0].current);
    p += GW_TLV_WriteFloat(&payload[p], GW_TAG_PA,   elecData.phase[0].power);
    p += GW_TLV_WriteFloat(&payload[p], GW_TAG_FA,   elecData.phase[0].frequency);
    p += GW_TLV_WriteFloat(&payload[p], GW_TAG_PFA,  elecData.phase[0].pf);
    p += GW_TLV_WriteFloat(&payload[p], GW_TAG_VAA,  elecData.phase[0].va);
    p += GW_TLV_WriteFloat(&payload[p], GW_TAG_VARA, elecData.phase[0].var);
    p += GW_TLV_WriteFloat(&payload[p], GW_TAG_PHIA, elecData.phase[0].phi);
    p += GW_TLV_WriteFloat(&payload[p], GW_TAG_THTA, elecData.phase[0].theta);

    /* Phase B */
    p += GW_TLV_WriteFloat(&payload[p], GW_TAG_VB,   elecData.phase[1].voltage);
    p += GW_TLV_WriteFloat(&payload[p], GW_TAG_IB,   elecData.phase[1].current);
    p += GW_TLV_WriteFloat(&payload[p], GW_TAG_PB,   elecData.phase[1].power);
    p += GW_TLV_WriteFloat(&payload[p], GW_TAG_FB,   elecData.phase[1].frequency);
    p += GW_TLV_WriteFloat(&payload[p], GW_TAG_PFB,  elecData.phase[1].pf);
    p += GW_TLV_WriteFloat(&payload[p], GW_TAG_VAB,  elecData.phase[1].va);
    p += GW_TLV_WriteFloat(&payload[p], GW_TAG_VARB, elecData.phase[1].var);
    p += GW_TLV_WriteFloat(&payload[p], GW_TAG_PHIB, elecData.phase[1].phi);
    p += GW_TLV_WriteFloat(&payload[p], GW_TAG_THTB, elecData.phase[1].theta);

    /* Phase C */
    p += GW_TLV_WriteFloat(&payload[p], GW_TAG_VC,   elecData.phase[2].voltage);
    p += GW_TLV_WriteFloat(&payload[p], GW_TAG_IC,   elecData.phase[2].current);
    p += GW_TLV_WriteFloat(&payload[p], GW_TAG_PC,   elecData.phase[2].power);
    p += GW_TLV_WriteFloat(&payload[p], GW_TAG_FC,   elecData.phase[2].frequency);
    p += GW_TLV_WriteFloat(&payload[p], GW_TAG_PFC,  elecData.phase[2].pf);
    p += GW_TLV_WriteFloat(&payload[p], GW_TAG_VAC,  elecData.phase[2].va);
    p += GW_TLV_WriteFloat(&payload[p], GW_TAG_VARC, elecData.phase[2].var);
    p += GW_TLV_WriteFloat(&payload[p], GW_TAG_PHIC, elecData.phase[2].phi);
    p += GW_TLV_WriteFloat(&payload[p], GW_TAG_THTC, elecData.phase[2].theta);

    /* Thermal max (sama seperti TMAX:/TMAX2: di CSV lama, ambil maxTemp) */
    float tmax1 = thermData.initialized  ? thermData.maxTemp  : 0.0f;
    float tmax2 = thermData2.initialized ? thermData2.maxTemp : 0.0f;
    p += GW_TLV_WriteFloat(&payload[p], GW_TAG_TMAX,  tmax1);
    p += GW_TLV_WriteFloat(&payload[p], GW_TAG_TMAX2, tmax2);

    /* Mic */
    p += GW_TLV_WriteFloat(&payload[p], GW_TAG_MIC_RMS,  micData.rms);
    p += GW_TLV_WriteFloat(&payload[p], GW_TAG_MIC_BAND, micData.band);
    p += GW_TLV_WriteFloat(&payload[p], GW_TAG_MIC_DB,   micData.dB);

    uint8_t micValid8 = (uint8_t)micData.valid;
    p += GW_TLV_WriteRaw(&payload[p], GW_TAG_MIC_VALID, &micValid8, 1U);

    uint16_t frameLen = GW_BuildFrame(gw_tx_buf, GW_TLV_TYPE_TELEMETRY, payload, p);
    GW_SendFrame(gw_tx_buf, frameLen);
}

/**
 * GW_BuildAndSendThermalFrame()
 *
 * Menggantikan ESP1_BuildAndSendThermalFrame(). Satu TLV besar
 * (TAG_THERMAL_GRID, 1536 byte) dibungkus frame TYPE_THERMAL1/2.
 * Sumber data (td->txBuffer, nilai suhu x100) dan kondisi
 * "skip jika !valid" identik dengan versi lama.
 */
static void GW_BuildAndSendThermalFrame(ThermalData_t *td, uint8_t frameType)
{
    if (!td->valid) return;

    static uint8_t payload[2 + 2 + GW_THERMAL_TLV_VALLEN];  /* TAG+LEN(2)+1536 */
    uint16_t p = 0;

    payload[p++] = GW_TAG_THERMAL_GRID;
    payload[p++] = (uint8_t)(GW_THERMAL_TLV_VALLEN & 0xFFU);
    payload[p++] = (uint8_t)((GW_THERMAL_TLV_VALLEN >> 8) & 0xFFU);
    /* Catatan: TLV thermal pakai LEN 2-byte (beda dari TLV float di
     * atas yang 1-byte), karena 1536 > 255. ESP32-TFT decoder harus
     * tahu TAG ini selalu diikuti LEN 2-byte, bukan 1-byte. */

    for (int i = 0; i < 768; i++)
    {
        uint16_t v  = td->txBuffer[i];
        payload[p++] = (uint8_t)(v & 0xFFU);
        payload[p++] = (uint8_t)((v >> 8) & 0xFFU);
    }

    uint16_t frameLen = GW_BuildFrame(gw_tx_buf, frameType, payload, p);
    GW_SendFrame(gw_tx_buf, frameLen);
}

/**
 * GW_SendThermalFrame()
 *
 * Menggantikan ESP1_SendThermalFrame(). Interval & urutan kirim
 * (MLX1 lalu delay 20ms lalu MLX2) identik dengan versi lama.
 */
void GW_SendThermalFrame(void)
{
    static uint32_t lastSend = 0;

    if (HAL_GetTick() - lastSend < ESP1_THERMAL_INTERVAL_MS) return;
    lastSend = HAL_GetTick();

    GW_BuildAndSendThermalFrame(&thermData,  GW_TLV_TYPE_THERMAL1);
    HAL_Delay(20);
    GW_BuildAndSendThermalFrame(&thermData2, GW_TLV_TYPE_THERMAL2);
}

void ESP1_Send(char *msg)
{
    RS485_1_TX();
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 200);
    RS485_1_RX();
}


/* ─────────────────── Fault helpers ─────────────────── */
static uint16_t CheckVoltageFault(float vA, float vB, float vC)
{
    const float vLow  = VOLT_NOMINAL * (1.0f - LTOL);   /* 198 V */
    const float vHigh = VOLT_NOMINAL * (1.0f + HTOL);   /* 231 V */

    if (vA > vHigh) return 4U;
    if (vB > vHigh) return 5U;
    if (vC > vHigh) return 6U;
    if (vA < vLow)  return 1U;
    if (vB < vLow)  return 2U;
    if (vC < vLow)  return 3U;
    return 0U;
}

static void UpdateFaultCode(uint16_t code)
{

}

static void PZEM_HardReset(PZEM6L24_t *dev) {
    HAL_UART_AbortReceive_IT(dev->huart);
    HAL_UART_AbortTransmit_IT(dev->huart);
    __HAL_UART_FLUSH_DRREGISTER(dev->huart); // Bersihkan buffer hardware
    dev->state = PZEM_STATE_IDLE;
    // Optional: jika UART hang parah, bisa lakukan DeInit/Init
    // HAL_UART_DeInit(dev->huart);
    // HAL_UART_Init(dev->huart);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
