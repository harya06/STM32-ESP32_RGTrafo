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
    uint32_t last_update_tick;   /* HAL_GetTick() saat terakhir data valid diterima */
    uint8_t  valid;              /* 1 = data segar (< MIC_TIMEOUT_MS), 0 = stale/no data */
} MicData_t;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define VOLT_NOMINAL     220.0f
#define LTOL             0.10f
#define HTOL             0.05f
#define PZEM_CONN_RETRY  3U

#define DS3231_ADDRESS   (0x68 << 1)

#define MLX90640_I2C_ADDR            0x33
#define MLX_MAX_CONSECUTIVE_ERRORS   5        /* Berapa kali error sebelum reset I2C */
#define MLX_RECOVERY_TIMEOUT_MS      10000UL  /* Timeout recovery: 10 detik */
#define MLX_RETRY_INTERVAL_MS        2000UL   /* Coba baca ulang tiap 2 detik saat error */
#define MLX_BLINK_ERROR_COUNT        3        /* Kedip LED 3x saat recovery */

#define PZEM_SLAVE_ADDRESS           0x01
#define PZEM_REQUEST_INTERVAL        500      /* Request tiap 500ms */


#define MIC_UART                     huart6
#define MIC_RX_BUF_LEN               64
#define MIC_TIMEOUT_MS               3000U    /* Data stale setelah 3 detik */
#define MIC_RESTART_INTERVAL_MS      2000U    /* Restart UART IT tiap 2 detik jika mati */

#define SD_LOG_INTERVAL_MS           5000UL   /* Log ke SD tiap 5 detik */
#define ESP1_TELEMETRY_INTERVAL_MS   1000UL   /* Kirim data ke ESP1 tiap 1 detik */

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

/* ----- MLX90640 ----- */
paramsMLX90640 mlx90640;
uint16_t       eeMLX90640[832];
uint16_t       mlx90640Frame[834];
float          mlx90640To[768];

/* ----- Data terstruktur ----- */
Electrical_t  elecData;
ThermalData_t thermData;
MLX_Debug_t   mlx_debug;

/* ----- PZEM ----- */
PZEM6L24_t pzem;
static uint8_t connFailCount = 0U;

/* ----- RTC DS3231 ----- */
volatile RTC_DS3231_t rtcData;

/* ----- SD Card ----- */
SDCard_Class sd;

static volatile uint32_t mlx_consecutive_errors = 0;
static volatile uint32_t pzem_last_request = 0;

static volatile uint8_t  mic_rx_raw[MIC_RX_BUF_LEN];
static volatile char     mic_frame_buf[MIC_RX_BUF_LEN];
static volatile uint8_t mic_frame_ready = 0;    /* Flag: frame baru tersedia */
static volatile uint8_t mic_rx_active = 0;
static uint32_t mic_check_last = 0;

MicData_t micData = {
    .rms              = 0.0f,
    .peak             = 0.0f,
    .band             = 0.0f,
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
static void MLX_Process(void);
static void MLX_HandleError(void);
static void MLX_RecoveryAttempt(void);

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

void Build_ESP1_Message(char *buffer);
void ESP1_SendTelemetry(void);

/* ─── Fungsi mikrofon ─── */
static void MIC_StartReceive(void);
static uint8_t MIC_ParseFrame(const char *frame, MicData_t *out);
static void MIC_UpdateValidity(MicData_t *mic);

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
  MLX_Init();  /* Tidak akan freeze meski sensor error */

  /* Inisialisasi struktur data */
  memset(&elecData, 0, sizeof(elecData));

  /* DS3231 RTC */
  rtcData.second = 00;
  rtcData.minute = 22;
  rtcData.hour   = 22;
  rtcData.day    = 3; /*Rabu*/
  rtcData.date   = 17;
  rtcData.month  = 6;
  rtcData.year   = 2026;
//   DS3231_SetTime((RTC_DS3 231_t*)&rtcData);  /* Uncomment untuk set waktu */

  /* SD Card */
  sd.mount = SD_mount;
  sd.createFile = SD_createFile;
  sd.append = SD_append;
  if (sd.mount(&sd)) {
      sd.createFile(&sd, "LOG.CSV");
  }

  /* Microphone UART */
  RS485_2_RX();
  MIC_StartReceive();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      /* WATCHDOG REFRESH */
      HAL_IWDG_Refresh(&hiwdg);

      if (HAL_GetTick() - mic_check_last > MIC_RESTART_INTERVAL_MS)
      {
          mic_check_last = HAL_GetTick();
          // Hanya restart jika receive TIDAK aktif (bukan sekadar tidak ada frame)
          if (!mic_rx_active)
          {
              MIC_StartReceive();
          }
      }

      /* Update validitas microphone */
      MIC_UpdateValidity(&micData);

      /* Proses frame mic jika ada */
      if (mic_frame_ready)
      {
          if (MIC_ParseFrame((const char*)mic_frame_buf, &micData))
          {
              micData.last_update_tick = HAL_GetTick();
              micData.valid = 1;
          }
          mic_frame_ready = 0;
          MIC_StartReceive();  /* Restart untuk frame berikutnya */
      }

      if (HAL_GetTick() - pzem_last_request >= PZEM_REQUEST_INTERVAL)
      {
          pzem_last_request = HAL_GetTick();
          if (pzem.state == PZEM_STATE_IDLE)
              PZEM_IT_RequestAll(&pzem);
      }

      if (pzem.state == PZEM_STATE_COMPLETE)
      {
          if (PZEM_IT_ProcessData(&pzem) == HAL_OK)
          {
              for (int i = 0; i < 3; i++)
              {
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
              connFailCount = 0;

              /* Update fault code (kecuali MLX critical) */
              if (elecData.faultCode != FAULT_MLX_CRITICAL)
              {
                  elecData.faultCode = CheckVoltageFault(
                      elecData.phase[0].voltage,
                      elecData.phase[1].voltage,
                      elecData.phase[2].voltage);
              }
          }
          pzem.state = PZEM_STATE_IDLE;
      }
      else if (pzem.state == PZEM_STATE_FAULT || pzem.state == PZEM_STATE_ERROR)
      {
          HAL_UART_Abort(pzem.huart);
          pzem.state = PZEM_STATE_IDLE;
          if (++connFailCount >= PZEM_CONN_RETRY)
              elecData.faultCode = FAULT_PZEM_LOST;
      }

      MLX_Process();

      DS3231_GetTime(&rtcData);

      static uint32_t lastLog = 0;
      if (HAL_GetTick() - lastLog > SD_LOG_INTERVAL_MS)
      {
          lastLog = HAL_GetTick();

          char line[640];
          sprintf(line,
              "%02d/%02d/%04d,%02d:%02d:%02d,"
              "%.2f,%.3f,%.2f,%.2f,%.3f,%.2f,%.2f,%.2f,%.2f,"
              "%.2f,%.3f,%.2f,%.2f,%.3f,%.2f,%.2f,%.2f,%.2f,"
              "%.2f,%.3f,%.2f,%.2f,%.3f,%.2f,%.2f,%.2f,%.2f,"
              "%.2f,%.2f,%.2f,%.2f,%.2f,"
              "%u,%lu,%lu,"
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

              thermData.initialized ? thermData.avgTemp : 0.0f,
              thermData.initialized ? thermData.maxTemp : 0.0f,
              thermData.initialized ? thermData.minTemp : 0.0f,
              thermData.initialized ? thermData.Ta : 0.0f,
              thermData.initialized ? thermData.centerTemp : 0.0f,

              (unsigned int)elecData.faultCode,
              (unsigned long)mlx_debug.frame_success_count,
              (unsigned long)mlx_debug.frame_fail_count,

              micData.rms, micData.peak, micData.band,
              (unsigned int)micData.valid
          );

          sd.append(&sd, "LOG.CSV", line);
      }

      ESP1_SendTelemetry();

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
 * @brief Proses pembacaan MLX90640 - NON-BLOCKING
 *        Dipanggil dari main loop
 */
static void MLX_Process(void)
{
    uint32_t now = HAL_GetTick();

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
                          buf, 7, 10) != HAL_OK)
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

/**
 * MIC_StartReceive()
 *
 * Memulai (atau me-restart) penerimaan UART dengan metode
 * ReceiveToIdle: interrupt akan dipanggil ketika bus IDLE
 * setelah sejumlah byte diterima, tanpa perlu tahu panjang
 * frame secara tepat sebelumnya.
 *
 * Lebih robust daripada HAL_UART_Receive_IT(N) yang
 * mengharuskan kita tahu panjang frame persis.
 */
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
}

/**
 * HAL_UARTEx_RxEventCallback()
 *
 * Dipanggil oleh HAL dalam interrupt context ketika:
 *   - UART IDLE event terdeteksi setelah data masuk, ATAU
 *   - Buffer penuh (Size byte diterima)
 *
 * Kita hanya proses event dari huart6 (jalur mikrofon).
 * Data disalin ke mic_frame_buf dan flag mic_frame_ready di-set.
 * Main loop yang melakukan parse (hindari operasi berat di ISR).
 *
 * Parameter:
 *   huart — UART handle yang memicu callback
 *   Size  — jumlah byte yang diterima
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART6)
    {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

        if (Size > 0 && Size < MIC_RX_BUF_LEN)
        {
            memcpy((void*)mic_frame_buf, (void*)mic_rx_raw, Size);
            ((volatile char*)mic_frame_buf)[Size] = '\0';
            mic_frame_ready = 1;
        }

        // Pastikan DIR tetap LOW, lalu restart receive
        RS485_2_RX();
        HAL_StatusTypeDef res = HAL_UARTEx_ReceiveToIdle_IT(
            &MIC_UART,
            (uint8_t*)mic_rx_raw,
            MIC_RX_BUF_LEN - 1
        );
        mic_rx_active = (res == HAL_OK) ? 1 : 0;
    }
}
/**
 * MIC_ParseFrame()
 *
 * Mem-parse string format: "RMS:x.xxx,PEAK:x.xxx,BAND:x.xxx\r\n"
 *
 * Algoritma:
 *   1. Cari substring "RMS:"  → strtof() dari posisi setelahnya
 *   2. Cari substring "PEAK:" → strtof()
 *   3. Cari substring "BAND:" → strtof()
 *   4. Validasi: semua nilai harus ≥ 0.0 (sinyal audio adalah nilai absolut)
 *
 * Return:
 *   1 jika parse berhasil dan nilai valid
 *   0 jika format tidak dikenali atau nilai out-of-range
 *
 * Mengapa tidak pakai sscanf("RMS:%f,PEAK:%f,BAND:%f")?
 *   sscanf lebih ringkas tapi rapuh terhadap whitespace dan
 *   karakter \r\n di tengah string. Metode strstr+strtof
 *   lebih toleran terhadap variasi kecil format.
 */
static uint8_t MIC_ParseFrame(const char *frame, MicData_t *out)
{
    if (frame == NULL || out == NULL) return 0;

    char *ptr;
    float rms, peak, band;

    /* ── Parse RMS ── */
    ptr = strstr(frame, "RMS:");
    if (ptr == NULL) return 0;
    ptr += 4;   /* Geser melewati "RMS:" */
    rms = strtof(ptr, NULL);

    /* ── Parse PEAK ── */
    ptr = strstr(frame, "PEAK:");
    if (ptr == NULL) return 0;
    ptr += 5;
    peak = strtof(ptr, NULL);

    /* ── Parse BAND ── */
    ptr = strstr(frame, "BAND:");
    if (ptr == NULL) return 0;
    ptr += 5;
    band = strtof(ptr, NULL);

    /* ── Validasi range ──
     *
     * INMP441 normalized ke [0.0, 1.0].
     * Nilai negatif tidak mungkin untuk RMS, PEAK, BAND energy.
     * Nilai > 2.0 curiga (clipping atau format salah).
     * Batas atas longgar (2.0) untuk toleransi variasi gain.
     */
    if (rms  < 0.0f || rms  > 2.0f) return 0;
    if (peak < 0.0f || peak > 2.0f) return 0;
    if (band < 0.0f)                 return 0;

    out->rms  = rms;
    out->peak = peak;
    out->band = band;

    return 1;
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

/* ══════════════════════════════════════════════════════════════
 *  BUILD_ESP1_MESSAGE — versi diperbarui dengan field MIC
 *
 *  Field tambahan di akhir pesan:
 *    MIC_RMS:x.xxxx,MIC_PEAK:x.xxxx,MIC_BAND:x.xxxx,MIC_VALID:n
 *
 *  MIC_VALID: 1 = data segar, 0 = ESP32 mikrofon tidak aktif
 *
 *  Parser di ESP1 (display) perlu diupdate untuk membaca
 *  empat field baru ini.
 * ══════════════════════════════════════════════════════════════ */
void Build_ESP1_Message(char *buffer)
{
    sprintf(buffer,
        /* Timestamp */
        "DATE:%02d/%02d/%04d,"
        "TIME:%02d:%02d:%02d,"

        /* Phase A */
        "VA:%.1f,IA:%.2f,PA:%.1f,FA:%.2f,PFA:%.3f,"
        "VAA:%.1f,VARA:%.1f,PHIA:%.1f,THETAA:%.1f,"

        /* Phase B */
        "VB:%.1f,IB:%.2f,PB:%.1f,FB:%.2f,PFB:%.3f,"
        "VAB:%.1f,VARB:%.1f,PHIB:%.1f,THETAB:%.1f,"

        /* Phase C */
        "VC:%.1f,IC:%.2f,PC:%.1f,FC:%.2f,PFC:%.3f,"
        "VAC:%.1f,VARC:%.1f,PHIC:%.1f,THETAC:%.1f,"

        /* Thermal */
        "TAVG:%.1f,TMAX:%.1f,TMIN:%.1f,TA:%.1f,TCENTER:%.1f,"

        /* System */
        "FAULT:%u,MLXOK:%lu,MLXFAIL:%lu,"

        /* ── BARU: MLX Valid Flag ── */
        "MLX_VALID:%u,"

        /* Akustik Mikrofon */
        "MIC_RMS:%.4f,"
        "MIC_PEAK:%.4f,"
        "MIC_BAND:%.4f,"
        "MIC_VALID:%u"

        "\r\n",

        /* Timestamp */
        rtcData.date, rtcData.month, rtcData.year,
        rtcData.hour, rtcData.minute, rtcData.second,

        /* Phase A */
        elecData.phase[0].voltage, elecData.phase[0].current,
        elecData.phase[0].power,   elecData.phase[0].frequency,
        elecData.phase[0].pf,      elecData.phase[0].va,
        elecData.phase[0].var,     elecData.phase[0].phi,
        elecData.phase[0].theta,

        /* Phase B */
        elecData.phase[1].voltage, elecData.phase[1].current,
        elecData.phase[1].power,   elecData.phase[1].frequency,
        elecData.phase[1].pf,      elecData.phase[1].va,
        elecData.phase[1].var,     elecData.phase[1].phi,
        elecData.phase[1].theta,

        /* Phase C */
        elecData.phase[2].voltage, elecData.phase[2].current,
        elecData.phase[2].power,   elecData.phase[2].frequency,
        elecData.phase[2].pf,      elecData.phase[2].va,
        elecData.phase[2].var,     elecData.phase[2].phi,
        elecData.phase[2].theta,

        /* Thermal */
        thermData.initialized ? thermData.avgTemp    : 0.0f,
        thermData.initialized ? thermData.maxTemp    : 0.0f,
        thermData.initialized ? thermData.minTemp    : 0.0f,
        thermData.initialized ? thermData.Ta         : 0.0f,
        thermData.initialized ? thermData.centerTemp : 0.0f,

        /* System */
        (unsigned int)elecData.faultCode,
        (unsigned long)mlx_debug.frame_success_count,
        (unsigned long)mlx_debug.frame_fail_count,

        /* ── MLX Valid Flag ── */
        (unsigned int)thermData.valid,

        /* Akustik */
        micData.rms,
        micData.peak,
        micData.band,
        (unsigned int)micData.valid
    );
}


void ESP1_SendTelemetry(void)
{
    static uint32_t lastSend = 0;

    if (HAL_GetTick() - lastSend >= 1000U)
    {
        lastSend = HAL_GetTick();

        char txBuffer[1024];
        memset(txBuffer, 0, sizeof(txBuffer));
        Build_ESP1_Message(txBuffer);
        ESP1_Send(txBuffer);
    }
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
    /* Aksi saat fault:
     *   code 0   : semua normal → matikan LED fault
     *   code 1-6 : tegangan → nyalakan LED
     *   code 404 : PZEM lost → nyalakan LED
     */
	(void)code;

    if (code == 0U) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
    }
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
