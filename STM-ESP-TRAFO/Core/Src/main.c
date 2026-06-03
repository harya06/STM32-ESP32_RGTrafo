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
    PhaseData_t phase[3];   /* indeks: 0 = A, 1 = B, 2 = C */
    uint16_t    faultCode;
} Electrical_t;

/* Struktur data thermal MLX90640 */
typedef struct {
    float Ta;           /* suhu ambient / sensor */
    float minTemp;
    float maxTemp;
    float avgTemp;
    float centerTemp;   /* pixel tengah (opsional) */
} ThermalData_t;

/* Struktur data RTC DS3231 */
typedef struct {
    uint8_t second;
    uint8_t minute;
    uint8_t hour;
    uint8_t day;      /* 1-7 */
    uint8_t date;     /* 1-31 */
    uint8_t month;    /* 1-12 */
    uint16_t year;    /* full year example 2026 */
} RTC_DS3231_t;

typedef struct
{
    FATFS fs;
    FIL file;
    FRESULT result;
    UINT bw;
    uint8_t mounted;

    /* Methods */
    uint8_t (*mount)(void *self);
    uint8_t (*createFile)(void *self, const char *filename);
    uint8_t (*append)(void *self, const char *filename, const char *text);

} SDCard_Class;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define VOLT_NOMINAL     220.0f
#define LTOL             0.10f   /* 10% undervolt */
#define HTOL             0.05f   /*  5% overvolt */
#define PZEM_CONN_RETRY  3U

#define DS3231_ADDRESS  (0x68 << 1)   /* Shifted for HAL */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

/* ----- MLX90640 ----- */
paramsMLX90640 mlx90640;
uint16_t eeMLX90640[832];
uint16_t mlx90640Frame[834];
float mlx90640To[768];

/* ----- Data terstruktur untuk debugging ----- */
volatile Electrical_t  elecData;
volatile ThermalData_t thermData;
/* ----- PZEM ----- */
PZEM6L24_t pzem;
static uint8_t connFailCount = 0U;

/* ----- RTC DS3131 ----- */
volatile RTC_DS3231_t  rtcData;
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
/* USER CODE BEGIN PFP */

static uint16_t CheckVoltageFault(float vA, float vB, float vC);
static void     UpdateFaultCode(uint16_t code);

static uint8_t BCD_To_Dec(uint8_t val);
static uint8_t Dec_To_BCD(uint8_t val);
static HAL_StatusTypeDef DS3231_GetTime(volatile RTC_DS3231_t *rtc);
static HAL_StatusTypeDef DS3231_SetTime(RTC_DS3231_t *rtc);

static uint8_t SD_mount(void *self);
static uint8_t SD_createFile(void *self, const char *filename);
static uint8_t SD_append(void *self, const char *filename, const char *text);
SDCard_Class sd;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  /* USER CODE BEGIN 2 */
  /* LED indikator koneksi PZEM (awalnya nyala = belum terhubung) */
   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);

   /* Inisialisasi PZEM */
   PZEM_Init(&pzem, &huart2, 0x01);

   /* ===================== */
   /* Inisialisasi MLX90640 */
   /* ===================== */
   if (MLX90640_DumpEE(0x33, eeMLX90640) != 0)
   {
       /* EEPROM error → fault permanen, LED merah */
       while (1)
       {
           HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
           HAL_Delay(200);
           HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
           HAL_Delay(200);
       }
   }

   if (MLX90640_ExtractParameters(eeMLX90640, &mlx90640) < 0)
   {
       /* Parameter error */
       while (1)
       {
           HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
           HAL_Delay(100);
           HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
           HAL_Delay(100);
       }
   }

   MLX90640_SetChessMode(0x33);
   MLX90640_SetRefreshRate(0x33, 0x03);  /* 4 Hz */

   /* Inisialisasi struktur data ke nol */
   memset(&elecData, 0, sizeof(elecData));
   memset(&thermData, 0, sizeof(thermData));

   /* ===================== */
   /* Inisialisasi DS3231   */
   /* ===================== */

   rtcData.second = 0;
   rtcData.minute = 18;
   rtcData.hour   = 18;
   rtcData.day    = 3;      /* Rabu */
   rtcData.date   = 3;
   rtcData.month  = 6;
   rtcData.year   = 2026;

   /* Set hanya sekali saat awal flashing */
   /* Setelah itu bisa dikomentari */
//   DS3231_SetTime(&rtcData);

   /* ===================== */
   /* SD Card Object Init   */
   /* ===================== */

   sd.mount      = SD_mount;
   sd.createFile = SD_createFile;
   sd.append     = SD_append;

   if (sd.mount(&sd))
   {
       sd.createFile(&sd, "LOG.CSV");
   }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  static uint32_t alive = 0;
	  alive++;

	   /* ============================================= */
	    /*               BACA PZEM (3 FASA)              */
	    /* ============================================= */
	    if (pzem.state == PZEM_STATE_FAULT || pzem.state == PZEM_STATE_ERROR)
	    {
	        HAL_UART_Abort(pzem.huart);
	        pzem.state = PZEM_STATE_IDLE;
	    }

	    if (PZEM_IT_RequestAll(&pzem) == HAL_OK)
	    {
	        uint32_t t0 = HAL_GetTick();
	        while (pzem.state == PZEM_STATE_TX || pzem.state == PZEM_STATE_RX)
	        {
	            if (HAL_GetTick() - t0 > 2000U)
	            {
	                HAL_UART_Abort(pzem.huart);
	                pzem.state = PZEM_STATE_ERROR;
	                break;
	            }
	        }

	        if (pzem.state == PZEM_STATE_COMPLETE)
	        {
	            if (PZEM_IT_ProcessData(&pzem) == HAL_OK)
	            {
	                /* Simpan ke struct 3 fasa */
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

	                connFailCount = 0U;
	                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);

	                /* Evaluasi fault tegangan */
	                elecData.faultCode = CheckVoltageFault(
	                    elecData.phase[0].voltage,
	                    elecData.phase[1].voltage,
	                    elecData.phase[2].voltage);
	                UpdateFaultCode(elecData.faultCode);
	            }
	        }
	        else
	        {
	            HAL_UART_Abort(pzem.huart);
	            pzem.state = PZEM_STATE_IDLE;

	            if (connFailCount < PZEM_CONN_RETRY)
	                connFailCount++;

	            if (connFailCount >= PZEM_CONN_RETRY)
	            {
	                elecData.faultCode = 404U;
	                UpdateFaultCode(404U);
	                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
	            }
	        }
	    }
	    else
	    {
	        HAL_UART_Abort(pzem.huart);
	        pzem.state = PZEM_STATE_IDLE;
	    }

	    /* ============================================= */
	    /*               BACA MLX90640                   */
	    /* ============================================= */
	    if (MLX90640_GetFrameData(0x33, mlx90640Frame) >= 0)
	    {
	        float Ta = MLX90640_GetTa(mlx90640Frame, &mlx90640);
	        float emissivity = 0.95f;

	        MLX90640_CalculateTo(mlx90640Frame, &mlx90640, emissivity, Ta, mlx90640To);

	        float minTemp = 1000.0f;
	        float maxTemp = -1000.0f;
	        float sumTemp = 0.0f;

	        for (int i = 0; i < 768; i++)
	        {
	            float t = mlx90640To[i];
	            sumTemp += t;
	            if (t < minTemp) minTemp = t;
	            if (t > maxTemp) maxTemp = t;
	        }

	        thermData.Ta         = Ta;
	        thermData.minTemp    = minTemp;
	        thermData.maxTemp    = maxTemp;
	        thermData.avgTemp    = sumTemp / 768.0f;
	        thermData.centerTemp = mlx90640To[384];   /* pixel tengah */
	    }

	    /* ----- Baca RTC ----- */
	    DS3231_GetTime(&rtcData);

	    static uint32_t lastLog = 0;

	    if (HAL_GetTick() - lastLog > 5000)
	    {
	        lastLog = HAL_GetTick();

	        char line[200];

	        sprintf(line,
	            "%02d/%02d/%04d,%02d:%02d:%02d,"
	            "%.2f,%.2f,%.2f,"
	            "%.2f,%.2f\r\n",

	            rtcData.date,
	            rtcData.month,
	            rtcData.year,
	            rtcData.hour,
	            rtcData.minute,
	            rtcData.second,

	            elecData.phase[0].voltage,
	            elecData.phase[1].voltage,
	            elecData.phase[2].voltage,

	            thermData.avgTemp,
	            thermData.maxTemp
	        );

	        sd.append(&sd, "LOG.CSV", line);
	    }

	    HAL_Delay(100);   /* ritme loop */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
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
  huart2.Init.BaudRate = 115200;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : SPI2_CS_Pin */
  GPIO_InitStruct.Pin = SPI2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI2_CS_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


/**
  * @brief  TX selesai — pindahkan ke mode RX interrupt.
  *         Dipanggil otomatis oleh HAL setelah TX IT selesai.
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        PZEM_IT_TxCpltCallback(&pzem);
    }
}

/**
  * @brief  1 byte RX diterima — kumpulkan ke buffer.
  *         Dipanggil otomatis oleh HAL setiap 1 byte masuk via IT.
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        PZEM_IT_RxCpltCallback(&pzem);
    }
}

/* ========================= */
/* DS3231 Helper Functions   */
/* ========================= */

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
    uint8_t buffer[7];

    if (HAL_I2C_Mem_Read(&hi2c3, DS3231_ADDRESS,
                         0x00, I2C_MEMADD_SIZE_8BIT,
                         buffer, 7, HAL_MAX_DELAY) != HAL_OK)
        return HAL_ERROR;

    rtc->second = BCD_To_Dec(buffer[0]);
    rtc->minute = BCD_To_Dec(buffer[1]);
    rtc->hour   = BCD_To_Dec(buffer[2] & 0x3F);
    rtc->day    = BCD_To_Dec(buffer[3]);
    rtc->date   = BCD_To_Dec(buffer[4]);
    rtc->month  = BCD_To_Dec(buffer[5] & 0x1F);
    rtc->year   = 2000 + BCD_To_Dec(buffer[6]);

    return HAL_OK;
}

static HAL_StatusTypeDef DS3231_SetTime(RTC_DS3231_t *rtc)
{
    uint8_t buffer[7];

    buffer[0] = Dec_To_BCD(rtc->second);
    buffer[1] = Dec_To_BCD(rtc->minute);
    buffer[2] = Dec_To_BCD(rtc->hour);
    buffer[3] = Dec_To_BCD(rtc->day);
    buffer[4] = Dec_To_BCD(rtc->date);
    buffer[5] = Dec_To_BCD(rtc->month);
    buffer[6] = Dec_To_BCD(rtc->year - 2000);

    return HAL_I2C_Mem_Write(&hi2c3, DS3231_ADDRESS,
                             0x00, I2C_MEMADD_SIZE_8BIT,
                             buffer, 7, HAL_MAX_DELAY);
}

static uint8_t SD_mount(void *self)
{
    SDCard_Class *sd = (SDCard_Class*)self;

    if (f_mount(&sd->fs, "", 1) == FR_OK)
    {
        sd->mounted = 1;
        return 1;
    }

    sd->mounted = 0;
    return 0;
}

static uint8_t SD_createFile(void *self, const char *filename)
{
    SDCard_Class *sd = (SDCard_Class*)self;

    if (!sd->mounted) return 0;

    sd->result = f_open(&sd->file, filename, FA_OPEN_ALWAYS | FA_WRITE);

    if (sd->result == FR_OK)
    {
        f_lseek(&sd->file, f_size(&sd->file));

        if (f_size(&sd->file) == 0)
        {
            char header[] =
            "Date,Time,VA,VB,VC,TempAvg,TempMax\r\n";

            f_write(&sd->file, header, strlen(header), &sd->bw);
        }

        f_close(&sd->file);
        return 1;
    }

    return 0;
}

static uint8_t SD_append(void *self, const char *filename, const char *text)
{
    SDCard_Class *sd = (SDCard_Class*)self;

    if (!sd->mounted) return 0;

    if (f_open(&sd->file, filename, FA_OPEN_APPEND | FA_WRITE) == FR_OK)
    {
        f_write(&sd->file, text, strlen(text), &sd->bw);
        f_close(&sd->file);
        return 1;
    }

    return 0;
}

/* ======================================================== */
/* Fungsi bantu                                                */
/* ======================================================== */
static uint16_t CheckVoltageFault(float vA, float vB, float vC)
{
    const float vLow  = VOLT_NOMINAL * (1.0f - LTOL);
    const float vHigh = VOLT_NOMINAL * (1.0f + HTOL);

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
    /* Tempat untuk aksi jika fault terjadi (LED, relay, dll.)
       Saat ini hanya menyimpan di elecData.faultCode */
    (void)code;
}

// FAULT CODE
void FaultCode(uint16_t code)
{
    switch (code) {
        case 0U:
            /* No fault - semua normal */
            break;

        case 1U:
            /* Undervoltage Phase A */
            break;

        case 2U:
            /* Undervoltage Phase B */
            break;

        case 3U:
            /* Undervoltage Phase C */
            break;

        case 4U:
            /* Overvoltage Phase A */
            break;

        case 5U:
            /* Overvoltage Phase B */
            break;

        case 6U:
            /* Overvoltage Phase C */
            break;

        case 404U:
            /* Koneksi PZEM hilang / kabel putus / terbalik */
            break;

        default:
            break;
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
