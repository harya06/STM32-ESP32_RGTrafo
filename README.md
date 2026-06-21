# RG Monitoring Trafo 3 Phasa

Industrial-Grade Real-Time 3-Phase Transformer Monitoring System.

Sistem ini dirancang untuk monitoring trafo 3 phasa secara kontinu dengan arsitektur multi-node berbasis STM32 dan ESP32 melalui komunikasi RS485.

---

## 1. System Overview

Fungsi utama sistem:

- Monitoring listrik 3 phasa (Voltage, Current, Power, PF, VA, VAR, Phi, Theta)
- Monitoring thermal menggunakan MLX90640
- Monitoring akustik trafo (FFT – INMP441)
- Logging data ke SD Card (format CSV)
- Telemetri RS485 multi-node
- HMI TFT 320x240 real-time

Arsitektur sistem:

ESP32 (Acoustic Node)  
        │  
        │ RS485  
        ▼  
STM32F411 (Main Controller)  
        │  
        │ RS485  
        ▼  
ESP32 S3 (TFT HMI)

Sensor:
- PZEM 3-Phase
- MLX90640 Thermal Camera
- RTC DS3231
- SD Card Logger

---

## 2. TFT Configuration

Library yang digunakan:

TFT_eSPI by Bodmer

File `User_Setup.h` HARUS dikonfigurasi sesuai berikut:

Driver dan Resolusi:

#define ILI9341_DRIVER
#define TFT_WIDTH  240
#define TFT_HEIGHT 320
#define TFT_RGB_ORDER TFT_BGR
#define TFT_INVERSION_OFF

SPI Configuration:

#define SPI_FREQUENCY 27000000
#define USE_HSPI_PORT

Pin Mapping :

#define TFT_MOSI 11
#define TFT_MISO 13
#define TFT_SCLK 12
#define TFT_CS   4
#define TFT_DC   5
#define TFT_RST  6
#define TFT_BL   7

Perubahan konfigurasi pin atau SPI port dapat menyebabkan:
- Display tidak inisialisasi
- Warna salah
- Rendering corrupt
- Konflik SPI

---

## 3. ESP32 S3 – TFT HMI Wiring

| Fungsi | ESP32 S3 | TFT |
|--------|----------|-----|
| SCLK   | GPIO12   | SCK |
| MOSI   | GPIO11   | SDA |
| MISO   | GPIO13   | MISO |
| CS     | GPIO4    | CS |
| DC     | GPIO5    | DC |
| RST    | GPIO6    | RST |
| BL     | GPIO7    | LED |
| VCC    | 3V3      | VCC |
| GND    | GND      | GND |

---

## 4. ESP32 WROOM 32U – Acoustic Node

INMP441 → I2S:

| INMP441 | ESP32 |
|----------|--------|
| VDD      | 3V3 |
| GND      | GND |
| SCK      | GPIO26 |
| WS       | GPIO25 |
| SD       | GPIO33 |
| L/R      | GND |

RS485:

| Fungsi | Pin |
|--------|-----|
| TX     | GPIO17 |
| RX     | GPIO16 |
| DE/RE  | GPIO27 |

---

## 5. STM32F411 – Main Controller

USART Configuration:

USART1 → RS485 ke ESP32 S3  
- PA9  → TX (DI)  
- PA10 → RX (RO)  
- PB1  → RS485 DIR  

USART6 → RS485 ke ESP32 Mic  
- PA11 → TX (DI)  
- PA12 → RX (RO)  
- PB5  → RS485 DIR  

USART2 → PZEM 3-Phase  
- PA2 / PA3  

I2C:

I2C1 → MLX90640  
- PB6 SCL  
- PB7 SDA  

I2C3 → RTC DS3231  
- PA8 SCL  
- PB4 SDA  

Power:
- VDD → 3.3V
- VSS → GND

---

## 6. RS485 Protocol Format

STM32 → ESP32 HMI:

DATE:dd/mm/yyyy,
TIME:hh:mm:ss,
VA:x.x,IA:x.xx,PA:x.x,...
VB:x.x,...
VC:x.x,...
TAVG:x.x,TMAX:x.x,...
MLX_VALID:n,
MIC_RMS:x.xxxx,
MIC_PEAK:x.xxxx,
MIC_BAND:x.xxxx,
MIC_VALID:n

ESP32 Mic → STM32:

RMS:x.xxxx,PEAK:x.xxxx,BAND:x.xxxx

---

## 7. Build Environment

STM32:
- STM32CubeIDE
- HAL Driver
- FATFS enabled
- UART Interrupt Mode
- UART ReceiveToIdle enabled
- Independent Watchdog enabled

ESP32:
- Arduino IDE / PlatformIO
- TFT_eSPI (custom User_Setup.h)
- ArduinoFFT

---

## 8. Industrial Design Notes

- Non-blocking architecture
- State-machine MLX90640 recovery
- UART interrupt-based communication
- RS485 direction control hardware managed
- Sensor timeout validation (MIC & MLX)
- SD logging interval: 5s
- Telemetry interval: 1s
- Watchdog active (STM32)

System dirancang untuk operasi kontinu 24/7 pada lingkungan industri.

---

## 9. Critical Requirement

JANGAN mengubah:

- SPI port (harus HSPI)
- Pin mapping TFT
- Resolusi TFT
- Driver ILI9341

Konfigurasi berbeda akan menyebabkan sistem HMI tidak bekerja.

---

## 10. Version

RG Transformer Monitoring System  
Rev: Industrial Prototype  
Target: Continuous Monitoring Deployment

---

RG Engineering Development