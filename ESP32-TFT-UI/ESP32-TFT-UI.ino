#include <Arduino.h>
#include <FS.h>
#include <SPIFFS.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// ================= PIN =================
#define RXD2 18
#define TXD2 17
#define DIR_PIN 15

// ================= SERVER =================
const char* SERVER_URL = "https://script.google.com/macros/s/AKfycbwmCigQPD9HdRObk-po-6jl-2PNZRsqsQPwmnfJ_7QZO_BYFkLYy4qHDe-uGQCBp_iT/exec";
const char* API_KEY = "RG_TRAFO_RHNAZ_26";

#define SEND_INTERVAL_MS 10 * 1000
unsigned long lastSend = 0;

// ================= WARNA =================
#define C_BG 0x0862
#define C_PANEL 0x0884
#define C_HDR 0x0883
#define C_BORDER 0x1948
#define C_ACC_BLUE 0x0294
#define C_ACC_GRN 0x0400
#define C_TEXT 0xFFFF
#define C_MUTED 0x8C51
#define C_OK 0x07E0
#define C_WARN 0xFD20
#define C_CRIT 0xF800
#define C_INFO 0x05FD
#define C_CYAN 0x07FF
#define C_TITLE 0x469C
#define C_PHASE 0xFD80

// ================= LAYOUT =================
#define HDR_H 26
#define BODY_Y (HDR_H)
#define BODY_H 193
#define FOOT_Y (BODY_Y + BODY_H)  // = 219
#define FOOT_H 21                 // = 219+21 = 240 pas

#define COL_PARAM 8
#define COL_A 132
#define COL_B 202
#define COL_C 275

#define ROW_H 17
#define ROW_START (BODY_Y + 26)

// Footer x-positions
#define FOOT_X_UP 3
#define FOOT_X_RMS 72
#define FOOT_X_BAND 148
#define FOOT_X_TMP 240

// ================= OBJECT =================
TFT_eSPI tft;
HardwareSerial RS485Serial(2);

// ================= DATA =================
String rxBuf = "";
String dateStr = "--/--/----";
String timeStr = "--:--:--";

float VA_v = 0, VB_v = 0, VC_v = 0;
float IA = 0, IB = 0, IC = 0;
float PA = 0, PB = 0, PC = 0;
float FA = 0, FB = 0, FC = 0;
float PFA = 0, PFB = 0, PFC = 0;
float VAA = 0, VAB = 0, VAC = 0;
float VARA = 0, VARB = 0, VARC = 0;
float PHIA = 0, PHIB = 0, PHIC = 0;
float THTA = 0, THTB = 0, THTC = 0;

float TMAX = 0;   // MLX
float TMAX2 = 0;  // MLX2

float micRMS = 0.0f;
float micBAND = 0.0f;
float dB = 0.0f;
uint8_t micValid = 0;

unsigned long lastRx = 0;
unsigned long upSec = 0;
unsigned long lastSec = 0;
bool blinkOn = false;
unsigned long lastBlink = 0;

// ================= PROTOKOL TLV + CRC16 (gateway STM32 -> TFT) =================
#define GW_TLV_SYNC1 0xAA
#define GW_TLV_SYNC2 0x99

#define GW_TLV_TYPE_TELEMETRY 0x20
#define GW_TLV_TYPE_THERMAL1 0x30
#define GW_TLV_TYPE_THERMAL2 0x31

#define GW_TAG_DATETIME 0x1F

#define GW_TAG_VA 0x01
#define GW_TAG_IA 0x02
#define GW_TAG_PA 0x03
#define GW_TAG_FA 0x04
#define GW_TAG_PFA 0x05
#define GW_TAG_VAA 0x06
#define GW_TAG_VARA 0x07
#define GW_TAG_PHIA 0x08
#define GW_TAG_THTA 0x09

#define GW_TAG_VB 0x0A
#define GW_TAG_IB 0x0B
#define GW_TAG_PB 0x0C
#define GW_TAG_FB 0x0D
#define GW_TAG_PFB 0x0E
#define GW_TAG_VAB 0x0F
#define GW_TAG_VARB 0x10
#define GW_TAG_PHIB 0x11
#define GW_TAG_THTB 0x12

#define GW_TAG_VC 0x13
#define GW_TAG_IC 0x14
#define GW_TAG_PC 0x15
#define GW_TAG_FC 0x16
#define GW_TAG_PFC 0x17
#define GW_TAG_VAC 0x18
#define GW_TAG_VARC 0x19
#define GW_TAG_PHIC 0x1A
#define GW_TAG_THTC 0x1B

#define GW_TAG_TMAX 0x1C
#define GW_TAG_TMAX2 0x1D

#define GW_TAG_MIC_RMS 0x40
#define GW_TAG_MIC_BAND 0x41
#define GW_TAG_MIC_DB 0x42
#define GW_TAG_MIC_VALID 0x43

#define GW_TAG_THERMAL_GRID 0x50 /* satu-satunya TAG dengan LEN 2-byte */

#define THERMAL_PIXEL_COUNT 768
#define GW_MAX_PAYLOAD_LEN 1600 /* cukup untuk payload thermal (1542) */

enum GwTLVState {
  GW_WAIT_SYNC1,
  GW_WAIT_SYNC2,
  GW_WAIT_TYPE,
  GW_WAIT_LEN_L,
  GW_WAIT_LEN_H,
  GW_WAIT_PAYLOAD,
  GW_WAIT_CRC_L,
  GW_WAIT_CRC_H
};

GwTLVState gwState = GW_WAIT_SYNC1;
uint8_t gwType = 0;
uint16_t gwExpectLen = 0;
uint16_t gwRxIdx = 0;
uint8_t gwPayload[GW_MAX_PAYLOAD_LEN];
uint16_t gwCrcRecv = 0;

// ---- DEBUG: counter & buffer kerja CRC (dipindah ke global agar TIDAK
// dialokasikan ulang di stack tiap frame -- sebelumnya crcBuf[3+1600]
// dibuat sbg local array di setiap event WAIT_CRC_H, boros stack tanpa
// perlu karena ukurannya selalu sama dgn gwPayload+header). ----
static uint8_t  gwCrcWorkBuf[3 + GW_MAX_PAYLOAD_LEN];
static uint32_t dbgByteCount   = 0;   // total byte mentah masuk dari RS485
static uint32_t dbgSyncCount   = 0;   // berapa kali SYNC1+SYNC2 valid ketemu
static uint32_t dbgFrameOkCount   = 0;  // frame lolos CRC
static uint32_t dbgFrameBadCount  = 0;  // frame CRC mismatch
static unsigned long dbgLastReport = 0;

uint16_t thermalGrid[THERMAL_PIXEL_COUNT];
uint16_t thermalGrid2[THERMAL_PIXEL_COUNT];  // MLX2 (BAA, I2C2)
volatile bool thermalFrameReady = false;
volatile bool thermalFrame2Ready = false;
bool thermalGrid2Valid = false;  // true setelah minimal 1 frame TYPE 0x02 diterima
unsigned long lastThermalRx = 0;
unsigned long lastThermalRx2 = 0;

// ================= FREERTOS: KIRIM DATA NON-BLOCKING =================
// Tiap SEND_INTERVAL_MS, loop() nyusun snapshot SEMUA data (listrik +
// thermal) ke struct ini, dorong ke queue, lalu task terpisah (core 0)
// yang nyusun JSON & HTTPClient.POST() — loop() utama (baca UART + TFT)
// di core 1 gak pernah ke-block nungguin jaringan.
typedef struct {
  float vA, vB, vC;
  float iA, iB, iC;
  float pfA, pfB, pfC;  // faktor daya
  float pA, pB, pC;     // W / daya aktif
  float sA, sB, sC;     // VA / daya semu
  float qA, qB, qC;     // VAR / daya reaktif
  float tmax;
  uint16_t thermal[THERMAL_PIXEL_COUNT];
  uint16_t thermal2[THERMAL_PIXEL_COUNT];
  bool thermal2Valid;  // false = belum ada frame MLX2 valid
} TelemetryPacket_t;

QueueHandle_t telemetryQueue = NULL;
TaskHandle_t telemetryTaskHandle = NULL;

// ================= COLOR LOGIC =================
uint16_t cVolt(float v) {
  if (v == 0) return C_MUTED;
  if (v < 198 || v > 231) return C_CRIT;
  if (v < 207 || v > 228) return C_WARN;
  return C_OK;
}
uint16_t cFreq(float f) {
  if (f == 0) return C_MUTED;
  if (f < 49.5f || f > 50.5f) return C_WARN;
  return C_OK;
}
uint16_t cPF(float p) {
  if (p == 0) return C_MUTED;
  if (p < 0.8f) return C_CRIT;
  if (p < 0.9f) return C_WARN;
  return C_OK;
}
uint16_t cTemp(float t) {
  if (t >= 120) return C_CRIT;
  if (t >= 80) return C_WARN;
  return C_OK;
}
uint16_t cBand(float b) {
  if (b > 0.15f) return C_CRIT;
  if (b > 0.05f) return C_WARN;
  return C_OK;
}

/**
 * crc16_ccitt()
 *
 * Identik dengan implementasi di ESP32-INMP441 (Tahap 1) dan STM32
 * (Tahap 2) — poly 0x1021, init 0xFFFF, dihitung atas byte array.
 */
uint16_t crc16_ccitt(const uint8_t* data, uint16_t len) {
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

/**
 * decodeTelemetryPayload()
 *
 * Mem-parse payload TYPE_TELEMETRY: serangkaian TLV kecil, masing²
 * [TAG:1][LEN:1][VALUE:LEN byte]. Mengisi langsung variabel global
 * yang sudah ada (VA_v, IA, ..., micValid) — variabel & nama TIDAK
 * diubah dari versi parseData() lama.
 */
void decodeTelemetryPayload(const uint8_t* payload, uint16_t len) {
  uint16_t p = 0;

  while (p + 2 <= len) {
    uint8_t tag = payload[p];
    uint8_t tlvLen = payload[p + 1];

    if (p + 2 + tlvLen > len) break;  // TLV korup/terpotong, hentikan parsing aman

    const uint8_t* val = &payload[p + 2];

    switch (tag) {
      case GW_TAG_DATETIME:
        {
          if (tlvLen == 7) {
            char buf[24];
            uint16_t year = (uint16_t)val[2] | ((uint16_t)val[3] << 8);
            snprintf(buf, sizeof(buf), "%02d/%02d/%04d", val[0], val[1], year);
            dateStr = String(buf);
            snprintf(buf, sizeof(buf), "%02d:%02d:%02d", val[4], val[5], val[6]);
            timeStr = String(buf);
          }
          break;
        }

      case GW_TAG_VA: memcpy(&VA_v, val, 4); break;
      case GW_TAG_IA: memcpy(&IA, val, 4); break;
      case GW_TAG_PA: memcpy(&PA, val, 4); break;
      case GW_TAG_FA: memcpy(&FA, val, 4); break;
      case GW_TAG_PFA: memcpy(&PFA, val, 4); break;
      case GW_TAG_VAA: memcpy(&VAA, val, 4); break;
      case GW_TAG_VARA: memcpy(&VARA, val, 4); break;
      case GW_TAG_PHIA: memcpy(&PHIA, val, 4); break;
      case GW_TAG_THTA: memcpy(&THTA, val, 4); break;

      case GW_TAG_VB: memcpy(&VB_v, val, 4); break;
      case GW_TAG_IB: memcpy(&IB, val, 4); break;
      case GW_TAG_PB: memcpy(&PB, val, 4); break;
      case GW_TAG_FB: memcpy(&FB, val, 4); break;
      case GW_TAG_PFB: memcpy(&PFB, val, 4); break;
      case GW_TAG_VAB: memcpy(&VAB, val, 4); break;
      case GW_TAG_VARB: memcpy(&VARB, val, 4); break;
      case GW_TAG_PHIB: memcpy(&PHIB, val, 4); break;
      case GW_TAG_THTB: memcpy(&THTB, val, 4); break;

      case GW_TAG_VC: memcpy(&VC_v, val, 4); break;
      case GW_TAG_IC: memcpy(&IC, val, 4); break;
      case GW_TAG_PC: memcpy(&PC, val, 4); break;
      case GW_TAG_FC: memcpy(&FC, val, 4); break;
      case GW_TAG_PFC: memcpy(&PFC, val, 4); break;
      case GW_TAG_VAC: memcpy(&VAC, val, 4); break;
      case GW_TAG_VARC: memcpy(&VARC, val, 4); break;
      case GW_TAG_PHIC: memcpy(&PHIC, val, 4); break;
      case GW_TAG_THTC: memcpy(&THTC, val, 4); break;

      case GW_TAG_TMAX: memcpy(&TMAX, val, 4); break;
      case GW_TAG_TMAX2: memcpy(&TMAX2, val, 4); break;

      case GW_TAG_MIC_RMS: memcpy(&micRMS, val, 4); break;
      case GW_TAG_MIC_BAND: memcpy(&micBAND, val, 4); break;
      case GW_TAG_MIC_DB: memcpy(&dB, val, 4); break;
      case GW_TAG_MIC_VALID: micValid = val[0]; break;

      default:
        // TAG tidak dikenal, lewati saja (forward-compatible)
        break;
    }

    p += (2 + tlvLen);
  }

  lastRx = millis();
}

/**
 * decodeThermalPayload()
 *
 * Mem-parse payload TYPE_THERMAL1/2: satu TLV besar TAG_THERMAL_GRID
 * dengan LEN 2-byte (bukan 1-byte seperti TLV telemetry), karena
 * payload-nya 1536 byte. Mengisi thermalGrid/thermalGrid2 — variabel
 * dan representasi (uint16, suhu x100) TIDAK diubah dari versi lama.
 */
void decodeThermalPayload(const uint8_t* payload, uint16_t len, uint8_t frameType) {
  if (len < 3) return;
  uint8_t tag = payload[0];
  if (tag != GW_TAG_THERMAL_GRID) return;

  uint16_t valLen = (uint16_t)payload[1] | ((uint16_t)payload[2] << 8);
  if (valLen != (THERMAL_PIXEL_COUNT * 2) || (3U + valLen) > len) return;

  const uint8_t* grid = &payload[3];
  uint16_t* targetGrid = (frameType == GW_TLV_TYPE_THERMAL1) ? thermalGrid : thermalGrid2;

  for (int i = 0; i < THERMAL_PIXEL_COUNT; i++) {
    targetGrid[i] = (uint16_t)grid[i * 2] | ((uint16_t)grid[i * 2 + 1] << 8);
  }

  if (frameType == GW_TLV_TYPE_THERMAL1) {
    thermalFrameReady = true;
    lastThermalRx = millis();
  } else {
    thermalFrame2Ready = true;
    thermalGrid2Valid = true;
    lastThermalRx2 = millis();
  }
}

/**
 * processGwByte()
 *
 * State machine 8-state generik untuk frame gateway STM32->TFT.
 * Menggantikan processBinaryByte() lama TOTAL. Menangani TYPE
 * telemetry maupun thermal lewat field TYPE yang sama, panjang
 * payload variabel (LENGTH 2-byte, bukan fixed seperti versi lama).
 */
void processGwByte(uint8_t b) {
  dbgByteCount++;

  switch (gwState) {

    case GW_WAIT_SYNC1:
      gwState = (b == GW_TLV_SYNC1) ? GW_WAIT_SYNC2 : GW_WAIT_SYNC1;
      break;

    case GW_WAIT_SYNC2:
      if (b == GW_TLV_SYNC2) {
        gwState = GW_WAIT_TYPE;
        dbgSyncCount++;
      } else if (b != GW_TLV_SYNC1) {
        gwState = GW_WAIT_SYNC1;
      }
      // jika b == SYNC1 lagi, tetap di WAIT_SYNC2 (toleransi sync dobel)
      break;

    case GW_WAIT_TYPE:
      gwType = b;
      gwState = GW_WAIT_LEN_L;
      break;

    case GW_WAIT_LEN_L:
      gwExpectLen = b;
      gwState = GW_WAIT_LEN_H;
      break;

    case GW_WAIT_LEN_H:
      gwExpectLen |= ((uint16_t)b << 8);
      if (gwExpectLen == 0 || gwExpectLen > GW_MAX_PAYLOAD_LEN) {
        gwState = GW_WAIT_SYNC1;  // panjang tidak wajar -> noise, resync
      } else {
        gwRxIdx = 0;
        gwState = GW_WAIT_PAYLOAD;
      }
      break;

    case GW_WAIT_PAYLOAD:
      // Guard tambahan: walau gwExpectLen sudah divalidasi <= GW_MAX_PAYLOAD_LEN
      // di atas, cek ulang gwRxIdx sebelum tulis supaya tidak mungkin
      // overrun gwPayload[] kalau suatu saat batas berubah.
      if (gwRxIdx < GW_MAX_PAYLOAD_LEN) {
        gwPayload[gwRxIdx++] = b;
      } else {
        gwState = GW_WAIT_SYNC1;  // tidak boleh terjadi, resync demi aman
        break;
      }
      if (gwRxIdx >= gwExpectLen) {
        gwState = GW_WAIT_CRC_L;
      }
      break;

    case GW_WAIT_CRC_L:
      gwCrcRecv = b;
      gwState = GW_WAIT_CRC_H;
      break;

    case GW_WAIT_CRC_H:
      {
        gwCrcRecv |= ((uint16_t)b << 8);

        // Hitung ulang CRC atas TYPE + LEN(2) + PAYLOAD.
        // gwCrcWorkBuf adalah buffer GLOBAL (lihat deklarasi di atas),
        // bukan local array -- sebelumnya dialokasikan ulang di stack
        // tiap frame (3+1600 byte), tidak perlu & boros stack runtime.
        gwCrcWorkBuf[0] = gwType;
        gwCrcWorkBuf[1] = (uint8_t)(gwExpectLen & 0xFF);
        gwCrcWorkBuf[2] = (uint8_t)((gwExpectLen >> 8) & 0xFF);
        memcpy(&gwCrcWorkBuf[3], gwPayload, gwExpectLen);

        uint16_t crcCalc = crc16_ccitt(gwCrcWorkBuf, 3 + gwExpectLen);

        if (crcCalc == gwCrcRecv) {
          dbgFrameOkCount++;
          if (gwType == GW_TLV_TYPE_TELEMETRY) {
            decodeTelemetryPayload(gwPayload, gwExpectLen);
            updateUI();
          } else if (gwType == GW_TLV_TYPE_THERMAL1 || gwType == GW_TLV_TYPE_THERMAL2) {
            decodeThermalPayload(gwPayload, gwExpectLen, gwType);
          }
        } else {
          dbgFrameBadCount++;
          Serial.printf("[GW] CRC MISMATCH type=0x%02X len=%u calc=0x%04X recv=0x%04X\n",
                        gwType, gwExpectLen, crcCalc, gwCrcRecv);
        }

        gwState = GW_WAIT_SYNC1;
        break;
      }
  }
}

// ================= DRAW STATIC =================
void drawStatic() {
  tft.fillScreen(C_BG);

  // ── Header ──
  tft.fillRect(0, 0, 320, HDR_H, C_HDR);
  tft.fillRect(0, 0, 3, HDR_H, C_ACC_BLUE);

  tft.setTextColor(C_TEXT, C_HDR);
  tft.setCursor(8, 3);
  tft.print("RG TRANSFORMER MONITOR");
  tft.setTextColor(C_MUTED, C_HDR);
  tft.setCursor(8, 15);
  tft.print("REAL-TIME ELECTRICAL MONITOR");

  // ── Body ──
  tft.fillRect(0, BODY_Y, 320, BODY_H, C_PANEL);
  tft.drawFastHLine(0, BODY_Y, 320, C_BORDER);

  tft.setTextColor(C_TITLE, C_PANEL);
  tft.setCursor(8, BODY_Y + 3);
  tft.print("ELECTRICAL - 3 PHASE");

  tft.setTextColor(C_PHASE, C_PANEL);
  tft.setCursor(COL_A - 18, BODY_Y + 15);
  tft.print("PHASE A");
  tft.setCursor(COL_B - 18, BODY_Y + 15);
  tft.print("PHASE B");
  tft.setCursor(COL_C - 18, BODY_Y + 15);
  tft.print("PHASE C");

  const char* label[] = {
    "Voltage (V)", "Current (A)", "Power (W)",
    "Freq (Hz)", "Power Factor",
    "VA (VA)", "VAR", "Phi", "Theta"
  };
  for (int i = 0; i < 9; i++) {
    tft.setTextColor(C_MUTED, C_PANEL);
    tft.setCursor(COL_PARAM, ROW_START + i * ROW_H);
    tft.print(label[i]);
  }

  // ── Footer ──
  tft.fillRect(0, FOOT_Y, 320, FOOT_H, C_PANEL);
  tft.fillRect(0, FOOT_Y, 3, FOOT_H, C_ACC_GRN);
  tft.drawFastHLine(0, FOOT_Y, 320, C_BORDER);

  tft.drawFastVLine(FOOT_X_RMS - 4, FOOT_Y + 3, FOOT_H - 6, C_BORDER);
  tft.drawFastVLine(FOOT_X_BAND - 4, FOOT_Y + 3, FOOT_H - 6, C_BORDER);
  tft.drawFastVLine(FOOT_X_TMP - 4, FOOT_Y + 3, FOOT_H - 6, C_BORDER);

  tft.setTextColor(C_MUTED, C_PANEL);
  tft.setCursor(FOOT_X_UP, FOOT_Y + 3);
  tft.print("UPTIME");
  tft.setCursor(FOOT_X_RMS, FOOT_Y + 3);
  tft.print("RMS");
  tft.setCursor(FOOT_X_BAND, FOOT_Y + 3);
  tft.print("BAND");
  tft.setCursor(FOOT_X_TMP, FOOT_Y + 3);
  tft.print("TEMP");
}

// ================= HELPER drawVal =================
void drawVal(float v, uint8_t d, int x, int y, uint16_t fg) {
  tft.fillRect(x - 45, y, 55, 10, C_PANEL);
  tft.setTextColor(fg, C_PANEL);
  tft.setTextDatum(TR_DATUM);
  tft.drawFloat(v, d, x, y);
  tft.setTextDatum(TL_DATUM);
}

// ================= UPDATE HEADER =================
void updateHeader() {
  tft.fillRect(138, 1, 180, HDR_H - 2, C_HDR);

  bool link = (millis() - lastRx < 5000);

  uint16_t c_stm = link ? (blinkOn ? C_OK : 0x0420) : C_CRIT;
  tft.fillCircle(144, 7, 3, c_stm);
  tft.setTextColor(C_OK, C_HDR);
  tft.setCursor(150, 3);
  tft.print("STM32");

  uint16_t c_esp = micValid ? (blinkOn ? C_CYAN : 0x0210) : C_CRIT;
  tft.fillCircle(144, 19, 3, c_esp);
  tft.setTextColor(C_CYAN, C_HDR);
  tft.setCursor(150, 15);
  tft.print("ESP32(2)");

  tft.setTextColor(C_OK, C_HDR);
  tft.setTextDatum(TR_DATUM);
  tft.drawString(dateStr, 318, 3);
  tft.drawString(timeStr, 318, 15);
  tft.setTextDatum(TL_DATUM);
}

// ================= UPDATE FOOTER =================
void updateFooter() {
  tft.fillRect(4, FOOT_Y + 11, 315, FOOT_H - 12, C_PANEL);

  char buf[16];

  unsigned long h = upSec / 3600;
  unsigned long m = (upSec % 3600) / 60;
  unsigned long s = upSec % 60;
  sprintf(buf, "%02lu:%02lu:%02lu", h, m, s);
  tft.setTextColor(C_INFO, C_PANEL);
  tft.setCursor(FOOT_X_UP, FOOT_Y + 12);
  tft.print(buf);

  tft.setTextColor(C_CYAN, C_PANEL);
  tft.setCursor(FOOT_X_RMS, FOOT_Y + 12);
  tft.printf("%.4f", micRMS);

  tft.setTextColor(cBand(micBAND), C_PANEL);
  tft.setCursor(FOOT_X_BAND, FOOT_Y + 12);
  if (!micValid) {
    tft.print("NO SIG");
  } else {
    tft.printf("%.4f", micBAND);
  }

  sprintf(buf, "%.1f C", TMAX);
  tft.setTextColor(cTemp(TMAX), C_PANEL);
  tft.setCursor(FOOT_X_TMP, FOOT_Y + 12);
  tft.print(buf);
}

// ================= UPDATE UI =================
void updateUI() {
#define ROW(y) (ROW_START + (y)*ROW_H)

  drawVal(VA_v, 1, COL_A + 8, ROW(0), cVolt(VA_v));
  drawVal(VB_v, 1, COL_B + 8, ROW(0), cVolt(VB_v));
  drawVal(VC_v, 1, COL_C + 8, ROW(0), cVolt(VC_v));

  drawVal(IA, 2, COL_A + 8, ROW(1), C_TEXT);
  drawVal(IB, 2, COL_B + 8, ROW(1), C_TEXT);
  drawVal(IC, 2, COL_C + 8, ROW(1), C_TEXT);

  drawVal(PA, 0, COL_A + 8, ROW(2), C_TEXT);
  drawVal(PB, 0, COL_B + 8, ROW(2), C_TEXT);
  drawVal(PC, 0, COL_C + 8, ROW(2), C_TEXT);

  drawVal(FA, 2, COL_A + 8, ROW(3), cFreq(FA));
  drawVal(FB, 2, COL_B + 8, ROW(3), cFreq(FB));
  drawVal(FC, 2, COL_C + 8, ROW(3), cFreq(FC));

  drawVal(PFA, 2, COL_A + 8, ROW(4), cPF(PFA));
  drawVal(PFB, 2, COL_B + 8, ROW(4), cPF(PFB));
  drawVal(PFC, 2, COL_C + 8, ROW(4), cPF(PFC));

  drawVal(VAA, 0, COL_A + 8, ROW(5), C_TEXT);
  drawVal(VAB, 0, COL_B + 8, ROW(5), C_TEXT);
  drawVal(VAC, 0, COL_C + 8, ROW(5), C_TEXT);

  drawVal(VARA, 0, COL_A + 8, ROW(6), C_TEXT);
  drawVal(VARB, 0, COL_B + 8, ROW(6), C_TEXT);
  drawVal(VARC, 0, COL_C + 8, ROW(6), C_TEXT);

  drawVal(PHIA, 1, COL_A + 8, ROW(7), C_TEXT);
  drawVal(PHIB, 1, COL_B + 8, ROW(7), C_TEXT);
  drawVal(PHIC, 1, COL_C + 8, ROW(7), C_TEXT);

  drawVal(THTA, 1, COL_A + 8, ROW(8), C_TEXT);
  drawVal(THTB, 1, COL_B + 8, ROW(8), C_TEXT);
  drawVal(THTC, 1, COL_C + 8, ROW(8), C_TEXT);

  updateFooter();
}

// ================= BUILD JSON & POST (jalan di task FreeRTOS, core 0) =================
void sendTelemetryJSON(TelemetryPacket_t& pkt) {
  if (WiFi.status() != WL_CONNECTED) return;

  String body;
  body.reserve(4500);  // 768 angka thermal (~5 char + koma) + field listrik lain
  body = "{";
  body += "\"api_key\":\"" + String(API_KEY) + "\",";
  body += "\"ip\":\"" + WiFi.localIP().toString() + "\",";
  body += "\"rssi\":" + String(WiFi.RSSI()) + ",";
  body += "\"firmware\":\"2.0-FAST\",";

  // Tegangan
  body += "\"v_r\":" + String(pkt.vA, 1) + ",";
  body += "\"v_s\":" + String(pkt.vB, 1) + ",";
  body += "\"v_t\":" + String(pkt.vC, 1) + ",";

  // Arus
  body += "\"i_r\":" + String(pkt.iA, 2) + ",";
  body += "\"i_s\":" + String(pkt.iB, 2) + ",";
  body += "\"i_t\":" + String(pkt.iC, 2) + ",";

  // Power Factor
  body += "\"pf_r\":" + String(pkt.pfA, 3) + ",";
  body += "\"pf_s\":" + String(pkt.pfB, 3) + ",";
  body += "\"pf_t\":" + String(pkt.pfC, 3) + ",";

  // Daya Aktif (P)
  body += "\"p_r\":" + String(pkt.pA, 2) + ",";
  body += "\"p_s\":" + String(pkt.pB, 2) + ",";
  body += "\"p_t\":" + String(pkt.pC, 2) + ",";

  // Daya Semu (S = VA)
  body += "\"s_r\":" + String(pkt.sA, 2) + ",";
  body += "\"s_s\":" + String(pkt.sB, 2) + ",";
  body += "\"s_t\":" + String(pkt.sC, 2) + ",";

  // Daya Reaktif (Q = VAR)
  body += "\"q_r\":" + String(pkt.qA, 2) + ",";
  body += "\"q_s\":" + String(pkt.qB, 2) + ",";
  body += "\"q_t\":" + String(pkt.qC, 2) + ",";

  // Thermal MLX1
  body += "\"thermal\":\"";
  for (int i = 0; i < THERMAL_PIXEL_COUNT; i++) {
    body += String(pkt.thermal[i]);
    if (i < THERMAL_PIXEL_COUNT - 1) body += ",";
  }
  body += "\",";
  body += "\"thermal_scale\":100,";

  // Thermal MLX2 — hanya kirim kalau sudah pernah ada frame valid
  if (pkt.thermal2Valid) {
    Serial.println("Thermal2 Valid");
    body += "\"thermal2\":\"";
    for (int i = 0; i < THERMAL_PIXEL_COUNT; i++) {
      body += String(pkt.thermal2[i]);
      if (i < THERMAL_PIXEL_COUNT - 1) body += ",";
    }
    body += "\",";
    body += "\"thermal2_scale\":100,";
  }
  body += "\"tmax\":" + String(pkt.tmax, 1) + ",";

  body += "\"noise_db\":" + String(dB, 2);

  body += "}";

  HTTPClient http;
  http.begin(SERVER_URL);
  http.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
  http.addHeader("Content-Type", "application/json");
  http.setTimeout(15000);

  int httpCode = http.POST(body);
  if (httpCode > 0) {
    // Serial.printf("[HTTP] OK Code: %d\n", httpCode);
    // Serial.println("[SERVER] " + http.getString());
  } else {
    // Serial.printf("[HTTP] Gagal: %s\n", http.errorToString(httpCode).c_str());
  }
  http.end();
}

// Task FreeRTOS khusus kirim data — nunggu snapshot masuk queue, lalu
// nyusun JSON & POST. Gak pernah nyentuh TFT/UART, jadi loop() utama
// (di core 1) gak pernah ke-block walau server/WiFi lagi lambat.
void telemetryUploadTask(void* param) {
  TelemetryPacket_t pkt;
  for (;;) {
    if (xQueueReceive(telemetryQueue, &pkt, portMAX_DELAY) == pdTRUE) {
      sendTelemetryJSON(pkt);
    }
  }
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);

  // TFT & RS485
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(DIR_PIN, LOW);

  RS485Serial.setRxBufferSize(4096);  // headroom buat frame biner thermal (1536 byte payload)
  RS485Serial.begin(115200, SERIAL_8N1, RXD2, TXD2);

  tft.init();
  tft.setRotation(1);
  tft.setTextSize(1);

  drawStatic();
  updateHeader();
  updateUI();

  // WiFiManager — portal "ESP32_R20" / "Ruliff20"
  //
  // PENTING: autoConnect() TANPA timeout akan BLOCKING SELAMANYA kalau
  // tidak ada yang connect ke portal & submit WiFi -- selama itu,
  // setup() belum return, jadi loop() (baca RS485 + update TFT) TIDAK
  // PERNAH jalan walau STM32 sudah kirim data. Ini sebelumnya bikin
  // tampak seperti "RS485 putus", padahal RS485 belum sempat diproses
  // sama sekali. Makanya:
  //   1) Diberi setConfigPortalTimeout supaya portal tidak menunggu
  //      selamanya kalau tidak ada yang setup WiFi.
  //   2) Jika gagal connect, TETAP LANJUT (bukan ESP.restart()), supaya
  //      TFT & RS485 (fungsi inti monitoring) tetap jalan secara lokal
  //      walau tanpa WiFi -- hanya upload ke cloud yang di-skip.
  WiFiManager wm;
  wm.setConfigPortalTimeout(60);  // portal nyala max 60 detik, lalu lanjut tanpa WiFi
  bool wifiOk = wm.autoConnect("ESP32_R20", "Ruliff20");

  if (wifiOk) {
    Serial.println("[WiFi] Terhubung! IP: " + WiFi.localIP().toString());
  } else {
    Serial.println("[WiFi] Gagal connect / portal timeout -- lanjut TANPA WiFi.");
    Serial.println("[WiFi] RS485 & TFT tetap jalan normal, upload cloud di-skip.");
    WiFi.mode(WIFI_OFF);  // matikan radio supaya tidak terus mencoba & tidak panas
  }

  // ── Setup queue + task upload (FreeRTOS, core 0) ──────────────
  // loop() utama (baca UART RS485 + update TFT) tetap di core 1,
  // jadi HTTPClient.POST() yang blocking gak pernah bikin UI nge-lag.
  telemetryQueue = xQueueCreate(2, sizeof(TelemetryPacket_t));  // buffer 2 snapshot

  xTaskCreatePinnedToCore(
    telemetryUploadTask,  // fungsi task
    "TelemetryUpload",    // nama (buat debug)
    12288,                // stack size (byte) — HTTPClient + JSON String ~4-5KB butuh lega
    NULL,                 // parameter
    1,                    // priority rendah-menengah
    &telemetryTaskHandle,
    0  // pin ke core 0
  );
}

// ================= LOOP =================
void loop() {
  // Baca RS485
  while (RS485Serial.available()) {
    uint8_t c = (uint8_t)RS485Serial.read();
    processGwByte(c);
  }

  unsigned long now = millis();

  // DEBUG: laporan tiap 2 detik -- byte mentah masuk, sync ketemu,
  // frame OK/BAD. Kalau dbgByteCount tetap 0 terus, artinya ESP32
  // sungguh tidak menerima APA PUN secara fisik di RXD2 (bukan soal
  // parsing) -- cek wiring/baud/transceiver, bukan kode parser.
  if (now - dbgLastReport >= 2000) {
    dbgLastReport = now;
    Serial.printf("[GW][DBG] bytesIn=%lu syncFound=%lu frameOK=%lu frameBAD=%lu state=%d\n",
                  (unsigned long)dbgByteCount, (unsigned long)dbgSyncCount,
                  (unsigned long)dbgFrameOkCount, (unsigned long)dbgFrameBadCount,
                  (int)gwState);
  }

  // Uptime & footer tiap 1 detik
  if (now - lastSec >= 1000) {
    lastSec = now;
    upSec++;
    updateFooter();
  }

  // Blink indicator tiap 800ms
  if (now - lastBlink >= 800) {
    lastBlink = now;
    blinkOn = !blinkOn;
    updateHeader();
  }

  // Kirim data ke server tiap SEND_INTERVAL_MS — non-blocking.
  // loop() cuma nyusun snapshot & dorong ke queue (timeout 0), POST
  // aslinya dikerjain telemetryUploadTask() di core lain.
  if (now - lastSend >= SEND_INTERVAL_MS) {
    lastSend = now;

    TelemetryPacket_t pkt;
    pkt.vA = VA_v;
    pkt.vB = VB_v;
    pkt.vC = VC_v;
    pkt.iA = IA;
    pkt.iB = IB;
    pkt.iC = IC;
    pkt.pfA = PFA;
    pkt.pfB = PFB;
    pkt.pfC = PFC;
    pkt.pA = PA;
    pkt.pB = PB;
    pkt.pC = PC;
    pkt.sA = VAA;
    pkt.sB = VAB;
    pkt.sC = VAC;
    pkt.qA = VARA;
    pkt.qB = VARB;
    pkt.qC = VARC;
    pkt.tmax = TMAX;
    memcpy(pkt.thermal, thermalGrid, sizeof(pkt.thermal));
    memcpy(pkt.thermal2, thermalGrid2, sizeof(pkt.thermal2));
    pkt.thermal2Valid = thermalGrid2Valid;

    xQueueSend(telemetryQueue, &pkt, 0);  // queue penuh -> drop diam-diam, ada snapshot baru 5 detik
  }
}