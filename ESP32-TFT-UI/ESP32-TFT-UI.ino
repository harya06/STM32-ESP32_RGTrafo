/*
 * ============================================================
 *  RG TRANSFORMER MONITOR — ESP32 + TFT 320×240
 *  Real-time 3-phase electrical monitor via RS485
 * ============================================================
 */

#include <TFT_eSPI.h>
#include <SPI.h>

// ── Pin ──────────────────────────────────────────────────────
#define RXD2     18
#define TXD2     17
#define DIR_PIN  15

// ── Warna (RGB565) ───────────────────────────────────────────
#define C_BG       0x0862
#define C_PANEL    0x0884
#define C_HDR      0x0883
#define C_BORDER   0x1948
#define C_ACC_BLUE 0x0294
#define C_ACC_GRN  0x0400
#define C_TEXT     0xFFFF
#define C_MUTED    0x8C51
#define C_OK       0x07E0
#define C_WARN     0xFD20
#define C_CRIT     0xF800
#define C_INFO     0x05FD
#define C_CYAN     0x07FF
#define C_TITLE    0x469C
#define C_PHASE    0xFD80

// ── Layout (320×240) ─────────────────────────────────────────
//
//  HDR  : y=0,   h=26  → y=25
//  BODY : y=26,  h=193 → y=218
//  FOOT : y=219, h=21  → y=239 (tepat batas layar)
//
//  Body: Title(14) + PhaseHdr(12) + 9×Row(17) = 179px ✓
//
#define HDR_H      26
#define BODY_Y     HDR_H
#define BODY_H     193
#define FOOT_Y     (BODY_Y + BODY_H)   // 219
#define FOOT_H     21                  // 219+21 = 240

#define COL_PARAM  8
#define COL_A      132
#define COL_B      202
#define COL_C      275
#define ROW_H      17
#define ROW_START  (BODY_Y + 26)       // title 14px + phase hdr 12px

#define FOOT_X_UP   3
#define FOOT_X_RMS  72
#define FOOT_X_BAND 148
#define FOOT_X_TMP  240

// ── Objects ──────────────────────────────────────────────────
TFT_eSPI       tft;
HardwareSerial RS485Serial(2);

// ── State ────────────────────────────────────────────────────
String rxBuf  = "";
String dateStr = "--/--/----";
String timeStr = "--:--:--";

// Electrical data — 3 phase
float VA_v=0, VB_v=0, VC_v=0;
float IA=0,   IB=0,   IC=0;
float PA=0,   PB=0,   PC=0;
float FA=0,   FB=0,   FC=0;
float PFA=0,  PFB=0,  PFC=0;
float VAA=0,  VAB=0,  VAC=0;
float VARA=0, VARB=0, VARC=0;
float PHIA=0, PHIB=0, PHIC=0;
float THTA=0, THTB=0, THTC=0;

// Sensor data
float   TMAX     = 0;
float   micRMS   = 0.0f;
float   micBAND  = 0.0f;
uint8_t micValid = 0;
uint8_t mlxValid = 0;

// Timing
unsigned long lastRx   = 0;
unsigned long upSec    = 0;
unsigned long lastSec  = 0;
unsigned long lastBlink = 0;
bool          blinkOn  = false;

// ============================================================
//  COLOR LOGIC
// ============================================================
uint16_t cVolt(float v) {
  if (v == 0)              return C_MUTED;
  if (v < 198 || v > 231) return C_CRIT;
  if (v < 207 || v > 228) return C_WARN;
  return C_OK;
}

uint16_t cFreq(float f) {
  if (f == 0)                    return C_MUTED;
  if (f < 49.5f || f > 50.5f)   return C_WARN;
  return C_OK;
}

uint16_t cPF(float p) {
  if (p == 0)    return C_MUTED;
  if (p < 0.8f)  return C_CRIT;
  if (p < 0.9f)  return C_WARN;
  return C_OK;
}

uint16_t cTemp(float t) {
  if (t >= 70) return C_CRIT;
  if (t >= 55) return C_WARN;
  return C_OK;
}

uint16_t cBand(float b) {
  if (b > 0.15f) return C_CRIT;
  if (b > 0.05f) return C_WARN;
  return C_OK;
}

// ============================================================
//  PARSER
// ============================================================

// Ambil nilai setelah key k, batas koma atau akhir string.
String getVal(String s, String k) {
  int start = s.indexOf(k);
  if (start < 0) return "";

  start += k.length();

  int end = s.indexOf(',', start);
  if (end < 0) {
    end = s.indexOf('\r', start);
    if (end < 0) end = s.indexOf('\n', start);
    if (end < 0) end = s.length();
  }

  String val = s.substring(start, end);
  val.trim();
  return val;
}

void parseData(String s) {
  if (s.length() == 0) return;

  int idx, endIdx;

  // DATE & TIME
  idx = s.indexOf("DATE:");
  if (idx >= 0) {
    idx   += 5;
    endIdx = s.indexOf(',', idx);
    if (endIdx > idx) dateStr = s.substring(idx, endIdx);
  }

  idx = s.indexOf("TIME:");
  if (idx >= 0) {
    idx   += 5;
    endIdx = s.indexOf(',', idx);
    if (endIdx > idx) timeStr = s.substring(idx, endIdx);
  }

  // Electrical
  VA_v = getVal(s, "VA:").toFloat();
  VB_v = getVal(s, "VB:").toFloat();
  VC_v = getVal(s, "VC:").toFloat();

  IA = getVal(s, "IA:").toFloat();
  IB = getVal(s, "IB:").toFloat();
  IC = getVal(s, "IC:").toFloat();

  PA = getVal(s, "PA:").toFloat();
  PB = getVal(s, "PB:").toFloat();
  PC = getVal(s, "PC:").toFloat();

  FA = getVal(s, "FA:").toFloat();
  FB = getVal(s, "FB:").toFloat();
  FC = getVal(s, "FC:").toFloat();

  PFA = getVal(s, "PFA:").toFloat();
  PFB = getVal(s, "PFB:").toFloat();
  PFC = getVal(s, "PFC:").toFloat();

  VAA = getVal(s, "VAA:").toFloat();
  VAB = getVal(s, "VAB:").toFloat();
  VAC = getVal(s, "VAC:").toFloat();

  VARA = getVal(s, "VARA:").toFloat();
  VARB = getVal(s, "VARB:").toFloat();
  VARC = getVal(s, "VARC:").toFloat();

  PHIA = getVal(s, "PHIA:").toFloat();
  PHIB = getVal(s, "PHIB:").toFloat();
  PHIC = getVal(s, "PHIC:").toFloat();

  THTA = getVal(s, "THETAA:").toFloat();
  THTB = getVal(s, "THETAB:").toFloat();
  THTC = getVal(s, "THETAC:").toFloat();

  // Sensor
  idx = s.indexOf("TMAX:");
  if (idx >= 0) {
    idx   += 5;
    endIdx = s.indexOf(',', idx);
    if (endIdx > idx) TMAX = s.substring(idx, endIdx).toFloat();
  }

  idx = s.indexOf("MLX_VALID:");
  if (idx >= 0) {
    idx   += 10;
    endIdx = s.indexOf(',', idx);
    if (endIdx > idx) mlxValid = (uint8_t)s.substring(idx, endIdx).toInt();
  }

  micRMS   = getVal(s, "MIC_RMS:").toFloat();
  micBAND  = getVal(s, "MIC_BAND:").toFloat();
  micValid = (uint8_t)getVal(s, "MIC_VALID:").toInt();

  lastRx = millis();
}

// ============================================================
//  DRAW STATIC (dipanggil sekali di setup)
// ============================================================
void drawStatic() {
  tft.fillScreen(C_BG);

  // Header
  tft.fillRect(0, 0, 320, HDR_H, C_HDR);
  tft.fillRect(0, 0, 3,   HDR_H, C_ACC_BLUE);
  tft.setTextColor(C_TEXT,  C_HDR); tft.setCursor(8,  3); tft.print("RG TRANSFORMER MONITOR");
  tft.setTextColor(C_MUTED, C_HDR); tft.setCursor(8, 15); tft.print("REAL-TIME ELECTRICAL MONITOR");

  // Body
  tft.fillRect(0, BODY_Y, 320, BODY_H, C_PANEL);
  tft.drawFastHLine(0, BODY_Y, 320, C_BORDER);

  tft.setTextColor(C_TITLE, C_PANEL);
  tft.setCursor(8, BODY_Y + 3);
  tft.print("ELECTRICAL - 3 PHASE");

  tft.setTextColor(C_PHASE, C_PANEL);
  tft.setCursor(COL_A - 18, BODY_Y + 15); tft.print("PHASE A");
  tft.setCursor(COL_B - 18, BODY_Y + 15); tft.print("PHASE B");
  tft.setCursor(COL_C - 18, BODY_Y + 15); tft.print("PHASE C");

  const char* rowLabel[] = {
    "Voltage (V)", "Current (A)", "Power (W)",
    "Freq (Hz)",   "Power Factor",
    "VA (VA)",     "VAR",         "Phi",  "Theta"
  };
  tft.setTextColor(C_MUTED, C_PANEL);
  for (int i = 0; i < 9; i++) {
    tft.setCursor(COL_PARAM, ROW_START + i * ROW_H);
    tft.print(rowLabel[i]);
  }

  // Footer
  tft.fillRect(0, FOOT_Y, 320, FOOT_H, C_PANEL);
  tft.fillRect(0, FOOT_Y, 3,   FOOT_H, C_ACC_GRN);
  tft.drawFastHLine(0, FOOT_Y, 320, C_BORDER);

  tft.drawFastVLine(FOOT_X_RMS  - 4, FOOT_Y + 3, FOOT_H - 6, C_BORDER);
  tft.drawFastVLine(FOOT_X_BAND - 4, FOOT_Y + 3, FOOT_H - 6, C_BORDER);
  tft.drawFastVLine(FOOT_X_TMP  - 4, FOOT_Y + 3, FOOT_H - 6, C_BORDER);

  tft.setTextColor(C_MUTED, C_PANEL);
  tft.setCursor(FOOT_X_UP,   FOOT_Y + 3); tft.print("UPTIME");
  tft.setCursor(FOOT_X_RMS,  FOOT_Y + 3); tft.print("RMS");
  tft.setCursor(FOOT_X_BAND, FOOT_Y + 3); tft.print("BAND");
  tft.setCursor(FOOT_X_TMP,  FOOT_Y + 3); tft.print("TEMP");
}

// ============================================================
//  HELPER
// ============================================================
void drawVal(float v, uint8_t decimals, int x, int y, uint16_t fg) {
  tft.fillRect(x - 45, y, 55, 10, C_PANEL);
  tft.setTextColor(fg, C_PANEL);
  tft.setTextDatum(TR_DATUM);
  tft.drawFloat(v, decimals, x, y);
  tft.setTextDatum(TL_DATUM);
}

// ============================================================
//  UPDATE HEADER
// ============================================================
void updateHeader() {
  tft.fillRect(138, 1, 180, HDR_H - 2, C_HDR);

  bool linked = (millis() - lastRx < 5000);

  // STM32 link indicator
  uint16_t cStm = linked ? (blinkOn ? C_OK : 0x0420) : C_CRIT;
  tft.fillCircle(144, 7, 3, cStm);
  tft.setTextColor(C_OK,   C_HDR); tft.setCursor(150,  3); tft.print("STM32");

  // ESP32(2) sensor indicator
  uint16_t cEsp = micValid ? (blinkOn ? C_CYAN : 0x0210) : C_CRIT;
  tft.fillCircle(144, 19, 3, cEsp);
  tft.setTextColor(C_CYAN, C_HDR); tft.setCursor(150, 15); tft.print("ESP32(2)");

  // Date & Time
  tft.setTextColor(C_OK, C_HDR);
  tft.setTextDatum(TR_DATUM);
  tft.drawString(dateStr, 318,  3);
  tft.drawString(timeStr, 318, 15);
  tft.setTextDatum(TL_DATUM);
}

// ============================================================
//  UPDATE FOOTER
// ============================================================
void updateFooter() {
  tft.fillRect(4, FOOT_Y + 11, 315, FOOT_H - 12, C_PANEL);

  char buf[16];

  // Uptime
  sprintf(buf, "%02lu:%02lu:%02lu", upSec / 3600, (upSec % 3600) / 60, upSec % 60);
  tft.setTextColor(C_INFO, C_PANEL);
  tft.setCursor(FOOT_X_UP, FOOT_Y + 12);
  tft.print(buf);

  // RMS
  tft.setTextColor(C_CYAN, C_PANEL);
  tft.setCursor(FOOT_X_RMS, FOOT_Y + 12);
  tft.printf("%.4f", micRMS);

  // Band
  tft.setCursor(FOOT_X_BAND, FOOT_Y + 12);
  if (!micValid) {
    tft.setTextColor(C_CRIT, C_PANEL);
    tft.print("NO SIG");
  } else {
    tft.setTextColor(cBand(micBAND), C_PANEL);
    tft.printf("%.4f", micBAND);
  }

  // Max Temp
  tft.setCursor(FOOT_X_TMP, FOOT_Y + 12);
  if (!mlxValid) {
    tft.setTextColor(C_CRIT, C_PANEL);
    tft.print("NO SIG");
  } else {
    sprintf(buf, "%.1f C", TMAX);
    tft.setTextColor(cTemp(TMAX), C_PANEL);
    tft.print(buf);
  }
}

// ============================================================
//  UPDATE UI (data rows)
// ============================================================
void updateUI() {
#define ROW(i) (ROW_START + (i) * ROW_H)

  drawVal(VA_v, 1, COL_A+8, ROW(0), cVolt(VA_v));
  drawVal(VB_v, 1, COL_B+8, ROW(0), cVolt(VB_v));
  drawVal(VC_v, 1, COL_C+8, ROW(0), cVolt(VC_v));

  drawVal(IA, 2, COL_A+8, ROW(1), C_TEXT);
  drawVal(IB, 2, COL_B+8, ROW(1), C_TEXT);
  drawVal(IC, 2, COL_C+8, ROW(1), C_TEXT);

  drawVal(PA, 0, COL_A+8, ROW(2), C_TEXT);
  drawVal(PB, 0, COL_B+8, ROW(2), C_TEXT);
  drawVal(PC, 0, COL_C+8, ROW(2), C_TEXT);

  drawVal(FA, 2, COL_A+8, ROW(3), cFreq(FA));
  drawVal(FB, 2, COL_B+8, ROW(3), cFreq(FB));
  drawVal(FC, 2, COL_C+8, ROW(3), cFreq(FC));

  drawVal(PFA, 2, COL_A+8, ROW(4), cPF(PFA));
  drawVal(PFB, 2, COL_B+8, ROW(4), cPF(PFB));
  drawVal(PFC, 2, COL_C+8, ROW(4), cPF(PFC));

  drawVal(VAA, 0, COL_A+8, ROW(5), C_TEXT);
  drawVal(VAB, 0, COL_B+8, ROW(5), C_TEXT);
  drawVal(VAC, 0, COL_C+8, ROW(5), C_TEXT);

  drawVal(VARA, 0, COL_A+8, ROW(6), C_TEXT);
  drawVal(VARB, 0, COL_B+8, ROW(6), C_TEXT);
  drawVal(VARC, 0, COL_C+8, ROW(6), C_TEXT);

  drawVal(PHIA, 1, COL_A+8, ROW(7), C_TEXT);
  drawVal(PHIB, 1, COL_B+8, ROW(7), C_TEXT);
  drawVal(PHIC, 1, COL_C+8, ROW(7), C_TEXT);

  drawVal(THTA, 1, COL_A+8, ROW(8), C_TEXT);
  drawVal(THTB, 1, COL_B+8, ROW(8), C_TEXT);
  drawVal(THTC, 1, COL_C+8, ROW(8), C_TEXT);

  updateFooter();
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);

  pinMode(TFT_BL,  OUTPUT); digitalWrite(TFT_BL,  HIGH);
  pinMode(DIR_PIN, OUTPUT); digitalWrite(DIR_PIN, LOW);

  RS485Serial.begin(115200, SERIAL_8N1, RXD2, TXD2);

  tft.init();
  tft.setRotation(1);
  tft.setTextSize(1);

  drawStatic();
  updateHeader();
  updateUI();
}

// ============================================================
//  LOOP
// ============================================================
void loop() {
  // ── Baca RS485 ──────────────────────────────────────────
  while (RS485Serial.available()) {
    char c = RS485Serial.read();

    if (c == '\n') {
      if (rxBuf.length() > 10) {
        // Buang garbage di depan buffer — anchor ke "DATE:"
        int anchor = rxBuf.indexOf("DATE:");
        if (anchor < 0) anchor = rxBuf.indexOf("VA:");
        if (anchor < 0) anchor = 0;

        parseData(rxBuf.substring(anchor));
        updateUI();
      }
      rxBuf = "";
    } else if (c != '\r') {
      // Terima hanya karakter printable ASCII
      if (c >= 0x20 && c <= 0x7E) {
        rxBuf += c;
        if (rxBuf.length() > 900) rxBuf = "";  // guard overflow
      }
    }
  }

  unsigned long now = millis();

  // ── Uptime tiap detik ───────────────────────────────────
  if (now - lastSec >= 1000) {
    lastSec = now;
    upSec++;
    updateFooter();
  }

  // ── Blink indicator tiap 800ms ──────────────────────────
  if (now - lastBlink >= 800) {
    lastBlink = now;
    blinkOn   = !blinkOn;
    updateHeader();
  }
}
