#include <Arduino.h>
#include <driver/i2s.h>
#include <arduinoFFT.h>
#include <math.h>
#include <string.h>

// --- RS485 ---
#define PIN_RS485_RXD 16
#define PIN_RS485_TXD 17
#define PIN_RS485_DIR 27
#define RS485_BAUD 115200

// --- INMP441 I2S ---
#define PIN_I2S_WS 25
#define PIN_I2S_SCK 26
#define PIN_I2S_SD 33
#define I2S_PORT I2S_NUM_0

#define SAMPLE_RATE 16000
#define FFT_SIZE 1024
#define I2S_BITS 32

#define FFT_BIN_MIN 7
#define FFT_BIN_MAX 32

#define I2S_SHIFT_BITS 8
#define INMP441_SCALE 8388608.0f
#define SEND_INTERVAL_MS 200

// ================================================================
// TLV + CRC16 PROTOCOL LAYER  (menggantikan sendRS485 CSV lama)
// ================================================================
#define TLV_SYNC1 0xAA
#define TLV_SYNC2 0x55
#define TLV_TYPE_MIC_DATA 0x10

#define TLV_TAG_RMS 0x01
#define TLV_TAG_PEAK 0x02
#define TLV_TAG_BAND 0x03
#define TLV_TAG_DB 0x04

#define TLV_VAL_LEN_FLOAT 4U                                 // panjang VALUE tiap TLV float32
#define TLV_PAYLOAD_LEN 24U                                  // 4 TLV x (TAG 1 + LEN 1 + VALUE 4) = 24
#define TLV_FRAME_LEN (2U + 1U + 2U + TLV_PAYLOAD_LEN + 2U)  // = 30

static uint8_t tlv_frame_buf[TLV_FRAME_LEN];

static int32_t i2s_raw_buf[FFT_SIZE];
static double fft_vReal[FFT_SIZE];
static double fft_vImag[FFT_SIZE];

ArduinoFFT<double> FFT(fft_vReal, fft_vImag, FFT_SIZE, SAMPLE_RATE);

HardwareSerial RS485Serial(2);

static uint32_t last_send_ms = 0;

void setupI2S();
void setupRS485();
bool acquireI2SSamples();
void removeDC(double* buf, uint16_t len);
void applyHammingWindow(double* buf, uint16_t len);
float computeRMS(const double* buf, uint16_t len);
float computePeak(const double* buf, uint16_t len);
float computeBandEnergy(const double* vReal, uint16_t bin_min, uint16_t bin_max);
float calculateDecibel(float amplitude);

uint16_t crc16_ccitt(const uint8_t* data, uint16_t len);
uint16_t buildTLVPacket(uint8_t* out, float rms, float peak, float band, float dB);
void sendTLVPacket(float rms, float peak, float band, float dB);

void setup() {
  Serial.begin(115200);
  Serial.println(F("[BOOT] Transformer Acoustic Monitor v1.0"));
  Serial.printf("[DSP]  FFT_SIZE=%d, Fs=%d Hz, Δf=%.3f Hz/bin\n",
                FFT_SIZE, SAMPLE_RATE, (float)SAMPLE_RATE / FFT_SIZE);
  Serial.printf("[BAND] Bin %d (%.1f Hz) — Bin %d (%.1f Hz)\n",
                FFT_BIN_MIN, FFT_BIN_MIN * (float)SAMPLE_RATE / FFT_SIZE,
                FFT_BIN_MAX, FFT_BIN_MAX * (float)SAMPLE_RATE / FFT_SIZE);

  setupRS485();
  setupI2S();

  Serial.println(F("[BOOT] System ready."));
  last_send_ms = millis();
}

void loop() {
  if (!acquireI2SSamples()) {
    Serial.println(F("[WARN] I2S read timeout"));
    return;
  }

  removeDC(fft_vReal, FFT_SIZE);

  float mic_rms = computeRMS(fft_vReal, FFT_SIZE);
  float mic_peak = computePeak(fft_vReal, FFT_SIZE);

  applyHammingWindow(fft_vReal, FFT_SIZE);

  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();

  float mic_band = computeBandEnergy(fft_vReal, FFT_BIN_MIN, FFT_BIN_MAX);

  float dB = calculateDecibel(mic_rms);

  uint32_t now = millis();
  if ((now - last_send_ms) >= SEND_INTERVAL_MS) {
    sendTLVPacket(mic_rms, mic_peak, mic_band, dB);
    last_send_ms = now;

    Serial.printf("[DATA] RMS=%.4f PEAK=%.4f BAND=%.4f dB=%.4f\n",
                  mic_rms, mic_peak, mic_band, dB);
  }
}

void setupRS485() {
  pinMode(PIN_RS485_DIR, OUTPUT);
  digitalWrite(PIN_RS485_DIR, LOW);  // Default RX mode

  RS485Serial.begin(RS485_BAUD, SERIAL_8N1, PIN_RS485_RXD, PIN_RS485_TXD);
  Serial.println(F("[RS485] Initialized (TX=17, RX=16, DIR=27)"));
}

void setupI2S() {
  const i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 128,
    .use_apll = true,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  const i2s_pin_config_t pin_config = {
    .bck_io_num = PIN_I2S_SCK,
    .ws_io_num = PIN_I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = PIN_I2S_SD
  };

  esp_err_t err;

  err = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  if (err != ESP_OK)
    Serial.printf("[I2S] Driver install error: %d\n", err);

  err = i2s_set_pin(I2S_PORT, &pin_config);
  if (err != ESP_OK)
    Serial.printf("[I2S] Pin config error: %d\n", err);

  i2s_zero_dma_buffer(I2S_PORT);

  Serial.println(F("[I2S] INMP441 initialized"));
}

bool acquireI2SSamples() {
  size_t bytes_read = 0;
  const size_t bytes_needed = FFT_SIZE * sizeof(int32_t);
  TickType_t timeout_ticks = pdMS_TO_TICKS(150);

  esp_err_t err = i2s_read(I2S_PORT,
                           i2s_raw_buf,
                           bytes_needed,
                           &bytes_read,
                           timeout_ticks);

  if (err != ESP_OK || bytes_read < bytes_needed)
    return false;

  for (uint16_t i = 0; i < FFT_SIZE; i++) {
    int32_t raw = i2s_raw_buf[i] >> I2S_SHIFT_BITS;
    fft_vReal[i] = (double)raw / INMP441_SCALE;
    fft_vImag[i] = 0.0;
  }

  return true;
}

void removeDC(double* buf, uint16_t len) {
  double sum = 0.0;
  for (uint16_t i = 0; i < len; i++)
    sum += buf[i];

  double mean = sum / len;

  for (uint16_t i = 0; i < len; i++)
    buf[i] -= mean;
}

void applyHammingWindow(double* buf, uint16_t len) {
  const double alpha = 0.54;
  const double beta = 0.46;
  const double two_pi_over_N1 = TWO_PI / (len - 1);

  for (uint16_t i = 0; i < len; i++) {
    double w = alpha - beta * cos(two_pi_over_N1 * i);
    buf[i] *= w;
  }
}

float computeRMS(const double* buf, uint16_t len) {
  double sum_sq = 0.0;
  for (uint16_t i = 0; i < len; i++)
    sum_sq += buf[i] * buf[i];

  return (float)sqrt(sum_sq / len);
}

float computePeak(const double* buf, uint16_t len) {
  double peak = 0.0;
  for (uint16_t i = 0; i < len; i++) {
    double absval = fabs(buf[i]);
    if (absval > peak) peak = absval;
  }
  return (float)peak;
}

float computeBandEnergy(const double* vReal, uint16_t bin_min, uint16_t bin_max) {
  double energy = 0.0;
  for (uint16_t k = bin_min; k <= bin_max; k++) {
    double mag = vReal[k];
    energy += mag * mag;
  }
  return (float)sqrt(energy);
}

float calculateDecibel(float amplitude) {
  if (amplitude < 0.00001f) return 0;  // Mencegah log(0)

  float dbfs = 20.0f * log10f(amplitude);  // Amplitudo referensi = 1 karena out I2S sudah ternormalisasi

  float dbspl = dbfs + 120.0f;  // Penambahan offset
  return dbspl;
}

// ================================================================
// TLV + CRC16 — implementasi
// ================================================================

/**
 * crc16_ccitt()
 *
 * CRC16-CCITT, polynomial 0x1021, initial value 0xFFFF.
 * Dihitung di atas array byte (TYPE + LENGTH + PAYLOAD), TIDAK
 * termasuk SYNC1, SYNC2, maupun CRC itu sendiri.
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
 * buildTLVPacket()
 *
 * Menyusun frame lengkap ke buffer 'out':
 *   [SYNC1][SYNC2][TYPE][LENGTH][PAYLOAD(4xTLV)][CRC_L][CRC_H]
 *
 * PAYLOAD berisi 4 TLV berurutan, masing-masing:
 *   [TAG:1][LEN:1][VALUE:4 byte float32 little-endian]
 *
 * Return: panjang total frame dalam byte (selalu TLV_FRAME_LEN = 30).
 */
uint16_t buildTLVPacket(uint8_t* out, float rms, float peak, float band, float dB) {
  uint16_t idx = 0;

  out[idx++] = TLV_SYNC1;
  out[idx++] = TLV_SYNC2;

  uint16_t crcStartIdx = idx;  // mulai dihitung CRC dari sini (TYPE)

  out[idx++] = TLV_TYPE_MIC_DATA;
  out[idx++] = (uint8_t)(TLV_PAYLOAD_LEN & 0xFF);         // LEN_L = 0x18
  out[idx++] = (uint8_t)((TLV_PAYLOAD_LEN >> 8) & 0xFF);  // LEN_H = 0x00

  // ---- TLV 1: RMS ----
  out[idx++] = TLV_TAG_RMS;
  out[idx++] = TLV_VAL_LEN_FLOAT;
  memcpy(&out[idx], &rms, sizeof(float));
  idx += sizeof(float);

  // ---- TLV 2: PEAK ----
  out[idx++] = TLV_TAG_PEAK;
  out[idx++] = TLV_VAL_LEN_FLOAT;
  memcpy(&out[idx], &peak, sizeof(float));
  idx += sizeof(float);

  // ---- TLV 3: BAND ----
  out[idx++] = TLV_TAG_BAND;
  out[idx++] = TLV_VAL_LEN_FLOAT;
  memcpy(&out[idx], &band, sizeof(float));
  idx += sizeof(float);

  // ---- TLV 4: dB ----
  out[idx++] = TLV_TAG_DB;
  out[idx++] = TLV_VAL_LEN_FLOAT;
  memcpy(&out[idx], &dB, sizeof(float));
  idx += sizeof(float);

  uint16_t crcLen = idx - crcStartIdx;  // = 1(TYPE) + 1(LEN) + 24(PAYLOAD) = 26
  uint16_t crc = crc16_ccitt(&out[crcStartIdx], crcLen);

  out[idx++] = (uint8_t)(crc & 0xFF);         // CRC low byte
  out[idx++] = (uint8_t)((crc >> 8) & 0xFF);  // CRC high byte

  return idx;  // = TLV_FRAME_LEN (30)
}

/**
 * sendTLVPacket()
 *
 * Menggantikan sendRS485() lama. Membangun frame TLV+CRC16, lalu
 * mengirim lewat RS485 dengan kontrol arah DIR pin yang sama persis
 * seperti sebelumnya (HIGH sebelum TX, settle 100us, flush, hitung
 * waktu transmisi untuk delay, lalu LOW kembali).
 */
void sendTLVPacket(float rms, float peak, float band, float dB) {
  uint16_t frameLen = buildTLVPacket(tlv_frame_buf, rms, peak, band, dB);

  digitalWrite(PIN_RS485_DIR, HIGH);
  delayMicroseconds(100);  // settling sebelum TX

  RS485Serial.write(tlv_frame_buf, frameLen);
  RS485Serial.flush();  // tunggu TX buffer kosong

  uint32_t wait_us = ((uint32_t)frameLen * 10 * 1000000UL / RS485_BAUD) + 500;
  delayMicroseconds(wait_us);

  digitalWrite(PIN_RS485_DIR, LOW);
}
