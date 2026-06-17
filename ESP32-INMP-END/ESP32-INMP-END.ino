#include <Arduino.h>
#include <driver/i2s.h>
#include <arduinoFFT.h>
#include <math.h>

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
void sendRS485(float rms, float peak, float band);

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

  uint32_t now = millis();
  if ((now - last_send_ms) >= SEND_INTERVAL_MS) {
    sendRS485(mic_rms, mic_peak, mic_band);
    last_send_ms = now;

    Serial.printf("[DATA] RMS=%.4f PEAK=%.4f BAND=%.4f\n",
                  mic_rms, mic_peak, mic_band);
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

void sendRS485(float rms, float peak, float band)
{
    char msg[80];
    snprintf(msg, sizeof(msg),
             "RMS:%.4f,PEAK:%.4f,BAND:%.4f\r\n",
             rms, peak, band);

    digitalWrite(PIN_RS485_DIR, HIGH);
    delayMicroseconds(100);  // settling sebelum TX
    
    RS485Serial.print(msg);
    RS485Serial.flush();     // tunggu TX buffer kosong

    uint32_t tx_len = strlen(msg);
    uint32_t wait_us = (tx_len * 10 * 1000000UL / RS485_BAUD) + 500;
    delayMicroseconds(wait_us);
    
    digitalWrite(PIN_RS485_DIR, LOW);
}