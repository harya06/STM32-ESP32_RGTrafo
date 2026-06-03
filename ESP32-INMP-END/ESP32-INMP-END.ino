#include <Arduino.h>
#include <driver/i2s.h>

// =========================
// INMP441 PIN CONFIG
// =========================
#define I2S_WS      25   // LRCLK / WS
#define I2S_SCK     26   // BCLK
#define I2S_SD      33   // DOUT

#define I2S_PORT    I2S_NUM_0

// =========================
// AUDIO CONFIG
// =========================
#define SAMPLE_RATE     16000
#define BUFFER_LEN      64

int32_t samples[BUFFER_LEN];

void setupI2S()
{
    // Konfigurasi I2S
    const i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,
        .dma_buf_len = 64,
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0
    };

    // Pin I2S
    const i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_SCK,
        .ws_io_num = I2S_WS,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = I2S_SD
    };

    // Install driver
    i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);

    // Set pin
    i2s_set_pin(I2S_PORT, &pin_config);

    // Clear DMA
    i2s_zero_dma_buffer(I2S_PORT);
}

void setup()
{
    Serial.begin(115200);

    delay(1000);

    Serial.println();
    Serial.println("INMP441 TEST START");

    setupI2S();
}

void loop()
{
    size_t bytes_read = 0;

    // Baca data microphone
    i2s_read(
        I2S_PORT,
        &samples,
        sizeof(samples),
        &bytes_read,
        portMAX_DELAY
    );

    int samples_read = bytes_read / 4;

    // Hitung amplitudo rata-rata
    long sum = 0;

    for (int i = 0; i < samples_read; i++)
    {
        int32_t sample = samples[i];

        // INMP441 hasil 24-bit dalam frame 32-bit
        sample = sample >> 14;

        sum += abs(sample);
    }-
    long average = sum / samples_read;

    // Output ke Serial Plotter
    Serial.println(average);

    delay(10);
}