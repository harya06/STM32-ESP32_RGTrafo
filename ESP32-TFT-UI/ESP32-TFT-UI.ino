#include <TFT_eSPI.h>
#include <SPI.h>

// ==========================
// TFT PIN DEFINE
// ==========================

#define TFT_MOSI 11
#define TFT_MISO 13
#define TFT_SCLK 12

#define TFT_CS    4
#define TFT_DC    5
#define TFT_RST   6
#define TFT_BL    7

#define USE_HSPI_PORT

// ==========================
// TFT OBJECT
// ==========================

TFT_eSPI tft = TFT_eSPI();

void setup()
{
    Serial.begin(115200);
    Serial.println("START");

    // Backlight ON
    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, HIGH);

    // Init TFT
    tft.init();

    // Rotasi layar
    tft.setRotation(1);

    // Test awal
    tft.fillScreen(TFT_BLACK);
    delay(1000);
}

void loop()
{
    // RED
    tft.fillScreen(TFT_RED);
    Serial.println("RED");
    delay(1000);

    // GREEN
    tft.fillScreen(TFT_GREEN);
    Serial.println("GREEN");
    delay(1000);

    // BLUE
    tft.fillScreen(TFT_BLUE);
    Serial.println("BLUE");
    delay(1000);

    // WHITE
    tft.fillScreen(TFT_WHITE);
    Serial.println("WHITE");
    delay(1000);

    // YELLOW
    tft.fillScreen(TFT_YELLOW);
    Serial.println("YELLOW");
    delay(1000);

    // CYAN
    tft.fillScreen(TFT_CYAN);
    Serial.println("CYAN");
    delay(1000);

    // MAGENTA
    tft.fillScreen(TFT_MAGENTA);
    Serial.println("MAGENTA");
    delay(1000);

    // BLACK
    tft.fillScreen(TFT_BLACK);
    Serial.println("BLACK");
    delay(1000);
}