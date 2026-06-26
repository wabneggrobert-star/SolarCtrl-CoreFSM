#include "feature_sdcard.h"

#include "config.h"

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>

namespace {
  SPIClass sdSpi(VSPI);
  bool g_available = false;

  bool mountCard() {
    pinMode(PIN_SD_CS, OUTPUT);
    digitalWrite(PIN_SD_CS, HIGH);

    sdSpi.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);
    delay(5);

    bool ok = SD.begin(PIN_SD_CS, sdSpi);
    return ok;
  }
}

namespace SDCard {

bool begin() {
  Serial.println("Initialisiere SD...");
  g_available = mountCard();

  Serial.print("SD.begin: ");
  Serial.println(g_available ? "OK" : "FEHLER");

  if (g_available) {
    uint64_t sizeMb = SD.cardSize() / (1024ULL * 1024ULL);
    Serial.print("SD Kartengroesse [MB]: ");
    Serial.println((unsigned long)sizeMb);
  }

  return g_available;
}

bool available() {
  return g_available;
}

}