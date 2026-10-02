#include "feature_io_expander_pcf8574.h"
#include "config.h"

#include <Arduino.h>
#include <Wire.h>

#include "feature_build_flags.h"
namespace {
  uint8_t g_state = 0xFF;
  bool g_started = false;

  bool applyState() {
    Wire.beginTransmission(PCF8574_MAX_ADDR);
    Wire.write(g_state);
    return (Wire.endTransmission() == 0);
  }

  bool validBit(uint8_t bitIndex) {
    return bitIndex < 8;
  }

  void setMaxCsHigh() {
    g_state |= (1u << PCF_BIT_MAX1_CS);
    g_state |= (1u << PCF_BIT_MAX2_CS);
    g_state |= (1u << PCF_BIT_MAX3_CS);
    g_state |= (1u << PCF_BIT_MAX4_CS);
  }
}

namespace Pcf8574Io {

bool begin() {
  DBG_PRINTLN("Pcf8574Io::begin MAX-PCF");
  DBG_FLUSH();

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

  Wire.beginTransmission(PCF8574_MAX_ADDR);
  uint8_t errMax = Wire.endTransmission();

  DBG_PRINT("MAX-PCF 0x");
  DBG_PRINT(PCF8574_MAX_ADDR, HEX);
  DBG_PRINT(" -> ");
  DBG_PRINTLN(errMax);
  DBG_FLUSH();

  if (errMax != 0) {
    g_started = false;
    return false;
  }

  g_state = 0xFF;
  setMaxCsHigh();

  g_started = applyState();

  DBG_PRINT("MAX-PCF write -> ");
  DBG_PRINTLN(g_started ? "OK" : "FEHLER");
  DBG_FLUSH();

  return g_started;
}

bool writeState(uint8_t value) {
  if (!g_started) return false;
  g_state = value;
  return applyState();
}

uint8_t currentState() {
  return g_state;
}

uint8_t readBack() {
  if (!g_started) return 0xFF;

  Wire.requestFrom((int)PCF8574_MAX_ADDR, 1);
  if (Wire.available()) {
    return Wire.read();
  }

  return 0xFF;
}

bool setBitHigh(uint8_t bitIndex) {
  if (!g_started || !validBit(bitIndex)) return false;
  g_state |= (1u << bitIndex);
  return applyState();
}

bool setBitLow(uint8_t bitIndex) {
  if (!g_started || !validBit(bitIndex)) return false;
  g_state &= ~(1u << bitIndex);
  return applyState();
}

bool deselectMaxCs() {
  if (!g_started) return false;

  setMaxCsHigh();
  return applyState();
}

bool selectMax(MaxChannel ch) {
  if (!g_started) return false;

  // Erst alle MAX-CS inaktiv HIGH
  setMaxCsHigh();

  switch (ch) {
    case MaxChannel::CH1:
      g_state &= ~(1u << PCF_BIT_MAX1_CS);
      break;

    case MaxChannel::CH2:
      g_state &= ~(1u << PCF_BIT_MAX2_CS);
      break;

    case MaxChannel::CH3:
      g_state &= ~(1u << PCF_BIT_MAX3_CS);
      break;

    case MaxChannel::CH4:
      g_state &= ~(1u << PCF_BIT_MAX4_CS);
      break;

    default:
      return false;
  }

  return applyState();
}

bool deselectAllCs() {
  return deselectMaxCs();
}

}