#pragma once
#include <Arduino.h>
#include "app_types.h"

namespace Pcf8574Io {

  bool begin();

  bool writeState(uint8_t value);
  uint8_t currentState();
  uint8_t readBack();

  bool setBitHigh(uint8_t bitIndex);
  bool setBitLow(uint8_t bitIndex);

  bool deselectMaxCs();
  bool selectMax(MaxChannel ch);
  // Diagnostic helper: set all PCF outputs HIGH, then pull exactly one bit LOW.
  // Used only by MAX31865 bus diagnostics.
  bool selectSingleBitLow(uint8_t bitIndex);
  bool deselectAllCs();
}
