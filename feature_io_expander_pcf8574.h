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
  bool deselectAllCs();
}
