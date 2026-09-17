#pragma once
#include "app_types.h"

namespace EnergyMeter {
  void begin(AppContext& ctx);
  void reconfigure(AppContext& ctx);
  void process(AppContext& ctx);
  void saveRuntime(const AppContext& ctx);
  uint8_t gpioForFeedbackInput(uint8_t inputIndex);
}
