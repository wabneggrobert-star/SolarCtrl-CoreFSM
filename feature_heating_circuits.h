#pragma once
#include "app_types.h"

namespace HeatingCircuits {
  void begin(AppContext& ctx);
  void process(AppContext& ctx);
  void allOff(AppContext& ctx);
}
