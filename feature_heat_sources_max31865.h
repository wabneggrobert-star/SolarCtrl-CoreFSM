#pragma once
#include "app_types.h"

namespace HeatSourcesMax {

  bool begin(AppContext& ctx);

  void startCycle(AppContext& ctx);
  void process(AppContext& ctx);
  bool cycleComplete(const AppContext& ctx);

}