#pragma once
#include "app_types.h"

namespace HeatSourcesMax {

  bool begin(AppContext& ctx);

  void startCycle(AppContext& ctx);
  void process(AppContext& ctx);
  bool cycleComplete(const AppContext& ctx);

  // Blocking one-shot read. This is the single authoritative MAX31865
  // read routine used by the normal runtime path and the debug endpoint.
  bool readChannelNow(AppContext& ctx, MaxChannel ch);
  void readAllNow(AppContext& ctx);

  // Blocking debug read for browser endpoint /api/max-debug.
  // Does not change application configuration; it only toggles MAX31865 CS lines
  // and optionally triggers one-shot conversions for diagnosis.
  String debugJson(AppContext& ctx, bool runOneShot);

}