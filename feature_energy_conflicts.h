#pragma once

#include "app_types.h"

namespace EnergyConflicts {

  void begin(AppContext& ctx);
  void clear(AppContext& ctx);

  bool canActivateRoute(
    AppContext& ctx,
    uint8_t pumpIndex,
    HeatSourceRole sourceRole,
    Ds18Role sinkRole,
    uint8_t pumpRelayIndex,
    uint8_t valveRelayIndex
  );

  void reserveRoute(
    AppContext& ctx,
    uint8_t pumpIndex,
    HeatSourceRole sourceRole,
    Ds18Role sinkRole,
    uint8_t pumpRelayIndex,
    uint8_t valveRelayIndex
  );

}
