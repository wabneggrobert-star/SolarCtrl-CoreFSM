#pragma once

#include "app_types.h"

namespace PumpRouting {

  struct RouteResult {
    bool active = false;
    uint8_t targetIndex = PIN_UNUSED;

    Ds18Role sinkRole = Ds18Role::NONE;
    float sinkC = NAN;
    bool sinkValid = false;

    float diffC = NAN;
    float targetDiff = NAN;
    float hysteresis = NAN;

    uint8_t valveIndex = PIN_UNUSED;
    OutputRef valveOutput;
    uint8_t valveRelayIndex = PIN_UNUSED; // Legacy-/Diagnosefeld fuer Relaisventile
    bool hasValve = false;

    bool valveMoving = false;
    uint32_t valveMoveRemainingMs = 0;
  };

  void begin(AppContext& ctx);
  void closeAllTargets(AppContext& ctx, uint8_t pumpIndex);

  RouteResult resolve(AppContext& ctx, uint8_t pumpIndex, float sourceC, bool sourceValid);
  RouteResult resolveHeatDumpCoolestTarget(AppContext& ctx, uint8_t pumpIndex, float sourceC, bool sourceValid);
}
