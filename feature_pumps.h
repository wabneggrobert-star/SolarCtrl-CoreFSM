#pragma once
#include "app_types.h"

namespace Pumps {

  void begin(AppContext& ctx);
  void process(AppContext& ctx);
  void allOff(AppContext& ctx);
  void safetyAllOff(AppContext& ctx);
  void safetyAllOffExceptOven(AppContext& ctx);
  bool safetyForceRunForSource(AppContext& ctx, HeatSourceRole sourceRole, float pwmPercent);
  bool safetyForceHeatDumpForSource(AppContext& ctx, HeatSourceRole sourceRole, float pwmPercent);
  bool safetyForceNightCooling(AppContext& ctx, float pwmPercent);


}