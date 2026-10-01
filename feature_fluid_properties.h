#pragma once
#include "app_types.h"

namespace FluidProperties {
  struct Result {
    float glycolConcentrationPercent = 0.0f;
    float waterConcentrationPercent = 100.0f;
    float volumetricHeatCapacityWhPerLiterK = 1.163f;
    bool freezePointInRange = true;
    bool safetyCompatible = true;
    bool safetyMarginMet = true;
  };

  Result evaluate(const ConfigData& cfg, float meanFluidTemperatureC = 40.0f);
  float glycolConcentrationPercent(const ConfigData& cfg);
  float volumetricHeatCapacityWhPerLiterK(const ConfigData& cfg, float meanFluidTemperatureC);
  const char* glycolTypeText(GlycolType type);
}
