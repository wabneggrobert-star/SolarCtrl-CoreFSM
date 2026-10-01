#include "feature_fluid_properties.h"
#include <Arduino.h>
#include <math.h>

namespace {
struct CurvePoint { float concentrationPct; float freezeC; float heatCapacity40C; };

// Engineering lookup tables for typical inhibited water/glycol mixtures.
// They are deliberately used as interpolation tables rather than as exact laboratory values.
// The measured freeze point remains the user's primary input; additives can shift real values.
const CurvePoint PROP[] = {
  {0,   0, 1.163f}, {20, -7, 1.105f}, {25,-10,1.089f}, {30,-13,1.071f},
  {35,-17,1.053f}, {40,-21,1.034f}, {45,-26,1.014f}, {50,-32,0.994f},
  {55,-40,0.973f}, {60,-50,0.950f}
};
const CurvePoint ETH[] = {
  {0,   0, 1.163f}, {20, -8, 1.117f}, {25,-12,1.104f}, {30,-15,1.091f},
  {35,-20,1.077f}, {40,-25,1.062f}, {45,-31,1.046f}, {50,-37,1.029f},
  {55,-45,1.011f}, {60,-52,0.992f}
};

template <size_t N>
float concentrationFromFreeze(const CurvePoint (&curve)[N], float freezeC, bool& inRange) {
  if (freezeC >= curve[0].freezeC) { inRange = freezeC <= 1.0f; return 0.0f; }
  if (freezeC <= curve[N-1].freezeC) { inRange = freezeC >= curve[N-1].freezeC - 3.0f; return curve[N-1].concentrationPct; }
  for (size_t i=1;i<N;i++) {
    if (freezeC >= curve[i].freezeC) {
      const float span = curve[i].freezeC - curve[i-1].freezeC;
      const float f = fabsf(span) > 0.001f ? (freezeC - curve[i-1].freezeC) / span : 0.0f;
      inRange = true;
      return curve[i-1].concentrationPct + (curve[i].concentrationPct - curve[i-1].concentrationPct) * f;
    }
  }
  inRange = false;
  return curve[N-1].concentrationPct;
}

template <size_t N>
float heatCapacityAt40(const CurvePoint (&curve)[N], float concentrationPct) {
  if (concentrationPct <= curve[0].concentrationPct) return curve[0].heatCapacity40C;
  if (concentrationPct >= curve[N-1].concentrationPct) return curve[N-1].heatCapacity40C;
  for (size_t i=1;i<N;i++) {
    if (concentrationPct <= curve[i].concentrationPct) {
      const float f=(concentrationPct-curve[i-1].concentrationPct)/(curve[i].concentrationPct-curve[i-1].concentrationPct);
      return curve[i-1].heatCapacity40C + (curve[i].heatCapacity40C-curve[i-1].heatCapacity40C)*f;
    }
  }
  return curve[N-1].heatCapacity40C;
}

float temperatureCorrect(float value40C, float temperatureC) {
  // Volumetric heat capacity varies only moderately in the normal solar loop range.
  // This bounded correction prevents unrealistic extrapolation while improving over a fixed factor.
  const float t = constrain(temperatureC, -10.0f, 120.0f);
  const float factor = 1.0f - 0.00055f * (t - 40.0f);
  return value40C * constrain(factor, 0.94f, 1.04f);
}
}

namespace FluidProperties {
const char* glycolTypeText(GlycolType type) {
  return type == GlycolType::ETHYLENE ? "Ethylenglykol" : "Propylenglykol";
}

float glycolConcentrationPercent(const ConfigData& cfg) {
  if (cfg.solarFluidType == SolarFluidType::WATER) return 0.0f;
  bool ok=false;
  return cfg.glycolType == GlycolType::ETHYLENE
      ? concentrationFromFreeze(ETH, cfg.glycolMeasuredFreezeProtectionC, ok)
      : concentrationFromFreeze(PROP, cfg.glycolMeasuredFreezeProtectionC, ok);
}

float volumetricHeatCapacityWhPerLiterK(const ConfigData& cfg, float meanFluidTemperatureC) {
  if (cfg.solarFluidType == SolarFluidType::WATER) return temperatureCorrect(1.163f, meanFluidTemperatureC);
  bool ok=false;
  const float c = cfg.glycolType == GlycolType::ETHYLENE
      ? concentrationFromFreeze(ETH, cfg.glycolMeasuredFreezeProtectionC, ok)
      : concentrationFromFreeze(PROP, cfg.glycolMeasuredFreezeProtectionC, ok);
  const float base = cfg.glycolType == GlycolType::ETHYLENE ? heatCapacityAt40(ETH,c) : heatCapacityAt40(PROP,c);
  return temperatureCorrect(base, meanFluidTemperatureC);
}

Result evaluate(const ConfigData& cfg, float meanFluidTemperatureC) {
  Result r;
  if (cfg.solarFluidType == SolarFluidType::WATER) {
    r.glycolConcentrationPercent=0.0f;
    r.waterConcentrationPercent=100.0f;
    r.volumetricHeatCapacityWhPerLiterK=volumetricHeatCapacityWhPerLiterK(cfg,meanFluidTemperatureC);
    r.freezePointInRange=true;
    r.safetyCompatible = cfg.frostRequiredProtectionC >= 0.0f;
    r.safetyMarginMet = r.safetyCompatible;
    return r;
  }
  bool inRange=false;
  const float c = cfg.glycolType == GlycolType::ETHYLENE
      ? concentrationFromFreeze(ETH, cfg.glycolMeasuredFreezeProtectionC, inRange)
      : concentrationFromFreeze(PROP, cfg.glycolMeasuredFreezeProtectionC, inRange);
  r.glycolConcentrationPercent=constrain(c,0.0f,70.0f);
  r.waterConcentrationPercent=100.0f-r.glycolConcentrationPercent;
  r.volumetricHeatCapacityWhPerLiterK=volumetricHeatCapacityWhPerLiterK(cfg,meanFluidTemperatureC);
  r.freezePointInRange=inRange;
  // More negative means better protection. The safety value is the required design protection.
  r.safetyCompatible = cfg.glycolMeasuredFreezeProtectionC <= cfg.frostRequiredProtectionC;
  r.safetyMarginMet = cfg.glycolMeasuredFreezeProtectionC <= (cfg.frostRequiredProtectionC - max(0.0f,cfg.frostProtectionReserveK));
  return r;
}
}
