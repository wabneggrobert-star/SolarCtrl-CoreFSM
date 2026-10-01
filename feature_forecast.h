#pragma once
#include "app_types.h"
#include <Arduino.h>

namespace Forecast {
struct Summary {
  bool enabled = false;
  bool valid = false;
  bool fetching = false;
  bool optimizerReady = false;
  uint32_t lastSuccessEpoch = 0;
  uint32_t lastAttemptMs = 0;
  char lastError[96] = "";
  float currentGtiWm2 = NAN;
  float next3hMeanGtiWm2 = NAN;
  float next3hIrradiationKWhM2 = NAN;
  float todayRemainingKWhM2 = NAN;
  float tomorrowKWhM2 = NAN;
  float tomorrowSunshineHours = NAN;
  float expectedThermalNext3hKWh = NAN;
  float learnedMeanEfficiency = NAN;
  float learnedMeanNext3hKWh = NAN;
  uint32_t learningSamples = 0;
};

void begin(AppContext& ctx);
void process(AppContext& ctx);
void forceRefresh();
Summary summary();
bool shouldDelayAuxHeater(const AppContext& ctx);
bool preferBufferForSolar(const AppContext& ctx, const PumpConfig& pump);
String json(const AppContext& ctx);
}
