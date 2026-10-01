#pragma once
#include "app_types.h"
#include <Arduino.h>

namespace MlOptimizer {
struct Status {
  bool enabled = false;
  bool learning = false;
  bool active = false;
  bool fallback = false;
  uint32_t trainingSamples = 0;
  uint16_t trainingDays = 0;
  uint16_t windowSamples7d = 0;
  float meanError7dPercent = NAN;
  float predictedNext3hKWh = NAN;
  float conservativeNext3hKWh = NAN;
  float lastMeasuredKWh = NAN;
  float lastPredictionErrorPercent = NAN;
  char lastDecision[128] = "";
  char recommendation[160] = "";
};

void begin(AppContext& ctx);
void process(AppContext& ctx, bool commissioningOrTestActive);
void resetLearning(AppContext& ctx);
Status status();
String json(const AppContext& ctx);

bool shouldDelayAuxHeater(const AppContext& ctx, float currentTemperatureC);
bool shouldStopAuxHeaterEarly(const AppContext& ctx, float currentTemperatureC);
bool preferBufferForSolar(const AppContext& ctx, const PumpConfig& pump);
float adjustSolarPumpPwm(const AppContext& ctx, uint8_t pumpIndex, float requestedPercent);
float adjustHeatingTarget(const AppContext& ctx, uint8_t circuitIndex, float requestedTargetC, float roomC, float outsideC);
}
