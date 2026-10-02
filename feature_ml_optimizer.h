#pragma once
#include "app_types.h"
#include <Arduino.h>

namespace MlOptimizer {

// ML V2 arbeitet modular. Jede Domaene lernt ab dem ersten gueltigen Datenpunkt
// unabhaengig von den anderen Domaenen. AUTOMATIC wird pro Domaene separat
// freigeschaltet, sobald Mindestdaten, Lerndauer und Qualitaet passen.
enum class Domain : uint8_t {
  SOLAR_YIELD = 0,
  SOLAR_PID,
  SOLAR_ROUTING,
  HEATING,
  AUX_HEATER,
  OVEN,
  SOLAR_CONTROL,
  THERMAL_DEMAND,
  COUNT
};

struct DomainStatus {
  bool enabled = false;
  bool learning = false;
  bool active = false;
  bool fallback = false;
  bool controlAvailable = false;

  uint32_t trainingSamples = 0;
  uint16_t trainingDays = 0;
  uint16_t windowSamples7d = 0;
  float meanError7dPercent = NAN;

  char name[24] = "";
  char reason[64] = "";
};

struct PidGains {
  float kp = 0.0f;
  float ki = 0.0f;
  float kd = 0.0f;
};

struct PidTuningStatus {
  bool available = false;
  bool active = false;

  float baseKp = 0.0f;
  float baseKi = 0.0f;
  float baseKd = 0.0f;

  float effectiveKp = 0.0f;
  float effectiveKi = 0.0f;
  float effectiveKd = 0.0f;

  float kpScale = 1.0f;
  float kiScale = 1.0f;
  float kdScale = 1.0f;

  float kpScaleMin = 0.60f;
  float kpScaleMax = 1.40f;
  float kiScaleMin = 0.50f;
  float kiScaleMax = 1.50f;
  float kdScaleMin = 0.50f;
  float kdScaleMax = 1.50f;

  float ewmaError = NAN;
  float ewmaAbsError = NAN;
  float oscillationScore = 0.0f;
};

struct SolarControlParams {
  bool available = false;
  bool active = false;

  float baseStartDiffC = 0.0f;
  float baseTargetDiffC = 0.0f;
  float baseHysteresisC = 0.0f;

  float learnedStartDiffC = 0.0f;
  float learnedTargetDiffC = 0.0f;
  float learnedHysteresisC = 0.0f;

  float effectiveStartDiffC = 0.0f;
  float effectiveTargetDiffC = 0.0f;
  float effectiveHysteresisC = 0.0f;

  float startMinC = 0.0f;
  float startMaxC = 0.0f;
  float targetMinC = 0.0f;
  float targetMaxC = 0.0f;
  float hysteresisMinC = 0.0f;
  float hysteresisMaxC = 0.0f;

  float startOffsetC = 0.0f;
  float targetOffsetC = 0.0f;
  float hysteresisOffsetC = 0.0f;
};

struct HeatingTuningStatus {
  bool available = false;
  bool active = false;

  float learnedOffsetC = 0.0f;
  float effectiveTargetC = NAN;

  float roomErrorC = NAN;
  float roomRateCPerHour = NAN;
  float predictedRoomError1hC = NAN;

  float outsideCoeff = 0.0f;
  float flowErrorEwmaC = NAN;
  float spreadEwmaC = NAN;
  float mixerOscillationScore = 0.0f;

  float baseMixerDeadbandC = NAN;
  float learnedMixerDeadbandC = NAN;
  float effectiveMixerDeadbandC = NAN;
};

struct OvenTuningStatus {
  bool available = false;
  bool active = false;

  float openingBiasPercent = 0.0f;
  float temperatureRateCPerMin = NAN;
  float predictedError10MinC = NAN;
  float errorEwmaC = NAN;
  float absErrorEwmaC = NAN;
  float oscillationScore = 0.0f;

  float baseDeadbandC = NAN;
  float learnedDeadbandC = NAN;
  float effectiveDeadbandC = NAN;

  bool rlaControlAvailable = false;
  char rlaState[48] = "";
};

struct ThermalDemandStatus {
  bool available = false;
  bool learning = false;
  bool qualityReady = false;

  uint8_t boilerSensorCount = 0;
  uint8_t bufferSensorCount = 0;

  float boilerMeanC = NAN;
  float bufferMeanC = NAN;

  float boilerEnergyAbove20KWh = NAN;
  float bufferEnergyAbove20KWh = NAN;

  float boilerDemandKwNow = NAN;
  float bufferDemandKwNow = NAN;
  float totalDemandKwNow = NAN;

  float predictedDemand3hKWh = NAN;
  float predictedDemand6hKWh = NAN;
  float predictedDemand12hKWh = NAN;

  float predictedBoiler3hKWh = NAN;
  float predictedBuffer3hKWh = NAN;

  float outsideTemperatureC = NAN;
  float bufferOutsideCoeffKwPerK = 0.0f;

  uint32_t validIntervals = 0;
  uint32_t rejectedIntervals = 0;

  char stateText[48] = "";
};

struct Status {
  bool enabled = false;
  bool learning = false;
  bool active = false;
  bool fallback = false;

  // Legacy-/UI-Kompatibilitaet: diese Felder spiegeln weiterhin das
  // Solarertragsmodell wider.
  uint32_t trainingSamples = 0;
  uint16_t trainingDays = 0;
  uint16_t windowSamples7d = 0;
  float meanError7dPercent = NAN;

  float predictedNext3hKWh = NAN;
  float conservativeNext3hKWh = NAN;
  float lastMeasuredKWh = NAN;
  float lastPredictionErrorPercent = NAN;

  uint8_t readyDomainCount = 0;
  uint8_t enabledDomainCount = 0;

  DomainStatus domains[(uint8_t)Domain::COUNT];

  char lastDecision[128] = "";
  char recommendation[192] = "";
};

void begin(AppContext& ctx);
void process(AppContext& ctx, bool commissioningOrTestActive);
void resetLearning(AppContext& ctx);

Status status();
DomainStatus domainStatus(Domain domain);
bool domainActive(Domain domain);
String json(const AppContext& ctx);

// Bestehende ML-Overlay-Schnittstellen. Die Aufrufer muessen nicht geaendert
// werden; intern werden sie in V2 jeweils von ihrer eigenen Domaene gegated.
bool shouldDelayAuxHeater(const AppContext& ctx, float currentTemperatureC);
bool shouldStopAuxHeaterEarly(const AppContext& ctx, float currentTemperatureC);
bool preferBufferForSolar(const AppContext& ctx, const PumpConfig& pump);

PidGains effectiveSolarPidGains(
  const AppContext& ctx,
  uint8_t pumpIndex,
  const PumpConfig& pump
);

PidTuningStatus pidTuningStatus(
  const AppContext& ctx,
  uint8_t pumpIndex,
  const PumpConfig& pump
);

SolarControlParams solarControlParams(
  const AppContext& ctx,
  uint8_t pumpIndex,
  const PumpConfig& pump
);

ThermalDemandStatus thermalDemandStatus(
  const AppContext& ctx
);

HeatingTuningStatus heatingTuningStatus(
  const AppContext& ctx,
  uint8_t circuitIndex
);

OvenTuningStatus ovenTuningStatus(
  const AppContext& ctx
);

float adjustSolarPumpPwm(const AppContext& ctx, uint8_t pumpIndex, float requestedPercent);
float adjustHeatingTarget(
  const AppContext& ctx,
  uint8_t circuitIndex,
  float requestedTargetC,
  float roomC,
  float outsideC
);

float adjustHeatingMixerDeadband(
  const AppContext& ctx,
  uint8_t circuitIndex,
  float baseDeadbandC
);

// Ofen V2: nur eine begrenzte Korrektur innerhalb der vom Benutzer
// konfigurierten Servo-Min/Max-Grenzen. Safety bleibt ausserhalb/oberhalb davon.
uint8_t adjustOvenServoOpening(
  const AppContext& ctx,
  uint8_t requestedOpeningPercent,
  float ovenTemperatureC
);

float adjustOvenServoDeadband(
  const AppContext& ctx,
  float baseDeadbandC
);

} // namespace MlOptimizer
