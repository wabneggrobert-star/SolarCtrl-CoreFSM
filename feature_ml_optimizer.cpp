#include "feature_ml_optimizer.h"
#include "feature_forecast.h"
#include "feature_time.h"
#include "feature_sensor_assignments.h"
#include "feature_safety_manager.h"
#include "feature_fluid_properties.h"
#include "feature_energy_meter.h"
#include "feature_oven.h"
#include "feature_aux_heater.h"
#include "config.h"

#include <SD.h>
#include <math.h>
#include <string.h>
#include <time.h>

namespace {

using MlDomain = MlOptimizer::Domain;
using MlDomainStatus = MlOptimizer::DomainStatus;

constexpr uint8_t DOMAIN_COUNT = (uint8_t)MlDomain::COUNT;
constexpr uint8_t FEATURE_COUNT = 6;
constexpr uint8_t MAX_PENDING = 12;
constexpr uint16_t ERROR_WINDOW_PER_DOMAIN = 64;

constexpr uint32_t PROCESS_INTERVAL_MS = 2000UL;
constexpr uint32_t SOLAR_YIELD_SAMPLE_INTERVAL_MS = 1800000UL; // 30 min
constexpr uint32_t SOLAR_PID_SAMPLE_INTERVAL_MS = 120000UL;    // 2 min bei laufender Solarpumpe
constexpr uint32_t ROUTING_SAMPLE_INTERVAL_MS = 1800000UL;     // 30 min
constexpr uint32_t HEATING_SAMPLE_INTERVAL_MS = 600000UL;      // 10 min
constexpr uint32_t AUX_SAMPLE_INTERVAL_MS = 1800000UL;         // 30 min
constexpr uint32_t OVEN_SAMPLE_INTERVAL_MS = 300000UL;         // 5 min
constexpr uint32_t SOLAR_CONTROL_SAMPLE_INTERVAL_MS = 300000UL; // 5 min
constexpr uint32_t THERMAL_DEMAND_SAMPLE_INTERVAL_MS = 900000UL; // 15 min
constexpr uint32_t RUNTIME_SAVE_INTERVAL_MS = 600000UL;        // 10 min

constexpr uint32_t PREDICTION_HORIZON_S = 3UL * 3600UL;
constexpr uint32_t WINDOW_7D_S = 7UL * 86400UL;

// V2 Freigaberegel: jede Domaene separat.
constexpr uint32_t MIN_TRAINING_SAMPLES = 200;
constexpr uint16_t MIN_TRAINING_DAYS = 7;
constexpr uint16_t MIN_WINDOW_SAMPLES = 50;
constexpr float MAX_WINDOW_ERROR_PCT = 10.0f;

constexpr float CONTROL_CONFIDENCE_FACTOR = 0.85f;
constexpr float SOLAR_SGD_RATE = 0.035f;

// Die eigentlichen Stellkorrekturen bleiben bewusst klein. ML ist ein Overlay,
// kein Ersatz fuer Grundregelung, Min/Max, Safety oder Benutzergrenzen.
constexpr float PID_KP_SCALE_MIN = 0.60f;
constexpr float PID_KP_SCALE_MAX = 1.40f;
constexpr float PID_KI_SCALE_MIN = 0.50f;
constexpr float PID_KI_SCALE_MAX = 1.50f;
constexpr float PID_KD_SCALE_MIN = 0.50f;
constexpr float PID_KD_SCALE_MAX = 1.50f;

constexpr float SOLAR_PWM_BIAS_LIMIT = 8.0f;
constexpr float OVEN_OPENING_BIAS_LIMIT = 8.0f;

// ML V2 Block 9:
// - vier jahreszeitliche Korrekturfaktoren
// - sehr langsames Vergessen alter Stellkorrekturen
// - Diagnose der Modellfrische
constexpr uint8_t SEASON_COUNT = 4;
constexpr uint32_t MODEL_STALE_S = 8UL * 86400UL;
constexpr float MODEL_AGE_DAILY = 0.9995f;
constexpr float SOLAR_SEASON_FACTOR_MIN = 0.75f;
constexpr float SOLAR_SEASON_FACTOR_MAX = 1.25f;
constexpr float THERMAL_SEASON_FACTOR_MIN = 0.70f;
constexpr float THERMAL_SEASON_FACTOR_MAX = 1.30f;

constexpr const char* FILE_ML_V2_DOMAIN_LOG = "/logs/ml_v2_domains.csv";
constexpr const char* FILE_ML_V2_THERMAL_LOG = "/logs/ml_v2_thermal.csv";

struct PendingSample {
  bool used = false;
  bool valid = true;
  uint32_t createdEpoch = 0;
  uint32_t dueEpoch = 0;
  double startEnergyKWh = 0.0;
  float predictedKWh = NAN;
  float features[FEATURE_COUNT] = {};
  char invalidReason[28] = "";
};

struct ErrorPoint {
  uint32_t epoch = 0;
  float errorPct = NAN;
};

struct DomainRuntime {
  uint32_t samples = 0;
  uint16_t days = 0;
  int32_t lastDayKey = -1;
  uint32_t lastSampleMs = 0;
  uint32_t lastObservationEpoch = 0;

  ErrorPoint errors[ERROR_WINDOW_PER_DOMAIN];
  uint8_t errorHead = 0;
  uint8_t errorCount = 0;
};

struct SolarControlAdaptiveState {
  float startOffsetC = 0.0f;
  float targetOffsetC = 0.0f;
  float hysteresisOffsetC = 0.0f;

  bool stateInitialized = false;
  bool lastPumpState = false;
  uint32_t cycleStartedMs = 0;
  uint32_t lastCompletedCycleMs = 0;
};

struct OvenAdaptiveState {
  bool initialized = false;
  float lastTemperatureC = NAN;
  float lastErrorC = NAN;
  uint32_t lastEpoch = 0;

  float temperatureRateCPerMin = NAN;
  float predictedError10MinC = NAN;
  float errorEwmaC = NAN;
  float absErrorEwmaC = NAN;
  float oscillationScore = 0.0f;
  float deadbandOffsetC = 0.0f;
};

struct HeatingAdaptiveState {
  bool initialized = false;
  float lastRoomC = NAN;
  float lastFlowErrorC = NAN;
  uint32_t lastEpoch = 0;

  float roomRateCPerHour = NAN;
  float projectedRoomError1hC = NAN;
  float flowErrorEwmaC = NAN;
  float spreadEwmaC = NAN;
  float mixerOscillationScore = 0.0f;
  float mixerDeadbandOffsetC = 0.0f;
};

struct ThermalDemandRuntime {
  bool initialized = false;
  bool intervalContaminated = false;

  uint32_t lastEpoch = 0;
  float lastBoilerEnergyKWh = NAN;
  float lastBufferEnergyKWh = NAN;

  // 0 = Werktag, 1 = Wochenende; je 24 Stunden.
  float boilerRateKw[2][24] = {};
  float bufferBaseRateKw[2][24] = {};
  uint16_t boilerCount[2][24] = {};
  uint16_t bufferCount[2][24] = {};

  // Puffer-Heizlastkorrektur:
  // rate = hourlyBase + coeff * max(0, 15 - outsideC)
  float bufferOutsideCoeffKwPerK = 0.0f;

  float lastBoilerDemandKw = NAN;
  float lastBufferDemandKw = NAN;

  uint32_t validIntervals = 0;
  uint32_t rejectedIntervals = 0;
};

struct PidAdaptiveState {
  float kpScale = 1.0f;
  float kiScale = 1.0f;
  float kdScale = 1.0f;

  float lastNormalizedError = 0.0f;
  bool hasLastError = false;

  float ewmaError = 0.0f;
  float ewmaAbsError = 0.0f;
  float ewmaDeltaError = 0.0f;
  float oscillationScore = 0.0f;
};

MlOptimizer::Status g_status;
PendingSample g_pending[MAX_PENDING];
DomainRuntime g_domains[DOMAIN_COUNT];

float g_weights[FEATURE_COUNT] = {};
float g_solarSeasonFactor[SEASON_COUNT] = {
  1.0f, 1.0f, 1.0f, 1.0f
};
float g_thermalSeasonFactor[SEASON_COUNT] = {
  1.0f, 1.0f, 1.0f, 1.0f
};
uint32_t g_lastAgingEpoch = 0;

float g_solarPwmBias[MAX_PUMPS] = {};
PidAdaptiveState g_pidAdaptive[MAX_PUMPS];
SolarControlAdaptiveState g_solarControl[MAX_PUMPS];
ThermalDemandRuntime g_thermalDemand;
OvenAdaptiveState g_ovenAdaptive;
HeatingAdaptiveState g_heatingAdaptive[MAX_HEATING_CIRCUITS];
float g_hcBias[MAX_HEATING_CIRCUITS] = {};
float g_hcOutsideCoeff[MAX_HEATING_CIRCUITS] = {};
float g_ovenOpeningBias = 0.0f;

uint32_t g_lastCreateMs = 0;
uint32_t g_lastProcessMs = 0;
uint32_t g_lastRuntimeSaveMs = 0;
uint32_t g_auxDelayStartedMs = 0;

bool g_loaded = false;
bool g_runtimeDirty = false;

float clampf(float value, float lo, float hi) {
  return value < lo ? lo : (value > hi ? hi : value);
}

uint8_t domainIndex(MlDomain domain) {
  return (uint8_t)domain;
}

const char* domainName(MlDomain domain) {
  switch (domain) {
    case MlDomain::SOLAR_YIELD: return "solar_yield";
    case MlDomain::SOLAR_PID: return "solar_pid";
    case MlDomain::SOLAR_ROUTING: return "solar_routing";
    case MlDomain::HEATING: return "heating";
    case MlDomain::AUX_HEATER: return "aux_heater";
    case MlDomain::OVEN: return "oven";
    case MlDomain::SOLAR_CONTROL: return "solar_control";
    case MlDomain::THERMAL_DEMAND: return "thermal_demand";
    default: return "unknown";
  }
}

bool readRole(const AppContext& ctx, Ds18Role role, float& value) {
  value = NAN;
  bool valid = false;
  if (role == Ds18Role::NONE) return false;

  return SensorAssignments::readByRole(
           ctx.ds18b20,
           ctx.assignments,
           role,
           value,
           valid
         ) &&
         valid &&
         !isnan(value);
}

bool isSolarRole(HeatSourceRole role) {
  return role == HeatSourceRole::SOLAR_COLLECTOR_1 ||
         role == HeatSourceRole::SOLAR_COLLECTOR_2 ||
         role == HeatSourceRole::SOLAR_COLLECTOR_3;
}

bool isBoilerRole(Ds18Role role) {
  return role == Ds18Role::SINK_BOILER_TOP ||
         role == Ds18Role::BOILER_BOTTOM;
}

bool isBufferRole(Ds18Role role) {
  return role == Ds18Role::SINK_BUFFER_TOP ||
         role == Ds18Role::BUFFER_HIGH ||
         role == Ds18Role::BUFFER_MID ||
         role == Ds18Role::BUFFER_BOTTOM;
}

int findSolarPump(const AppContext& ctx) {
  for (uint8_t i = 0; i < MAX_PUMPS; i++) {
    const PumpConfig& pump = ctx.config.pumps[i];
    if (!pump.enabled) continue;
    if (pump.sourceType != PumpSourceType::HEAT_SOURCE_ROLE) continue;
    if (isSolarRole(pump.sourceRole)) return i;
  }
  return -1;
}

int32_t localDayKey(uint32_t epoch) {
  time_t t = (time_t)epoch;
  struct tm tmv = {};
  localtime_r(&t, &tmv);
  return (tmv.tm_year + 1900) * 1000 + tmv.tm_yday;
}

float daySin(uint32_t epoch) {
  time_t t = (time_t)epoch;
  struct tm tmv = {};
  localtime_r(&t, &tmv);
  return sinf(2.0f * PI * (float)tmv.tm_yday / 365.25f);
}

float dayCos(uint32_t epoch) {
  time_t t = (time_t)epoch;
  struct tm tmv = {};
  localtime_r(&t, &tmv);
  return cosf(2.0f * PI * (float)tmv.tm_yday / 365.25f);
}

float roleMeanStorage(const AppContext& ctx) {
  float buffer = NAN;
  float boiler = NAN;

  const bool bufferValid = readRole(ctx, Ds18Role::SINK_BUFFER_TOP, buffer);
  const bool boilerValid = readRole(ctx, Ds18Role::SINK_BOILER_TOP, boiler);

  if (bufferValid && boilerValid) return 0.5f * (buffer + boiler);
  if (bufferValid) return buffer;
  if (boilerValid) return boiler;
  return 45.0f;
}

float outsideTemp(const AppContext& ctx) {
  float value = NAN;
  return readRole(ctx, Ds18Role::OUTSIDE_TEMPERATURE, value) ? value : 10.0f;
}

void featuresFor(
  const AppContext& ctx,
  float out[FEATURE_COUNT],
  float& incidentKWh
) {
  const Forecast::Summary fs = Forecast::summary();
  const float irradiation =
    isnan(fs.next3hIrradiationKWhM2)
      ? 0.0f
      : max(0.0f, fs.next3hIrradiationKWhM2);

  incidentKWh =
    irradiation * max(0.1f, ctx.config.collectorApertureM2);

  const float outsideC = outsideTemp(ctx);
  const float storageC = roleMeanStorage(ctx);

  const int solarPumpIndex = findSolarPump(ctx);
  const float pwmPercent =
    solarPumpIndex >= 0
      ? ctx.config.pumps[solarPumpIndex].lastPwmPercent
      : 50.0f;

  const uint32_t now =
    TimeService::valid() ? (uint32_t)time(nullptr) : 0;

  out[0] = incidentKWh;
  out[1] =
    incidentKWh *
    clampf((outsideC - 10.0f) / 30.0f, -1.0f, 1.5f);
  out[2] =
    incidentKWh *
    clampf((storageC - 45.0f) / 45.0f, -1.0f, 1.2f);
  out[3] = incidentKWh * (now ? daySin(now) : 0.0f);
  out[4] = incidentKWh * (now ? dayCos(now) : 0.0f);
  out[5] =
    incidentKWh *
    clampf((pwmPercent - 50.0f) / 50.0f, -1.0f, 1.0f);
}

float rawPredict(
  const float features[FEATURE_COUNT],
  float incidentKWh
) {
  float predicted = 0.0f;

  for (uint8_t i = 0; i < FEATURE_COUNT; i++) {
    predicted += g_weights[i] * features[i];
  }

  return clampf(
    predicted,
    0.0f,
    max(0.0f, incidentKWh * 0.95f)
  );
}

uint8_t seasonIndex(uint32_t epoch) {
  if (!epoch) return 0;

  time_t t = (time_t)epoch;
  struct tm tmv = {};
  localtime_r(&t, &tmv);

  const uint8_t month =
    (uint8_t)(tmv.tm_mon + 1);

  // Meteorologische Jahreszeiten:
  // 0 Winter, 1 Fruehling, 2 Sommer, 3 Herbst
  if (month == 12 || month <= 2) return 0;
  if (month <= 5) return 1;
  if (month <= 8) return 2;
  return 3;
}

const char* seasonName(uint8_t season) {
  switch (season) {
    case 0: return "winter";
    case 1: return "spring";
    case 2: return "summer";
    case 3: return "autumn";
    default: return "unknown";
  }
}

float seasonalSolarPredict(
  const float features[FEATURE_COUNT],
  float incidentKWh,
  uint32_t epoch
) {
  const float raw =
    rawPredict(features, incidentKWh);

  const float factor =
    g_solarSeasonFactor[
      min((uint8_t)(SEASON_COUNT - 1), seasonIndex(epoch))
    ];

  return clampf(
    raw * factor,
    0.0f,
    max(0.0f, incidentKWh * 0.95f)
  );
}

void resetModelParameters(const ConfigData& cfg) {
  for (float& weight : g_weights) weight = 0.0f;

  for (uint8_t season = 0; season < SEASON_COUNT; season++) {
    g_solarSeasonFactor[season] = 1.0f;
    g_thermalSeasonFactor[season] = 1.0f;
  }

  g_lastAgingEpoch = 0;

  g_weights[0] =
    cfg.solarCollectorType == SolarCollectorType::EVACUATED_TUBE
      ? 0.55f
      : 0.45f;

  for (uint8_t i = 0; i < MAX_PUMPS; i++) {
    g_solarPwmBias[i] = 0.0f;
    g_pidAdaptive[i] = PidAdaptiveState{};
    g_solarControl[i] = SolarControlAdaptiveState{};
  }

  for (uint8_t i = 0; i < MAX_HEATING_CIRCUITS; i++) {
    g_hcBias[i] = 0.0f;
    g_hcOutsideCoeff[i] = 0.0f;
  }

  g_ovenOpeningBias = 0.0f;
  g_ovenAdaptive = OvenAdaptiveState{};
  g_thermalDemand = ThermalDemandRuntime{};

  for (uint8_t i = 0; i < MAX_HEATING_CIRCUITS; i++) {
    g_heatingAdaptive[i] = HeatingAdaptiveState{};
  }
}

void resetDomainRuntime() {
  for (uint8_t i = 0; i < DOMAIN_COUNT; i++) {
    g_domains[i] = DomainRuntime{};
  }
}

void addDomainError(
  MlDomain domain,
  uint32_t epoch,
  float errorPct
) {
  if (isnan(errorPct)) return;

  DomainRuntime& rt = g_domains[domainIndex(domain)];
  rt.errors[rt.errorHead] = {epoch, errorPct};
  rt.errorHead =
    (uint8_t)((rt.errorHead + 1) % ERROR_WINDOW_PER_DOMAIN);

  if (rt.errorCount < ERROR_WINDOW_PER_DOMAIN) {
    rt.errorCount++;
  }
}

void observeDomain(
  MlDomain domain,
  uint32_t epoch,
  float errorPct
) {
  DomainRuntime& rt = g_domains[domainIndex(domain)];

  rt.samples++;

  const int32_t dayKey = localDayKey(epoch);
  if (dayKey != rt.lastDayKey) {
    rt.days++;
    rt.lastDayKey = dayKey;
  }

  rt.lastObservationEpoch = epoch;

  addDomainError(domain, epoch, errorPct);
  g_runtimeDirty = true;
}

void calculateDomainWindow(
  MlDomain domain,
  uint32_t now,
  uint16_t& count,
  float& mean
) {
  count = 0;
  mean = NAN;

  const DomainRuntime& rt = g_domains[domainIndex(domain)];
  float sum = 0.0f;

  for (uint8_t j = 0; j < rt.errorCount; j++) {
    const uint8_t idx =
      (uint8_t)(
        (rt.errorHead + ERROR_WINDOW_PER_DOMAIN - rt.errorCount + j) %
        ERROR_WINDOW_PER_DOMAIN
      );

    const ErrorPoint& point = rt.errors[idx];

    if (!point.epoch || isnan(point.errorPct)) continue;
    if (now < point.epoch) continue;
    if ((now - point.epoch) > WINDOW_7D_S) continue;

    sum += point.errorPct;
    count++;
  }

  if (count > 0) {
    mean = sum / (float)count;
  }
}

void applyDailyModelAging(uint32_t now) {
  if (!now) return;

  if (g_lastAgingEpoch == 0) {
    g_lastAgingEpoch = now;
    g_runtimeDirty = true;
    return;
  }

  if (now <= g_lastAgingEpoch) return;

  const uint32_t elapsedS =
    now - g_lastAgingEpoch;

  if (elapsedS < 86400UL) return;

  uint32_t daysElapsed =
    elapsedS / 86400UL;

  // Bei langer Offlinezeit die Alterung begrenzen.
  daysElapsed =
    min((uint32_t)30, max((uint32_t)1, daysElapsed));

  for (uint32_t day = 0; day < daysElapsed; day++) {
    // Haupt-Solarwirkungsgrad w0 bleibt als Anlagenbasis erhalten.
    // Nur die Korrekturfeatures altern langsam.
    for (uint8_t i = 1; i < FEATURE_COUNT; i++) {
      g_weights[i] *= MODEL_AGE_DAILY;
    }

    for (uint8_t season = 0; season < SEASON_COUNT; season++) {
      g_solarSeasonFactor[season] =
        1.0f +
        (
          g_solarSeasonFactor[season] -
          1.0f
        ) * MODEL_AGE_DAILY;

      g_thermalSeasonFactor[season] =
        1.0f +
        (
          g_thermalSeasonFactor[season] -
          1.0f
        ) * MODEL_AGE_DAILY;
    }

    for (uint8_t i = 0; i < MAX_PUMPS; i++) {
      g_solarPwmBias[i] *= MODEL_AGE_DAILY;

      g_pidAdaptive[i].kpScale =
        1.0f +
        (
          g_pidAdaptive[i].kpScale -
          1.0f
        ) * MODEL_AGE_DAILY;

      g_pidAdaptive[i].kiScale =
        1.0f +
        (
          g_pidAdaptive[i].kiScale -
          1.0f
        ) * MODEL_AGE_DAILY;

      g_pidAdaptive[i].kdScale =
        1.0f +
        (
          g_pidAdaptive[i].kdScale -
          1.0f
        ) * MODEL_AGE_DAILY;

      g_solarControl[i].startOffsetC *=
        MODEL_AGE_DAILY;
      g_solarControl[i].targetOffsetC *=
        MODEL_AGE_DAILY;
      g_solarControl[i].hysteresisOffsetC *=
        MODEL_AGE_DAILY;
    }

    for (uint8_t i = 0; i < MAX_HEATING_CIRCUITS; i++) {
      g_hcBias[i] *= MODEL_AGE_DAILY;
      g_hcOutsideCoeff[i] *= MODEL_AGE_DAILY;
      g_heatingAdaptive[i].mixerDeadbandOffsetC *=
        MODEL_AGE_DAILY;
    }

    g_ovenOpeningBias *= MODEL_AGE_DAILY;
    g_ovenAdaptive.deadbandOffsetC *=
      MODEL_AGE_DAILY;
  }

  g_lastAgingEpoch +=
    daysElapsed * 86400UL;

  g_runtimeDirty = true;
}

bool storageLimited(const AppContext& ctx, int pumpIndex) {
  if (pumpIndex < 0) return true;

  const PumpConfig& pump = ctx.config.pumps[pumpIndex];

  bool anyTarget = false;
  bool anyAvailable = false;

  for (uint8_t i = 0; i < PUMP_ROUTE_TARGET_COUNT; i++) {
    const PumpRouteTargetConfig& target = pump.targets[i];

    if (!target.enabled || target.sinkRole == Ds18Role::NONE) {
      continue;
    }

    float value = NAN;
    if (!readRole(ctx, target.sinkRole, value)) continue;

    anyTarget = true;

    if (target.maxTempC <= 0.01f ||
        value < target.maxTempC - 0.3f) {
      anyAvailable = true;
    }
  }

  return anyTarget && !anyAvailable;
}

bool safetyAllowsLearning(bool testActive) {
  if (testActive) return false;

  const auto& safety = SafetyManager::status();

  return
    !safety.blockNormalPumpControl &&
    !safety.storageCriticalOvertemperatureActive &&
    !safety.sensorFaultActive;
}

bool commonSolarTrainingValid(
  const AppContext& ctx,
  bool testActive,
  char* reason,
  size_t reasonSize
) {
  if (testActive) {
    strncpy(reason, "test_mode", reasonSize);
    return false;
  }

  if (!ctx.config.forecastEnabled ||
      !Forecast::summary().valid) {
    strncpy(reason, "forecast_invalid", reasonSize);
    return false;
  }

  if (!TimeService::valid()) {
    strncpy(reason, "time_invalid", reasonSize);
    return false;
  }

  if (!ctx.config.energyMeter.enabled) {
    strncpy(reason, "energy_meter_off", reasonSize);
    return false;
  }

  if (!safetyAllowsLearning(false)) {
    strncpy(reason, "safety_active", reasonSize);
    return false;
  }

  const int solarPumpIndex = findSolarPump(ctx);

  if (solarPumpIndex < 0) {
    strncpy(reason, "no_solar_pump", reasonSize);
    return false;
  }

  if (storageLimited(ctx, solarPumpIndex)) {
    strncpy(reason, "storage_limited", reasonSize);
    return false;
  }

  if (ctx.config.collectorApertureM2 < 0.1f) {
    strncpy(reason, "aperture_invalid", reasonSize);
    return false;
  }

  reason[0] = '\0';
  return true;
}

void invalidatePending(const char* reason) {
  for (PendingSample& sample : g_pending) {
    if (!sample.used || !sample.valid) continue;

    sample.valid = false;
    strncpy(
      sample.invalidReason,
      reason,
      sizeof(sample.invalidReason) - 1
    );
  }
}

void appendSolarTrainingLog(
  uint32_t created,
  uint32_t due,
  float predicted,
  float measured,
  float errorPct,
  const char* result
) {
  if (!SD.exists("/logs")) SD.mkdir("/logs");

  const bool headerNeeded = !SD.exists(FILE_ML_LOG);
  File file = SD.open(FILE_ML_LOG, FILE_APPEND);
  if (!file) return;

  if (headerNeeded || file.size() == 0) {
    file.println(
      "completed_at,prediction_created_at,prediction_until,"
      "predicted_kwh,measured_kwh,error_pct,result,"
      "model_version,training_samples"
    );
  }

  time_t now = time(nullptr);
  time_t createdTime = created;
  time_t dueTime = due;

  struct tm tmv = {};
  char completedText[32];
  char createdText[32];
  char dueText[32];

  localtime_r(&now, &tmv);
  strftime(
    completedText,
    sizeof(completedText),
    "%Y-%m-%dT%H:%M:%S%z",
    &tmv
  );

  localtime_r(&createdTime, &tmv);
  strftime(
    createdText,
    sizeof(createdText),
    "%Y-%m-%dT%H:%M:%S%z",
    &tmv
  );

  localtime_r(&dueTime, &tmv);
  strftime(
    dueText,
    sizeof(dueText),
    "%Y-%m-%dT%H:%M:%S%z",
    &tmv
  );

  file.print(completedText);
  file.print(',');
  file.print(createdText);
  file.print(',');
  file.print(dueText);
  file.print(',');
  file.print(predicted, 4);
  file.print(',');
  file.print(measured, 4);
  file.print(',');

  if (!isnan(errorPct)) file.print(errorPct, 2);

  file.print(',');
  file.print(result);
  file.print(",mlv2,");
  file.println(
    g_domains[domainIndex(MlDomain::SOLAR_YIELD)].samples
  );

  file.close();
}

void appendDomainLog(
  MlDomain domain,
  uint32_t epoch,
  float errorPct,
  float learnedValue
) {
  if (!SD.exists("/logs")) SD.mkdir("/logs");

  const bool headerNeeded = !SD.exists(FILE_ML_V2_DOMAIN_LOG);
  File file = SD.open(FILE_ML_V2_DOMAIN_LOG, FILE_APPEND);
  if (!file) return;

  if (headerNeeded || file.size() == 0) {
    file.println(
      "epoch,domain,error_pct,learned_value,samples,days"
    );
  }

  const DomainRuntime& rt = g_domains[domainIndex(domain)];

  file.print(epoch);
  file.print(',');
  file.print(domainName(domain));
  file.print(',');

  if (!isnan(errorPct)) file.print(errorPct, 3);

  file.print(',');
  if (!isnan(learnedValue)) file.print(learnedValue, 5);

  file.print(',');
  file.print(rt.samples);
  file.print(',');
  file.println(rt.days);

  file.close();
}

float activePumpTargetDiff(const PumpConfig& pump) {
  float target = pump.targetDiff;

  if (pump.activeTargetIndex < PUMP_ROUTE_TARGET_COUNT) {
    const PumpRouteTargetConfig& routeTarget =
      pump.targets[pump.activeTargetIndex];

    if (routeTarget.enabled &&
        routeTarget.targetDiffOverride > 0.0f) {
      target = routeTarget.targetDiffOverride;
    }
  }

  return max(0.1f, target);
}

void resolveSolarControlBounds(
  const PumpConfig& pump,
  float& startMin,
  float& startMax,
  float& targetMin,
  float& targetMax,
  float& hystMin,
  float& hystMax
) {
  // Wenn kein gueltiges User-Min/Max-Paar konfiguriert wurde, gelten
  // konservative automatische Grenzen relativ zum Basiswert.
  if (pump.mlStartDiffMinC > 0.0f &&
      pump.mlStartDiffMaxC >= pump.mlStartDiffMinC) {
    startMin = pump.mlStartDiffMinC;
    startMax = pump.mlStartDiffMaxC;
  } else {
    startMin = max(1.0f, pump.startDiff - 2.0f);
    startMax = pump.startDiff + 2.0f;
  }

  if (pump.mlTargetDiffMinC > 0.0f &&
      pump.mlTargetDiffMaxC >= pump.mlTargetDiffMinC) {
    targetMin = pump.mlTargetDiffMinC;
    targetMax = pump.mlTargetDiffMaxC;
  } else {
    targetMin = max(3.0f, pump.targetDiff - 3.0f);
    targetMax = pump.targetDiff + 3.0f;
  }

  if (pump.mlHysteresisMinC > 0.0f &&
      pump.mlHysteresisMaxC >= pump.mlHysteresisMinC) {
    hystMin = pump.mlHysteresisMinC;
    hystMax = pump.mlHysteresisMaxC;
  } else {
    hystMin = max(0.5f, pump.hysteresis - 1.0f);
    hystMax = pump.hysteresis + 1.0f;
  }

  // Physikalische Mindestkonsistenz.
  targetMin = max(targetMin, startMin + 1.0f);
  targetMax = max(targetMax, targetMin);
  hystMax = max(hystMin, hystMax);
}

MlOptimizer::SolarControlParams solarControlParamsInternal(
  const AppContext& ctx,
  uint8_t pumpIndex,
  const PumpConfig& pump,
  bool applyLearned
) {
  MlOptimizer::SolarControlParams out;

  out.available =
    pumpIndex < MAX_PUMPS &&
    pump.enabled &&
    pump.mode == PumpMode::PWM &&
    isSolarRole(pump.sourceRole);

  out.baseStartDiffC = pump.startDiff;
  out.baseTargetDiffC = pump.targetDiff;
  out.baseHysteresisC = pump.hysteresis;

  resolveSolarControlBounds(
    pump,
    out.startMinC,
    out.startMaxC,
    out.targetMinC,
    out.targetMaxC,
    out.hysteresisMinC,
    out.hysteresisMaxC
  );

  if (pumpIndex < MAX_PUMPS) {
    const SolarControlAdaptiveState& learned =
      g_solarControl[pumpIndex];

    out.startOffsetC = learned.startOffsetC;
    out.targetOffsetC = learned.targetOffsetC;
    out.hysteresisOffsetC = learned.hysteresisOffsetC;
  }

  out.learnedStartDiffC =
    clampf(
      pump.startDiff + out.startOffsetC,
      out.startMinC,
      out.startMaxC
    );

  out.learnedTargetDiffC =
    clampf(
      pump.targetDiff + out.targetOffsetC,
      out.targetMinC,
      out.targetMaxC
    );

  // Target muss immer mindestens 1 K oberhalb des Startwerts bleiben.
  out.learnedTargetDiffC =
    max(
      out.learnedTargetDiffC,
      out.learnedStartDiffC + 1.0f
    );
  out.learnedTargetDiffC =
    min(out.learnedTargetDiffC, out.targetMaxC);

  out.learnedHysteresisC =
    clampf(
      pump.hysteresis + out.hysteresisOffsetC,
      out.hysteresisMinC,
      out.hysteresisMaxC
    );

  // Stop-Schwelle muss positiv bleiben.
  out.learnedHysteresisC =
    min(
      out.learnedHysteresisC,
      max(0.1f, out.learnedStartDiffC - 0.5f)
    );

  out.active =
    applyLearned &&
    out.available;

  out.effectiveStartDiffC =
    out.active
      ? out.learnedStartDiffC
      : pump.startDiff;

  out.effectiveTargetDiffC =
    out.active
      ? out.learnedTargetDiffC
      : pump.targetDiff;

  out.effectiveHysteresisC =
    out.active
      ? out.learnedHysteresisC
      : pump.hysteresis;

  return out;
}

MlOptimizer::PidGains effectivePidGainsInternal(
  uint8_t pumpIndex,
  const PumpConfig& pump,
  bool applyLearned
) {
  MlOptimizer::PidGains gains;
  gains.kp = pump.pidKp;
  gains.ki = pump.pidKi;
  gains.kd = pump.pidKd;

  if (!applyLearned || pumpIndex >= MAX_PUMPS) return gains;

  const PidAdaptiveState& learned = g_pidAdaptive[pumpIndex];

  gains.kp =
    pump.pidKp *
    clampf(learned.kpScale, PID_KP_SCALE_MIN, PID_KP_SCALE_MAX);

  gains.ki =
    pump.pidKi <= 0.0f
      ? 0.0f
      : pump.pidKi *
        clampf(learned.kiScale, PID_KI_SCALE_MIN, PID_KI_SCALE_MAX);

  gains.kd =
    pump.pidKd <= 0.0f
      ? 0.0f
      : pump.pidKd *
        clampf(learned.kdScale, PID_KD_SCALE_MIN, PID_KD_SCALE_MAX);

  return gains;
}

void saveRuntime() {
  if (!SD.exists("/runtime")) SD.mkdir("/runtime");

  File file = SD.open(FILE_ML_RUNTIME, FILE_WRITE);
  if (!file) return;

  file.println("modelVersion=2");

  for (uint8_t i = 0; i < FEATURE_COUNT; i++) {
    file.printf("w%u=%.8f\n", i, g_weights[i]);
  }

  file.printf("agingLastEpoch=%lu\n", (unsigned long)g_lastAgingEpoch);

  for (uint8_t season = 0; season < SEASON_COUNT; season++) {
    file.printf(
      "solarSeason%u=%.7f\n",
      season,
      g_solarSeasonFactor[season]
    );
    file.printf(
      "thermalSeason%u=%.7f\n",
      season,
      g_thermalSeasonFactor[season]
    );
  }

  for (uint8_t i = 0; i < MAX_PUMPS; i++) {
    file.printf(
      "solarPwmBias%u=%.5f\n",
      i,
      g_solarPwmBias[i]
    );
    file.printf("pidKpScale%u=%.7f\n", i, g_pidAdaptive[i].kpScale);
    file.printf("pidKiScale%u=%.7f\n", i, g_pidAdaptive[i].kiScale);
    file.printf("pidKdScale%u=%.7f\n", i, g_pidAdaptive[i].kdScale);
    file.printf("pidEwmaError%u=%.7f\n", i, g_pidAdaptive[i].ewmaError);
    file.printf("pidEwmaAbs%u=%.7f\n", i, g_pidAdaptive[i].ewmaAbsError);
    file.printf("pidEwmaDelta%u=%.7f\n", i, g_pidAdaptive[i].ewmaDeltaError);
    file.printf("pidOscillation%u=%.7f\n", i, g_pidAdaptive[i].oscillationScore);
    file.printf("solarStartOffset%u=%.5f\n", i, g_solarControl[i].startOffsetC);
    file.printf("solarTargetOffset%u=%.5f\n", i, g_solarControl[i].targetOffsetC);
    file.printf("solarHystOffset%u=%.5f\n", i, g_solarControl[i].hysteresisOffsetC);
  }

  for (uint8_t i = 0; i < MAX_HEATING_CIRCUITS; i++) {
    file.printf("hcBias%u=%.5f\n", i, g_hcBias[i]);
    file.printf(
      "hcOutside%u=%.6f\n",
      i,
      g_hcOutsideCoeff[i]
    );
    file.printf("hcRoomRate%u=%.6f\n", i,
      isnan(g_heatingAdaptive[i].roomRateCPerHour) ? 0.0f : g_heatingAdaptive[i].roomRateCPerHour);
    file.printf("hcFlowEwma%u=%.6f\n", i,
      isnan(g_heatingAdaptive[i].flowErrorEwmaC) ? 0.0f : g_heatingAdaptive[i].flowErrorEwmaC);
    file.printf("hcSpreadEwma%u=%.6f\n", i,
      isnan(g_heatingAdaptive[i].spreadEwmaC) ? 0.0f : g_heatingAdaptive[i].spreadEwmaC);
    file.printf("hcMixerOsc%u=%.6f\n", i, g_heatingAdaptive[i].mixerOscillationScore);
    file.printf("hcDeadbandOffset%u=%.6f\n", i, g_heatingAdaptive[i].mixerDeadbandOffsetC);
  }

  file.printf("ovenOpeningBias=%.5f\n", g_ovenOpeningBias);
  file.printf(
    "ovenTempRate=%.6f\n",
    isnan(g_ovenAdaptive.temperatureRateCPerMin)
      ? 0.0f
      : g_ovenAdaptive.temperatureRateCPerMin
  );
  file.printf(
    "ovenErrorEwma=%.6f\n",
    isnan(g_ovenAdaptive.errorEwmaC)
      ? 0.0f
      : g_ovenAdaptive.errorEwmaC
  );
  file.printf(
    "ovenAbsErrorEwma=%.6f\n",
    isnan(g_ovenAdaptive.absErrorEwmaC)
      ? 0.0f
      : g_ovenAdaptive.absErrorEwmaC
  );
  file.printf("ovenOscillation=%.6f\n", g_ovenAdaptive.oscillationScore);
  file.printf("ovenDeadbandOffset=%.6f\n", g_ovenAdaptive.deadbandOffsetC);

  for (uint8_t d = 0; d < DOMAIN_COUNT; d++) {
    const DomainRuntime& rt = g_domains[d];

    file.printf("d%u_samples=%lu\n", d, (unsigned long)rt.samples);
    file.printf("d%u_days=%u\n", d, rt.days);
    file.printf("d%u_lastDay=%ld\n", d, (long)rt.lastDayKey);
    file.printf(
      "d%u_lastObs=%lu\n",
      d,
      (unsigned long)rt.lastObservationEpoch
    );
    file.printf("d%u_errorCount=%u\n", d, rt.errorCount);

    for (uint8_t j = 0; j < rt.errorCount; j++) {
      const uint8_t idx =
        (uint8_t)(
          (rt.errorHead + ERROR_WINDOW_PER_DOMAIN - rt.errorCount + j) %
          ERROR_WINDOW_PER_DOMAIN
        );

      file.printf(
        "d%u_e%u=%lu,%.4f\n",
        d,
        j,
        (unsigned long)rt.errors[idx].epoch,
        rt.errors[idx].errorPct
      );
    }
  }

  file.println("thermalModelVersion=1");
  file.printf("thermalOutsideCoeff=%.7f\n", g_thermalDemand.bufferOutsideCoeffKwPerK);
  file.printf("thermalValidIntervals=%lu\n", (unsigned long)g_thermalDemand.validIntervals);
  file.printf("thermalRejectedIntervals=%lu\n", (unsigned long)g_thermalDemand.rejectedIntervals);

  for (uint8_t profile = 0; profile < 2; profile++) {
    for (uint8_t hour = 0; hour < 24; hour++) {
      file.printf(
        "thermalB_%u_%u=%.6f,%u\n",
        profile,
        hour,
        g_thermalDemand.boilerRateKw[profile][hour],
        g_thermalDemand.boilerCount[profile][hour]
      );

      file.printf(
        "thermalP_%u_%u=%.6f,%u\n",
        profile,
        hour,
        g_thermalDemand.bufferBaseRateKw[profile][hour],
        g_thermalDemand.bufferCount[profile][hour]
      );
    }
  }

  file.close();

  g_runtimeDirty = false;
  g_lastRuntimeSaveMs = millis();
}

bool parseDomainKey(
  const String& key,
  uint8_t& domain,
  String& suffix
) {
  if (!key.startsWith("d")) return false;

  const int underscore = key.indexOf('_');
  if (underscore <= 1) return false;

  const int parsedDomain = key.substring(1, underscore).toInt();
  if (parsedDomain < 0 || parsedDomain >= DOMAIN_COUNT) return false;

  domain = (uint8_t)parsedDomain;
  suffix = key.substring(underscore + 1);
  return true;
}

void loadRuntime(const ConfigData& cfg) {
  resetModelParameters(cfg);
  resetDomainRuntime();

  int loadedVersion = 1;

  if (!SD.exists(FILE_ML_RUNTIME)) {
    g_loaded = true;
    return;
  }

  File file = SD.open(FILE_ML_RUNTIME, FILE_READ);
  if (!file) {
    g_loaded = true;
    return;
  }

  // Legacy V1 compatibility. Old samples/days/errors become SOLAR_YIELD V2.
  uint32_t legacySamples = 0;
  uint16_t legacyDays = 0;
  int32_t legacyLastDay = -1;
  ErrorPoint legacyErrors[ERROR_WINDOW_PER_DOMAIN] = {};
  uint8_t legacyErrorCount = 0;

  while (file.available()) {
    String line = file.readStringUntil('\n');
    line.trim();

    const int equals = line.indexOf('=');
    if (equals < 0) continue;

    const String key = line.substring(0, equals);
    const String value = line.substring(equals + 1);

    if (key == "modelVersion") {
      loadedVersion = value.toInt();
      continue;
    }

    if (key == "samples") {
      legacySamples = (uint32_t)value.toInt();
      continue;
    }

    if (key == "days") {
      legacyDays = (uint16_t)value.toInt();
      continue;
    }

    if (key == "lastDay") {
      legacyLastDay = value.toInt();
      continue;
    }

    if (key == "errorCount") {
      continue;
    }

    if (key == "agingLastEpoch") {
      g_lastAgingEpoch =
        (uint32_t)value.toInt();
      continue;
    }

    if (key.startsWith("solarSeason")) {
      const int index = key.substring(11).toInt();
      if (index >= 0 && index < SEASON_COUNT) {
        g_solarSeasonFactor[index] =
          clampf(
            value.toFloat(),
            SOLAR_SEASON_FACTOR_MIN,
            SOLAR_SEASON_FACTOR_MAX
          );
      }
      continue;
    }

    if (key.startsWith("thermalSeason")) {
      const int index = key.substring(13).toInt();
      if (index >= 0 && index < SEASON_COUNT) {
        g_thermalSeasonFactor[index] =
          clampf(
            value.toFloat(),
            THERMAL_SEASON_FACTOR_MIN,
            THERMAL_SEASON_FACTOR_MAX
          );
      }
      continue;
    }

    if (key.startsWith("w")) {
      const int index = key.substring(1).toInt();
      if (index >= 0 && index < FEATURE_COUNT) {
        g_weights[index] = value.toFloat();
      }
      continue;
    }

    if (key.startsWith("solarPwmBias")) {
      const int index = key.substring(12).toInt();
      if (index >= 0 && index < MAX_PUMPS) {
        g_solarPwmBias[index] = value.toFloat();
      }
      continue;
    }

    if (key.startsWith("pidKpScale")) {
      const int index = key.substring(10).toInt();
      if (index >= 0 && index < MAX_PUMPS)
        g_pidAdaptive[index].kpScale = clampf(value.toFloat(), PID_KP_SCALE_MIN, PID_KP_SCALE_MAX);
      continue;
    }
    if (key.startsWith("pidKiScale")) {
      const int index = key.substring(10).toInt();
      if (index >= 0 && index < MAX_PUMPS)
        g_pidAdaptive[index].kiScale = clampf(value.toFloat(), PID_KI_SCALE_MIN, PID_KI_SCALE_MAX);
      continue;
    }
    if (key.startsWith("pidKdScale")) {
      const int index = key.substring(10).toInt();
      if (index >= 0 && index < MAX_PUMPS)
        g_pidAdaptive[index].kdScale = clampf(value.toFloat(), PID_KD_SCALE_MIN, PID_KD_SCALE_MAX);
      continue;
    }
    if (key.startsWith("pidEwmaError")) {
      const int index = key.substring(12).toInt();
      if (index >= 0 && index < MAX_PUMPS) g_pidAdaptive[index].ewmaError = value.toFloat();
      continue;
    }
    if (key.startsWith("pidEwmaAbs")) {
      const int index = key.substring(10).toInt();
      if (index >= 0 && index < MAX_PUMPS) g_pidAdaptive[index].ewmaAbsError = value.toFloat();
      continue;
    }
    if (key.startsWith("pidEwmaDelta")) {
      const int index = key.substring(12).toInt();
      if (index >= 0 && index < MAX_PUMPS) g_pidAdaptive[index].ewmaDeltaError = value.toFloat();
      continue;
    }
    if (key.startsWith("pidOscillation")) {
      const int index = key.substring(14).toInt();
      if (index >= 0 && index < MAX_PUMPS)
        g_pidAdaptive[index].oscillationScore = clampf(value.toFloat(), 0.0f, 1.0f);
      continue;
    }

    if (key.startsWith("solarStartOffset")) {
      const int index = key.substring(16).toInt();
      if (index >= 0 && index < MAX_PUMPS) {
        g_solarControl[index].startOffsetC = value.toFloat();
      }
      continue;
    }

    if (key.startsWith("solarTargetOffset")) {
      const int index = key.substring(17).toInt();
      if (index >= 0 && index < MAX_PUMPS) {
        g_solarControl[index].targetOffsetC = value.toFloat();
      }
      continue;
    }

    if (key.startsWith("solarHystOffset")) {
      const int index = key.substring(15).toInt();
      if (index >= 0 && index < MAX_PUMPS) {
        g_solarControl[index].hysteresisOffsetC = value.toFloat();
      }
      continue;
    }

    if (key.startsWith("hcBias")) {
      const int index = key.substring(6).toInt();
      if (index >= 0 && index < MAX_HEATING_CIRCUITS) {
        g_hcBias[index] = value.toFloat();
      }
      continue;
    }

    if (key.startsWith("hcOutside")) {
      const int index = key.substring(9).toInt();
      if (index >= 0 && index < MAX_HEATING_CIRCUITS) {
        g_hcOutsideCoeff[index] = value.toFloat();
      }
      continue;
    }

    if (key.startsWith("hcRoomRate")) {
      const int index = key.substring(10).toInt();
      if (index >= 0 && index < MAX_HEATING_CIRCUITS)
        g_heatingAdaptive[index].roomRateCPerHour = clampf(value.toFloat(), -3.0f, 3.0f);
      continue;
    }

    if (key.startsWith("hcFlowEwma")) {
      const int index = key.substring(10).toInt();
      if (index >= 0 && index < MAX_HEATING_CIRCUITS)
        g_heatingAdaptive[index].flowErrorEwmaC = clampf(value.toFloat(), -10.0f, 10.0f);
      continue;
    }

    if (key.startsWith("hcSpreadEwma")) {
      const int index = key.substring(12).toInt();
      if (index >= 0 && index < MAX_HEATING_CIRCUITS)
        g_heatingAdaptive[index].spreadEwmaC = clampf(value.toFloat(), -20.0f, 30.0f);
      continue;
    }

    if (key.startsWith("hcMixerOsc")) {
      const int index = key.substring(10).toInt();
      if (index >= 0 && index < MAX_HEATING_CIRCUITS)
        g_heatingAdaptive[index].mixerOscillationScore = clampf(value.toFloat(), 0.0f, 1.0f);
      continue;
    }

    if (key.startsWith("hcDeadbandOffset")) {
      const int index = key.substring(16).toInt();
      if (index >= 0 && index < MAX_HEATING_CIRCUITS)
        g_heatingAdaptive[index].mixerDeadbandOffsetC = clampf(value.toFloat(), -0.20f, 0.80f);
      continue;
    }

    if (key == "ovenOpeningBias") {
      g_ovenOpeningBias = value.toFloat();
      continue;
    }

    if (key == "ovenTempRate") {
      g_ovenAdaptive.temperatureRateCPerMin =
        clampf(value.toFloat(), -5.0f, 5.0f);
      continue;
    }

    if (key == "ovenErrorEwma") {
      g_ovenAdaptive.errorEwmaC =
        clampf(value.toFloat(), -30.0f, 30.0f);
      continue;
    }

    if (key == "ovenAbsErrorEwma") {
      g_ovenAdaptive.absErrorEwmaC =
        clampf(value.toFloat(), 0.0f, 30.0f);
      continue;
    }

    if (key == "ovenOscillation") {
      g_ovenAdaptive.oscillationScore =
        clampf(value.toFloat(), 0.0f, 1.0f);
      continue;
    }

    if (key == "ovenDeadbandOffset") {
      g_ovenAdaptive.deadbandOffsetC =
        clampf(value.toFloat(), -1.0f, 3.0f);
      continue;
    }

    if (key == "thermalOutsideCoeff") {
      g_thermalDemand.bufferOutsideCoeffKwPerK =
        clampf(value.toFloat(), 0.0f, 0.50f);
      continue;
    }

    if (key == "thermalValidIntervals") {
      g_thermalDemand.validIntervals =
        (uint32_t)value.toInt();
      continue;
    }

    if (key == "thermalRejectedIntervals") {
      g_thermalDemand.rejectedIntervals =
        (uint32_t)value.toInt();
      continue;
    }

    if (key.startsWith("thermalB_") ||
        key.startsWith("thermalP_")) {
      const bool boiler =
        key.startsWith("thermalB_");

      const int firstUnderscore =
        key.indexOf('_');
      const int secondUnderscore =
        key.indexOf('_', firstUnderscore + 1);

      if (firstUnderscore > 0 &&
          secondUnderscore > firstUnderscore) {
        const int profile =
          key.substring(
            firstUnderscore + 1,
            secondUnderscore
          ).toInt();

        const int hour =
          key.substring(
            secondUnderscore + 1
          ).toInt();

        const int comma =
          value.indexOf(',');

        if (profile >= 0 &&
            profile < 2 &&
            hour >= 0 &&
            hour < 24 &&
            comma > 0) {
          const float rate =
            max(
              0.0f,
              value.substring(0, comma).toFloat()
            );

          const int parsedCount =
            value.substring(comma + 1).toInt();

          const uint16_t count =
            parsedCount > 0
              ? (uint16_t)min(parsedCount, 65535)
              : 0;

          if (boiler) {
            g_thermalDemand.boilerRateKw[profile][hour] =
              rate;
            g_thermalDemand.boilerCount[profile][hour] =
              count;
          } else {
            g_thermalDemand.bufferBaseRateKw[profile][hour] =
              rate;
            g_thermalDemand.bufferCount[profile][hour] =
              count;
          }
        }
      }

      continue;
    }

    uint8_t parsedDomain = 0;
    String suffix;

    if (parseDomainKey(key, parsedDomain, suffix)) {
      DomainRuntime& rt = g_domains[parsedDomain];

      if (suffix == "samples") {
        rt.samples = (uint32_t)value.toInt();
      } else if (suffix == "days") {
        rt.days = (uint16_t)value.toInt();
      } else if (suffix == "lastDay") {
        rt.lastDayKey = value.toInt();
      } else if (suffix == "lastObs") {
        rt.lastObservationEpoch =
          (uint32_t)value.toInt();
      } else if (suffix.startsWith("e")) {
        const int comma = value.indexOf(',');
        if (comma > 0 &&
            rt.errorCount < ERROR_WINDOW_PER_DOMAIN) {
          const ErrorPoint point = {
            (uint32_t)value.substring(0, comma).toInt(),
            value.substring(comma + 1).toFloat()
          };

          rt.errors[rt.errorHead] = point;
          rt.errorHead =
            (uint8_t)(
              (rt.errorHead + 1) % ERROR_WINDOW_PER_DOMAIN
            );
          rt.errorCount++;
        }
      }

      continue;
    }

    // Alte V1 Fehlerpunkte: e0=epoch,error
    if (loadedVersion < 2 &&
        key.length() > 1 &&
        key[0] == 'e' &&
        isDigit(key[1])) {
      const int comma = value.indexOf(',');

      if (comma > 0 &&
          legacyErrorCount < ERROR_WINDOW_PER_DOMAIN) {
        legacyErrors[legacyErrorCount++] = {
          (uint32_t)value.substring(0, comma).toInt(),
          value.substring(comma + 1).toFloat()
        };
      }
    }
  }

  file.close();

  if (loadedVersion < 2) {
    DomainRuntime& solar =
      g_domains[domainIndex(MlDomain::SOLAR_YIELD)];

    solar.samples = legacySamples;
    solar.days = legacyDays;
    solar.lastDayKey = legacyLastDay;

    for (uint8_t i = 0; i < legacyErrorCount; i++) {
      solar.lastObservationEpoch =
        max(
          solar.lastObservationEpoch,
          legacyErrors[i].epoch
        );

      solar.errors[solar.errorHead] = legacyErrors[i];
      solar.errorHead =
        (uint8_t)(
          (solar.errorHead + 1) % ERROR_WINDOW_PER_DOMAIN
        );
      solar.errorCount++;
    }

    // Nach dem ersten V2-Lauf wird das alte Runtimeformat automatisch in das
    // neue Domainformat geschrieben.
    g_runtimeDirty = true;
  }

  g_loaded = true;
}

void maturePending(const AppContext& ctx, uint32_t now) {
  for (PendingSample& sample : g_pending) {
    if (!sample.used || now < sample.dueEpoch) continue;

    const float measured =
      (float)(
        ctx.energyMeter.totalEnergyKWh -
        sample.startEnergyKWh
      );

    if (!sample.valid || measured < 0.0f) {
      appendSolarTrainingLog(
        sample.createdEpoch,
        sample.dueEpoch,
        sample.predictedKWh,
        max(0.0f, measured),
        NAN,
        sample.invalidReason[0]
          ? sample.invalidReason
          : "invalid"
      );

      sample.used = false;
      continue;
    }

    const float denominator = max(0.25f, measured);
    const float errorPct =
      fabsf(sample.predictedKWh - measured) /
      denominator *
      100.0f;

    float norm = 0.1f;
    for (float x : sample.features) {
      norm += x * x;
    }

    const float delta = measured - sample.predictedKWh;

    if (sample.predictedKWh > 0.05f) {
      const uint8_t season =
        seasonIndex(sample.createdEpoch);

      const float oldFactor =
        g_solarSeasonFactor[season];

      const float targetFactor =
        clampf(
          oldFactor *
          measured /
          max(0.05f, sample.predictedKWh),
          SOLAR_SEASON_FACTOR_MIN,
          SOLAR_SEASON_FACTOR_MAX
        );

      g_solarSeasonFactor[season] =
        clampf(
          0.95f * oldFactor +
          0.05f * targetFactor,
          SOLAR_SEASON_FACTOR_MIN,
          SOLAR_SEASON_FACTOR_MAX
        );
    }

    for (uint8_t i = 0; i < FEATURE_COUNT; i++) {
      g_weights[i] +=
        SOLAR_SGD_RATE *
        delta *
        sample.features[i] /
        norm;

      g_weights[i] =
        clampf(g_weights[i], -0.8f, 1.2f);
    }

    g_weights[0] =
      clampf(g_weights[0], 0.05f, 0.90f);

    observeDomain(
      MlDomain::SOLAR_YIELD,
      now,
      errorPct
    );

    g_status.lastMeasuredKWh = measured;
    g_status.lastPredictionErrorPercent = errorPct;

    appendSolarTrainingLog(
      sample.createdEpoch,
      sample.dueEpoch,
      sample.predictedKWh,
      measured,
      errorPct,
      "valid"
    );

    appendDomainLog(
      MlDomain::SOLAR_YIELD,
      now,
      errorPct,
      g_weights[0]
    );

    sample.used = false;
  }
}

void maybeCreateSolarYieldSample(
  const AppContext& ctx,
  bool testActive,
  uint32_t now
) {
  if (g_lastCreateMs &&
      (uint32_t)(millis() - g_lastCreateMs) <
        SOLAR_YIELD_SAMPLE_INTERVAL_MS) {
    return;
  }

  char reason[28];

  if (!commonSolarTrainingValid(
        ctx,
        testActive,
        reason,
        sizeof(reason)
      )) {
    return;
  }

  float features[FEATURE_COUNT];
  float incidentKWh = 0.0f;

  featuresFor(ctx, features, incidentKWh);

  if (incidentKWh < 0.05f) return;

  int slot = -1;

  for (uint8_t i = 0; i < MAX_PENDING; i++) {
    if (!g_pending[i].used) {
      slot = i;
      break;
    }
  }

  if (slot < 0) return;

  PendingSample& sample = g_pending[slot];
  sample = PendingSample{};

  sample.used = true;
  sample.valid = true;
  sample.createdEpoch = now;
  sample.dueEpoch = now + PREDICTION_HORIZON_S;
  sample.startEnergyKWh = ctx.energyMeter.totalEnergyKWh;

  for (uint8_t i = 0; i < FEATURE_COUNT; i++) {
    sample.features[i] = features[i];
  }

  sample.predictedKWh =
    seasonalSolarPredict(
      features,
      incidentKWh,
      now
    );

  g_lastCreateMs = millis();
}

bool intervalDue(
  MlDomain domain,
  uint32_t intervalMs
) {
  DomainRuntime& rt = g_domains[domainIndex(domain)];
  const uint32_t nowMs = millis();

  if (rt.lastSampleMs != 0 &&
      (uint32_t)(nowMs - rt.lastSampleMs) < intervalMs) {
    return false;
  }

  rt.lastSampleMs = nowMs;
  return true;
}

void learnSolarPid(
  const AppContext& ctx,
  bool testActive,
  uint32_t now
) {
  if (!safetyAllowsLearning(testActive)) return;
  if (!intervalDue(MlDomain::SOLAR_PID, SOLAR_PID_SAMPLE_INTERVAL_MS)) return;

  const int solarPumpIndex = findSolarPump(ctx);
  if (solarPumpIndex < 0) return;

  const PumpConfig& pump = ctx.config.pumps[solarPumpIndex];

  if (!pump.enabled ||
      pump.mode != PumpMode::PWM ||
      !pump.state ||
      isnan(pump.lastDiffC)) {
    return;
  }

  const float targetDiff = activePumpTargetDiff(pump);
  if (targetDiff <= 0.1f) return;

  const float errorC = pump.lastDiffC - targetDiff;
  const float normalizedError = errorC / max(1.0f, targetDiff);

  PidAdaptiveState& learned = g_pidAdaptive[solarPumpIndex];

  const float alpha = 0.20f;
  const float absError = fabsf(normalizedError);
  const float deltaError =
    learned.hasLastError
      ? normalizedError - learned.lastNormalizedError
      : 0.0f;

  const bool signChanged =
    learned.hasLastError &&
    normalizedError != 0.0f &&
    learned.lastNormalizedError != 0.0f &&
    ((normalizedError > 0.0f) != (learned.lastNormalizedError > 0.0f));

  if (!learned.hasLastError) {
    learned.ewmaError = normalizedError;
    learned.ewmaAbsError = absError;
    learned.ewmaDeltaError = fabsf(deltaError);
    learned.oscillationScore = 0.0f;
  } else {
    learned.ewmaError =
      (1.0f - alpha) * learned.ewmaError + alpha * normalizedError;
    learned.ewmaAbsError =
      (1.0f - alpha) * learned.ewmaAbsError + alpha * absError;
    learned.ewmaDeltaError =
      (1.0f - alpha) * learned.ewmaDeltaError + alpha * fabsf(deltaError);
    learned.oscillationScore =
      (1.0f - alpha) * learned.oscillationScore + alpha * (signChanged ? 1.0f : 0.0f);
  }

  if (learned.ewmaAbsError > 0.15f &&
      learned.oscillationScore < 0.25f) {
    learned.kpScale += 0.0030f;
  }
  if (learned.oscillationScore > 0.35f) {
    learned.kpScale -= 0.0060f;
  }

  if (pump.pidKi > 0.0f) {
    if (fabsf(learned.ewmaError) > 0.08f &&
        learned.oscillationScore < 0.25f) {
      learned.kiScale += 0.0020f;
    }
    if (learned.oscillationScore > 0.35f) {
      learned.kiScale -= 0.0040f;
    }
  } else {
    learned.kiScale = 1.0f;
  }

  if (pump.pidKd > 0.0f) {
    if (learned.oscillationScore > 0.25f ||
        learned.ewmaDeltaError > 0.20f) {
      learned.kdScale += 0.0030f;
    } else {
      learned.kdScale += (1.0f - learned.kdScale) * 0.01f;
    }
  } else {
    learned.kdScale = 1.0f;
  }

  learned.kpScale = clampf(learned.kpScale, PID_KP_SCALE_MIN, PID_KP_SCALE_MAX);
  learned.kiScale = clampf(learned.kiScale, PID_KI_SCALE_MIN, PID_KI_SCALE_MAX);
  learned.kdScale = clampf(learned.kdScale, PID_KD_SCALE_MIN, PID_KD_SCALE_MAX);

  learned.lastNormalizedError = normalizedError;
  learned.hasLastError = true;

  const float errorPct = absError * 100.0f;

  observeDomain(MlDomain::SOLAR_PID, now, errorPct);
  appendDomainLog(
    MlDomain::SOLAR_PID,
    now,
    errorPct,
    learned.kpScale
  );
}

bool findSolarRouteTargets(
  const PumpConfig& pump,
  int& boilerIndex,
  int& bufferIndex
) {
  boilerIndex = -1;
  bufferIndex = -1;

  for (uint8_t i = 0; i < PUMP_ROUTE_TARGET_COUNT; i++) {
    const PumpRouteTargetConfig& target = pump.targets[i];
    if (!target.enabled) continue;

    if (isBoilerRole(target.sinkRole)) boilerIndex = i;
    if (isBufferRole(target.sinkRole)) bufferIndex = i;
  }

  return boilerIndex >= 0 && bufferIndex >= 0;
}

void learnSolarControl(
  const AppContext& ctx,
  bool testActive,
  uint32_t now
) {
  if (!safetyAllowsLearning(testActive)) return;

  const int solarPumpIndex = findSolarPump(ctx);
  if (solarPumpIndex < 0) return;

  const PumpConfig& pump =
    ctx.config.pumps[solarPumpIndex];

  if (!pump.enabled ||
      pump.mode != PumpMode::PWM ||
      isnan(pump.lastDiffC)) {
    return;
  }

  SolarControlAdaptiveState& learned =
    g_solarControl[solarPumpIndex];

  const uint32_t nowMs = millis();

  if (!learned.stateInitialized) {
    learned.stateInitialized = true;
    learned.lastPumpState = pump.state;
    if (pump.state) learned.cycleStartedMs = nowMs;
  } else if (pump.state != learned.lastPumpState) {
    if (pump.state) {
      // Neuer Lauf: Startpunkt merken.
      learned.cycleStartedMs = nowMs;
    } else if (learned.cycleStartedMs != 0) {
      const uint32_t durationMs =
        (uint32_t)(nowMs - learned.cycleStartedMs);

      learned.lastCompletedCycleMs = durationMs;

      // Sehr kurze Zyklen: etwas spaeter starten und etwas weiter nachlaufen.
      if (durationMs < 5UL * 60UL * 1000UL) {
        learned.startOffsetC += 0.05f;
        learned.hysteresisOffsetC += 0.03f;
      }
      // Lange stabile Zyklen: vorsichtig etwas frueher Solarertrag abholen und
      // die Nachlaufspanne wieder Richtung kleinerer Werte bewegen.
      else if (durationMs > 30UL * 60UL * 1000UL) {
        learned.startOffsetC -= 0.02f;
        learned.hysteresisOffsetC -= 0.01f;
      }

      learned.cycleStartedMs = 0;
      g_runtimeDirty = true;
    }

    learned.lastPumpState = pump.state;
  }

  if (!pump.state) return;

  if (!intervalDue(
        MlDomain::SOLAR_CONTROL,
        SOLAR_CONTROL_SAMPLE_INTERVAL_MS
      )) {
    return;
  }

  const MlOptimizer::SolarControlParams current =
    solarControlParamsInternal(
      ctx,
      (uint8_t)solarPumpIndex,
      pump,
      true
    );

  const float target =
    max(1.0f, current.learnedTargetDiffC);

  const float errorC =
    pump.lastDiffC - target;

  const float errorPct =
    fabsf(errorC) / target * 100.0f;

  // TargetDiff lernt nur, wenn die Hydraulik offensichtlich an einer
  // Leistungsgrenze arbeitet. Damit konkurriert TargetDiff nicht permanent
  // mit Kp/Ki/Kd.
  const float nearMin =
    pump.minPwmPercent + 3.0f;

  const float nearMax =
    pump.maxPwmPercent - 3.0f;

  if (pump.lastPwmPercent >= nearMax &&
      errorC > 0.5f) {
    // Max-PWM und Spreizung immer noch zu gross -> physikalisch erreichbares
    // Ziel vorsichtig nach oben korrigieren.
    learned.targetOffsetC += 0.05f;
  } else if (pump.lastPwmPercent <= nearMin &&
             errorC < -0.5f) {
    // Min-PWM und Spreizung trotzdem zu klein -> Ziel vorsichtig senken.
    learned.targetOffsetC -= 0.05f;
  } else {
    // Ohne Sättigung langsam zum Benutzer-Basiswert zurueck.
    learned.targetOffsetC *= 0.995f;
  }

  // Alle Shadow-Werte sofort an den aktuell gueltigen Bounds halten.
  MlOptimizer::SolarControlParams bounded =
    solarControlParamsInternal(
      ctx,
      (uint8_t)solarPumpIndex,
      pump,
      true
    );

  learned.startOffsetC =
    bounded.learnedStartDiffC - pump.startDiff;
  learned.targetOffsetC =
    bounded.learnedTargetDiffC - pump.targetDiff;
  learned.hysteresisOffsetC =
    bounded.learnedHysteresisC - pump.hysteresis;

  observeDomain(
    MlDomain::SOLAR_CONTROL,
    now,
    errorPct
  );

  appendDomainLog(
    MlDomain::SOLAR_CONTROL,
    now,
    errorPct,
    bounded.learnedTargetDiffC
  );
}

void learnSolarRouting(
  const AppContext& ctx,
  bool testActive,
  uint32_t now
) {
  if (!safetyAllowsLearning(testActive)) return;
  if (!intervalDue(
        MlDomain::SOLAR_ROUTING,
        ROUTING_SAMPLE_INTERVAL_MS
      )) {
    return;
  }

  const int solarPumpIndex = findSolarPump(ctx);
  if (solarPumpIndex < 0) return;

  const PumpConfig& pump = ctx.config.pumps[solarPumpIndex];

  int boilerIndex = -1;
  int bufferIndex = -1;

  if (!findSolarRouteTargets(
        pump,
        boilerIndex,
        bufferIndex
      )) {
    return;
  }

  float boilerC = NAN;
  float bufferC = NAN;

  if (!readRole(
        ctx,
        pump.targets[boilerIndex].sinkRole,
        boilerC
      )) {
    return;
  }

  if (!readRole(
        ctx,
        pump.targets[bufferIndex].sinkRole,
        bufferC
      )) {
    return;
  }

  const float boilerMinimum =
    pump.targets[boilerIndex].minTempC;

  if (boilerMinimum > 0.01f &&
      boilerC < boilerMinimum) {
    return;
  }

  if (pump.targets[bufferIndex].maxTempC > 0.01f &&
      bufferC >= pump.targets[bufferIndex].maxTempC) {
    return;
  }

  // Routing V2 lernt nur an physikalisch tatsaechlich freien
  // Umschaltgelegenheiten. Als Qualitaetsmass dient der aktuell gemessene
  // Solarprognosefehler, weil genau diese Prognose spaeter die
  // Boiler/Puffer-Entscheidung korrigiert.
  const float inheritedError =
    g_status.lastPredictionErrorPercent;

  if (isnan(inheritedError)) return;

  observeDomain(
    MlDomain::SOLAR_ROUTING,
    now,
    inheritedError
  );

  appendDomainLog(
    MlDomain::SOLAR_ROUTING,
    now,
    inheritedError,
    bufferC - boilerC
  );
}

void learnHeatingCircuits(
  const AppContext& ctx,
  bool testActive,
  uint32_t now
) {
  if (!safetyAllowsLearning(testActive)) return;
  if (!intervalDue(MlDomain::HEATING, HEATING_SAMPLE_INTERVAL_MS)) return;

  bool observed = false;
  float errorSum = 0.0f;
  uint8_t errorCount = 0;

  for (uint8_t i = 0; i < MAX_HEATING_CIRCUITS; i++) {
    const HeatingCircuitConfig& cfg = ctx.config.heatingCircuits[i];
    const HeatingCircuitRuntime& rt = ctx.heatingCircuitRuntime[i];

    if (!cfg.enabled ||
        !cfg.roomControlEnabled ||
        isnan(rt.roomTemperatureC) ||
        isnan(rt.flowTemperatureC)) {
      continue;
    }

    HeatingAdaptiveState& a = g_heatingAdaptive[i];

    const float roomTarget =
      !isnan(rt.effectiveRoomTargetTemperatureC)
        ? rt.effectiveRoomTargetTemperatureC
        : cfg.roomTargetTemperatureC;

    const float roomErrorC = roomTarget - rt.roomTemperatureC;

    const float outsideC =
      isnan(rt.outsideTemperatureC) ? 10.0f : rt.outsideTemperatureC;

    const float outsideNorm =
      clampf((20.0f - outsideC) / 30.0f, -0.5f, 1.5f);

    float roomRate = a.roomRateCPerHour;

    if (a.initialized && a.lastEpoch > 0 && now > a.lastEpoch) {
      const float dtH = (float)(now - a.lastEpoch) / 3600.0f;

      if (dtH >= 0.05f && dtH <= 1.0f) {
        const float rawRate =
          clampf((rt.roomTemperatureC - a.lastRoomC) / dtH, -3.0f, 3.0f);

        roomRate =
          isnan(roomRate)
            ? rawRate
            : 0.80f * roomRate + 0.20f * rawRate;
      }
    }

    a.roomRateCPerHour = roomRate;

    const float projectedError =
      isnan(roomRate)
        ? roomErrorC
        : roomErrorC - roomRate;

    a.projectedRoomError1hC = projectedError;

    g_hcBias[i] =
      clampf(
        g_hcBias[i] + 0.018f * projectedError,
        -4.0f,
        4.0f
      );

    g_hcOutsideCoeff[i] =
      clampf(
        g_hcOutsideCoeff[i] +
          0.0018f * projectedError * outsideNorm,
        -1.5f,
        1.5f
      );

    const float flowErrorC =
      !isnan(rt.targetFlowTemperatureC)
        ? rt.targetFlowTemperatureC - rt.flowTemperatureC
        : 0.0f;

    a.flowErrorEwmaC =
      isnan(a.flowErrorEwmaC)
        ? flowErrorC
        : 0.80f * a.flowErrorEwmaC + 0.20f * flowErrorC;

    if (!isnan(rt.spreadTemperatureC)) {
      a.spreadEwmaC =
        isnan(a.spreadEwmaC)
          ? rt.spreadTemperatureC
          : 0.85f * a.spreadEwmaC + 0.15f * rt.spreadTemperatureC;
    }

    const bool signFlip =
      a.initialized &&
      !isnan(a.lastFlowErrorC) &&
      fabsf(flowErrorC) > 0.4f &&
      fabsf(a.lastFlowErrorC) > 0.4f &&
      ((flowErrorC > 0.0f && a.lastFlowErrorC < 0.0f) ||
       (flowErrorC < 0.0f && a.lastFlowErrorC > 0.0f));

    if (signFlip) {
      a.mixerOscillationScore =
        clampf(a.mixerOscillationScore + 0.10f, 0.0f, 1.0f);
    } else {
      a.mixerOscillationScore =
        clampf(a.mixerOscillationScore - 0.015f, 0.0f, 1.0f);
    }

    if (a.mixerOscillationScore > 0.35f) {
      a.mixerDeadbandOffsetC =
        clampf(a.mixerDeadbandOffsetC + 0.02f, -0.20f, 0.80f);
    } else if (
      fabsf(a.flowErrorEwmaC) > 1.5f &&
      a.mixerOscillationScore < 0.15f
    ) {
      a.mixerDeadbandOffsetC =
        clampf(a.mixerDeadbandOffsetC - 0.01f, -0.20f, 0.80f);
    } else {
      a.mixerDeadbandOffsetC *= 0.995f;
    }

    a.lastRoomC = rt.roomTemperatureC;
    a.lastFlowErrorC = flowErrorC;
    a.lastEpoch = now;
    a.initialized = true;

    // Qualitaet bleibt an realem Raumkomfort gemessen.
    const float errorPct =
      fabsf(roomErrorC) / max(1.0f, roomTarget) * 100.0f;

    errorSum += errorPct;
    errorCount++;
    observed = true;
  }

  if (!observed || errorCount == 0) return;

  const float meanError = errorSum / (float)errorCount;

  observeDomain(MlDomain::HEATING, now, meanError);
  appendDomainLog(MlDomain::HEATING, now, meanError, g_hcBias[0]);
  g_runtimeDirty = true;
}

bool readMeanOfRoles(
  const AppContext& ctx,
  const Ds18Role* roles,
  uint8_t roleCount,
  float& meanC,
  uint8_t& validCount
) {
  float sum = 0.0f;
  validCount = 0;
  meanC = NAN;

  for (uint8_t i = 0; i < roleCount; i++) {
    float value = NAN;
    if (!readRole(ctx, roles[i], value)) continue;
    sum += value;
    validCount++;
  }

  if (validCount == 0) return false;
  meanC = sum / (float)validCount;
  return true;
}

bool boilerThermalState(
  const AppContext& ctx,
  float& meanC,
  float& energyAbove20KWh,
  uint8_t& sensorCount
) {
  static const Ds18Role roles[] = {
    Ds18Role::SINK_BOILER_TOP,
    Ds18Role::BOILER_BOTTOM
  };

  if (!readMeanOfRoles(
        ctx,
        roles,
        sizeof(roles) / sizeof(roles[0]),
        meanC,
        sensorCount
      )) {
    energyAbove20KWh = NAN;
    return false;
  }

  if (ctx.config.boilerVolumeLiters <= 1.0f) {
    energyAbove20KWh = NAN;
    return false;
  }

  energyAbove20KWh =
    ctx.config.boilerVolumeLiters *
    max(0.0f, meanC - 20.0f) *
    1.163f /
    1000.0f;

  return true;
}

bool bufferThermalState(
  const AppContext& ctx,
  float& meanC,
  float& energyAbove20KWh,
  uint8_t& sensorCount
) {
  static const Ds18Role roles[] = {
    Ds18Role::SINK_BUFFER_TOP,
    Ds18Role::BUFFER_HIGH,
    Ds18Role::BUFFER_MID,
    Ds18Role::BUFFER_BOTTOM
  };

  if (!readMeanOfRoles(
        ctx,
        roles,
        sizeof(roles) / sizeof(roles[0]),
        meanC,
        sensorCount
      )) {
    energyAbove20KWh = NAN;
    return false;
  }

  if (ctx.config.bufferVolumeLiters <= 1.0f) {
    energyAbove20KWh = NAN;
    return false;
  }

  energyAbove20KWh =
    ctx.config.bufferVolumeLiters *
    max(0.0f, meanC - 20.0f) *
    1.163f /
    1000.0f;

  return true;
}

bool pumpTouchesStorage(const PumpConfig& pump) {
  if (!pump.enabled || !pump.state) return false;

  if (pump.sourceType == PumpSourceType::SENSOR_ROLE &&
      (isBoilerRole(pump.sourceSensorRole) ||
       isBufferRole(pump.sourceSensorRole))) {
    return true;
  }

  if (isBoilerRole(pump.sinkRole) ||
      isBufferRole(pump.sinkRole)) {
    return true;
  }

  if (pump.activeTargetIndex < PUMP_ROUTE_TARGET_COUNT) {
    const PumpRouteTargetConfig& target =
      pump.targets[pump.activeTargetIndex];

    if (target.enabled &&
        (isBoilerRole(target.sinkRole) ||
         isBufferRole(target.sinkRole))) {
      return true;
    }
  }

  return false;
}

bool thermalIntervalContaminated(
  const AppContext& ctx,
  bool testActive
) {
  if (testActive) return true;
  if (!safetyAllowsLearning(false)) return true;

  if (AuxHeater::heatingActive() ||
      AuxHeater::pumpActive()) {
    return true;
  }

  if (OvenControl::active()) {
    return true;
  }

  for (uint8_t i = 0; i < MAX_PUMPS; i++) {
    if (pumpTouchesStorage(ctx.config.pumps[i])) {
      return true;
    }
  }

  return false;
}

bool isWeekendEpoch(uint32_t epoch) {
  time_t t = (time_t)epoch;
  struct tm tmv = {};
  localtime_r(&t, &tmv);
  return tmv.tm_wday == 0 || tmv.tm_wday == 6;
}

uint8_t hourOfEpoch(uint32_t epoch) {
  time_t t = (time_t)epoch;
  struct tm tmv = {};
  localtime_r(&t, &tmv);
  return (uint8_t)tmv.tm_hour;
}

float outsideColdK(const AppContext& ctx) {
  return max(0.0f, 15.0f - outsideTemp(ctx));
}

float thermalPredictedBoilerRateKw(
  uint32_t epoch
) {
  const uint8_t profile =
    isWeekendEpoch(epoch) ? 1 : 0;
  const uint8_t hour =
    hourOfEpoch(epoch);

  const float base =
    max(
      0.0f,
      g_thermalDemand.boilerRateKw[profile][hour]
    );

  return
    base *
    g_thermalSeasonFactor[
      seasonIndex(epoch)
    ];
}

float thermalPredictedBufferRateKw(
  const AppContext& ctx,
  uint32_t epoch
) {
  const uint8_t profile =
    isWeekendEpoch(epoch) ? 1 : 0;
  const uint8_t hour =
    hourOfEpoch(epoch);

  const float base =
    max(
      0.0f,
      g_thermalDemand.bufferBaseRateKw[profile][hour]
    );

  const float predicted =
    max(
      0.0f,
      base +
        g_thermalDemand.bufferOutsideCoeffKwPerK *
        outsideColdK(ctx)
    );

  return
    predicted *
    g_thermalSeasonFactor[
      seasonIndex(epoch)
    ];
}

float thermalPredictDemandKWh(
  const AppContext& ctx,
  uint32_t now,
  uint8_t hours,
  bool boilerOnly,
  bool bufferOnly
) {
  float sum = 0.0f;

  for (uint8_t h = 0; h < hours; h++) {
    const uint32_t epoch =
      now + (uint32_t)h * 3600UL;

    if (!bufferOnly) {
      sum += thermalPredictedBoilerRateKw(epoch);
    }

    if (!boilerOnly) {
      sum += thermalPredictedBufferRateKw(ctx, epoch);
    }
  }

  return max(0.0f, sum);
}

float thermalPredictedDemandForRoleKWh(
  const AppContext& ctx,
  Ds18Role role,
  uint32_t now,
  uint8_t hours
) {
  if (isBoilerRole(role)) {
    return thermalPredictDemandKWh(
      ctx,
      now,
      hours,
      true,
      false
    );
  }

  if (isBufferRole(role)) {
    return thermalPredictDemandKWh(
      ctx,
      now,
      hours,
      false,
      true
    );
  }

  return 0.0f;
}

float storageEnergyAboveReferenceKWh(
  const ConfigData& cfg,
  Ds18Role role,
  float currentTemperatureC,
  float referenceTemperatureC
) {
  float volume = 0.0f;

  if (isBoilerRole(role)) {
    volume = cfg.boilerVolumeLiters;
  } else if (isBufferRole(role)) {
    volume = cfg.bufferVolumeLiters;
  }

  if (volume <= 1.0f ||
      isnan(currentTemperatureC)) {
    return 0.0f;
  }

  return
    volume *
    max(
      0.0f,
      currentTemperatureC -
      referenceTemperatureC
    ) *
    1.163f /
    1000.0f;
}

void appendThermalLog(
  uint32_t epoch,
  float boilerDemandKw,
  float bufferDemandKw,
  float predictedBoilerKw,
  float predictedBufferKw,
  float errorPct,
  float outsideC,
  const char* result
) {
  if (!SD.exists("/logs")) SD.mkdir("/logs");

  const bool headerNeeded =
    !SD.exists(FILE_ML_V2_THERMAL_LOG);

  File file =
    SD.open(FILE_ML_V2_THERMAL_LOG, FILE_APPEND);

  if (!file) return;

  if (headerNeeded || file.size() == 0) {
    file.println(
      "epoch,boiler_demand_kw,buffer_demand_kw,"
      "predicted_boiler_kw,predicted_buffer_kw,"
      "error_pct,outside_c,result"
    );
  }

  file.print(epoch);
  file.print(',');
  if (!isnan(boilerDemandKw)) file.print(boilerDemandKw, 4);
  file.print(',');
  if (!isnan(bufferDemandKw)) file.print(bufferDemandKw, 4);
  file.print(',');
  if (!isnan(predictedBoilerKw)) file.print(predictedBoilerKw, 4);
  file.print(',');
  if (!isnan(predictedBufferKw)) file.print(predictedBufferKw, 4);
  file.print(',');
  if (!isnan(errorPct)) file.print(errorPct, 2);
  file.print(',');
  if (!isnan(outsideC)) file.print(outsideC, 2);
  file.print(',');
  file.println(result);
  file.close();
}

void updateThermalProfile(
  float& value,
  uint16_t& count,
  float sample
) {
  if (sample < 0.0f || isnan(sample)) return;

  const float alpha =
    count < 8
      ? 0.30f
      : 0.08f;

  if (count == 0) {
    value = sample;
  } else {
    value =
      (1.0f - alpha) * value +
      alpha * sample;
  }

  if (count < 65535) count++;
}

void learnThermalDemand(
  const AppContext& ctx,
  bool testActive,
  uint32_t now
) {
  if (!TimeService::valid()) return;

  if (thermalIntervalContaminated(ctx, testActive)) {
    g_thermalDemand.intervalContaminated = true;
  }

  DomainRuntime& domain =
    g_domains[domainIndex(MlDomain::THERMAL_DEMAND)];

  const uint32_t nowMs = millis();

  if (domain.lastSampleMs != 0 &&
      (uint32_t)(nowMs - domain.lastSampleMs) <
        THERMAL_DEMAND_SAMPLE_INTERVAL_MS) {
    return;
  }

  domain.lastSampleMs = nowMs;

  float boilerMeanC = NAN;
  float boilerEnergy = NAN;
  uint8_t boilerSensors = 0;

  float bufferMeanC = NAN;
  float bufferEnergy = NAN;
  uint8_t bufferSensors = 0;

  const bool boilerValid =
    boilerThermalState(
      ctx,
      boilerMeanC,
      boilerEnergy,
      boilerSensors
    );

  const bool bufferValid =
    bufferThermalState(
      ctx,
      bufferMeanC,
      bufferEnergy,
      bufferSensors
    );

  if (!boilerValid && !bufferValid) {
    g_thermalDemand.rejectedIntervals++;
    g_thermalDemand.intervalContaminated = false;
    return;
  }

  if (!g_thermalDemand.initialized) {
    g_thermalDemand.initialized = true;
    g_thermalDemand.lastEpoch = now;
    g_thermalDemand.lastBoilerEnergyKWh = boilerEnergy;
    g_thermalDemand.lastBufferEnergyKWh = bufferEnergy;
    g_thermalDemand.intervalContaminated = false;
    return;
  }

  if (now <= g_thermalDemand.lastEpoch) {
    g_thermalDemand.lastEpoch = now;
    g_thermalDemand.lastBoilerEnergyKWh = boilerEnergy;
    g_thermalDemand.lastBufferEnergyKWh = bufferEnergy;
    g_thermalDemand.intervalContaminated = false;
    return;
  }

  const uint32_t dtS =
    now - g_thermalDemand.lastEpoch;

  const float dtH =
    (float)dtS / 3600.0f;

  const bool dtValid =
    dtH >= 0.15f &&
    dtH <= 0.60f;

  if (!dtValid ||
      g_thermalDemand.intervalContaminated) {
    g_thermalDemand.rejectedIntervals++;

    appendThermalLog(
      now,
      NAN,
      NAN,
      NAN,
      NAN,
      NAN,
      outsideTemp(ctx),
      g_thermalDemand.intervalContaminated
        ? "charging_or_safety"
        : "interval_invalid"
    );

    g_thermalDemand.lastEpoch = now;
    g_thermalDemand.lastBoilerEnergyKWh = boilerEnergy;
    g_thermalDemand.lastBufferEnergyKWh = bufferEnergy;
    g_thermalDemand.intervalContaminated = false;
    g_runtimeDirty = true;
    return;
  }

  const uint32_t midpoint =
    g_thermalDemand.lastEpoch + dtS / 2UL;

  const uint8_t profile =
    isWeekendEpoch(midpoint) ? 1 : 0;

  const uint8_t hour =
    hourOfEpoch(midpoint);

  float boilerDemandKw = NAN;
  float bufferDemandKw = NAN;
  bool anyDemand = false;

  if (boilerValid &&
      !isnan(g_thermalDemand.lastBoilerEnergyKWh)) {
    boilerDemandKw =
      max(
        0.0f,
        (
          g_thermalDemand.lastBoilerEnergyKWh -
          boilerEnergy
        ) / dtH
      );
    anyDemand = true;
  }

  if (bufferValid &&
      !isnan(g_thermalDemand.lastBufferEnergyKWh)) {
    bufferDemandKw =
      max(
        0.0f,
        (
          g_thermalDemand.lastBufferEnergyKWh -
          bufferEnergy
        ) / dtH
      );
    anyDemand = true;
  }

  if (!anyDemand) {
    g_thermalDemand.rejectedIntervals++;
    g_thermalDemand.lastEpoch = now;
    g_thermalDemand.lastBoilerEnergyKWh = boilerEnergy;
    g_thermalDemand.lastBufferEnergyKWh = bufferEnergy;
    g_thermalDemand.intervalContaminated = false;
    return;
  }

  const float predictedBoilerKw =
    thermalPredictedBoilerRateKw(midpoint);

  const float predictedBufferKw =
    thermalPredictedBufferRateKw(ctx, midpoint);

  const float actualTotalKw =
    (isnan(boilerDemandKw) ? 0.0f : boilerDemandKw) +
    (isnan(bufferDemandKw) ? 0.0f : bufferDemandKw);

  const float predictedTotalKw =
    (isnan(boilerDemandKw) ? 0.0f : predictedBoilerKw) +
    (isnan(bufferDemandKw) ? 0.0f : predictedBufferKw);

  const float errorPct =
    fabsf(
      predictedTotalKw - actualTotalKw
    ) /
    max(0.25f, actualTotalKw) *
    100.0f;

  if (predictedTotalKw > 0.10f &&
      actualTotalKw >= 0.0f) {
    const uint8_t season =
      seasonIndex(midpoint);

    const float oldFactor =
      g_thermalSeasonFactor[season];

    const float targetFactor =
      clampf(
        oldFactor *
        actualTotalKw /
        max(0.10f, predictedTotalKw),
        THERMAL_SEASON_FACTOR_MIN,
        THERMAL_SEASON_FACTOR_MAX
      );

    g_thermalSeasonFactor[season] =
      clampf(
        0.96f * oldFactor +
        0.04f * targetFactor,
        THERMAL_SEASON_FACTOR_MIN,
        THERMAL_SEASON_FACTOR_MAX
      );
  }

  if (!isnan(boilerDemandKw)) {
    updateThermalProfile(
      g_thermalDemand.boilerRateKw[profile][hour],
      g_thermalDemand.boilerCount[profile][hour],
      boilerDemandKw
    );
  }

  if (!isnan(bufferDemandKw)) {
    const float coldK =
      outsideColdK(ctx);

    const float basePrediction =
      g_thermalDemand.bufferBaseRateKw[profile][hour];

    const float residual =
      bufferDemandKw -
      (
        basePrediction +
        g_thermalDemand.bufferOutsideCoeffKwPerK *
        coldK
      );

    if (coldK > 0.5f) {
      g_thermalDemand.bufferOutsideCoeffKwPerK +=
        0.002f *
        residual *
        coldK /
        (1.0f + coldK * coldK);

      g_thermalDemand.bufferOutsideCoeffKwPerK =
        clampf(
          g_thermalDemand.bufferOutsideCoeffKwPerK,
          0.0f,
          0.50f
        );
    }

    const float baseSample =
      max(
        0.0f,
        bufferDemandKw -
        g_thermalDemand.bufferOutsideCoeffKwPerK *
        coldK
      );

    updateThermalProfile(
      g_thermalDemand.bufferBaseRateKw[profile][hour],
      g_thermalDemand.bufferCount[profile][hour],
      baseSample
    );
  }

  g_thermalDemand.lastBoilerDemandKw =
    boilerDemandKw;
  g_thermalDemand.lastBufferDemandKw =
    bufferDemandKw;
  g_thermalDemand.validIntervals++;

  observeDomain(
    MlDomain::THERMAL_DEMAND,
    now,
    errorPct
  );

  appendDomainLog(
    MlDomain::THERMAL_DEMAND,
    now,
    errorPct,
    actualTotalKw
  );

  appendThermalLog(
    now,
    boilerDemandKw,
    bufferDemandKw,
    predictedBoilerKw,
    predictedBufferKw,
    errorPct,
    outsideTemp(ctx),
    "valid"
  );

  g_thermalDemand.lastEpoch = now;
  g_thermalDemand.lastBoilerEnergyKWh = boilerEnergy;
  g_thermalDemand.lastBufferEnergyKWh = bufferEnergy;
  g_thermalDemand.intervalContaminated = false;
  g_runtimeDirty = true;
}

float storageVolumeForRole(
  const ConfigData& cfg,
  Ds18Role role
) {
  if (isBoilerRole(role)) return cfg.boilerVolumeLiters;
  if (isBufferRole(role)) return cfg.bufferVolumeLiters;
  return 0.0f;
}

float requiredStorageEnergyKWh(
  const ConfigData& cfg,
  Ds18Role role,
  float currentC,
  float targetC
) {
  const float volume =
    storageVolumeForRole(cfg, role);

  if (volume <= 1.0f || targetC <= currentC) {
    return 0.0f;
  }

  return
    volume *
    (targetC - currentC) *
    1.163f /
    1000.0f;
}

void learnAuxHeater(
  const AppContext& ctx,
  bool testActive,
  uint32_t now
) {
  if (!safetyAllowsLearning(testActive)) return;
  if (!intervalDue(MlDomain::AUX_HEATER, AUX_SAMPLE_INTERVAL_MS)) return;

  const AuxHeaterConfig& aux = ctx.config.auxHeater;

  if (!aux.enabled ||
      aux.sinkRole == Ds18Role::NONE ||
      !ctx.config.forecastEnabled ||
      !Forecast::summary().valid) {
    return;
  }

  float sinkC = NAN;
  if (!readRole(ctx, aux.sinkRole, sinkC)) return;

  // Nur Situationen sind Lernpunkte, in denen die Zusatzheizung thermisch
  // relevant werden koennte.
  if (sinkC >= aux.targetTemperatureC + aux.hysteresisC) {
    return;
  }

  const float inheritedError =
    g_status.lastPredictionErrorPercent;

  if (isnan(inheritedError)) return;

  observeDomain(
    MlDomain::AUX_HEATER,
    now,
    inheritedError
  );

  appendDomainLog(
    MlDomain::AUX_HEATER,
    now,
    inheritedError,
    sinkC
  );
}

void learnOven(
  const AppContext& ctx,
  bool testActive,
  uint32_t now
) {
  if (!safetyAllowsLearning(testActive)) return;
  if (!intervalDue(MlDomain::OVEN, OVEN_SAMPLE_INTERVAL_MS)) return;

  const OvenConfig& cfg = ctx.config.oven;

  if (!cfg.enabled ||
      !OvenControl::active() ||
      !OvenControl::targetTemperatureReached()) {
    return;
  }

  const float ovenC =
    ctx.sensors.heatSources.altSourceOvenC;

  if (!ctx.sensors.heatSources.altSourceOvenValid ||
      isnan(ovenC) ||
      cfg.targetOvenTemperatureC <= 1.0f) {
    return;
  }

  OvenAdaptiveState& a = g_ovenAdaptive;

  const float errorC =
    cfg.targetOvenTemperatureC - ovenC;

  float rateCPerMin = a.temperatureRateCPerMin;

  if (a.initialized &&
      a.lastEpoch > 0 &&
      now > a.lastEpoch &&
      !isnan(a.lastTemperatureC)) {
    const float dtMin =
      (float)(now - a.lastEpoch) / 60.0f;

    if (dtMin >= 1.0f && dtMin <= 30.0f) {
      const float rawRate =
        clampf(
          (ovenC - a.lastTemperatureC) / dtMin,
          -5.0f,
          5.0f
        );

      rateCPerMin =
        isnan(rateCPerMin)
          ? rawRate
          : 0.75f * rateCPerMin +
            0.25f * rawRate;
    }
  }

  a.temperatureRateCPerMin = rateCPerMin;

  const float predictedTemperature10MinC =
    isnan(rateCPerMin)
      ? ovenC
      : ovenC + rateCPerMin * 10.0f;

  const float predictedError10MinC =
    cfg.targetOvenTemperatureC -
    predictedTemperature10MinC;

  a.predictedError10MinC =
    predictedError10MinC;

  a.errorEwmaC =
    isnan(a.errorEwmaC)
      ? errorC
      : 0.80f * a.errorEwmaC +
        0.20f * errorC;

  a.absErrorEwmaC =
    isnan(a.absErrorEwmaC)
      ? fabsf(errorC)
      : 0.85f * a.absErrorEwmaC +
        0.15f * fabsf(errorC);

  const bool errorSignFlip =
    a.initialized &&
    !isnan(a.lastErrorC) &&
    fabsf(errorC) > 1.0f &&
    fabsf(a.lastErrorC) > 1.0f &&
    (
      (errorC > 0.0f && a.lastErrorC < 0.0f) ||
      (errorC < 0.0f && a.lastErrorC > 0.0f)
    );

  if (errorSignFlip) {
    a.oscillationScore =
      clampf(a.oscillationScore + 0.12f, 0.0f, 1.0f);
  } else {
    a.oscillationScore =
      clampf(a.oscillationScore - 0.02f, 0.0f, 1.0f);
  }

  g_ovenOpeningBias +=
    0.035f * predictedError10MinC;

  g_ovenOpeningBias =
    clampf(
      g_ovenOpeningBias,
      -OVEN_OPENING_BIAS_LIMIT,
      OVEN_OPENING_BIAS_LIMIT
    );

  if (a.oscillationScore > 0.35f) {
    a.deadbandOffsetC =
      clampf(a.deadbandOffsetC + 0.10f, -1.0f, 3.0f);
  } else if (
    !isnan(a.absErrorEwmaC) &&
    a.absErrorEwmaC > 4.0f &&
    a.oscillationScore < 0.15f
  ) {
    a.deadbandOffsetC =
      clampf(a.deadbandOffsetC - 0.05f, -1.0f, 3.0f);
  } else {
    a.deadbandOffsetC *= 0.995f;
  }

  a.lastTemperatureC = ovenC;
  a.lastErrorC = errorC;
  a.lastEpoch = now;
  a.initialized = true;

  const float errorPct =
    fabsf(errorC) /
    cfg.targetOvenTemperatureC *
    100.0f;

  observeDomain(
    MlDomain::OVEN,
    now,
    errorPct
  );

  appendDomainLog(
    MlDomain::OVEN,
    now,
    errorPct,
    g_ovenOpeningBias
  );

  g_runtimeDirty = true;
}

bool solarYieldPrerequisites(const AppContext& ctx) {
  return
    ctx.config.forecastEnabled &&
    Forecast::summary().valid &&
    TimeService::valid() &&
    ctx.config.energyMeter.enabled &&
    findSolarPump(ctx) >= 0 &&
    ctx.config.collectorApertureM2 >= 0.1f;
}

bool solarPidPrerequisites(const AppContext& ctx) {
  const int index = findSolarPump(ctx);
  if (index < 0) return false;

  const PumpConfig& pump = ctx.config.pumps[index];

  return
    pump.enabled &&
    pump.mode == PumpMode::PWM;
}

bool routingPrerequisites(const AppContext& ctx) {
  const int index = findSolarPump(ctx);
  if (index < 0) return false;

  int boiler = -1;
  int buffer = -1;

  return findSolarRouteTargets(
    ctx.config.pumps[index],
    boiler,
    buffer
  );
}

bool heatingPrerequisites(const AppContext& ctx) {
  for (uint8_t i = 0; i < MAX_HEATING_CIRCUITS; i++) {
    const HeatingCircuitConfig& cfg =
      ctx.config.heatingCircuits[i];

    if (cfg.enabled &&
        cfg.roomControlEnabled &&
        cfg.roomSensorRole != Ds18Role::NONE) {
      return true;
    }
  }

  return false;
}

bool auxPrerequisites(const AppContext& ctx) {
  return
    ctx.config.auxHeater.enabled &&
    ctx.config.auxHeater.sinkRole != Ds18Role::NONE &&
    ctx.config.forecastEnabled;
}

bool ovenPrerequisites(const AppContext& ctx) {
  return ctx.config.oven.enabled;
}

bool domainPrerequisites(
  const AppContext& ctx,
  MlDomain domain
) {
  switch (domain) {
    case MlDomain::SOLAR_YIELD:
      return solarYieldPrerequisites(ctx);

    case MlDomain::SOLAR_PID:
      return solarPidPrerequisites(ctx);

    case MlDomain::SOLAR_ROUTING:
      return routingPrerequisites(ctx);

    case MlDomain::HEATING:
      return heatingPrerequisites(ctx);

    case MlDomain::AUX_HEATER:
      return auxPrerequisites(ctx);

    case MlDomain::OVEN:
      return ovenPrerequisites(ctx);

    case MlDomain::SOLAR_CONTROL:
      return solarPidPrerequisites(ctx);

    case MlDomain::THERMAL_DEMAND:
      return
        TimeService::valid() &&
        (
          ctx.config.boilerVolumeLiters > 1.0f ||
          ctx.config.bufferVolumeLiters > 1.0f
        );

    default:
      return false;
  }
}

bool domainControlAvailable(MlDomain domain) {
  // Alle V2-Core-Domaenen haben bereits eine konkrete Overlay-Schnittstelle.
  switch (domain) {
    case MlDomain::SOLAR_YIELD:
      return true; // liefert Basis fuer Routing/Aux
    case MlDomain::SOLAR_PID:
    case MlDomain::SOLAR_ROUTING:
    case MlDomain::HEATING:
    case MlDomain::AUX_HEATER:
    case MlDomain::OVEN:
    case MlDomain::SOLAR_CONTROL:
      return true;

    case MlDomain::THERMAL_DEMAND:
      return false;

    default:
      return false;
  }
}

bool thermalDemandQualityReadyForAssist(
  const AppContext& ctx
) {
  if (ctx.config.mlMode != MlMode::AUTOMATIC) {
    return false;
  }

  if (!domainPrerequisites(
        ctx,
        MlDomain::THERMAL_DEMAND
      )) {
    return false;
  }

  const DomainRuntime& rt =
    g_domains[
      domainIndex(MlDomain::THERMAL_DEMAND)
    ];

  if (rt.samples < MIN_TRAINING_SAMPLES ||
      rt.days < MIN_TRAINING_DAYS) {
    return false;
  }

  uint16_t windowCount = 0;
  float meanError = NAN;

  calculateDomainWindow(
    MlDomain::THERMAL_DEMAND,
    TimeService::valid()
      ? (uint32_t)time(nullptr)
      : 0,
    windowCount,
    meanError
  );

  return
    windowCount >= MIN_WINDOW_SAMPLES &&
    !isnan(meanError) &&
    meanError < MAX_WINDOW_ERROR_PCT;
}

bool domainDependencyReady(
  const AppContext& ctx,
  MlDomain domain
) {
  switch (domain) {
    case MlDomain::SOLAR_ROUTING:
    case MlDomain::AUX_HEATER:
      return
        g_status.domains[
          domainIndex(MlDomain::SOLAR_YIELD)
        ].active &&
        thermalDemandQualityReadyForAssist(ctx);

    case MlDomain::SOLAR_CONTROL:
      // Betriebsparameter werden erst nach einem stabil freigegebenen
      // Kp/Ki/Kd-Regler wirksam. Lernen darf vorher bereits stattfinden.
      return
        g_status.domains[
          domainIndex(MlDomain::SOLAR_PID)
        ].active;

    default:
      return true;
  }
}

bool domainQualityReady(
  const AppContext& ctx,
  MlDomain domain,
  uint16_t& windowCount,
  float& meanError
) {
  const uint32_t now =
    TimeService::valid()
      ? (uint32_t)time(nullptr)
      : 0;

  calculateDomainWindow(
    domain,
    now,
    windowCount,
    meanError
  );

  const DomainRuntime& rt =
    g_domains[domainIndex(domain)];

  const bool fresh =
    now > 0 &&
    rt.lastObservationEpoch > 0 &&
    now >= rt.lastObservationEpoch &&
    (now - rt.lastObservationEpoch) <= MODEL_STALE_S;

  return
    ctx.config.mlMode == MlMode::AUTOMATIC &&
    domainPrerequisites(ctx, domain) &&
    domainControlAvailable(domain) &&
    rt.samples >= MIN_TRAINING_SAMPLES &&
    rt.days >= MIN_TRAINING_DAYS &&
    windowCount >= MIN_WINDOW_SAMPLES &&
    !isnan(meanError) &&
    meanError < MAX_WINDOW_ERROR_PCT &&
    fresh &&
    domainDependencyReady(ctx, domain);
}

void fillDomainStatus(
  const AppContext& ctx,
  MlDomain domain
) {
  MlDomainStatus& status =
    g_status.domains[domainIndex(domain)];

  memset(&status, 0, sizeof(status));

  strncpy(
    status.name,
    domainName(domain),
    sizeof(status.name) - 1
  );

  const bool prereq =
    domainPrerequisites(ctx, domain);

  status.enabled =
    ctx.config.mlMode != MlMode::OFF &&
    prereq;

  status.learning = status.enabled;

  status.controlAvailable =
    domainControlAvailable(domain);

  const DomainRuntime& rt =
    g_domains[domainIndex(domain)];

  status.trainingSamples = rt.samples;
  status.trainingDays = rt.days;

  uint16_t windowCount = 0;
  float meanError = NAN;

  const bool ready =
    domainQualityReady(
      ctx,
      domain,
      windowCount,
      meanError
    );

  status.windowSamples7d = windowCount;
  status.meanError7dPercent = meanError;
  status.active = ready;

  status.fallback =
    status.controlAvailable &&
    ctx.config.mlMode == MlMode::AUTOMATIC &&
    status.enabled &&
    !ready &&
    rt.samples >= MIN_TRAINING_SAMPLES;

  if (!status.enabled) {
    strncpy(
      status.reason,
      prereq ? "ml_off" : "prerequisite_missing",
      sizeof(status.reason) - 1
    );
  } else if (ctx.config.mlMode == MlMode::LEARN_ONLY) {
    strncpy(
      status.reason,
      "learn_only",
      sizeof(status.reason) - 1
    );
  } else if (rt.samples < MIN_TRAINING_SAMPLES) {
    snprintf(
      status.reason,
      sizeof(status.reason),
      "samples_%lu_of_%lu",
      (unsigned long)rt.samples,
      (unsigned long)MIN_TRAINING_SAMPLES
    );
  } else if (rt.days < MIN_TRAINING_DAYS) {
    snprintf(
      status.reason,
      sizeof(status.reason),
      "days_%u_of_%u",
      rt.days,
      MIN_TRAINING_DAYS
    );
  } else if (windowCount < MIN_WINDOW_SAMPLES) {
    snprintf(
      status.reason,
      sizeof(status.reason),
      "quality_samples_%u_of_%u",
      windowCount,
      MIN_WINDOW_SAMPLES
    );
  } else if (isnan(meanError) ||
             meanError >= MAX_WINDOW_ERROR_PCT) {
    snprintf(
      status.reason,
      sizeof(status.reason),
      "quality_%.1f_pct",
      meanError
    );
  } else {
    const uint32_t nowEpoch =
      TimeService::valid()
        ? (uint32_t)time(nullptr)
        : 0;

    const bool stale =
      nowEpoch == 0 ||
      rt.lastObservationEpoch == 0 ||
      nowEpoch < rt.lastObservationEpoch ||
      (nowEpoch - rt.lastObservationEpoch) > MODEL_STALE_S;

    if (stale) {
      strncpy(
        status.reason,
        "model_stale",
        sizeof(status.reason) - 1
      );
    } else if (!domainDependencyReady(ctx, domain)) {
    strncpy(
      status.reason,
      "dependency_not_ready",
      sizeof(status.reason) - 1
    );
    } else if (!status.controlAvailable) {
      strncpy(
        status.reason,
        "monitor_only",
        sizeof(status.reason) - 1
      );
    } else {
      strncpy(
        status.reason,
        "ready",
        sizeof(status.reason) - 1
      );
    }
  }
}

void refreshStatus(const AppContext& ctx) {
  float features[FEATURE_COUNT];
  float incidentKWh = 0.0f;

  featuresFor(ctx, features, incidentKWh);

  const uint32_t predictionEpoch =
    TimeService::valid()
      ? (uint32_t)time(nullptr)
      : 0;

  const float predicted =
    seasonalSolarPredict(
      features,
      incidentKWh,
      predictionEpoch
    );

  g_status.enabled =
    ctx.config.mlMode != MlMode::OFF;

  g_status.learning = false;
  g_status.active = false;
  g_status.fallback = false;
  g_status.readyDomainCount = 0;
  g_status.enabledDomainCount = 0;

  for (uint8_t d = 0; d < DOMAIN_COUNT; d++) {
    fillDomainStatus(
      ctx,
      (MlDomain)d
    );

    const MlDomainStatus& ds =
      g_status.domains[d];

    if (ds.enabled) g_status.enabledDomainCount++;
    if (ds.learning) g_status.learning = true;
    if (ds.active) {
      g_status.active = true;
      g_status.readyDomainCount++;
    }
    if (ds.fallback) g_status.fallback = true;
  }

  const MlDomainStatus& solarYield =
    g_status.domains[
      domainIndex(MlDomain::SOLAR_YIELD)
    ];

  g_status.trainingSamples =
    solarYield.trainingSamples;

  g_status.trainingDays =
    solarYield.trainingDays;

  g_status.windowSamples7d =
    solarYield.windowSamples7d;

  g_status.meanError7dPercent =
    solarYield.meanError7dPercent;

  g_status.predictedNext3hKWh = predicted;
  g_status.conservativeNext3hKWh =
    predicted * CONTROL_CONFIDENCE_FACTOR;

  if (ctx.config.mlMode == MlMode::OFF) {
    snprintf(
      g_status.recommendation,
      sizeof(g_status.recommendation),
      "ML V2 aus: Grundregelung bleibt allein aktiv."
    );
  } else if (ctx.config.mlMode == MlMode::LEARN_ONLY) {
    snprintf(
      g_status.recommendation,
      sizeof(g_status.recommendation),
      "ML V2 lernt modular: %u Domaenen aktiv, automatische Eingriffe gesperrt.",
      g_status.enabledDomainCount
    );
  } else {
    snprintf(
      g_status.recommendation,
      sizeof(g_status.recommendation),
      "ML V2: %u/%u Domaenen automatisch freigegeben. Jede Domaene braucht >=200 Punkte, >=7 Tage und <10%% 7-Tage-Fehler.",
      g_status.readyDomainCount,
      g_status.enabledDomainCount
    );
  }
}

bool currentSafetyBlocksControl() {
  const auto& safety = SafetyManager::status();

  return
    safety.blockNormalPumpControl ||
    safety.storageCriticalOvertemperatureActive ||
    safety.sensorFaultActive;
}

bool domainControlAllowed(MlDomain domain) {
  return
    g_status.domains[domainIndex(domain)].active &&
    !currentSafetyBlocksControl();
}

void maybeSaveRuntime() {
  if (!g_runtimeDirty) return;

  const uint32_t nowMs = millis();

  if (g_lastRuntimeSaveMs != 0 &&
      (uint32_t)(nowMs - g_lastRuntimeSaveMs) <
        RUNTIME_SAVE_INTERVAL_MS) {
    return;
  }

  saveRuntime();
}

} // namespace

namespace MlOptimizer {

void begin(AppContext& ctx) {
  memset(&g_status, 0, sizeof(g_status));

  for (PendingSample& pending : g_pending) {
    pending = PendingSample{};
  }

  loadRuntime(ctx.config);
  refreshStatus(ctx);

  // Eine migrierte V1-Datei zeitnah als V2 persistieren.
  if (g_runtimeDirty) {
    saveRuntime();
  }
}

void process(
  AppContext& ctx,
  bool testActive
) {
  if (!g_loaded) {
    loadRuntime(ctx.config);
  }

  const uint32_t nowMs = millis();

  if (g_lastProcessMs != 0 &&
      (uint32_t)(nowMs - g_lastProcessMs) <
        PROCESS_INTERVAL_MS) {
    return;
  }

  g_lastProcessMs = nowMs;

  if (ctx.config.mlMode == MlMode::OFF) {
    refreshStatus(ctx);
    return;
  }

  const uint32_t now =
    TimeService::valid()
      ? (uint32_t)time(nullptr)
      : 0;

  if (!now) {
    refreshStatus(ctx);
    return;
  }

  applyDailyModelAging(now);

  char reason[28];

  if (!commonSolarTrainingValid(
        ctx,
        testActive,
        reason,
        sizeof(reason)
      )) {
    invalidatePending(reason);
  }

  const int solarPumpIndex = findSolarPump(ctx);

  if (solarPumpIndex >= 0 &&
      ctx.config.pumps[solarPumpIndex].state &&
      (
        !ctx.energyMeter.temperatureValid ||
        ctx.energyMeter.flowLitersPerMinute < 0.01f
      )) {
    invalidatePending("meter_invalid");
  }

  // Jede Domaene sammelt ab Tag 1 unabhaengig ihre eigenen Daten.
  maturePending(ctx, now);
  maybeCreateSolarYieldSample(ctx, testActive, now);
  learnSolarPid(ctx, testActive, now);
  learnSolarControl(ctx, testActive, now);
  learnSolarRouting(ctx, testActive, now);
  learnHeatingCircuits(ctx, testActive, now);
  learnAuxHeater(ctx, testActive, now);
  learnOven(ctx, testActive, now);
  learnThermalDemand(ctx, testActive, now);

  refreshStatus(ctx);
  maybeSaveRuntime();
}

void resetLearning(AppContext& ctx) {
  for (PendingSample& pending : g_pending) {
    pending = PendingSample{};
  }

  resetDomainRuntime();
  resetModelParameters(ctx.config);

  g_lastCreateMs = 0;
  g_lastProcessMs = 0;
  g_lastRuntimeSaveMs = 0;
  g_auxDelayStartedMs = 0;

  g_status.lastMeasuredKWh = NAN;
  g_status.lastPredictionErrorPercent = NAN;
  g_status.lastDecision[0] = '\0';

  if (SD.exists(FILE_ML_RUNTIME)) {
    SD.remove(FILE_ML_RUNTIME);
  }

  if (SD.exists(FILE_FORECAST_LEARNING)) {
    SD.remove(FILE_FORECAST_LEARNING);
  }

  g_runtimeDirty = true;
  saveRuntime();
  refreshStatus(ctx);
}

Status status() {
  return g_status;
}

DomainStatus domainStatus(Domain domain) {
  const uint8_t index = domainIndex(domain);

  if (index >= DOMAIN_COUNT) {
    return DomainStatus{};
  }

  return g_status.domains[index];
}

bool domainActive(Domain domain) {
  const uint8_t index = domainIndex(domain);
  if (index >= DOMAIN_COUNT) return false;
  return g_status.domains[index].active;
}

bool shouldDelayAuxHeater(
  const AppContext& ctx,
  float currentTemperatureC
) {
  if (!domainControlAllowed(MlDomain::AUX_HEATER) ||
      !ctx.config.forecastAuxDelayEnabled ||
      isnan(g_status.conservativeNext3hKWh)) {
    g_auxDelayStartedMs = 0;
    return false;
  }

  const AuxHeaterConfig& aux =
    ctx.config.auxHeater;

  const float needKWh =
    requiredStorageEnergyKWh(
      ctx.config,
      aux.sinkRole,
      currentTemperatureC,
      aux.minimumTemperatureC
    );

  if (needKWh <= 0.01f) {
    g_auxDelayStartedMs = 0;
    return false;
  }

  const uint32_t nowEpoch =
    TimeService::valid()
      ? (uint32_t)time(nullptr)
      : 0;

  const float predictedDemand3hKWh =
    nowEpoch
      ? thermalPredictedDemandForRoleKWh(
          ctx,
          aux.sinkRole,
          nowEpoch,
          3
        )
      : 0.0f;

  // Die vorhandene Mindesttemperatur bleibt die harte Komfortbasis.
  // Fuer eine Verzoegerung muss der konservative Solarertrag jetzt sowohl
  // die unmittelbare Aufheizenergie als auch den erwarteten Verbrauch der
  // naechsten drei Stunden abdecken.
  const float requiredWithDemandKWh =
    needKWh +
    max(0.0f, predictedDemand3hKWh);

  const bool solarOpportunity =
    g_status.conservativeNext3hKWh >=
    requiredWithDemandKWh;

  if (!solarOpportunity) {
    g_auxDelayStartedMs = 0;

    snprintf(
      g_status.lastDecision,
      sizeof(g_status.lastDecision),
      "ML V2 Zusatzheizung: Start freigeben (Solar %.2f, Aufheizen %.2f, Last3h %.2f kWh)",
      g_status.conservativeNext3hKWh,
      needKWh,
      predictedDemand3hKWh
    );

    return false;
  }

  const uint32_t nowMs = millis();

  if (g_auxDelayStartedMs == 0) {
    g_auxDelayStartedMs = nowMs;
  }

  const uint32_t maxWait =
    max(
      (uint32_t)60000UL,
      (uint32_t)ctx.config.forecastAuxMaxWaitMs
    );

  if ((uint32_t)(nowMs - g_auxDelayStartedMs) >= maxWait) {
    g_auxDelayStartedMs = 0;

    snprintf(
      g_status.lastDecision,
      sizeof(g_status.lastDecision),
      "ML V2 Zusatzheizung: maximale Nutzer-Wartezeit erreicht"
    );

    return false;
  }

  snprintf(
    g_status.lastDecision,
    sizeof(g_status.lastDecision),
    "ML V2 Zusatzheizung: warten (Solar %.2f, Aufheizen %.2f, Last3h %.2f kWh)",
    g_status.conservativeNext3hKWh,
    needKWh,
    predictedDemand3hKWh
  );

  return true;
}

bool shouldStopAuxHeaterEarly(
  const AppContext& ctx,
  float currentTemperatureC
) {
  if (!domainControlAllowed(MlDomain::AUX_HEATER) ||
      !ctx.config.forecastAuxDelayEnabled ||
      isnan(g_status.conservativeNext3hKWh)) {
    return false;
  }

  const AuxHeaterConfig& aux =
    ctx.config.auxHeater;

  // Harte Komfortgrenze: niemals unter der konfigurierten Mindesttemperatur
  // wegen ML vorzeitig abschalten.
  if (currentTemperatureC < aux.minimumTemperatureC) {
    return false;
  }

  const float target =
    aux.targetTemperatureC +
    max(0.0f, aux.hysteresisC);

  const float needKWh =
    requiredStorageEnergyKWh(
      ctx.config,
      aux.sinkRole,
      currentTemperatureC,
      target
    );

  if (needKWh <= 0.01f) return false;

  const uint32_t nowEpoch =
    TimeService::valid()
      ? (uint32_t)time(nullptr)
      : 0;

  const float predictedDemand3hKWh =
    nowEpoch
      ? thermalPredictedDemandForRoleKWh(
          ctx,
          aux.sinkRole,
          nowEpoch,
          3
        )
      : 0.0f;

  const float requiredWithDemandKWh =
    needKWh +
    max(0.0f, predictedDemand3hKWh);

  const bool stopEarly =
    g_status.conservativeNext3hKWh >=
    requiredWithDemandKWh * 1.10f;

  if (stopEarly) {
    snprintf(
      g_status.lastDecision,
      sizeof(g_status.lastDecision),
      "ML V2 Zusatzheizung frueh AUS: Solar %.2f >= Rest %.2f + Last3h %.2f kWh",
      g_status.conservativeNext3hKWh,
      needKWh,
      predictedDemand3hKWh
    );
  }

  return stopEarly;
}

bool preferBufferForSolar(
  const AppContext& ctx,
  const PumpConfig& pump
) {
  if (!domainControlAllowed(MlDomain::SOLAR_ROUTING) ||
      !ctx.config.forecastSolarPriorityEnabled ||
      pump.sourceType != PumpSourceType::HEAT_SOURCE_ROLE ||
      !isSolarRole(pump.sourceRole)) {
    return false;
  }

  int boilerIndex = -1;
  int bufferIndex = -1;

  if (!findSolarRouteTargets(
        pump,
        boilerIndex,
        bufferIndex
      )) {
    return false;
  }

  float boilerC = NAN;

  if (!readRole(
        ctx,
        pump.targets[boilerIndex].sinkRole,
        boilerC
      )) {
    return false;
  }

  const float boilerMinimum =
    pump.targets[boilerIndex].minTempC;

  // Harte Valve-V2-Prioritaet bleibt unangetastet.
  if (boilerMinimum <= 0.01f ||
      boilerC < boilerMinimum) {
    return false;
  }

  float bufferC = NAN;

  if (!readRole(
        ctx,
        pump.targets[bufferIndex].sinkRole,
        bufferC
      )) {
    return false;
  }

  if (pump.targets[bufferIndex].maxTempC > 0.01f &&
      bufferC >= pump.targets[bufferIndex].maxTempC) {
    return false;
  }

  if (isnan(g_status.conservativeNext3hKWh) ||
      g_status.conservativeNext3hKWh <= 0.25f ||
      !TimeService::valid()) {
    return false;
  }

  const uint32_t nowEpoch =
    (uint32_t)time(nullptr);

  const float boilerDemand3hKWh =
    thermalPredictedDemandForRoleKWh(
      ctx,
      pump.targets[boilerIndex].sinkRole,
      nowEpoch,
      3
    );

  const float bufferDemand3hKWh =
    thermalPredictedDemandForRoleKWh(
      ctx,
      pump.targets[bufferIndex].sinkRole,
      nowEpoch,
      3
    );

  // Boilerreserve wird bewusst nur oberhalb der harten
  // Mindesttemperatur bewertet. Damit kann ML niemals die Valve-V2-
  // Mindesttemperatur-Prioritaet aushebeln.
  const float boilerReserveKWh =
    storageEnergyAboveReferenceKWh(
      ctx.config,
      pump.targets[boilerIndex].sinkRole,
      boilerC,
      boilerMinimum
    );

  // Fuer den Puffer wird dieselbe 20-C-Referenz wie im Thermal-Demand-
  // Modell verwendet. Sie ist nur eine relative Vergleichsbasis.
  const float bufferReserveKWh =
    storageEnergyAboveReferenceKWh(
      ctx.config,
      pump.targets[bufferIndex].sinkRole,
      bufferC,
      20.0f
    );

  const float boilerUncoveredKWh =
    max(
      0.0f,
      boilerDemand3hKWh -
      boilerReserveKWh
    );

  const float bufferUncoveredKWh =
    max(
      0.0f,
      bufferDemand3hKWh -
      bufferReserveKWh
    );

  // Thermal Demand darf Boiler nur dann zugunsten des Puffers
  // zurueckstellen, wenn die prognostizierte Boilerlast durch die Reserve
  // oberhalb der Mindesttemperatur abgedeckt ist.
  if (boilerUncoveredKWh > 0.10f) {
    snprintf(
      g_status.lastDecision,
      sizeof(g_status.lastDecision),
      "ML V2 Solarziel: Boilerreserve schuetzen (Last3h %.2f, Reserve %.2f kWh)",
      boilerDemand3hKWh,
      boilerReserveKWh
    );
    return false;
  }

  const bool bufferPressureHigher =
    bufferUncoveredKWh >
      boilerUncoveredKWh + 0.10f ||
    (
      bufferDemand3hKWh >
        boilerDemand3hKWh + 0.25f &&
      bufferReserveKWh <
        bufferDemand3hKWh * 2.0f
    );

  const bool preferBuffer =
    bufferPressureHigher ||
    (
      boilerDemand3hKWh <= 0.10f &&
      bufferDemand3hKWh > 0.10f
    );

  if (preferBuffer) {
    snprintf(
      g_status.lastDecision,
      sizeof(g_status.lastDecision),
      "ML V2 Solarziel: Puffer (Last3h Boi %.2f / Puf %.2f kWh)",
      boilerDemand3hKWh,
      bufferDemand3hKWh
    );
  }

  return preferBuffer;
}

PidGains effectiveSolarPidGains(
  const AppContext& ctx,
  uint8_t pumpIndex,
  const PumpConfig& pump
) {
  const bool applyLearned =
    pumpIndex < MAX_PUMPS &&
    domainControlAllowed(MlDomain::SOLAR_PID) &&
    pump.enabled &&
    pump.mode == PumpMode::PWM &&
    isSolarRole(pump.sourceRole);

  return effectivePidGainsInternal(pumpIndex, pump, applyLearned);
}

PidTuningStatus pidTuningStatus(
  const AppContext& ctx,
  uint8_t pumpIndex,
  const PumpConfig& pump
) {
  PidTuningStatus status;
  status.available =
    pumpIndex < MAX_PUMPS &&
    pump.enabled &&
    pump.mode == PumpMode::PWM &&
    isSolarRole(pump.sourceRole);

  status.active =
    status.available &&
    domainControlAllowed(MlDomain::SOLAR_PID);

  status.baseKp = pump.pidKp;
  status.baseKi = pump.pidKi;
  status.baseKd = pump.pidKd;

  status.kpScaleMin = PID_KP_SCALE_MIN;
  status.kpScaleMax = PID_KP_SCALE_MAX;
  status.kiScaleMin = PID_KI_SCALE_MIN;
  status.kiScaleMax = PID_KI_SCALE_MAX;
  status.kdScaleMin = PID_KD_SCALE_MIN;
  status.kdScaleMax = PID_KD_SCALE_MAX;

  if (pumpIndex < MAX_PUMPS) {
    const PidAdaptiveState& learned = g_pidAdaptive[pumpIndex];
    status.kpScale = learned.kpScale;
    status.kiScale = learned.kiScale;
    status.kdScale = learned.kdScale;
    status.ewmaError = learned.ewmaError;
    status.ewmaAbsError = learned.ewmaAbsError;
    status.oscillationScore = learned.oscillationScore;
  }

  const PidGains gains =
    effectiveSolarPidGains(ctx, pumpIndex, pump);

  status.effectiveKp = gains.kp;
  status.effectiveKi = gains.ki;
  status.effectiveKd = gains.kd;

  return status;
}

SolarControlParams solarControlParams(
  const AppContext& ctx,
  uint8_t pumpIndex,
  const PumpConfig& pump
) {
  const bool applyLearned =
    pumpIndex < MAX_PUMPS &&
    domainControlAllowed(MlDomain::SOLAR_CONTROL) &&
    pump.enabled &&
    pump.mode == PumpMode::PWM &&
    isSolarRole(pump.sourceRole);

  return solarControlParamsInternal(
    ctx,
    pumpIndex,
    pump,
    applyLearned
  );
}

float adjustSolarPumpPwm(
  const AppContext& ctx,
  uint8_t pumpIndex,
  float requestedPercent
) {
  (void)ctx;
  (void)pumpIndex;
  return requestedPercent;
}

float adjustHeatingTarget(
  const AppContext& ctx,
  uint8_t circuitIndex,
  float requestedTargetC,
  float roomC,
  float outsideC
) {
  if (!domainControlAllowed(MlDomain::HEATING) ||
      circuitIndex >= MAX_HEATING_CIRCUITS ||
      isnan(roomC)) {
    return requestedTargetC;
  }

  const HeatingCircuitConfig& cfg =
    ctx.config.heatingCircuits[circuitIndex];

  const HeatingAdaptiveState& a =
    g_heatingAdaptive[circuitIndex];

  const float outsideNorm =
    isnan(outsideC)
      ? 0.0f
      : clampf((20.0f - outsideC) / 30.0f, -0.5f, 1.5f);

  const float inertiaCorrection =
    isnan(a.roomRateCPerHour)
      ? 0.0f
      : clampf(-0.75f * a.roomRateCPerHour, -1.5f, 1.5f);

  const float offset =
    clampf(
      g_hcBias[circuitIndex] +
      g_hcOutsideCoeff[circuitIndex] * outsideNorm +
      inertiaCorrection,
      -5.0f,
      5.0f
    );

  return clampf(
    requestedTargetC + offset,
    cfg.minimumFlowTemperatureC,
    cfg.maximumFlowTemperatureC
  );
}

float adjustHeatingMixerDeadband(
  const AppContext& ctx,
  uint8_t circuitIndex,
  float baseDeadbandC
) {
  if (!domainControlAllowed(MlDomain::HEATING) ||
      circuitIndex >= MAX_HEATING_CIRCUITS) {
    return baseDeadbandC;
  }

  const HeatingCircuitConfig& cfg =
    ctx.config.heatingCircuits[circuitIndex];

  const float minDeadband =
    cfg.mixerType == HeatingCircuitMixerType::THERMAL ? 1.0f : 0.5f;
  const float maxDeadband =
    cfg.mixerType == HeatingCircuitMixerType::THERMAL ? 2.5f : 1.5f;

  return clampf(
    baseDeadbandC + g_heatingAdaptive[circuitIndex].mixerDeadbandOffsetC,
    minDeadband,
    maxDeadband
  );
}

HeatingTuningStatus heatingTuningStatus(
  const AppContext& ctx,
  uint8_t circuitIndex
) {
  HeatingTuningStatus out;
  if (circuitIndex >= MAX_HEATING_CIRCUITS) return out;

  const HeatingCircuitConfig& cfg =
    ctx.config.heatingCircuits[circuitIndex];
  const HeatingCircuitRuntime& rt =
    ctx.heatingCircuitRuntime[circuitIndex];

  if (!cfg.enabled) return out;

  const HeatingAdaptiveState& a =
    g_heatingAdaptive[circuitIndex];

  out.available = true;
  out.active = domainControlAllowed(MlDomain::HEATING);

  const float outsideNorm =
    isnan(rt.outsideTemperatureC)
      ? 0.0f
      : clampf((20.0f - rt.outsideTemperatureC) / 30.0f, -0.5f, 1.5f);

  const float inertiaCorrection =
    isnan(a.roomRateCPerHour)
      ? 0.0f
      : clampf(-0.75f * a.roomRateCPerHour, -1.5f, 1.5f);

  out.learnedOffsetC =
    clampf(
      g_hcBias[circuitIndex] +
      g_hcOutsideCoeff[circuitIndex] * outsideNorm +
      inertiaCorrection,
      -5.0f,
      5.0f
    );

  out.effectiveTargetC = rt.targetFlowTemperatureC;

  const float roomTarget =
    !isnan(rt.effectiveRoomTargetTemperatureC)
      ? rt.effectiveRoomTargetTemperatureC
      : cfg.roomTargetTemperatureC;

  out.roomErrorC =
    isnan(rt.roomTemperatureC) ? NAN : roomTarget - rt.roomTemperatureC;

  out.roomRateCPerHour = a.roomRateCPerHour;
  out.predictedRoomError1hC = a.projectedRoomError1hC;
  out.outsideCoeff = g_hcOutsideCoeff[circuitIndex];
  out.flowErrorEwmaC = a.flowErrorEwmaC;
  out.spreadEwmaC = a.spreadEwmaC;
  out.mixerOscillationScore = a.mixerOscillationScore;

  const float baseDeadband =
    cfg.mixerType == HeatingCircuitMixerType::THERMAL ? 1.5f : 0.7f;

  out.baseMixerDeadbandC = baseDeadband;
  out.learnedMixerDeadbandC =
    clampf(
      baseDeadband + a.mixerDeadbandOffsetC,
      cfg.mixerType == HeatingCircuitMixerType::THERMAL ? 1.0f : 0.5f,
      cfg.mixerType == HeatingCircuitMixerType::THERMAL ? 2.5f : 1.5f
    );
  out.effectiveMixerDeadbandC =
    out.active ? out.learnedMixerDeadbandC : baseDeadband;

  return out;
}

uint8_t adjustOvenServoOpening(
  const AppContext& ctx,
  uint8_t requestedOpeningPercent,
  float ovenTemperatureC
) {
  if (!domainControlAllowed(MlDomain::OVEN) ||
      isnan(ovenTemperatureC)) {
    return requestedOpeningPercent;
  }

  const OvenConfig& cfg = ctx.config.oven;

  const uint8_t minOpening =
    min(
      cfg.servoMinimumOpeningPercent,
      cfg.servoMaximumOpeningPercent
    );

  const uint8_t maxOpening =
    max(
      cfg.servoMinimumOpeningPercent,
      cfg.servoMaximumOpeningPercent
    );

  float correction =
    g_ovenOpeningBias;

  if (!isnan(g_ovenAdaptive.temperatureRateCPerMin)) {
    correction +=
      clampf(
        -2.0f *
        g_ovenAdaptive.temperatureRateCPerMin,
        -4.0f,
        4.0f
      );
  }

  correction =
    clampf(
      correction,
      -OVEN_OPENING_BIAS_LIMIT,
      OVEN_OPENING_BIAS_LIMIT
    );

  if (ovenTemperatureC >
      cfg.targetOvenTemperatureC) {
    correction = min(0.0f, correction);
  } else if (ovenTemperatureC <
             cfg.targetOvenTemperatureC) {
    correction = max(0.0f, correction);
  } else {
    correction = 0.0f;
  }

  const float corrected =
    (float)requestedOpeningPercent + correction;

  return (uint8_t)lroundf(
    clampf(
      corrected,
      (float)minOpening,
      (float)maxOpening
    )
  );
}

float adjustOvenServoDeadband(
  const AppContext& ctx,
  float baseDeadbandC
) {
  if (!domainControlAllowed(MlDomain::OVEN)) {
    return baseDeadbandC;
  }

  const float safeBase =
    max(0.0f, baseDeadbandC);

  return clampf(
    safeBase +
      g_ovenAdaptive.deadbandOffsetC,
    1.5f,
    6.0f
  );
}

OvenTuningStatus ovenTuningStatus(
  const AppContext& ctx
) {
  OvenTuningStatus out;

  const OvenConfig& cfg = ctx.config.oven;
  if (!cfg.enabled) return out;

  out.available = true;
  out.active =
    domainControlAllowed(MlDomain::OVEN);

  out.openingBiasPercent =
    g_ovenOpeningBias;
  out.temperatureRateCPerMin =
    g_ovenAdaptive.temperatureRateCPerMin;
  out.predictedError10MinC =
    g_ovenAdaptive.predictedError10MinC;
  out.errorEwmaC =
    g_ovenAdaptive.errorEwmaC;
  out.absErrorEwmaC =
    g_ovenAdaptive.absErrorEwmaC;
  out.oscillationScore =
    g_ovenAdaptive.oscillationScore;

  out.baseDeadbandC =
    max(0.0f, cfg.servoDeadbandC);

  out.learnedDeadbandC =
    clampf(
      out.baseDeadbandC +
        g_ovenAdaptive.deadbandOffsetC,
      1.5f,
      6.0f
    );

  out.effectiveDeadbandC =
    out.active
      ? out.learnedDeadbandC
      : out.baseDeadbandC;

  out.rlaControlAvailable = false;
  snprintf(
    out.rlaState,
    sizeof(out.rlaState),
    "kein eigenstaendiger RLA-Regler vorhanden"
  );

  return out;
}


ThermalDemandStatus thermalDemandStatus(
  const AppContext& ctx
) {
  ThermalDemandStatus out;

  float boilerEnergy = NAN;
  float bufferEnergy = NAN;

  const bool boilerValid =
    boilerThermalState(
      ctx,
      out.boilerMeanC,
      boilerEnergy,
      out.boilerSensorCount
    );

  const bool bufferValid =
    bufferThermalState(
      ctx,
      out.bufferMeanC,
      bufferEnergy,
      out.bufferSensorCount
    );

  out.available =
    boilerValid || bufferValid;

  out.boilerEnergyAbove20KWh =
    boilerEnergy;
  out.bufferEnergyAbove20KWh =
    bufferEnergy;

  out.boilerDemandKwNow =
    g_thermalDemand.lastBoilerDemandKw;
  out.bufferDemandKwNow =
    g_thermalDemand.lastBufferDemandKw;

  if (!isnan(out.boilerDemandKwNow) ||
      !isnan(out.bufferDemandKwNow)) {
    out.totalDemandKwNow =
      (isnan(out.boilerDemandKwNow)
        ? 0.0f
        : out.boilerDemandKwNow) +
      (isnan(out.bufferDemandKwNow)
        ? 0.0f
        : out.bufferDemandKwNow);
  }

  const uint32_t now =
    TimeService::valid()
      ? (uint32_t)time(nullptr)
      : 0;

  if (now) {
    out.predictedDemand3hKWh =
      thermalPredictDemandKWh(
        ctx,
        now,
        3,
        false,
        false
      );

    out.predictedDemand6hKWh =
      thermalPredictDemandKWh(
        ctx,
        now,
        6,
        false,
        false
      );

    out.predictedDemand12hKWh =
      thermalPredictDemandKWh(
        ctx,
        now,
        12,
        false,
        false
      );

    out.predictedBoiler3hKWh =
      thermalPredictDemandKWh(
        ctx,
        now,
        3,
        true,
        false
      );

    out.predictedBuffer3hKWh =
      thermalPredictDemandKWh(
        ctx,
        now,
        3,
        false,
        true
      );
  }

  out.outsideTemperatureC =
    outsideTemp(ctx);

  out.bufferOutsideCoeffKwPerK =
    g_thermalDemand.bufferOutsideCoeffKwPerK;

  out.validIntervals =
    g_thermalDemand.validIntervals;

  out.rejectedIntervals =
    g_thermalDemand.rejectedIntervals;

  const DomainStatus ds =
    domainStatus(Domain::THERMAL_DEMAND);

  out.learning = ds.learning;

  out.qualityReady =
    ds.trainingSamples >= MIN_TRAINING_SAMPLES &&
    ds.trainingDays >= MIN_TRAINING_DAYS &&
    ds.windowSamples7d >= MIN_WINDOW_SAMPLES &&
    !isnan(ds.meanError7dPercent) &&
    ds.meanError7dPercent < MAX_WINDOW_ERROR_PCT;

  if (!out.available) {
    strncpy(
      out.stateText,
      "Speicherdaten fehlen",
      sizeof(out.stateText) - 1
    );
  } else if (!out.qualityReady) {
    strncpy(
      out.stateText,
      "Learning / Shadow",
      sizeof(out.stateText) - 1
    );
  } else {
    strncpy(
      out.stateText,
      "Modell qualifiziert",
      sizeof(out.stateText) - 1
    );
  }

  return out;
}

String json(const AppContext& ctx) {
  const Status current = g_status;

  String json = "{";

  json += "\"modelVersion\":2,";
  json += "\"mode\":" + String((int)ctx.config.mlMode) + ",";
  json += "\"active\":" + String(current.active ? "true" : "false") + ",";
  json += "\"learning\":" + String(current.learning ? "true" : "false") + ",";
  json += "\"fallback\":" + String(current.fallback ? "true" : "false") + ",";

  // Legacy/UI fields continue to represent solar yield.
  json += "\"trainingSamples\":" + String(current.trainingSamples) + ",";
  json += "\"trainingDays\":" + String(current.trainingDays) + ",";
  json += "\"windowSamples7d\":" + String(current.windowSamples7d) + ",";

  json += "\"meanError7dPercent\":";
  json += isnan(current.meanError7dPercent)
    ? String("null")
    : String(current.meanError7dPercent, 2);
  json += ",";

  json += "\"predictedNext3hKWh\":";
  json += isnan(current.predictedNext3hKWh)
    ? String("null")
    : String(current.predictedNext3hKWh, 3);
  json += ",";

  json += "\"conservativeNext3hKWh\":";
  json += isnan(current.conservativeNext3hKWh)
    ? String("null")
    : String(current.conservativeNext3hKWh, 3);
  json += ",";

  json += "\"lastMeasuredKWh\":";
  json += isnan(current.lastMeasuredKWh)
    ? String("null")
    : String(current.lastMeasuredKWh, 3);
  json += ",";

  json += "\"lastPredictionErrorPercent\":";
  json += isnan(current.lastPredictionErrorPercent)
    ? String("null")
    : String(current.lastPredictionErrorPercent, 2);
  json += ",";

  json += "\"readyDomainCount\":" +
          String(current.readyDomainCount) + ",";

  json += "\"enabledDomainCount\":" +
          String(current.enabledDomainCount) + ",";

  const uint32_t nowEpoch =
    TimeService::valid()
      ? (uint32_t)time(nullptr)
      : 0;

  const uint8_t currentSeason =
    seasonIndex(nowEpoch);

  uint8_t staleDomains = 0;
  uint8_t fallbackDomains = 0;
  uint8_t enabledControlDomains = 0;
  uint8_t activeControlDomains = 0;

  for (uint8_t d = 0; d < DOMAIN_COUNT; d++) {
    const DomainStatus& ds =
      current.domains[d];

    if (ds.fallback) fallbackDomains++;

    if (ds.enabled &&
        ds.controlAvailable) {
      enabledControlDomains++;
      if (ds.active) activeControlDomains++;
    }

    const DomainRuntime& rt =
      g_domains[d];

    if (ds.enabled &&
        (
          nowEpoch == 0 ||
          rt.lastObservationEpoch == 0 ||
          nowEpoch < rt.lastObservationEpoch ||
          (nowEpoch - rt.lastObservationEpoch) > MODEL_STALE_S
        )) {
      staleDomains++;
    }
  }

  const bool thermalQualified =
    thermalDemandQualityReadyForAssist(ctx);

  const bool mlComplete =
    ctx.config.mlMode == MlMode::AUTOMATIC &&
    enabledControlDomains > 0 &&
    activeControlDomains == enabledControlDomains &&
    thermalQualified;

  json += "\"modelHealth\":{";
  json += "\"seasonIndex\":" + String(currentSeason) + ",";
  json += "\"season\":\"" + String(seasonName(currentSeason)) + "\",";
  json += "\"solarSeasonFactor\":" +
          String(g_solarSeasonFactor[currentSeason], 4) + ",";
  json += "\"thermalSeasonFactor\":" +
          String(g_thermalSeasonFactor[currentSeason], 4) + ",";
  json += "\"staleDomains\":" + String(staleDomains) + ",";
  json += "\"fallbackDomains\":" + String(fallbackDomains) + ",";
  json += "\"enabledControlDomains\":" + String(enabledControlDomains) + ",";
  json += "\"activeControlDomains\":" + String(activeControlDomains) + ",";
  json += "\"thermalDemandQualified\":" +
          String(thermalQualified ? "true" : "false") + ",";
  json += "\"mlComplete\":" +
          String(mlComplete ? "true" : "false") + ",";
  json += "\"agingDailyFactor\":" + String(MODEL_AGE_DAILY, 5) + ",";
  json += "\"staleAfterDays\":" + String(MODEL_STALE_S / 86400UL);
  json += "},";

  json += "\"domains\":[";

  for (uint8_t d = 0; d < DOMAIN_COUNT; d++) {
    if (d) json += ",";

    const DomainStatus& ds = current.domains[d];

    json += "{";
    json += "\"id\":" + String(d) + ",";
    json += "\"name\":\"" + String(ds.name) + "\",";
    json += "\"enabled\":" + String(ds.enabled ? "true" : "false") + ",";
    json += "\"learning\":" + String(ds.learning ? "true" : "false") + ",";
    json += "\"active\":" + String(ds.active ? "true" : "false") + ",";
    json += "\"fallback\":" + String(ds.fallback ? "true" : "false") + ",";
    json += "\"controlAvailable\":" +
            String(ds.controlAvailable ? "true" : "false") + ",";
    json += "\"trainingSamples\":" +
            String(ds.trainingSamples) + ",";
    json += "\"trainingDays\":" +
            String(ds.trainingDays) + ",";
    json += "\"windowSamples7d\":" +
            String(ds.windowSamples7d) + ",";

    json += "\"meanError7dPercent\":";
    json += isnan(ds.meanError7dPercent)
      ? String("null")
      : String(ds.meanError7dPercent, 2);
    json += ",";

    json += "\"reason\":\"" + String(ds.reason) + "\",";

    const DomainRuntime& rt =
      g_domains[d];

    json += "\"lastObservationEpoch\":" +
            String(rt.lastObservationEpoch) + ",";

    json += "\"ageHours\":";
    if (nowEpoch == 0 ||
        rt.lastObservationEpoch == 0 ||
        nowEpoch < rt.lastObservationEpoch) {
      json += "null";
    } else {
      json += String(
        (nowEpoch - rt.lastObservationEpoch) /
        3600UL
      );
    }

    json += "}";
  }

  json += "],";

  json += "\"pidTuning\":[";
  bool firstPidTuning = true;

  for (uint8_t i = 0; i < MAX_PUMPS; i++) {
    const PumpConfig& pump = ctx.config.pumps[i];

    if (!pump.enabled ||
        pump.mode != PumpMode::PWM ||
        !isSolarRole(pump.sourceRole)) {
      continue;
    }

    if (!firstPidTuning) json += ",";
    firstPidTuning = false;

    const PidTuningStatus tuning =
      pidTuningStatus(ctx, i, pump);

    json += "{";
    json += "\"pumpIndex\":" + String(i) + ",";
    json += "\"available\":" + String(tuning.available ? "true" : "false") + ",";
    json += "\"active\":" + String(tuning.active ? "true" : "false") + ",";
    json += "\"baseKp\":" + String(tuning.baseKp, 5) + ",";
    json += "\"baseKi\":" + String(tuning.baseKi, 6) + ",";
    json += "\"baseKd\":" + String(tuning.baseKd, 6) + ",";
    json += "\"effectiveKp\":" + String(tuning.effectiveKp, 5) + ",";
    json += "\"effectiveKi\":" + String(tuning.effectiveKi, 6) + ",";
    json += "\"effectiveKd\":" + String(tuning.effectiveKd, 6) + ",";
    json += "\"kpScale\":" + String(tuning.kpScale, 5) + ",";
    json += "\"kiScale\":" + String(tuning.kiScale, 5) + ",";
    json += "\"kdScale\":" + String(tuning.kdScale, 5) + ",";
    json += "\"kpScaleMin\":" + String(tuning.kpScaleMin, 2) + ",";
    json += "\"kpScaleMax\":" + String(tuning.kpScaleMax, 2) + ",";
    json += "\"kiScaleMin\":" + String(tuning.kiScaleMin, 2) + ",";
    json += "\"kiScaleMax\":" + String(tuning.kiScaleMax, 2) + ",";
    json += "\"kdScaleMin\":" + String(tuning.kdScaleMin, 2) + ",";
    json += "\"kdScaleMax\":" + String(tuning.kdScaleMax, 2) + ",";
    json += "\"ewmaError\":" + String(tuning.ewmaError, 5) + ",";
    json += "\"ewmaAbsError\":" + String(tuning.ewmaAbsError, 5) + ",";
    json += "\"oscillationScore\":" + String(tuning.oscillationScore, 5);
    json += "}";
  }

  json += "],";

  json += "\"solarControl\":[";
  bool firstSolarControl = true;

  for (uint8_t i = 0; i < MAX_PUMPS; i++) {
    const PumpConfig& pump = ctx.config.pumps[i];
    if (!pump.enabled ||
        pump.mode != PumpMode::PWM ||
        !isSolarRole(pump.sourceRole)) {
      continue;
    }

    if (!firstSolarControl) json += ",";
    firstSolarControl = false;

    const SolarControlParams params =
      solarControlParams(ctx, i, pump);

    json += "{";
    json += "\"pumpIndex\":" + String(i) + ",";
    json += "\"available\":" + String(params.available ? "true" : "false") + ",";
    json += "\"active\":" + String(params.active ? "true" : "false") + ",";

    json += "\"baseStartDiffC\":" + String(params.baseStartDiffC, 2) + ",";
    json += "\"baseTargetDiffC\":" + String(params.baseTargetDiffC, 2) + ",";
    json += "\"baseHysteresisC\":" + String(params.baseHysteresisC, 2) + ",";

    json += "\"learnedStartDiffC\":" + String(params.learnedStartDiffC, 2) + ",";
    json += "\"learnedTargetDiffC\":" + String(params.learnedTargetDiffC, 2) + ",";
    json += "\"learnedHysteresisC\":" + String(params.learnedHysteresisC, 2) + ",";

    json += "\"effectiveStartDiffC\":" + String(params.effectiveStartDiffC, 2) + ",";
    json += "\"effectiveTargetDiffC\":" + String(params.effectiveTargetDiffC, 2) + ",";
    json += "\"effectiveHysteresisC\":" + String(params.effectiveHysteresisC, 2) + ",";

    json += "\"startMinC\":" + String(params.startMinC, 2) + ",";
    json += "\"startMaxC\":" + String(params.startMaxC, 2) + ",";
    json += "\"targetMinC\":" + String(params.targetMinC, 2) + ",";
    json += "\"targetMaxC\":" + String(params.targetMaxC, 2) + ",";
    json += "\"hysteresisMinC\":" + String(params.hysteresisMinC, 2) + ",";
    json += "\"hysteresisMaxC\":" + String(params.hysteresisMaxC, 2);
    json += "}";
  }

  json += "],";

  json += "\"heatingTuning\":[";
  bool firstHeating = true;

  for (uint8_t i = 0; i < MAX_HEATING_CIRCUITS; i++) {
    if (!ctx.config.heatingCircuits[i].enabled) continue;

    if (!firstHeating) json += ",";
    firstHeating = false;

    const HeatingTuningStatus tuning =
      heatingTuningStatus(ctx, i);

    json += "{";
    json += "\"circuitIndex\":" + String(i) + ",";
    json += "\"available\":" + String(tuning.available ? "true" : "false") + ",";
    json += "\"active\":" + String(tuning.active ? "true" : "false") + ",";
    json += "\"learnedOffsetC\":" + String(tuning.learnedOffsetC, 3) + ",";

    json += "\"effectiveTargetC\":";
    json += isnan(tuning.effectiveTargetC) ? String("null") : String(tuning.effectiveTargetC, 2);
    json += ",";

    json += "\"roomErrorC\":";
    json += isnan(tuning.roomErrorC) ? String("null") : String(tuning.roomErrorC, 3);
    json += ",";

    json += "\"roomRateCPerHour\":";
    json += isnan(tuning.roomRateCPerHour) ? String("null") : String(tuning.roomRateCPerHour, 4);
    json += ",";

    json += "\"predictedRoomError1hC\":";
    json += isnan(tuning.predictedRoomError1hC) ? String("null") : String(tuning.predictedRoomError1hC, 3);
    json += ",";

    json += "\"outsideCoeff\":" + String(tuning.outsideCoeff, 5) + ",";

    json += "\"flowErrorEwmaC\":";
    json += isnan(tuning.flowErrorEwmaC) ? String("null") : String(tuning.flowErrorEwmaC, 3);
    json += ",";

    json += "\"spreadEwmaC\":";
    json += isnan(tuning.spreadEwmaC) ? String("null") : String(tuning.spreadEwmaC, 3);
    json += ",";

    json += "\"mixerOscillationScore\":" + String(tuning.mixerOscillationScore, 4) + ",";
    json += "\"baseMixerDeadbandC\":" + String(tuning.baseMixerDeadbandC, 2) + ",";
    json += "\"learnedMixerDeadbandC\":" + String(tuning.learnedMixerDeadbandC, 2) + ",";
    json += "\"effectiveMixerDeadbandC\":" + String(tuning.effectiveMixerDeadbandC, 2);
    json += "}";
  }

  json += "],";

  const OvenTuningStatus oven =
    ovenTuningStatus(ctx);

  json += "\"ovenTuning\":{";
  json += "\"available\":" + String(oven.available ? "true" : "false") + ",";
  json += "\"active\":" + String(oven.active ? "true" : "false") + ",";
  json += "\"openingBiasPercent\":" + String(oven.openingBiasPercent, 3) + ",";

  json += "\"temperatureRateCPerMin\":";
  json += isnan(oven.temperatureRateCPerMin)
    ? String("null")
    : String(oven.temperatureRateCPerMin, 4);
  json += ",";

  json += "\"predictedError10MinC\":";
  json += isnan(oven.predictedError10MinC)
    ? String("null")
    : String(oven.predictedError10MinC, 3);
  json += ",";

  json += "\"errorEwmaC\":";
  json += isnan(oven.errorEwmaC)
    ? String("null")
    : String(oven.errorEwmaC, 3);
  json += ",";

  json += "\"absErrorEwmaC\":";
  json += isnan(oven.absErrorEwmaC)
    ? String("null")
    : String(oven.absErrorEwmaC, 3);
  json += ",";

  json += "\"oscillationScore\":" + String(oven.oscillationScore, 4) + ",";
  json += "\"baseDeadbandC\":" + String(oven.baseDeadbandC, 2) + ",";
  json += "\"learnedDeadbandC\":" + String(oven.learnedDeadbandC, 2) + ",";
  json += "\"effectiveDeadbandC\":" + String(oven.effectiveDeadbandC, 2) + ",";
  json += "\"rlaControlAvailable\":" + String(oven.rlaControlAvailable ? "true" : "false") + ",";
  json += "\"rlaState\":\"" + String(oven.rlaState) + "\"";
  json += "},";

  const ThermalDemandStatus thermal =
    thermalDemandStatus(ctx);

  json += "\"thermalDemand\":{";
  json += "\"available\":" + String(thermal.available ? "true" : "false") + ",";
  json += "\"learning\":" + String(thermal.learning ? "true" : "false") + ",";
  json += "\"qualityReady\":" + String(thermal.qualityReady ? "true" : "false") + ",";
  json += "\"boilerSensorCount\":" + String(thermal.boilerSensorCount) + ",";
  json += "\"bufferSensorCount\":" + String(thermal.bufferSensorCount) + ",";

  json += "\"boilerMeanC\":";
  json += isnan(thermal.boilerMeanC) ? String("null") : String(thermal.boilerMeanC, 2);
  json += ",";

  json += "\"bufferMeanC\":";
  json += isnan(thermal.bufferMeanC) ? String("null") : String(thermal.bufferMeanC, 2);
  json += ",";

  json += "\"boilerEnergyAbove20KWh\":";
  json += isnan(thermal.boilerEnergyAbove20KWh) ? String("null") : String(thermal.boilerEnergyAbove20KWh, 3);
  json += ",";

  json += "\"bufferEnergyAbove20KWh\":";
  json += isnan(thermal.bufferEnergyAbove20KWh) ? String("null") : String(thermal.bufferEnergyAbove20KWh, 3);
  json += ",";

  json += "\"boilerDemandKwNow\":";
  json += isnan(thermal.boilerDemandKwNow) ? String("null") : String(thermal.boilerDemandKwNow, 3);
  json += ",";

  json += "\"bufferDemandKwNow\":";
  json += isnan(thermal.bufferDemandKwNow) ? String("null") : String(thermal.bufferDemandKwNow, 3);
  json += ",";

  json += "\"totalDemandKwNow\":";
  json += isnan(thermal.totalDemandKwNow) ? String("null") : String(thermal.totalDemandKwNow, 3);
  json += ",";

  json += "\"predictedDemand3hKWh\":";
  json += isnan(thermal.predictedDemand3hKWh) ? String("null") : String(thermal.predictedDemand3hKWh, 3);
  json += ",";

  json += "\"predictedDemand6hKWh\":";
  json += isnan(thermal.predictedDemand6hKWh) ? String("null") : String(thermal.predictedDemand6hKWh, 3);
  json += ",";

  json += "\"predictedDemand12hKWh\":";
  json += isnan(thermal.predictedDemand12hKWh) ? String("null") : String(thermal.predictedDemand12hKWh, 3);
  json += ",";

  json += "\"predictedBoiler3hKWh\":";
  json += isnan(thermal.predictedBoiler3hKWh) ? String("null") : String(thermal.predictedBoiler3hKWh, 3);
  json += ",";

  json += "\"predictedBuffer3hKWh\":";
  json += isnan(thermal.predictedBuffer3hKWh) ? String("null") : String(thermal.predictedBuffer3hKWh, 3);
  json += ",";

  json += "\"outsideTemperatureC\":";
  json += isnan(thermal.outsideTemperatureC) ? String("null") : String(thermal.outsideTemperatureC, 2);
  json += ",";

  json += "\"bufferOutsideCoeffKwPerK\":" +
          String(thermal.bufferOutsideCoeffKwPerK, 5) + ",";

  json += "\"validIntervals\":" +
          String(thermal.validIntervals) + ",";

  json += "\"rejectedIntervals\":" +
          String(thermal.rejectedIntervals) + ",";

  json += "\"stateText\":\"" +
          String(thermal.stateText) + "\"";
  json += "},";

  json += "\"lastDecision\":\"" +
          String(current.lastDecision) + "\",";
  json += "\"recommendation\":\"" +
          String(current.recommendation) + "\"";
  json += "}";

  return json;
}

} // namespace MlOptimizer
