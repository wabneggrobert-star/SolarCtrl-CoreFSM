#pragma once

#include "app_types.h"

namespace SafetyManager {

enum class SafetyMode : uint8_t {
  NORMAL = 0,
  FROST_PROTECTION,
  COLLECTOR_STAGNATION,
  STORAGE_OVERTEMPERATURE,
  OVEN_OVERTEMPERATURE,
  SENSOR_FAULT
};

struct SafetyStatus {
  SafetyMode mode = SafetyMode::NORMAL;

  bool frostProtectionActive = false;
  bool collectorStagnationActive = false;
  bool storageOvertemperatureActive = false;
  bool ovenOvertemperatureActive = false;
  bool sensorFaultActive = false;

  bool blockNormalPumpControl = false;
  bool forceSolarHeatDump = false;
  bool forceOvenPumpRun = false;

  bool ds18SensorMissing = false;
  bool frostStorageAvailable = false;
  bool controlledStagnation = false;
  bool safetyNightCoolingAllowed = false;

  float lowestCollectorTemperatureC = NAN;
  float highestCollectorTemperatureC = NAN;
  float highestStorageTemperatureC = NAN;
  float frostStorageTemperatureC = NAN;
  float ovenTemperatureC = NAN;

  bool safetyNightCoolingActive = false;

  float nightCoolingCollectorTemperatureC = NAN;
  float nightCoolingStorageTemperatureC = NAN;
  float nightCoolingDeltaC = NAN;
  float nightCoolingPumpPercent = 0.0f;

  char message[128] = "";
};

void begin(AppContext& ctx);
void evaluate(AppContext& ctx);
void applyOutputs(AppContext& ctx);

const SafetyStatus& status();
const char* modeToText(SafetyMode mode);

}
