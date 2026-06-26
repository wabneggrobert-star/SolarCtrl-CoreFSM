#include "feature_safety_manager.h"

#include "feature_pumps.h"
#include "feature_sensor_assignments.h"

#include <Arduino.h>
#include <math.h>
#include <string.h>

namespace {

static constexpr float SENSOR_MIN_VALID_C = -40.0f;
static constexpr float SENSOR_MAX_VALID_C = 180.0f;
static constexpr uint32_t ONE_DAY_MS = 24UL * 60UL * 60UL * 1000UL;

SafetyManager::SafetyStatus g_status;

bool isValidTemperature(float value) {
  return !isnan(value) &&
         value >= SENSOR_MIN_VALID_C &&
         value <= SENSOR_MAX_VALID_C;
}

void setMessage(const char* text) {
  strncpy(g_status.message, text, sizeof(g_status.message) - 1);
  g_status.message[sizeof(g_status.message) - 1] = '\0';
}

void resetStatus() {
  g_status = SafetyManager::SafetyStatus{};
  setMessage("Normalbetrieb");
}

bool isSolarRole(HeatSourceRole role) {
  return role == HeatSourceRole::SOLAR_COLLECTOR_1 ||
         role == HeatSourceRole::SOLAR_COLLECTOR_2 ||
         role == HeatSourceRole::SOLAR_COLLECTOR_3;
}

bool heatSourceRoleUsedByEnabledPump(const AppContext& ctx, HeatSourceRole role) {
  if (role == HeatSourceRole::NONE) return false;

  for (uint8_t i = 0; i < MAX_PUMPS; i++) {
    const PumpConfig& pump = ctx.config.pumps[i];

    if (!pump.enabled) continue;
    if (pump.sourceType != PumpSourceType::HEAT_SOURCE_ROLE) continue;
    if (pump.sourceRole == role) return true;
  }

  return false;
}

bool maxChannelRelevantForSafety(const AppContext& ctx, MaxChannel channel) {
  const MaxChannelConfig* config = nullptr;

  switch (channel) {
    case MaxChannel::CH1: config = &ctx.config.max1; break;
    case MaxChannel::CH2: config = &ctx.config.max2; break;
    case MaxChannel::CH3: config = &ctx.config.max3; break;
  }

  if (config == nullptr || !config->enabled) {
    return false;
  }

  for (uint8_t i = 0; i < ctx.heatSourceAssignments.count; i++) {
    const HeatSourceAssignment& assignment = ctx.heatSourceAssignments.items[i];

    if (!assignment.assigned) continue;
    if (assignment.channel != channel) continue;
    if (assignment.role == HeatSourceRole::NONE) continue;

    if (heatSourceRoleUsedByEnabledPump(ctx, assignment.role)) {
      return true;
    }
  }

  return false;
}

void updateCollectorTemperatures(const AppContext& ctx) {
  g_status.lowestCollectorTemperatureC = NAN;
  g_status.highestCollectorTemperatureC = NAN;

  auto consider = [](float temperatureC) {
    if (!isValidTemperature(temperatureC)) return;

    if (isnan(g_status.lowestCollectorTemperatureC) ||
        temperatureC < g_status.lowestCollectorTemperatureC) {
      g_status.lowestCollectorTemperatureC = temperatureC;
    }

    if (isnan(g_status.highestCollectorTemperatureC) ||
        temperatureC > g_status.highestCollectorTemperatureC) {
      g_status.highestCollectorTemperatureC = temperatureC;
    }
  };

  if (ctx.sensors.heatSources.solarCollector1Valid) consider(ctx.sensors.heatSources.solarCollector1C);
  if (ctx.sensors.heatSources.solarCollector2Valid) consider(ctx.sensors.heatSources.solarCollector2C);
  if (ctx.sensors.heatSources.solarCollector3Valid) consider(ctx.sensors.heatSources.solarCollector3C);
}

bool isStorageRole(Ds18Role role) {
  switch (role) {
    case Ds18Role::SINK_BOILER_TOP:
    case Ds18Role::SINK_BUFFER_TOP:
    case Ds18Role::BOILER_BOTTOM:
    case Ds18Role::BUFFER_HIGH:
    case Ds18Role::BUFFER_MID:
    case Ds18Role::BUFFER_BOTTOM:
      return true;
    default:
      return false;
  }
}

void updateDs18StorageTemperatures(AppContext& ctx) {
  g_status.highestStorageTemperatureC = NAN;
  g_status.ds18SensorMissing = (ctx.ds18b20.count == 0);

  for (uint8_t i = 0; i < ctx.ds18b20.count; i++) {
    const Ds18b20DeviceInfo& device = ctx.ds18b20.devices[i];

    if (!device.present || !device.lastValid || !isValidTemperature(device.lastTempC)) {
      continue;
    }

    if (!isStorageRole(device.role)) {
      continue;
    }

    if (isnan(g_status.highestStorageTemperatureC) ||
        device.lastTempC > g_status.highestStorageTemperatureC) {
      g_status.highestStorageTemperatureC = device.lastTempC;
    }
  }

  if (!isnan(g_status.highestStorageTemperatureC) &&
      g_status.highestStorageTemperatureC >= ctx.config.sinkMaxC) {
    ctx.diag.storageReachedMaximumAtMs = millis();
  }

  if (ctx.config.safetyNightCoolingEnabled && ctx.diag.storageReachedMaximumAtMs != 0) {
    const uint32_t elapsed = (uint32_t)(millis() - ctx.diag.storageReachedMaximumAtMs);
    g_status.safetyNightCoolingAllowed = elapsed <= ONE_DAY_MS;
  }
}

void updateFrostStorageTemperature(const AppContext& ctx) {
  g_status.frostStorageTemperatureC = NAN;
  g_status.frostStorageAvailable = false;

  float temperatureC = NAN;
  bool valid = false;

  SensorAssignments::readByRole(
    ctx.assignments,
    ctx.config.frostProtectionStorageRole,
    temperatureC,
    valid
  );

  if (valid && isValidTemperature(temperatureC)) {
    g_status.frostStorageTemperatureC = temperatureC;
    g_status.frostStorageAvailable = temperatureC >= ctx.config.frostSinkMinC;
  }
}

void updateOvenTemperature(const AppContext& ctx) {
  g_status.ovenTemperatureC = NAN;

  if (ctx.sensors.heatSources.altSourceOvenValid &&
      isValidTemperature(ctx.sensors.heatSources.altSourceOvenC)) {
    g_status.ovenTemperatureC = ctx.sensors.heatSources.altSourceOvenC;
  }
}

void evaluateSensorFaults(const AppContext& ctx) {
  g_status.sensorFaultActive = false;

  auto checkMax = [&](uint8_t index, MaxChannel channel, const MaxChannelReading& reading) {
    if (!maxChannelRelevantForSafety(ctx, channel)) {
      return;
    }

    if (!reading.valid) {
      Serial.print("SAFETY SENSOR ERROR MAX ");
      Serial.println(index + 1);
      g_status.sensorFaultActive = true;
    }
  };

  checkMax(0, MaxChannel::CH1, ctx.maxReadings[0]);
  checkMax(1, MaxChannel::CH2, ctx.maxReadings[1]);
  checkMax(2, MaxChannel::CH3, ctx.maxReadings[2]);

  // Keine DS18B20 bedeutet: keine Solar-/Speichersteuerung.
  // Der Ofen wird später separat über die Ofenlogik betreibbar bleiben.
  if (ctx.ds18b20.count == 0) {
    bool anyNonOvenPumpEnabled = false;

    for (uint8_t i = 0; i < MAX_PUMPS; i++) {
      const PumpConfig& pump = ctx.config.pumps[i];
      if (!pump.enabled) continue;

      if (pump.sourceType == PumpSourceType::HEAT_SOURCE_ROLE &&
          pump.sourceRole == HeatSourceRole::ALT_SOURCE_OVEN) {
        continue;
      }

      anyNonOvenPumpEnabled = true;
      break;
    }

    if (anyNonOvenPumpEnabled) {
      g_status.ds18SensorMissing = true;
      g_status.sensorFaultActive = true;
    }
  }
}

void evaluateFrostProtection(const AppContext& ctx) {
  if (!ctx.config.frostEnabled) return;
  if (ctx.config.solarHydraulicType == SolarHydraulicType::DRAINBACK) return;
  if (ctx.config.solarFluidType == SolarFluidType::GLYCOL &&
      ctx.config.frostCollectorOnC < -5.0f) return;

  if (isnan(g_status.lowestCollectorTemperatureC)) return;

  if (g_status.lowestCollectorTemperatureC <= ctx.config.frostCollectorOnC) {
    g_status.frostProtectionActive = true;
    g_status.blockNormalPumpControl = true;
    g_status.mode = SafetyManager::SafetyMode::FROST_PROTECTION;

    if (!g_status.frostStorageAvailable) {
      setMessage("Frostschutz aktiv, aber Frostschutz-Speicher ist nicht warm genug");
      return;
    }

    setMessage("Frostschutz aktiv");
  }
}

void evaluateCollectorStagnation(const AppContext& ctx) {
  if (!ctx.config.stagnationEnabled) return;
  if (isnan(g_status.highestCollectorTemperatureC)) return;

  float effectiveOnTemperatureC = ctx.config.stagnationCollectorOnC;

  if (ctx.config.solarCollectorType == SolarCollectorType::EVACUATED_TUBE) {
    effectiveOnTemperatureC += 10.0f;
  }

  if (ctx.config.solarHydraulicType == SolarHydraulicType::DRAINBACK) {
    // Drainback-Anlagen koennen bewusst leer stagnieren.
    effectiveOnTemperatureC += 15.0f;
  }

  if (g_status.highestCollectorTemperatureC >= effectiveOnTemperatureC) {
    g_status.collectorStagnationActive = true;
    g_status.forceSolarHeatDump = true;
    g_status.blockNormalPumpControl = true;
    g_status.mode = SafetyManager::SafetyMode::COLLECTOR_STAGNATION;
    setMessage("Kollektor-Stagnationsschutz aktiv");
  }
}

void evaluateStorageOvertemperature(const AppContext& ctx) {
  if (!ctx.config.storageProtectionEnabled) return;
  if (isnan(g_status.highestStorageTemperatureC)) return;

  if (g_status.highestStorageTemperatureC >= ctx.config.sinkMaxC) {
    g_status.storageOvertemperatureActive = true;
    g_status.blockNormalPumpControl = true;
    g_status.mode = SafetyManager::SafetyMode::STORAGE_OVERTEMPERATURE;
    setMessage("Speicher-Maximaltemperatur erreicht");
  }

  if (g_status.highestStorageTemperatureC >= ctx.config.storageCriticalTemperatureC) {
    g_status.storageOvertemperatureActive = true;
    g_status.blockNormalPumpControl = true;
    g_status.mode = SafetyManager::SafetyMode::STORAGE_OVERTEMPERATURE;
    setMessage("Speicher kritisch heiß");
  }
}

void evaluateOvenOvertemperature(const AppContext& ctx) {
  if (!ctx.config.ovenProtectionEnabled) return;
  if (isnan(g_status.ovenTemperatureC)) return;

  if (g_status.ovenTemperatureC >= ctx.config.ovenOvertemperatureOnC) {
    g_status.ovenOvertemperatureActive = true;
    g_status.forceOvenPumpRun = true;
    g_status.blockNormalPumpControl = true;
    g_status.mode = SafetyManager::SafetyMode::OVEN_OVERTEMPERATURE;
    setMessage("Ofen-Übertemperaturschutz aktiv");
  }
}

}

namespace SafetyManager {

void begin(AppContext& ctx) {
  (void)ctx;
  resetStatus();

  Serial.println("SafetyManager::begin OK");
  Serial.flush();
}

static void evaluateNightCooling(AppContext& ctx, SafetyStatus& st)
{
    st.safetyNightCoolingActive = false;

    if (!ctx.config.safetyNightCoolingEnabled)
        return;

    if (!ctx.config.storageProtectionEnabled)
        return;

    if (st.sensorFaultActive)
        return;

    if (st.frostProtectionActive)
        return;

    if (st.forceOvenPumpRun)
        return;

    if (st.highestStorageTemperatureC < ctx.config.sinkMaxC)
        return;

    float collector = st.lowestCollectorTemperatureC;
    float storage  = st.highestStorageTemperatureC;

    st.nightCoolingCollectorTemperatureC = collector;
    st.nightCoolingStorageTemperatureC   = storage;

    if (isnan(collector) || isnan(storage))
        return;

    float delta = storage - collector;
    st.nightCoolingDeltaC = delta;

    if (collector > 25.0f)
        return;

    if (delta < 8.0f)
        return;

    if (storage <= ctx.config.safetyNightCoolingTargetTemperatureC)
        return;

    st.safetyNightCoolingActive = true;

    float pwm;

    if (delta <= 8.0f)
        pwm = 30.0f;
    else if (delta >= 20.0f)
        pwm = 100.0f;
    else
        pwm = 30.0f + (delta - 8.0f) * (70.0f / 12.0f);

    st.nightCoolingPumpPercent = constrain(pwm,30.0f,100.0f);

    snprintf(
    st.message,
    sizeof(st.message),
    "Nachtkuehlung aktiv"
    );
}

void evaluate(AppContext& ctx) {
  resetStatus();

  updateCollectorTemperatures(ctx);
  updateDs18StorageTemperatures(ctx);
  updateFrostStorageTemperature(ctx);
  updateOvenTemperature(ctx);

  evaluateSensorFaults(ctx);

  if (g_status.sensorFaultActive) {
    g_status.mode = SafetyMode::SENSOR_FAULT;
    g_status.blockNormalPumpControl = true;
    setMessage(g_status.ds18SensorMissing
      ? "Kein DS18B20 vorhanden: Solar-/Speichersteuerung gesperrt"
      : "Sensorfehler erkannt");
  }

  // Solar hat Vorrang vor Ofen, danach Speicher- und Frostschutz.
  evaluateOvenOvertemperature(ctx);
  evaluateStorageOvertemperature(ctx);
  evaluateNightCooling(ctx, st);
  evaluateCollectorStagnation(ctx);
  evaluateFrostProtection(ctx);

  if (g_status.mode != SafetyMode::NORMAL) {
    Serial.print("SAFETY: ");
    Serial.print(modeToText(g_status.mode));
    Serial.print(" | ");
    Serial.println(g_status.message);
    Serial.flush();
  }
}

const SafetyStatus& status() {
  return g_status;
}

const char* modeToText(SafetyMode mode) {
  switch (mode) {
    case SafetyMode::NORMAL: return "Normalbetrieb";
    case SafetyMode::FROST_PROTECTION: return "Frostschutz";
    case SafetyMode::COLLECTOR_STAGNATION: return "Kollektor-Stagnation";
    case SafetyMode::STORAGE_OVERTEMPERATURE: return "Speicher-Übertemperatur";
    case SafetyMode::OVEN_OVERTEMPERATURE: return "Ofen-Übertemperatur";
    case SafetyMode::SENSOR_FAULT: return "Sensorfehler";
    default: return "Unbekannt";
  }
}

void applyOutputs(AppContext& ctx) {
  const SafetyStatus& s = status();

  if (s.mode == SafetyMode::NORMAL) {
    return;
  }

  Serial.print("SAFETY APPLY: ");
  Serial.println(s.message);
  Serial.flush();

  if (s.sensorFaultActive) {
    // Ofenbetrieb wird später separat über die Ofenlogik zugelassen.
    // Solar-/Speicherpumpen werden hier sicher abgeschaltet.
    Pumps::safetyAllOffExceptOven(ctx);
    return;
  }

  if (s.storageOvertemperatureActive) {
    Pumps::safetyAllOff(ctx);
    return;
  }

  if (s.forceSolarHeatDump) {
    bool anyStarted = false;
    anyStarted |= Pumps::safetyForceHeatDumpForSource(ctx, HeatSourceRole::SOLAR_COLLECTOR_1, ctx.config.stagnationPumpPercent);
    anyStarted |= Pumps::safetyForceHeatDumpForSource(ctx, HeatSourceRole::SOLAR_COLLECTOR_2, ctx.config.stagnationPumpPercent);
    anyStarted |= Pumps::safetyForceHeatDumpForSource(ctx, HeatSourceRole::SOLAR_COLLECTOR_3, ctx.config.stagnationPumpPercent);

    if (!anyStarted) {
      g_status.controlledStagnation = true;
      Serial.println("SAFETY: Kein kuehles Ziel verfuegbar, kontrollierte Stagnation");
      Serial.flush();
    }
  }

  if (s.safetyNightCoolingActive)
{
    Pumps::safetyForceRunForSource(
        ctx,
        HeatSourceRole::SOLAR_COLLECTOR_1,
        s.nightCoolingPumpPercent);

    Pumps::safetyForceRunForSource(
        ctx,
        HeatSourceRole::SOLAR_COLLECTOR_2,
        s.nightCoolingPumpPercent);

    Pumps::safetyForceRunForSource(
        ctx,
        HeatSourceRole::SOLAR_COLLECTOR_3,
        s.nightCoolingPumpPercent);

    return;
}

  if (s.forceOvenPumpRun) {
    Pumps::safetyForceRunForSource(ctx, HeatSourceRole::ALT_SOURCE_OVEN, 100.0f);
  }

  if (s.frostProtectionActive) {
    if (!s.frostStorageAvailable) {
      Pumps::safetyAllOff(ctx);
      return;
    }

    Pumps::safetyForceRunForSource(ctx, HeatSourceRole::SOLAR_COLLECTOR_1, ctx.config.frostPumpPercent);
    Pumps::safetyForceRunForSource(ctx, HeatSourceRole::SOLAR_COLLECTOR_2, ctx.config.frostPumpPercent);
    Pumps::safetyForceRunForSource(ctx, HeatSourceRole::SOLAR_COLLECTOR_3, ctx.config.frostPumpPercent);
  }
}

}
