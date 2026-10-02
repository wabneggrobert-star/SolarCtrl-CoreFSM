#include "feature_safety_manager.h"

#include "feature_pumps.h"
#include "feature_sensor_assignments.h"
#include "feature_alarms.h"
#include "feature_aux_heater.h"
#include "feature_oven.h"
#include "feature_heating_circuits.h"

#include <Arduino.h>
#include <math.h>
#include <string.h>

#include "feature_build_flags.h"
namespace {

static constexpr float SENSOR_MIN_VALID_C = -40.0f;
static constexpr float SENSOR_MAX_VALID_C = 180.0f;
static constexpr uint32_t ONE_DAY_MS = 24UL * 60UL * 60UL * 1000UL;

SafetyManager::SafetyStatus g_status;
bool g_storageMaximumLatched = false;
bool g_storageCriticalLatched = false;

static constexpr float STORAGE_MAX_HYSTERESIS_C = 2.0f;
static constexpr float STORAGE_CRITICAL_HYSTERESIS_C = 3.0f;
static constexpr float STORAGE_COOLING_MIN_DELTA_C = 5.0f;
static constexpr float STORAGE_COOLING_PWM_PERCENT = 100.0f;

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
    case MaxChannel::CH4: config = &ctx.config.max4; break;
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

  // IMPORTANT: Die Rollen der DS18B20 werden in ctx.assignments gespeichert.
  // inventory.devices[].role ist nach dem Scan nicht die verlaessliche Quelle
  // fuer die aktuelle Zuordnung. Der alte Safety-Code hat deshalb konfigurierte
  // Puffer-/Boilersensoren uebersehen und keine Uebertemperatur erkannt.
  static const Ds18Role storageRoles[] = {
    Ds18Role::SINK_BOILER_TOP,
    Ds18Role::BOILER_BOTTOM,
    Ds18Role::SINK_BUFFER_TOP,
    Ds18Role::BUFFER_HIGH,
    Ds18Role::BUFFER_MID,
    Ds18Role::BUFFER_BOTTOM
  };

  for (Ds18Role role : storageRoles) {
    float temperatureC = NAN;
    bool valid = false;
    if (!SensorAssignments::readByRole(ctx.ds18b20, ctx.assignments, role, temperatureC, valid)) continue;
    if (!valid || !isValidTemperature(temperatureC)) continue;

    if (isnan(g_status.highestStorageTemperatureC) ||
        temperatureC > g_status.highestStorageTemperatureC) {
      g_status.highestStorageTemperatureC = temperatureC;
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
    ctx.ds18b20,
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
      DBG_PRINT("SAFETY SENSOR ERROR MAX ");
      DBG_PRINTLN(index + 1);
      g_status.sensorFaultActive = true;
    }
  };

  checkMax(0, MaxChannel::CH1, ctx.maxReadings[0]);
  checkMax(1, MaxChannel::CH2, ctx.maxReadings[1]);
  checkMax(2, MaxChannel::CH3, ctx.maxReadings[2]);
  checkMax(3, MaxChannel::CH4, ctx.maxReadings[3]);

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
  if (!ctx.config.storageProtectionEnabled || isnan(g_status.highestStorageTemperatureC)) {
    g_storageMaximumLatched = false;
    g_storageCriticalLatched = false;
    return;
  }

  const float storageC = g_status.highestStorageTemperatureC;
  const float maximumC = ctx.config.sinkMaxC;
  const float criticalC = ctx.config.storageCriticalTemperatureC;

  if (!g_storageMaximumLatched && storageC >= maximumC) g_storageMaximumLatched = true;
  if (g_storageMaximumLatched && storageC <= maximumC - STORAGE_MAX_HYSTERESIS_C) g_storageMaximumLatched = false;

  if (!g_storageCriticalLatched && storageC >= criticalC) g_storageCriticalLatched = true;
  if (g_storageCriticalLatched && storageC <= criticalC - STORAGE_CRITICAL_HYSTERESIS_C) g_storageCriticalLatched = false;

  g_status.storageMaximumActive = g_storageMaximumLatched;
  g_status.storageCriticalOvertemperatureActive = g_storageCriticalLatched;
  g_status.storageOvertemperatureActive = g_storageMaximumLatched || g_storageCriticalLatched;

  if (g_status.storageMaximumActive) {
    g_status.blockNormalPumpControl = true;
    g_status.mode = SafetyManager::SafetyMode::STORAGE_OVERTEMPERATURE;
    setMessage("Speicher-Maximaltemperatur erreicht");
  }

  if (g_status.storageCriticalOvertemperatureActive) {
    g_status.blockNormalPumpControl = true;
    g_status.mode = SafetyManager::SafetyMode::STORAGE_OVERTEMPERATURE;
    setMessage("KRITISCH: Speicher-Uebertemperatur - Zwangskuehlung aktiv");
  }
}

void syncStorageTemperatureAlarms(const AppContext& ctx) {
  if (!ctx.config.storageProtectionEnabled || isnan(g_status.highestStorageTemperatureC)) {
    Alarms::clear("safety_storage_maximum");
    Alarms::clear("safety_storage_overtemperature");
    return;
  }

  char message[120];
  if (g_status.storageCriticalOvertemperatureActive) {
    snprintf(message, sizeof(message),
             "KRITISCHE Speichertemperatur %.1f C (Grenze %.1f C) - Zwangskuehlung",
             g_status.highestStorageTemperatureC, ctx.config.storageCriticalTemperatureC);
    Alarms::raise("safety_storage_overtemperature", Alarms::Severity::CRITICAL, message, false);
    Alarms::clear("safety_storage_maximum");
  } else {
    Alarms::clear("safety_storage_overtemperature");
    if (g_status.storageMaximumActive) {
      snprintf(message, sizeof(message),
               "Speicher-Maximaltemperatur %.1f C erreicht (Grenze %.1f C)",
               g_status.highestStorageTemperatureC, ctx.config.sinkMaxC);
      Alarms::raise("safety_storage_maximum", Alarms::Severity::WARNING, message, true);
    } else {
      Alarms::clear("safety_storage_maximum");
    }
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

  DBG_PRINTLN("SafetyManager::begin OK");
  DBG_FLUSH();
}

static void evaluateNightCooling(AppContext& ctx, SafetyStatus& st)
{
    g_status.safetyNightCoolingActive = false;

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
  evaluateNightCooling(ctx, g_status);
  evaluateCollectorStagnation(ctx);
  evaluateFrostProtection(ctx);

  // Alarm synchron zum schnellen Safety-Zyklus erzeugen, nicht erst im 60-s-Runtime-Update.
  syncStorageTemperatureAlarms(ctx);

  if (g_status.mode != SafetyMode::NORMAL) {
    DBG_PRINT("SAFETY: ");
    DBG_PRINT(modeToText(g_status.mode));
    DBG_PRINT(" | ");
    DBG_PRINTLN(g_status.message);
    DBG_FLUSH();
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

  DBG_PRINT("SAFETY APPLY: ");
  DBG_PRINTLN(s.message);
  DBG_FLUSH();

  // Kritische Speicheruebertemperatur hat Vorrang vor einem gleichzeitigen
  // allgemeinen Sensorfehler. Jede einzelne Notkuehlroute prueft ihre benoetigten
  // Sensoren nochmals, sodass nur sicher messbare Wege aktiviert werden.
  if (s.storageCriticalOvertemperatureActive) {
    // Keine aktive Waermeerzeugung mehr in den Speicher. Der Ofen kann physisch
    // nicht abrupt gestoppt werden, deshalb nur Luftklappe sicher schliessen und
    // seine Pumpenhardware nicht zwangsweise abschalten.
    AuxHeater::allOff(ctx);
    OvenControl::applyStandbyServoPosition(ctx);
    Pumps::safetyAllOffExceptOven(ctx);

    g_status.storageForcedCoolingPumpCount = Pumps::safetyForceStorageCooling(
      ctx,
      ctx.config.storageCriticalTemperatureC - STORAGE_CRITICAL_HYSTERESIS_C,
      STORAGE_COOLING_MIN_DELTA_C,
      STORAGE_COOLING_PWM_PERCENT
    );
    g_status.storageForcedCoolingPumpCount += HeatingCircuits::safetyHeatDump(
      ctx,
      ctx.config.storageCriticalTemperatureC - STORAGE_CRITICAL_HYSTERESIS_C,
      STORAGE_COOLING_MIN_DELTA_C
    );
    g_status.storageForcedCoolingActive = g_status.storageForcedCoolingPumpCount > 0;

    if (!g_status.storageForcedCoolingActive) {
      setMessage("KRITISCH: Speicher heiss - kein ausreichend kuehler konfigurierter Abnehmer verfuegbar");
    }
    return;
  }

  if (s.sensorFaultActive) {
    // Ofenbetrieb wird spaeter separat ueber die Ofenlogik zugelassen.
    // Solar-/Speicherpumpen werden hier sicher abgeschaltet.
    Pumps::safetyAllOffExceptOven(ctx);
    return;
  }

  if (s.storageMaximumActive) {
    // Maximaltemperatur ist noch nicht die kritische Stufe. Falls die explizit
    // konfigurierte Nachtkuehlung moeglich ist, darf sie arbeiten; andernfalls
    // wird weitere Speicherbeladung sicher gestoppt.
    AuxHeater::allOff(ctx);
    if (s.safetyNightCoolingActive) {
      Pumps::safetyForceNightCooling(ctx, s.nightCoolingPumpPercent);
    } else {
      Pumps::safetyAllOffExceptOven(ctx);
    }
    return;
  }

  if (s.forceSolarHeatDump) {
    bool anyStarted = false;
    anyStarted |= Pumps::safetyForceHeatDumpForSource(ctx, HeatSourceRole::SOLAR_COLLECTOR_1, ctx.config.stagnationPumpPercent);
    anyStarted |= Pumps::safetyForceHeatDumpForSource(ctx, HeatSourceRole::SOLAR_COLLECTOR_2, ctx.config.stagnationPumpPercent);
    anyStarted |= Pumps::safetyForceHeatDumpForSource(ctx, HeatSourceRole::SOLAR_COLLECTOR_3, ctx.config.stagnationPumpPercent);

    if (!anyStarted) {
      g_status.controlledStagnation = true;
      DBG_PRINTLN("SAFETY: Kein kuehles Ziel verfuegbar, kontrollierte Stagnation");
      DBG_FLUSH();
    }
  }

if (s.safetyNightCoolingActive)
{
    Pumps::safetyForceNightCooling(ctx, s.nightCoolingPumpPercent);
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
