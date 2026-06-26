#include "feature_storage.h"

#include "config.h"
#include "feature_sensor_roles.h"
#include "feature_sensor_assignments.h"
#include "feature_sink_ds18b20.h"

#include <Arduino.h>
#include <SD.h>
#include <string.h>
#include <math.h>

namespace {
  bool g_sdAvailable = false;

  void copyText(char* dst, size_t dstSize, const char* src) {
    if (!dst || dstSize == 0) return;
    strncpy(dst, (src != nullptr) ? src : "", dstSize - 1);
    dst[dstSize - 1] = '\0';
  }

  String valueOf(const String& text, const String& key) {
    String needle = key + "=";
    int start = text.indexOf(needle);
    if (start < 0) return "";

    start += needle.length();
    int end = text.indexOf('\n', start);
    if (end < 0) end = text.length();

    String value = text.substring(start, end);
    value.trim();
    return value;
  }

  bool ensureParentDir(const char* path) {
    if (!path || path[0] != '/') return false;

    String s(path);
    int slash = s.lastIndexOf('/');
    if (slash <= 0) return true;

    String dir = s.substring(0, slash);
    if (dir.length() == 0) return true;

    if (SD.exists(dir.c_str())) return true;
    return SD.mkdir(dir.c_str());
  }

  String readFileText(const char* path) {
    if (!g_sdAvailable) return "";

    File f = SD.open(path, FILE_READ);
    if (!f) return "";

    String text;
    while (f.available()) {
      text += char(f.read());
    }
    f.close();
    return text;
  }

  bool writeFileText(const char* path, const String& text) {
    if (!g_sdAvailable) return false;

    ensureParentDir(path);
    SD.remove(path);

    File f = SD.open(path, FILE_WRITE);
    if (!f) return false;

    size_t written = f.print(text);
    f.close();

    return written == text.length();
  }

  String floatOrEmpty(float v, uint8_t decimals = 2) {
    if (isnan(v)) return "";
    return String((double)v, (unsigned int)decimals);
  }

  RelayFunction relayFunctionFromInt(int v) {
    switch (v) {
      case 1: return RelayFunction::PUMP_ENABLE;
      case 2: return RelayFunction::ZONE_VALVE;
      case 3: return RelayFunction::HEATER_ROD;
      case 4: return RelayFunction::MIXER;
      default: return RelayFunction::NONE;
    }
  }

  int relayFunctionToInt(RelayFunction f) {
    switch (f) {
      case RelayFunction::PUMP_ENABLE: return 1;
      case RelayFunction::ZONE_VALVE: return 2;
      case RelayFunction::HEATER_ROD: return 3;
      case RelayFunction::MIXER: return 4;
      case RelayFunction::NONE:
      default: return 0;
    }
  }


  PwmOutputMode pwmOutputModeFromInt(int v) {
    return (v == 1) ? PwmOutputMode::SWITCH : PwmOutputMode::PWM;
  }

  int pwmOutputModeToInt(PwmOutputMode mode) {
    return (mode == PwmOutputMode::SWITCH) ? 1 : 0;
  }

  PwmProfile pwmProfileFromInt(int v) {
    return (v == 1) ? PwmProfile::HEATING : PwmProfile::SOLAR;
  }

  int pwmProfileToInt(PwmProfile profile) {
    return (profile == PwmProfile::HEATING) ? 1 : 0;
  }

  PumpMode pumpModeFromInt(int v) {
    switch (v) {
      case 1: return PumpMode::RELAY;
      case 2: return PumpMode::PWM;
      default: return PumpMode::OFF;
    }
  }

  int pumpModeToInt(PumpMode m) {
    switch (m) {
      case PumpMode::RELAY: return 1;
      case PumpMode::PWM: return 2;
      case PumpMode::OFF:
      default: return 0;
    }
  }

  PumpSourceType pumpSourceTypeFromInt(int v) {
    return (v == 1) ? PumpSourceType::SENSOR_ROLE : PumpSourceType::HEAT_SOURCE_ROLE;
  }

  int pumpSourceTypeToInt(PumpSourceType sourceType) {
    return (sourceType == PumpSourceType::SENSOR_ROLE) ? 1 : 0;
  }

  HeatSourceRole heatSourceRoleFromInt(int v) {
    if (v < 0 || v > (int)HeatSourceRole::SENSOR_HK4_RETURN) {
      return HeatSourceRole::NONE;
    }
    return (HeatSourceRole)v;
  }

  int heatSourceRoleToInt(HeatSourceRole role) {
    return (int)role;
  }

  Ds18Role ds18RoleFromInt(int v) {
    if (v < 0 || v > (int)Ds18Role::HK4_RETURN) {
      return Ds18Role::NONE;
    }
    return (Ds18Role)v;
  }

  int ds18RoleToInt(Ds18Role role) {
    return (int)role;
  }

  void setDefaults(ConfigData& cfg) {
    cfg.activeSinkTarget = SinkTarget::BOILER_TOP;

    cfg.diffOnC = DEFAULT_DIFF_ON;
    cfg.diffOffC = DEFAULT_DIFF_OFF;

    cfg.pwmStartDiffC = DEFAULT_PWM_START_DIFF;
    cfg.pwmStartPercent = DEFAULT_PWM_START_PCT;

    cfg.pwmFullDiffC = DEFAULT_PWM_FULL_DIFF;
    cfg.pwmFullPercent = DEFAULT_PWM_FULL_PCT;

    cfg.sampleIntervalMs = DEFAULT_SAMPLE_INTERVAL_MS;
    cfg.runtimeSaveIntervalMs = DEFAULT_RUNTIME_SAVE_INTERVAL_MS;

    copyText(cfg.apName, sizeof(cfg.apName), DEFAULT_AP_SSID);
    copyText(cfg.apPassword, sizeof(cfg.apPassword), DEFAULT_AP_PASSWORD);
    copyText(cfg.servicePin, sizeof(cfg.servicePin), DEFAULT_SERVICE_PIN);

    cfg.solarFluidType = SolarFluidType::GLYCOL;
    cfg.solarHydraulicType = SolarHydraulicType::CLOSED_PRESSURIZED;
    cfg.solarCollectorType = SolarCollectorType::FLAT_PLATE;

    cfg.frostEnabled = DEFAULT_FROST_ENABLED;
    cfg.frostCollectorOnC = DEFAULT_FROST_COLLECTOR_ON_C;
    cfg.frostCollectorOffC = DEFAULT_FROST_COLLECTOR_OFF_C;
    cfg.frostSinkMinC = DEFAULT_FROST_SINK_MIN_C;
    cfg.frostPumpPercent = DEFAULT_FROST_PUMP_PERCENT;
    cfg.frostProtectionStorageRole = Ds18Role::SINK_BUFFER_TOP;
    cfg.frostSafeCollectorTemperatureC = 8.0f;

    cfg.stagnationEnabled = DEFAULT_STAGNATION_ENABLED;
    cfg.stagnationCollectorOnC = DEFAULT_STAGNATION_COLLECTOR_ON_C;
    cfg.stagnationCollectorOffC = DEFAULT_STAGNATION_COLLECTOR_OFF_C;
    cfg.sinkMaxC = DEFAULT_SINK_MAX_C;
    cfg.stagnationPumpPercent = DEFAULT_STAGNATION_PUMP_PERCENT;

    cfg.storageProtectionEnabled = true;
    cfg.storageCriticalTemperatureC = 90.0f;
    cfg.safetyNightCoolingEnabled = false;
    cfg.safetyNightCoolingTargetTemperatureC = 75.0f;

    cfg.ovenProtectionEnabled = true;
    cfg.ovenOvertemperatureOnC = 85.0f;
    cfg.ovenOvertemperatureOffC = 78.0f;

    // MAX31865 Kanal-Defaults
    cfg.max1.enabled = true;
    cfg.max1.offsetC = 0.0f;
    cfg.max1.calFactor = 1.0f;

    cfg.max2.enabled = false;
    cfg.max2.offsetC = 0.0f;
    cfg.max2.calFactor = 1.0f;

    cfg.max3.enabled = false;
    cfg.max3.offsetC = 0.0f;
    cfg.max3.calFactor = 1.0f;

    for (uint8_t i = 0; i < RELAY_COUNT; i++) {
      cfg.relays[i].enabled = false;
      cfg.relays[i].function = RelayFunction::NONE;
      cfg.relays[i].activeLow = true;
    }

    for (uint8_t i = 0; i < PWM_OUTPUT_COUNT; i++) {
      cfg.pwmOutputs[i].enabled = true;
      cfg.pwmOutputs[i].mode = PwmOutputMode::PWM;
      cfg.pwmOutputs[i].function = RelayFunction::NONE;
      cfg.pwmOutputs[i].profile = PwmProfile::SOLAR;
    }

    for (uint8_t i = 0; i < MAX_PUMPS; i++) {
      cfg.pumps[i].enabled = false;
      cfg.pumps[i].mode = PumpMode::OFF;
      cfg.pumps[i].targetDiff = 5.0f;
      cfg.pumps[i].hysteresis = 1.0f;
      cfg.pumps[i].startDiff = 3.0f;
      cfg.pumps[i].pidKp = 10.0f;
      cfg.pumps[i].pidKi = 0.2f;
      cfg.pumps[i].pidKd = 0.0f;
      cfg.pumps[i].sourceType = PumpSourceType::HEAT_SOURCE_ROLE;
      cfg.pumps[i].sourceRole = HeatSourceRole::NONE;
      cfg.pumps[i].sourceSensorRole = Ds18Role::NONE;
      cfg.pumps[i].sinkRole = Ds18Role::NONE;
      cfg.pumps[i].relayIndex = PIN_UNUSED;
      cfg.pumps[i].pwmChannel = (i < 5) ? PUMP_PWM_CHANNELS[i] : PIN_UNUSED;
      cfg.pumps[i].pwmProfile = PwmProfile::SOLAR;
      cfg.pumps[i].feedbackPin =(i < FEEDBACK_INPUT_COUNT) ? i : PIN_UNUSED;
      cfg.pumps[i].minPwmPercent = 10.0f;
      cfg.pumps[i].maxPwmPercent = 100.0f;
      cfg.pumps[i].state = false;
      cfg.pumps[i].lastSourceC = NAN;
      cfg.pumps[i].lastSinkC = NAN;
      cfg.pumps[i].lastDiffC = NAN;
      cfg.pumps[i].lastPwmPercent = 0.0f;
      cfg.pumps[i].switchValveEnabled = false;
      cfg.pumps[i].switchValveRelayIndex = PIN_UNUSED;
      cfg.pumps[i].switchValveTravelTimeMs = 15000;
      cfg.pumps[i].switchValveMoving = false;
      cfg.pumps[i].switchValveMoveStartedMs = 0;
      cfg.pumps[i].switchValvePendingTargetIndex = PIN_UNUSED;
      cfg.pumps[i].switchValveStateForTargetA = false;
      cfg.pumps[i].activeTargetIndex = PIN_UNUSED;
      for (uint8_t t = 0; t < PUMP_ROUTE_TARGET_COUNT; t++) {
        cfg.pumps[i].targets[t].enabled = false;
        cfg.pumps[i].targets[t].sinkRole = Ds18Role::NONE;
        cfg.pumps[i].targets[t].targetDiffOverride = 0.0f;
        cfg.pumps[i].targets[t].hysteresisOverride = 0.0f;
        cfg.pumps[i].targets[t].minTempC = 0.0f;
        cfg.pumps[i].targets[t].maxTempC = 0.0f;
        cfg.pumps[i].targets[t].active = false;
        cfg.pumps[i].targets[t].lastSinkC = NAN;
        cfg.pumps[i].targets[t].lastDiffC = NAN;
      }
    }

    cfg.auxHeater.enabled = false;
    cfg.auxHeater.minimumTemperatureC = 45.0f;
    cfg.auxHeater.targetTemperatureC = 55.0f;
    cfg.auxHeater.hysteresisC = 2.0f;
    cfg.auxHeater.sinkRole = Ds18Role::NONE;
    cfg.auxHeater.pumpRelay = PIN_UNUSED;
    cfg.auxHeater.heaterRelay1 = PIN_UNUSED;
    cfg.auxHeater.heaterRelay2 = PIN_UNUSED;
    cfg.auxHeater.heaterRelay3 = PIN_UNUSED;
    cfg.auxHeater.preRunMs = 300000UL;
    cfg.auxHeater.cooldownMs = 300000UL;

    for (uint8_t i = 0; i < MAX_HEATING_CIRCUITS; i++) {
      HeatingCircuitConfig& hk = cfg.heatingCircuits[i];
      hk.enabled = false;
      hk.mixerType = HeatingCircuitMixerType::THREE_POINT;
      hk.controlMode = HeatingCircuitControlMode::FIXED_FLOW;
      hk.mixerOpenOutput = OutputRef{};
      hk.mixerCloseOutput = OutputRef{};
      hk.pumpMode = HeatingCircuitPumpMode::NONE;
      hk.pumpOutput = OutputRef{};
      hk.pumpMinPercent = 30;
      hk.pumpMaxPercent = 100;
      hk.flowSensorRole = Ds18Role::NONE;
      hk.returnSensorRole = Ds18Role::NONE;
      hk.roomSensorRole = Ds18Role::NONE;
      hk.bufferReferenceRole = Ds18Role::NONE;
      hk.outsideSensorRole = Ds18Role::OUTSIDE_TEMPERATURE;
      hk.fixedFlowTemperatureC = 35.0f;
      hk.maximumFlowTemperatureC = 55.0f;
      hk.minimumFlowTemperatureC = 20.0f;
      hk.roomTargetTemperatureC = 21.0f;
      hk.roomInfluenceK = 3.0f;
      hk.heatingCurveBaseC = 25.0f;
      hk.heatingCurveSlope = 1.0f;
      hk.frostProtectionEnabled = true;
      hk.frostStartTemperatureC = 3.0f;
      hk.frostTargetFlowTemperatureC = 12.0f;
      hk.mixerFullTravelMs = 120000UL;
      hk.mixerPulseMs = 1500UL;
      hk.mixerPauseMs = 8000UL;
    }
  }

  void setDiagDefaults(DiagnosticData& d) {
    d.faultActive = false;
    d.faultText[0] = '\0';

    d.bootCount = 0;
    d.sensorErrorCount = 0;
    d.sdErrorCount = 0;
    d.faultCount = 0;
    d.maxErrorCount = 0;

    d.lastCollectorC = NAN;
    d.lastSinkC = NAN;
    d.lastDiffC = NAN;

    d.frostEventCount = 0;
    d.stagnationEventCount = 0;
    d.lastProtectionMode = 0;
  }

  void setMaintenanceDefaults(MaintenanceData& m) {
    m.pumpStarts = 0;
    m.relaySwitchCount = 0;
    m.pumpRuntimeSeconds = 0;
    m.controllerRuntimeSeconds = 0;
    m.lastServiceDate[0] = '\0';
    m.serviceDue = false;
  }

  void setAssignmentDefaults(SensorAssignmentTable& table) {
    SensorAssignments::clearTable(table);
  }
}

namespace Storage {

void begin(bool sdAvailable) {
  g_sdAvailable = sdAvailable;
}

bool loadConfig(ConfigData& cfg) {
  setDefaults(cfg);

  String text = readFileText(FILE_CONFIG);
  if (text.isEmpty()) {
    return false;
  }

  String v;

  v = valueOf(text, "activeSinkTarget");
  if (v.length()) {
    cfg.activeSinkTarget = (v.toInt() == 1) ? SinkTarget::BUFFER_TOP : SinkTarget::BOILER_TOP;
  }

  v = valueOf(text, "diffOnC");
  if (v.length()) cfg.diffOnC = v.toFloat();

  v = valueOf(text, "diffOffC");
  if (v.length()) cfg.diffOffC = v.toFloat();

  v = valueOf(text, "pwmStartDiffC");
  if (v.length()) cfg.pwmStartDiffC = v.toFloat();

  v = valueOf(text, "pwmStartPercent");
  if (v.length()) cfg.pwmStartPercent = (uint8_t)v.toInt();

  v = valueOf(text, "pwmFullDiffC");
  if (v.length()) cfg.pwmFullDiffC = v.toFloat();

  v = valueOf(text, "pwmFullPercent");
  if (v.length()) cfg.pwmFullPercent = (uint8_t)v.toInt();

  v = valueOf(text, "sampleIntervalMs");
  if (v.length()) cfg.sampleIntervalMs = (uint32_t)v.toInt();

  v = valueOf(text, "runtimeSaveIntervalMs");
  if (v.length()) cfg.runtimeSaveIntervalMs = (uint32_t)v.toInt();

  v = valueOf(text, "apName");
  if (v.length()) copyText(cfg.apName, sizeof(cfg.apName), v.c_str());

  v = valueOf(text, "apPassword");
  if (v.length()) copyText(cfg.apPassword, sizeof(cfg.apPassword), v.c_str());

  v = valueOf(text, "servicePin");
  if (v.length()) copyText(cfg.servicePin, sizeof(cfg.servicePin), v.c_str());

  v = valueOf(text, "solarFluidType");
  if (v.length()) cfg.solarFluidType = (v.toInt() == 1) ? SolarFluidType::WATER : SolarFluidType::GLYCOL;

  v = valueOf(text, "solarHydraulicType");
  if (v.length()) cfg.solarHydraulicType = (v.toInt() == 1) ? SolarHydraulicType::DRAINBACK : SolarHydraulicType::CLOSED_PRESSURIZED;

  v = valueOf(text, "solarCollectorType");
  if (v.length()) cfg.solarCollectorType = (v.toInt() == 1) ? SolarCollectorType::EVACUATED_TUBE : SolarCollectorType::FLAT_PLATE;

  v = valueOf(text, "frostEnabled");
  if (v.length()) cfg.frostEnabled = (v.toInt() != 0);

  v = valueOf(text, "frostCollectorOnC");
  if (v.length()) cfg.frostCollectorOnC = v.toFloat();

  v = valueOf(text, "frostCollectorOffC");
  if (v.length()) cfg.frostCollectorOffC = v.toFloat();

  v = valueOf(text, "frostSinkMinC");
  if (v.length()) cfg.frostSinkMinC = v.toFloat();

  v = valueOf(text, "frostPumpPercent");
  if (v.length()) cfg.frostPumpPercent = (uint8_t)v.toInt();

  v = valueOf(text, "frostProtectionStorageRole");
  if (v.length()) cfg.frostProtectionStorageRole = ds18RoleFromInt(v.toInt());

  v = valueOf(text, "frostSafeCollectorTemperatureC");
  if (v.length()) cfg.frostSafeCollectorTemperatureC = v.toFloat();

  v = valueOf(text, "stagnationEnabled");
  if (v.length()) cfg.stagnationEnabled = (v.toInt() != 0);

  v = valueOf(text, "stagnationCollectorOnC");
  if (v.length()) cfg.stagnationCollectorOnC = v.toFloat();

  v = valueOf(text, "stagnationCollectorOffC");
  if (v.length()) cfg.stagnationCollectorOffC = v.toFloat();

  v = valueOf(text, "sinkMaxC");
  if (v.length()) cfg.sinkMaxC = v.toFloat();

  v = valueOf(text, "stagnationPumpPercent");
  if (v.length()) cfg.stagnationPumpPercent = (uint8_t)v.toInt();

  v = valueOf(text, "storageProtectionEnabled");
  if (v.length()) cfg.storageProtectionEnabled = (v.toInt() != 0);

  v = valueOf(text, "storageCriticalTemperatureC");
  if (v.length()) cfg.storageCriticalTemperatureC = v.toFloat();

  v = valueOf(text, "safetyNightCoolingEnabled");
  if (v.length()) cfg.safetyNightCoolingEnabled = (v.toInt() != 0);

  v = valueOf(text, "safetyNightCoolingTargetTemperatureC");
  if (v.length()) cfg.safetyNightCoolingTargetTemperatureC = v.toFloat();

  v = valueOf(text, "ovenProtectionEnabled");
  if (v.length()) cfg.ovenProtectionEnabled = (v.toInt() != 0);

  v = valueOf(text, "ovenOvertemperatureOnC");
  if (v.length()) cfg.ovenOvertemperatureOnC = v.toFloat();

  v = valueOf(text, "ovenOvertemperatureOffC");
  if (v.length()) cfg.ovenOvertemperatureOffC = v.toFloat();

  // MAX1
  v = valueOf(text, "max1Enabled");
  if (v.length()) cfg.max1.enabled = (v.toInt() != 0);

  v = valueOf(text, "max1OffsetC");
  if (v.length()) cfg.max1.offsetC = v.toFloat();

  v = valueOf(text, "max1CalFactor");
  if (v.length()) cfg.max1.calFactor = v.toFloat();

  // MAX2
  v = valueOf(text, "max2Enabled");
  if (v.length()) cfg.max2.enabled = (v.toInt() != 0);

  v = valueOf(text, "max2OffsetC");
  if (v.length()) cfg.max2.offsetC = v.toFloat();

  v = valueOf(text, "max2CalFactor");
  if (v.length()) cfg.max2.calFactor = v.toFloat();

  // MAX3
  v = valueOf(text, "max3Enabled");
  if (v.length()) cfg.max3.enabled = (v.toInt() != 0);

  v = valueOf(text, "max3OffsetC");
  if (v.length()) cfg.max3.offsetC = v.toFloat();

  v = valueOf(text, "max3CalFactor");
  if (v.length()) cfg.max3.calFactor = v.toFloat();

  // Relais-Konfiguration
  for (uint8_t i = 0; i < RELAY_COUNT; i++) {
    const String prefix = "relay" + String(i) + "_";

    v = valueOf(text, prefix + "enabled");
    if (v.length()) cfg.relays[i].enabled = (v.toInt() != 0);

    v = valueOf(text, prefix + "function");
    if (v.length()) cfg.relays[i].function = relayFunctionFromInt(v.toInt());

    v = valueOf(text, prefix + "activeLow");
    if (v.length()) cfg.relays[i].activeLow = (v.toInt() != 0);
  }

  // PCA/PO-Ausgangs-Konfiguration
  for (uint8_t i = 0; i < PWM_OUTPUT_COUNT; i++) {
    const String prefix = "pwmOutput" + String(i) + "_";

    v = valueOf(text, prefix + "enabled");
    if (v.length()) cfg.pwmOutputs[i].enabled = (v.toInt() != 0);

    v = valueOf(text, prefix + "mode");
    if (v.length()) cfg.pwmOutputs[i].mode = pwmOutputModeFromInt(v.toInt());

    v = valueOf(text, prefix + "function");
    if (v.length()) cfg.pwmOutputs[i].function = relayFunctionFromInt(v.toInt());

    v = valueOf(text, prefix + "profile");
    if (v.length()) cfg.pwmOutputs[i].profile = pwmProfileFromInt(v.toInt());
  }

  // Pumpen-Konfiguration wird hier bereits vorbereitet, UI folgt im naechsten Schritt.
  for (uint8_t i = 0; i < MAX_PUMPS; i++) {
    const String prefix = "pump" + String(i) + "_";

    v = valueOf(text, prefix + "enabled");
    if (v.length()) cfg.pumps[i].enabled = (v.toInt() != 0);

    v = valueOf(text, prefix + "mode");
    if (v.length()) cfg.pumps[i].mode = pumpModeFromInt(v.toInt());

    v = valueOf(text, prefix + "sourceType");
    if (v.length()) cfg.pumps[i].sourceType = pumpSourceTypeFromInt(v.toInt());

    v = valueOf(text, prefix + "sourceRole");
    if (v.length()) cfg.pumps[i].sourceRole = heatSourceRoleFromInt(v.toInt());

    v = valueOf(text, prefix + "sourceSensorRole");
    if (v.length()) cfg.pumps[i].sourceSensorRole = ds18RoleFromInt(v.toInt());

    v = valueOf(text, prefix + "sinkRole");
    if (v.length()) cfg.pumps[i].sinkRole = ds18RoleFromInt(v.toInt());

    v = valueOf(text, prefix + "relayIndex");
    if (v.length()) cfg.pumps[i].relayIndex = (uint8_t)v.toInt();

    v = valueOf(text, prefix + "pwmChannel");
    if (v.length()) cfg.pumps[i].pwmChannel = (uint8_t)v.toInt();

    v = valueOf(text, prefix + "pwmProfile");
    if (v.length()) cfg.pumps[i].pwmProfile = (v.toInt() == 1) ? PwmProfile::HEATING : PwmProfile::SOLAR;

    v = valueOf(text, prefix + "feedbackPin");
    if (v.length()) cfg.pumps[i].feedbackPin = (uint8_t)v.toInt();

    v = valueOf(text, prefix + "targetDiff");
    if (v.length()) cfg.pumps[i].targetDiff = v.toFloat();

    v = valueOf(text, prefix + "hysteresis");
    if (v.length()) cfg.pumps[i].hysteresis = v.toFloat();

    v = valueOf(text, prefix + "startDiff");
    if (v.length()) cfg.pumps[i].startDiff = v.toFloat();

    v = valueOf(text, prefix + "pidKp");
    if (v.length()) cfg.pumps[i].pidKp = v.toFloat();

    v = valueOf(text, prefix + "pidKi");
    if (v.length()) cfg.pumps[i].pidKi = v.toFloat();

    v = valueOf(text, prefix + "pidKd");
    if (v.length()) cfg.pumps[i].pidKd = v.toFloat();

    v = valueOf(text, prefix + "minPwmPercent");
    if (v.length()) cfg.pumps[i].minPwmPercent = v.toFloat();

    v = valueOf(text, prefix + "maxPwmPercent");
    if (v.length()) cfg.pumps[i].maxPwmPercent = v.toFloat();

    v = valueOf(text, prefix + "switchValveEnabled");
    if (v.length()) cfg.pumps[i].switchValveEnabled = (v.toInt() != 0);

    v = valueOf(text, prefix + "switchValveRelayIndex");
    if (v.length()) cfg.pumps[i].switchValveRelayIndex = (uint8_t)v.toInt();

    v = valueOf(text, prefix + "switchValveTravelTimeMs");
    if (v.length()) cfg.pumps[i].switchValveTravelTimeMs = (uint32_t)v.toInt();

    v = valueOf(text, prefix + "switchValveStateForTargetA");
    if (v.length()) cfg.pumps[i].switchValveStateForTargetA = (v.toInt() != 0);

    for (uint8_t t = 0; t < PUMP_ROUTE_TARGET_COUNT; t++) {
      const String tPrefix = prefix + "target" + String(t) + "_";

      v = valueOf(text, tPrefix + "enabled");
      if (v.length()) cfg.pumps[i].targets[t].enabled = (v.toInt() != 0);

      v = valueOf(text, tPrefix + "sinkRole");
      if (v.length()) cfg.pumps[i].targets[t].sinkRole = ds18RoleFromInt(v.toInt());

      v = valueOf(text, tPrefix + "targetDiffOverride");
      if (v.length()) cfg.pumps[i].targets[t].targetDiffOverride = v.toFloat();

      v = valueOf(text, tPrefix + "hysteresisOverride");
      if (v.length()) cfg.pumps[i].targets[t].hysteresisOverride = v.toFloat();

      v = valueOf(text, tPrefix + "maxTempC");
      if (v.length()) cfg.pumps[i].targets[t].maxTempC = v.toFloat();
    }
  }


  v = valueOf(text, "auxHeaterEnabled");
  if (v.length()) cfg.auxHeater.enabled = (v.toInt() != 0);

  v = valueOf(text, "auxHeaterMinimumTemperatureC");
  if (v.length()) cfg.auxHeater.minimumTemperatureC = v.toFloat();

  v = valueOf(text, "auxHeaterTargetTemperatureC");
  if (v.length()) cfg.auxHeater.targetTemperatureC = v.toFloat();

  v = valueOf(text, "auxHeaterHysteresisC");
  if (v.length()) cfg.auxHeater.hysteresisC = v.toFloat();

  v = valueOf(text, "auxHeaterSinkRole");
  if (v.length()) cfg.auxHeater.sinkRole = ds18RoleFromInt(v.toInt());

  v = valueOf(text, "auxHeaterPumpRelay");
  if (v.length()) cfg.auxHeater.pumpRelay = (uint8_t)v.toInt();

  v = valueOf(text, "auxHeaterRelay1");
  if (v.length()) cfg.auxHeater.heaterRelay1 = (uint8_t)v.toInt();

  v = valueOf(text, "auxHeaterRelay2");
  if (v.length()) cfg.auxHeater.heaterRelay2 = (uint8_t)v.toInt();

  v = valueOf(text, "auxHeaterRelay3");
  if (v.length()) cfg.auxHeater.heaterRelay3 = (uint8_t)v.toInt();

  v = valueOf(text, "auxHeaterPreRunMs");
  if (v.length()) cfg.auxHeater.preRunMs = (uint32_t)v.toInt();

  v = valueOf(text, "auxHeaterCooldownMs");
  if (v.length()) cfg.auxHeater.cooldownMs = (uint32_t)v.toInt();

  v = valueOf(text, "ovenEnabled");
  if (v.length()) cfg.oven.enabled = (v.toInt() != 0);

  v = valueOf(text, "ovenPumpRelay");
  if (v.length()) cfg.oven.pumpRelay = (uint8_t)v.toInt();

  v = valueOf(text, "ovenTargetOvenTemperatureC");
  if (v.length()) cfg.oven.targetOvenTemperatureC = v.toFloat();

  v = valueOf(text, "ovenCriticalOvenTemperatureC");
  if (v.length()) cfg.oven.criticalOvenTemperatureC = v.toFloat();

  v = valueOf(text, "ovenPumpOnTemperatureC");
  if (v.length()) cfg.oven.pumpOnTemperatureC = v.toFloat();

  v = valueOf(text, "ovenPumpHysteresisC");
  if (v.length()) cfg.oven.pumpHysteresisC = v.toFloat();

  v = valueOf(text, "ovenPumpOnTemperatureDifferenceC");
  if (v.length()) cfg.oven.pumpOnTemperatureDifferenceC = v.toFloat();

  v = valueOf(text, "ovenPumpOffTemperatureDifferenceC");
  if (v.length()) cfg.oven.pumpOffTemperatureDifferenceC = v.toFloat();

  v = valueOf(text, "ovenServoMinimumAngle");
  if (v.length()) cfg.oven.servoMinimumAngle = (uint8_t)v.toInt();

  v = valueOf(text, "ovenServoMaximumAngle");
  if (v.length()) cfg.oven.servoMaximumAngle = (uint8_t)v.toInt();

  v = valueOf(text, "ovenServoBaseAngle");
  if (v.length()) cfg.oven.servoBaseAngle = (uint8_t)v.toInt();

  v = valueOf(text, "ovenPidKp");
  if (v.length()) cfg.oven.pidKp = v.toFloat();

  v = valueOf(text, "ovenPidKi");
  if (v.length()) cfg.oven.pidKi = v.toFloat();

  v = valueOf(text, "ovenPidKd");
  if (v.length()) cfg.oven.pidKd = v.toFloat();
  
  v = valueOf(text, "ovenTargetSinkRole");
  if (v.length()) cfg.oven.targetSinkRole = ds18RoleFromInt(v.toInt());
  v = valueOf(text, "ovenAutoStartEnabled");
  if (v.length()) cfg.oven.autoStartEnabled = (v.toInt() != 0);

  v = valueOf(text, "ovenAutoStartTemperatureC");
  if (v.length()) cfg.oven.autoStartTemperatureC = v.toFloat();

  v = valueOf(text, "ovenAutoStartRiseC");
  if (v.length()) cfg.oven.autoStartRiseC = v.toFloat();

  v = valueOf(text, "ovenAutoReturnToStandbyTemperatureC");
  if (v.length()) cfg.oven.autoReturnToStandbyTemperatureC = v.toFloat();

  v = valueOf(text, "ovenPumpStopDropFromPeakC");
  if (v.length()) cfg.oven.pumpStopDropFromPeakC = v.toFloat();


  for (uint8_t i = 0; i < MAX_HEATING_CIRCUITS; i++) {
    const String prefix = "heatingCircuit" + String(i) + "_";
    HeatingCircuitConfig& hk = cfg.heatingCircuits[i];

    v = valueOf(text, prefix + "enabled");
    if (v.length()) hk.enabled = (v.toInt() != 0);

    v = valueOf(text, prefix + "mixerType");
    if (v.length()) hk.mixerType = (v.toInt() == 1) ? HeatingCircuitMixerType::THERMAL : HeatingCircuitMixerType::THREE_POINT;

    v = valueOf(text, prefix + "controlMode");
    if (v.length()) hk.controlMode = (v.toInt() == 1) ? HeatingCircuitControlMode::WEATHER_COMPENSATED : HeatingCircuitControlMode::FIXED_FLOW;

    v = valueOf(text, prefix + "mixerOpenOutputKind");
    if (v.length()) hk.mixerOpenOutput.kind = (OutputKind)v.toInt();
    v = valueOf(text, prefix + "mixerOpenOutputIndex");
    if (v.length()) hk.mixerOpenOutput.index = (uint8_t)v.toInt();

    v = valueOf(text, prefix + "mixerCloseOutputKind");
    if (v.length()) hk.mixerCloseOutput.kind = (OutputKind)v.toInt();
    v = valueOf(text, prefix + "mixerCloseOutputIndex");
    if (v.length()) hk.mixerCloseOutput.index = (uint8_t)v.toInt();

    v = valueOf(text, prefix + "pumpMode");
    if (v.length()) hk.pumpMode = (HeatingCircuitPumpMode)v.toInt();
    v = valueOf(text, prefix + "pumpOutputKind");
    if (v.length()) hk.pumpOutput.kind = (OutputKind)v.toInt();
    v = valueOf(text, prefix + "pumpOutputIndex");
    if (v.length()) hk.pumpOutput.index = (uint8_t)v.toInt();

    v = valueOf(text, prefix + "pumpMinPercent");
    if (v.length()) hk.pumpMinPercent = (uint8_t)v.toInt();
    v = valueOf(text, prefix + "pumpMaxPercent");
    if (v.length()) hk.pumpMaxPercent = (uint8_t)v.toInt();

    v = valueOf(text, prefix + "flowSensorRole");
    if (v.length()) hk.flowSensorRole = ds18RoleFromInt(v.toInt());
    v = valueOf(text, prefix + "returnSensorRole");
    if (v.length()) hk.returnSensorRole = ds18RoleFromInt(v.toInt());
    v = valueOf(text, prefix + "roomSensorRole");
    if (v.length()) hk.roomSensorRole = ds18RoleFromInt(v.toInt());
    v = valueOf(text, prefix + "bufferReferenceRole");
    if (v.length()) hk.bufferReferenceRole = ds18RoleFromInt(v.toInt());
    v = valueOf(text, prefix + "outsideSensorRole");
    if (v.length()) hk.outsideSensorRole = ds18RoleFromInt(v.toInt());

    v = valueOf(text, prefix + "fixedFlowTemperatureC");
    if (v.length()) hk.fixedFlowTemperatureC = v.toFloat();
    v = valueOf(text, prefix + "maximumFlowTemperatureC");
    if (v.length()) hk.maximumFlowTemperatureC = v.toFloat();
    v = valueOf(text, prefix + "minimumFlowTemperatureC");
    if (v.length()) hk.minimumFlowTemperatureC = v.toFloat();
    v = valueOf(text, prefix + "roomTargetTemperatureC");
    if (v.length()) hk.roomTargetTemperatureC = v.toFloat();
    v = valueOf(text, prefix + "roomInfluenceK");
    if (v.length()) hk.roomInfluenceK = v.toFloat();
    v = valueOf(text, prefix + "heatingCurveBaseC");
    if (v.length()) hk.heatingCurveBaseC = v.toFloat();
    v = valueOf(text, prefix + "heatingCurveSlope");
    if (v.length()) hk.heatingCurveSlope = v.toFloat();
    v = valueOf(text, prefix + "frostProtectionEnabled");
    if (v.length()) hk.frostProtectionEnabled = (v.toInt() != 0);
    v = valueOf(text, prefix + "frostStartTemperatureC");
    if (v.length()) hk.frostStartTemperatureC = v.toFloat();
    v = valueOf(text, prefix + "frostTargetFlowTemperatureC");
    if (v.length()) hk.frostTargetFlowTemperatureC = v.toFloat();
    v = valueOf(text, prefix + "mixerFullTravelMs");
    if (v.length()) hk.mixerFullTravelMs = (uint32_t)v.toInt();
    v = valueOf(text, prefix + "mixerPulseMs");
    if (v.length()) hk.mixerPulseMs = (uint32_t)v.toInt();
    v = valueOf(text, prefix + "mixerPauseMs");
    if (v.length()) hk.mixerPauseMs = (uint32_t)v.toInt();
  }

  return true;
}

bool saveConfig(const ConfigData& cfg) {
  String text;

  text += "activeSinkTarget=" + String(cfg.activeSinkTarget == SinkTarget::BUFFER_TOP ? 1 : 0) + "\n";

  text += "diffOnC=" + String(cfg.diffOnC, 2) + "\n";
  text += "diffOffC=" + String(cfg.diffOffC, 2) + "\n";

  text += "pwmStartDiffC=" + String(cfg.pwmStartDiffC, 2) + "\n";
  text += "pwmStartPercent=" + String(cfg.pwmStartPercent) + "\n";

  text += "pwmFullDiffC=" + String(cfg.pwmFullDiffC, 2) + "\n";
  text += "pwmFullPercent=" + String(cfg.pwmFullPercent) + "\n";

  text += "sampleIntervalMs=" + String(cfg.sampleIntervalMs) + "\n";
  text += "runtimeSaveIntervalMs=" + String(cfg.runtimeSaveIntervalMs) + "\n";

  text += "apName=" + String(cfg.apName) + "\n";
  text += "apPassword=" + String(cfg.apPassword) + "\n";
  text += "servicePin=" + String(cfg.servicePin) + "\n";

  text += "solarFluidType=" + String((int)cfg.solarFluidType) + "\n";
  text += "solarHydraulicType=" + String((int)cfg.solarHydraulicType) + "\n";
  text += "solarCollectorType=" + String((int)cfg.solarCollectorType) + "\n";

  text += "frostEnabled=" + String(cfg.frostEnabled ? 1 : 0) + "\n";
  text += "frostCollectorOnC=" + String(cfg.frostCollectorOnC, 2) + "\n";
  text += "frostCollectorOffC=" + String(cfg.frostCollectorOffC, 2) + "\n";
  text += "frostSinkMinC=" + String(cfg.frostSinkMinC, 2) + "\n";
  text += "frostPumpPercent=" + String(cfg.frostPumpPercent) + "\n";
  text += "frostProtectionStorageRole=" + String(ds18RoleToInt(cfg.frostProtectionStorageRole)) + "\n";
  text += "frostSafeCollectorTemperatureC=" + String(cfg.frostSafeCollectorTemperatureC, 2) + "\n";

  text += "stagnationEnabled=" + String(cfg.stagnationEnabled ? 1 : 0) + "\n";
  text += "stagnationCollectorOnC=" + String(cfg.stagnationCollectorOnC, 2) + "\n";
  text += "stagnationCollectorOffC=" + String(cfg.stagnationCollectorOffC, 2) + "\n";
  text += "sinkMaxC=" + String(cfg.sinkMaxC, 2) + "\n";
  text += "stagnationPumpPercent=" + String(cfg.stagnationPumpPercent) + "\n";

  text += "storageProtectionEnabled=" + String(cfg.storageProtectionEnabled ? 1 : 0) + "\n";
  text += "storageCriticalTemperatureC=" + String(cfg.storageCriticalTemperatureC, 2) + "\n";
  text += "safetyNightCoolingEnabled=" + String(cfg.safetyNightCoolingEnabled ? 1 : 0) + "\n";
  text += "safetyNightCoolingTargetTemperatureC=" + String(cfg.safetyNightCoolingTargetTemperatureC, 2) + "\n";

  text += "ovenProtectionEnabled=" + String(cfg.ovenProtectionEnabled ? 1 : 0) + "\n";
  text += "ovenOvertemperatureOnC=" + String(cfg.ovenOvertemperatureOnC, 2) + "\n";
  text += "ovenOvertemperatureOffC=" + String(cfg.ovenOvertemperatureOffC, 2) + "\n";

  // MAX1
  text += "max1Enabled=" + String(cfg.max1.enabled ? 1 : 0) + "\n";
  text += "max1OffsetC=" + String(cfg.max1.offsetC, 2) + "\n";
  text += "max1CalFactor=" + String(cfg.max1.calFactor, 4) + "\n";

  // MAX2
  text += "max2Enabled=" + String(cfg.max2.enabled ? 1 : 0) + "\n";
  text += "max2OffsetC=" + String(cfg.max2.offsetC, 2) + "\n";
  text += "max2CalFactor=" + String(cfg.max2.calFactor, 4) + "\n";

  // MAX3
  text += "max3Enabled=" + String(cfg.max3.enabled ? 1 : 0) + "\n";
  text += "max3OffsetC=" + String(cfg.max3.offsetC, 2) + "\n";
  text += "max3CalFactor=" + String(cfg.max3.calFactor, 4) + "\n";

  // Relais-Konfiguration
  for (uint8_t i = 0; i < RELAY_COUNT; i++) {
    const String prefix = "relay" + String(i) + "_";
    text += prefix + "enabled=" + String(cfg.relays[i].enabled ? 1 : 0) + "\n";
    text += prefix + "function=" + String(relayFunctionToInt(cfg.relays[i].function)) + "\n";
    text += prefix + "activeLow=" + String(cfg.relays[i].activeLow ? 1 : 0) + "\n";
  }

  // PCA/PO-Ausgangs-Konfiguration
  for (uint8_t i = 0; i < PWM_OUTPUT_COUNT; i++) {
    const String prefix = "pwmOutput" + String(i) + "_";
    text += prefix + "enabled=" + String(cfg.pwmOutputs[i].enabled ? 1 : 0) + "\n";
    text += prefix + "mode=" + String(pwmOutputModeToInt(cfg.pwmOutputs[i].mode)) + "\n";
    text += prefix + "function=" + String(relayFunctionToInt(cfg.pwmOutputs[i].function)) + "\n";
    text += prefix + "profile=" + String(pwmProfileToInt(cfg.pwmOutputs[i].profile)) + "\n";
  }

  // Pumpen-Konfiguration
  for (uint8_t i = 0; i < MAX_PUMPS; i++) {
    const String prefix = "pump" + String(i) + "_";
    text += prefix + "enabled=" + String(cfg.pumps[i].enabled ? 1 : 0) + "\n";
    text += prefix + "mode=" + String(pumpModeToInt(cfg.pumps[i].mode)) + "\n";
    text += prefix + "sourceType=" + String(pumpSourceTypeToInt(cfg.pumps[i].sourceType)) + "\n";
    text += prefix + "sourceRole=" + String(heatSourceRoleToInt(cfg.pumps[i].sourceRole)) + "\n";
    text += prefix + "sourceSensorRole=" + String(ds18RoleToInt(cfg.pumps[i].sourceSensorRole)) + "\n";
    text += prefix + "sinkRole=" + String(ds18RoleToInt(cfg.pumps[i].sinkRole)) + "\n";
    text += prefix + "relayIndex=" + String(cfg.pumps[i].relayIndex) + "\n";
    text += prefix + "pwmChannel=" + String(cfg.pumps[i].pwmChannel) + "\n";
    text += prefix + "pwmProfile=" + String(cfg.pumps[i].pwmProfile == PwmProfile::HEATING ? 1 : 0) + "\n";
    text += prefix + "feedbackPin=" + String(cfg.pumps[i].feedbackPin) + "\n";
    text += prefix + "targetDiff=" + String(cfg.pumps[i].targetDiff, 2) + "\n";
    text += prefix + "hysteresis=" + String(cfg.pumps[i].hysteresis, 2) + "\n";
    text += prefix + "startDiff=" + String(cfg.pumps[i].startDiff, 2) + "\n";
    text += prefix + "pidKp=" + String(cfg.pumps[i].pidKp, 4) + "\n";
    text += prefix + "pidKi=" + String(cfg.pumps[i].pidKi, 4) + "\n";
    text += prefix + "pidKd=" + String(cfg.pumps[i].pidKd, 4) + "\n";
    text += prefix + "minPwmPercent=" + String(cfg.pumps[i].minPwmPercent, 2) + "\n";
    text += prefix + "maxPwmPercent=" + String(cfg.pumps[i].maxPwmPercent, 2) + "\n";
    text += prefix + "switchValveEnabled=" + String(cfg.pumps[i].switchValveEnabled ? 1 : 0) + "\n";
    text += prefix + "switchValveRelayIndex=" + String(cfg.pumps[i].switchValveRelayIndex) + "\n";
    text += prefix + "switchValveTravelTimeMs=" + String(cfg.pumps[i].switchValveTravelTimeMs) + "\n";
    text += prefix + "switchValveStateForTargetA=" + String(cfg.pumps[i].switchValveStateForTargetA ? 1 : 0) + "\n";

    for (uint8_t t = 0; t < PUMP_ROUTE_TARGET_COUNT; t++) {
      const String tPrefix = prefix + "target" + String(t) + "_";
      text += tPrefix + "enabled=" + String(cfg.pumps[i].targets[t].enabled ? 1 : 0) + "\n";
      text += tPrefix + "sinkRole=" + String(ds18RoleToInt(cfg.pumps[i].targets[t].sinkRole)) + "\n";
      text += tPrefix + "targetDiffOverride=" + String(cfg.pumps[i].targets[t].targetDiffOverride, 2) + "\n";
      text += tPrefix + "hysteresisOverride=" + String(cfg.pumps[i].targets[t].hysteresisOverride, 2) + "\n";
      text += tPrefix + "minTempC=" + String(cfg.pumps[i].targets[t].minTempC, 2) + "\n";
      text += tPrefix + "maxTempC=" + String(cfg.pumps[i].targets[t].maxTempC, 2) + "\n";
    }
  }


  text += "auxHeaterEnabled=" + String(cfg.auxHeater.enabled ? 1 : 0) + "\n";
  text += "auxHeaterMinimumTemperatureC=" + String(cfg.auxHeater.minimumTemperatureC, 2) + "\n";
  text += "auxHeaterTargetTemperatureC=" + String(cfg.auxHeater.targetTemperatureC, 2) + "\n";
  text += "auxHeaterHysteresisC=" + String(cfg.auxHeater.hysteresisC, 2) + "\n";
  text += "auxHeaterSinkRole=" + String(ds18RoleToInt(cfg.auxHeater.sinkRole)) + "\n";
  text += "auxHeaterPumpRelay=" + String(cfg.auxHeater.pumpRelay) + "\n";
  text += "auxHeaterRelay1=" + String(cfg.auxHeater.heaterRelay1) + "\n";
  text += "auxHeaterRelay2=" + String(cfg.auxHeater.heaterRelay2) + "\n";
  text += "auxHeaterRelay3=" + String(cfg.auxHeater.heaterRelay3) + "\n";
  text += "auxHeaterPreRunMs=" + String(cfg.auxHeater.preRunMs) + "\n";
  text += "auxHeaterCooldownMs=" + String(cfg.auxHeater.cooldownMs) + "\n";

  text += "ovenEnabled=" + String(cfg.oven.enabled ? 1 : 0) + "\n";
  text += "ovenPumpRelay=" + String(cfg.oven.pumpRelay) + "\n";
  text += "ovenTargetOvenTemperatureC=" + String(cfg.oven.targetOvenTemperatureC) + "\n";
  text += "ovenCriticalOvenTemperatureC=" + String(cfg.oven.criticalOvenTemperatureC) + "\n";
  text += "ovenPumpOnTemperatureC=" + String(cfg.oven.pumpOnTemperatureC) + "\n";
  text += "ovenPumpHysteresisC=" + String(cfg.oven.pumpHysteresisC) + "\n";
  text += "ovenPumpOnTemperatureDifferenceC=" + String(cfg.oven.pumpOnTemperatureDifferenceC) + "\n";
  text += "ovenPumpOffTemperatureDifferenceC=" + String(cfg.oven.pumpOffTemperatureDifferenceC) + "\n";
  text += "ovenServoMinimumAngle=" +  String(cfg.oven.servoMinimumAngle) + "\n";
  text += "ovenServoMaximumAngle=" + String(cfg.oven.servoMaximumAngle) + "\n";
  text += "ovenServoBaseAngle=" + String(cfg.oven.servoBaseAngle) + "\n";
  text += "ovenPidKp=" + String(cfg.oven.pidKp) + "\n";
  text += "ovenPidKi=" + String(cfg.oven.pidKi) + "\n";
  text += "ovenPidKd=" + String(cfg.oven.pidKd) + "\n";
  text += "ovenTargetSinkRole=" + String((int)cfg.oven.targetSinkRole) + "\n";
  text += "ovenAutoStartEnabled=" + String(cfg.oven.autoStartEnabled ? 1 : 0) + "\n";
  text += "ovenAutoStartTemperatureC=" + String(cfg.oven.autoStartTemperatureC) + "\n";
  text += "ovenAutoStartRiseC=" + String(cfg.oven.autoStartRiseC) + "\n";
  text += "ovenAutoReturnToStandbyTemperatureC=" + String(cfg.oven.autoReturnToStandbyTemperatureC) + "\n";
  text += "ovenPumpStopDropFromPeakC=" + String(cfg.oven.pumpStopDropFromPeakC) + "\n"; 


  



  for (uint8_t i = 0; i < MAX_HEATING_CIRCUITS; i++) {
    const String prefix = "heatingCircuit" + String(i) + "_";
    const HeatingCircuitConfig& hk = cfg.heatingCircuits[i];
    text += prefix + "enabled=" + String(hk.enabled ? 1 : 0) + "\n";
    text += prefix + "mixerType=" + String((int)hk.mixerType) + "\n";
    text += prefix + "controlMode=" + String((int)hk.controlMode) + "\n";
    text += prefix + "mixerOpenOutputKind=" + String((int)hk.mixerOpenOutput.kind) + "\n";
    text += prefix + "mixerOpenOutputIndex=" + String(hk.mixerOpenOutput.index) + "\n";
    text += prefix + "mixerCloseOutputKind=" + String((int)hk.mixerCloseOutput.kind) + "\n";
    text += prefix + "mixerCloseOutputIndex=" + String(hk.mixerCloseOutput.index) + "\n";
    text += prefix + "pumpMode=" + String((int)hk.pumpMode) + "\n";
    text += prefix + "pumpOutputKind=" + String((int)hk.pumpOutput.kind) + "\n";
    text += prefix + "pumpOutputIndex=" + String(hk.pumpOutput.index) + "\n";
    text += prefix + "pumpMinPercent=" + String(hk.pumpMinPercent) + "\n";
    text += prefix + "pumpMaxPercent=" + String(hk.pumpMaxPercent) + "\n";
    text += prefix + "flowSensorRole=" + String(ds18RoleToInt(hk.flowSensorRole)) + "\n";
    text += prefix + "returnSensorRole=" + String(ds18RoleToInt(hk.returnSensorRole)) + "\n";
    text += prefix + "roomSensorRole=" + String(ds18RoleToInt(hk.roomSensorRole)) + "\n";
    text += prefix + "bufferReferenceRole=" + String(ds18RoleToInt(hk.bufferReferenceRole)) + "\n";
    text += prefix + "outsideSensorRole=" + String(ds18RoleToInt(hk.outsideSensorRole)) + "\n";
    text += prefix + "fixedFlowTemperatureC=" + String(hk.fixedFlowTemperatureC, 2) + "\n";
    text += prefix + "maximumFlowTemperatureC=" + String(hk.maximumFlowTemperatureC, 2) + "\n";
    text += prefix + "minimumFlowTemperatureC=" + String(hk.minimumFlowTemperatureC, 2) + "\n";
    text += prefix + "roomTargetTemperatureC=" + String(hk.roomTargetTemperatureC, 2) + "\n";
    text += prefix + "roomInfluenceK=" + String(hk.roomInfluenceK, 2) + "\n";
    text += prefix + "heatingCurveBaseC=" + String(hk.heatingCurveBaseC, 2) + "\n";
    text += prefix + "heatingCurveSlope=" + String(hk.heatingCurveSlope, 3) + "\n";
    text += prefix + "frostProtectionEnabled=" + String(hk.frostProtectionEnabled ? 1 : 0) + "\n";
    text += prefix + "frostStartTemperatureC=" + String(hk.frostStartTemperatureC, 2) + "\n";
    text += prefix + "frostTargetFlowTemperatureC=" + String(hk.frostTargetFlowTemperatureC, 2) + "\n";
    text += prefix + "mixerFullTravelMs=" + String(hk.mixerFullTravelMs) + "\n";
    text += prefix + "mixerPulseMs=" + String(hk.mixerPulseMs) + "\n";
    text += prefix + "mixerPauseMs=" + String(hk.mixerPauseMs) + "\n";
  }

  return writeFileText(FILE_CONFIG, text);
}

bool loadDiagnostics(DiagnosticData& d) {
  setDiagDefaults(d);

  String text = readFileText(FILE_DIAGNOSTICS);
  if (text.isEmpty()) {
    return false;
  }

  String v;

  v = valueOf(text, "faultActive");
  if (v.length()) d.faultActive = (v.toInt() != 0);

  v = valueOf(text, "faultText");
  if (v.length()) copyText(d.faultText, sizeof(d.faultText), v.c_str());

  v = valueOf(text, "bootCount");
  if (v.length()) d.bootCount = (uint32_t)v.toInt();

  v = valueOf(text, "sensorErrorCount");
  if (v.length()) d.sensorErrorCount = (uint32_t)v.toInt();

  v = valueOf(text, "sdErrorCount");
  if (v.length()) d.sdErrorCount = (uint32_t)v.toInt();

  v = valueOf(text, "faultCount");
  if (v.length()) d.faultCount = (uint32_t)v.toInt();

  v = valueOf(text, "maxErrorCount");
  if (v.length()) d.maxErrorCount = (uint32_t)v.toInt();

  v = valueOf(text, "lastCollectorC");
  if (v.length()) d.lastCollectorC = v.toFloat();

  v = valueOf(text, "lastSinkC");
  if (v.length()) d.lastSinkC = v.toFloat();

  v = valueOf(text, "lastDiffC");
  if (v.length()) d.lastDiffC = v.toFloat();

  v = valueOf(text, "frostEventCount");
  if (v.length()) d.frostEventCount = (uint32_t)v.toInt();

  v = valueOf(text, "stagnationEventCount");
  if (v.length()) d.stagnationEventCount = (uint32_t)v.toInt();

  v = valueOf(text, "storageReachedMaximumAtMs");
  if (v.length()) d.storageReachedMaximumAtMs = (uint32_t)v.toInt();

  v = valueOf(text, "lastProtectionMode");
  if (v.length()) d.lastProtectionMode = (uint8_t)v.toInt();

  return true;
}

bool saveDiagnostics(const DiagnosticData& d) {
  String text;

  text += "faultActive=" + String(d.faultActive ? 1 : 0) + "\n";
  text += "faultText=" + String(d.faultText) + "\n";
  text += "bootCount=" + String(d.bootCount) + "\n";
  text += "sensorErrorCount=" + String(d.sensorErrorCount) + "\n";
  text += "sdErrorCount=" + String(d.sdErrorCount) + "\n";
  text += "faultCount=" + String(d.faultCount) + "\n";
  text += "maxErrorCount=" + String(d.maxErrorCount) + "\n";

  text += "lastCollectorC=" + floatOrEmpty(d.lastCollectorC, 2) + "\n";
  text += "lastSinkC=" + floatOrEmpty(d.lastSinkC, 2) + "\n";
  text += "lastDiffC=" + floatOrEmpty(d.lastDiffC, 2) + "\n";

  text += "frostEventCount=" + String(d.frostEventCount) + "\n";
  text += "stagnationEventCount=" + String(d.stagnationEventCount) + "\n";
  text += "storageReachedMaximumAtMs=" + String(d.storageReachedMaximumAtMs) + "\n";
  text += "lastProtectionMode=" + String(d.lastProtectionMode) + "\n";

  return writeFileText(FILE_DIAGNOSTICS, text);
}

bool loadMaintenance(MaintenanceData& m) {
  setMaintenanceDefaults(m);

  String text = readFileText(FILE_MAINTENANCE);
  if (text.isEmpty()) {
    return false;
  }

  String v;

  v = valueOf(text, "pumpStarts");
  if (v.length()) m.pumpStarts = (uint32_t)v.toInt();

  v = valueOf(text, "relaySwitchCount");
  if (v.length()) m.relaySwitchCount = (uint32_t)v.toInt();

  v = valueOf(text, "pumpRuntimeSeconds");
  if (v.length()) m.pumpRuntimeSeconds = (uint64_t)v.toInt();

  v = valueOf(text, "controllerRuntimeSeconds");
  if (v.length()) m.controllerRuntimeSeconds = (uint64_t)v.toInt();

  v = valueOf(text, "lastServiceDate");
  if (v.length()) copyText(m.lastServiceDate, sizeof(m.lastServiceDate), v.c_str());

  v = valueOf(text, "serviceDue");
  if (v.length()) m.serviceDue = (v.toInt() != 0);

  return true;
}

bool saveMaintenance(const MaintenanceData& m) {
  String text;

  text += "pumpStarts=" + String(m.pumpStarts) + "\n";
  text += "relaySwitchCount=" + String(m.relaySwitchCount) + "\n";
  text += "pumpRuntimeSeconds=" + String((unsigned long)m.pumpRuntimeSeconds) + "\n";
  text += "controllerRuntimeSeconds=" + String((unsigned long)m.controllerRuntimeSeconds) + "\n";
  text += "lastServiceDate=" + String(m.lastServiceDate) + "\n";
  text += "serviceDue=" + String(m.serviceDue ? 1 : 0) + "\n";

  return writeFileText(FILE_MAINTENANCE, text);
}

bool loadSensorAssignments(SensorAssignmentTable& table) {
  setAssignmentDefaults(table);

  String text = readFileText(FILE_SENSOR_ASSIGNMENTS);
  if (text.isEmpty()) {
    return false;
  }

  String v = valueOf(text, "count");
  int count = v.length() ? v.toInt() : 0;

  if (count < 0) count = 0;
  if (count > MAX_DS18B20_SENSORS) count = MAX_DS18B20_SENSORS;

  for (int i = 0; i < count; i++) {
    String roleKey = valueOf(text, "role_" + String(i));
    String addr    = valueOf(text, "addr_" + String(i));

    if (!roleKey.length() || !addr.length()) {
      continue;
    }

    Ds18Role role = SensorRoles::fromKey(roleKey);
    if (role == Ds18Role::NONE) {
      continue;
    }

    uint8_t parsed[8] = {0};
    if (!SinkSensor::parseAddressString(addr, parsed)) {
      continue;
    }

    SensorAssignments::setRole(table, role, parsed, addr.c_str());
  }

  return table.count > 0;
}

bool saveSensorAssignments(const SensorAssignmentTable& table) {
  String text;
  text += "count=" + String(table.count) + "\n";

  for (uint8_t i = 0; i < table.count; i++) {
    if (!table.items[i].assigned || table.items[i].role == Ds18Role::NONE) {
      continue;
    }

    text += "role_" + String(i) + "=" + String(SensorRoles::toKey(table.items[i].role)) + "\n";
    text += "addr_" + String(i) + "=" + String(table.items[i].addressText) + "\n";
  }

  return writeFileText(FILE_SENSOR_ASSIGNMENTS, text);
}

}