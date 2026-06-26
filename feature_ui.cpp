#include "feature_ui.h"
#include "config.h"
#include "feature_sdcard.h"
#include "feature_storage.h"
#include "feature_sensor_assignments.h"
#include "feature_sensor_roles.h"
#include "feature_heat_source_roles.h"
#include "feature_heat_source_assignments.h"
#include "feature_heat_source_storage.h"
#include "feature_relay_outputs.h"
#include "feature_pwm_pca9685.h"
#include "feature_sink_ds18b20.h"
#include "feature_safety_manager.h"
#include "feature_oven.h"
#include "feature_output_validation.h"
#include <WebServer.h>
#include <SD.h>

static WebServer server(80);

namespace {
  String adcHardwareLabel(uint8_t index) {
  return "ADC" + String(index + 1);
}

String pwmOutputHardwareLabel(uint8_t index) {
  return "PO-" + String(index);
}

String feedbackHardwareLabel(uint8_t index) {
  return "FB" + String(index);
}

String relayHardwareLabel(uint8_t index) {
  return "R" + String(index);
}

String pwmOutputModeToKey(PwmOutputMode mode) {
  return (mode == PwmOutputMode::SWITCH) ? "switch" : "pwm";
}

PwmOutputMode pwmOutputModeFromKey(const String& key) {
  return (key == "switch") ? PwmOutputMode::SWITCH : PwmOutputMode::PWM;
}

String pwmProfileToKey(PwmProfile profile) {
  return (profile == PwmProfile::HEATING) ? "heating" : "solar";
}

String pwmOutputModeLabel(PwmOutputMode mode) {
  return (mode == PwmOutputMode::SWITCH) ? "Schaltausgang" : "PWM";
}
  bool g_commissioningTestActive = false;
  uint32_t g_commissioningTestStartedMs = 0;
   AppContext* s_ctx = nullptr;
  static uint32_t s_commissioningTestUntilMs = 0;
  static constexpr uint32_t COMMISSIONING_TEST_TIMEOUT_MS = 10UL * 60UL * 1000UL;

  void activateCommissioningTestMode() {
    g_commissioningTestActive = true;
    g_commissioningTestStartedMs = millis();
    s_commissioningTestUntilMs = millis() + COMMISSIONING_TEST_TIMEOUT_MS;

    Serial.println("INBETRIEBNAHME TESTMODUS AKTIV - REGELUNG PAUSIERT");
    Serial.flush();
  }

  void deactivateCommissioningTestMode() {
    if (s_ctx) {
      RelayOutputs::allOff(*s_ctx);
    }
    PwmDriver::allOff();

    g_commissioningTestActive = false;
    g_commissioningTestStartedMs = 0;
    s_commissioningTestUntilMs = 0;

    Serial.println("INBETRIEBNAHME TESTMODUS AUS - ALLE AUSGAENGE AUS");
    Serial.flush();
  }

  bool serviceSessionValid() {
    return true;
  }

  bool testSessionValid() {
    if (!s_ctx) return false;
    if (!server.hasArg("pin")) return false;
    return server.arg("pin") == String(s_ctx->config.servicePin);
  }

  bool commissioningTestAuthorized() {
    return g_commissioningTestActive || testSessionValid();
  }

  String contentType(const String& path) {
    if (path.endsWith(".html")) return "text/html";
    if (path.endsWith(".css")) return "text/css";
    if (path.endsWith(".js")) return "application/javascript";
    if (path.endsWith(".json")) return "application/json";
    return "text/plain";
  }

  bool serveSdFile(const String& path) {
    if (!SD.exists(path)) return false;

    File file = SD.open(path);
    if (!file) return false;

    server.streamFile(file, contentType(path));
    file.close();
    return true;
  }

  void handleRoot() {
    Serial.println("HTTP GET ROOT");

    if (serveSdFile("/index.html")) return;
    if (serveSdFile("/www/index.html")) return;

    server.send(404, "text/plain", "Keine index.html gefunden");
  }

  void handleStatic() {
    String path = server.uri();

    if (serveSdFile(path)) return;
    if (serveSdFile("/www" + path)) return;

    server.send(404, "text/plain", "Datei nicht gefunden");
  }

  // ================================
  // STATUS API
  // ================================
void handleStatusJson() {
  if (!s_ctx) {
    server.send(500, "application/json", "{\"error\":\"no_context\"}");
    return;
  }

  Ds18Role sinkRole = SensorAssignments::activeSinkRole(s_ctx->config);
  String activeSinkLabel = SensorRoles::toLabel(sinkRole);
  String activeSinkKey = SensorRoles::toKey(sinkRole);

  for (uint8_t i = 0; i < MAX_PUMPS; i++) {
    const PumpConfig& pump = s_ctx->config.pumps[i];
    if (!pump.state) continue;

    if (pump.switchValveEnabled && pump.activeTargetIndex < PUMP_ROUTE_TARGET_COUNT) {
      const Ds18Role targetRole = pump.targets[pump.activeTargetIndex].sinkRole;
      if (targetRole != Ds18Role::NONE) {
        activeSinkLabel = SensorRoles::toLabel(targetRole);
        activeSinkKey = SensorRoles::toKey(targetRole);
      }
    } else if (pump.sinkRole != Ds18Role::NONE) {
      activeSinkLabel = SensorRoles::toLabel(pump.sinkRole);
      activeSinkKey = SensorRoles::toKey(pump.sinkRole);
    }
    break;
  }

  String json = "{";
  json += "\"collectorC\":" + String(s_ctx->sensors.collectorValid ? String(s_ctx->sensors.collectorC, 2) : "null") + ",";
  json += "\"sinkC\":" + String(s_ctx->sensors.sinkValid ? String(s_ctx->sensors.sinkC, 2) : "null") + ",";
  json += "\"pumpOn\":" + String(s_ctx->control.relayEnable ? "true" : "false") + ",";
  json += "\"pwmPercent\":" + String(s_ctx->control.pwmPercent) + ",";
  json += "\"faultActive\":" + String(s_ctx->diag.faultActive ? "true" : "false") + ",";
  json += "\"faultText\":\"" + String(s_ctx->diag.faultText) + "\",";
  json += "\"activeSinkRole\":\"" + activeSinkKey + "\",";
  json += "\"activeSink\":\"" + activeSinkLabel + "\"";
  json += "}";

  server.send(200, "application/json", json);
}

  // ================================
  // SENSOR LISTE
  // ================================
void handleDs18b20Json() {
  if (!s_ctx) {
    server.send(500, "application/json", "{\"error\":\"no_context\"}");
    return;
  }

  SinkSensor::refreshInventoryTemps(s_ctx->ds18b20);

  String json = "{ \"devices\": [";

  for (uint8_t i = 0; i < s_ctx->ds18b20.count; i++) {
    if (i > 0) json += ",";

    json += "{";
    json += "\"id\":" + String(i + 1) + ",";
    json += "\"name\":\"Sensor " + String(i + 1) + "\",";
    json += "\"address\":\"" + String(s_ctx->ds18b20.devices[i].addressText) + "\",";
    json += "\"temp\":" + String(s_ctx->ds18b20.devices[i].lastValid ? String(s_ctx->ds18b20.devices[i].lastTempC, 2) : "null");
    json += "}";
  }

  json += "]}";
  server.send(200, "application/json", json);
}

  //=========
  //
  //=========
void handleAssignmentsJson() {
  if (!s_ctx) {
    server.send(500, "application/json", "{\"error\":\"no_context\"}");
    return;
  }

  String json = "{ \"assignments\": [";

  for (uint8_t i = 0; i < s_ctx->assignments.count; i++) {
    if (i > 0) json += ",";

    const Ds18RoleAssignment& a = s_ctx->assignments.items[i];

    json += "{";
    json += "\"role\":\"" + String(SensorRoles::toKey(a.role)) + "\",";
    json += "\"address\":\"" + String(a.addressText) + "\"";
    json += "}";
  }

  json += "]}";
  server.send(200, "application/json", json);
}
  // ================================
  // ROLLEN LISTE
  // ================================
  void handleRolesJson() {
    String json = "{ \"roles\": [";

    bool firstRole = true;
    for (int i = 0; i <= (int)Ds18Role::HK4_RETURN; i++) {
      Ds18Role role = (Ds18Role)i;
      if (!SensorRoles::isAssignable(role)) continue;
      if (!firstRole) json += ",";
      firstRole = false;

      json += "{";
      json += "\"key\":\"" + String(SensorRoles::toKey(role)) + "\",";
      json += "\"label\":\"" + String(SensorRoles::toLabel(role)) + "\"";
      json += "}";
    }

    json += "]}";
    server.send(200, "application/json", json);
  }

  // ================================
  // ROLLE ZUWEISEN
  // ================================
  void handleAssignRole() {
    if (!serviceSessionValid()) {
      server.send(403, "text/plain", "Nicht erlaubt");
      return;
    }

    String roleKey = server.arg("role");
    String address = server.arg("address");

    Ds18Role role = SensorRoles::fromKey(roleKey);

    if (role == Ds18Role::NONE || !SensorRoles::isAssignable(role)) {
      server.send(400, "text/plain", "Ungueltige oder fuer DS18 nicht erlaubte Rolle");
      return;
    }

    for (uint8_t i = 0; i < s_ctx->assignments.count; i++) {
      const Ds18RoleAssignment& existing = s_ctx->assignments.items[i];
      if (!existing.assigned) continue;

      if (existing.role == role && String(existing.addressText) != address) {
        server.send(409, "text/plain", "Diese Sensorrolle ist bereits einem anderen DS18B20 zugeordnet");
        return;
      }

      if (String(existing.addressText) == address && existing.role != role) {
        server.send(409, "text/plain", "Dieser DS18B20 ist bereits einer anderen Rolle zugeordnet");
        return;
      }
    }

    if (!SensorAssignments::assignRoleByAddressText(
          s_ctx->ds18b20,
          address,
          role,
          s_ctx->assignments)) {
      server.send(400, "text/plain", "Sensor nicht gefunden");
      return;
    }

    Storage::saveSensorAssignments(s_ctx->assignments);

    server.send(200, "text/plain", "OK");
  }

  // ================================
  // AKTIVEN SINK SETZEN
  // ================================
  void handleSetSinkTarget() {
    String target = server.arg("target");

    if (target == "buffer") {
      s_ctx->config.activeSinkTarget = SinkTarget::BUFFER_TOP;
    } else {
      s_ctx->config.activeSinkTarget = SinkTarget::BOILER_TOP;
    }

    Storage::saveConfig(s_ctx->config);

    server.send(200, "text/plain", "OK");
  }

  bool readRoleTemp(Ds18Role role, float& tempC, bool& valid) {
  return SensorAssignments::readByRole(
    s_ctx->assignments,
    role,
    tempC,
    valid
  );
}

void handlePlantJson() {
  if (!s_ctx) {
    server.send(500, "application/json", "{\"error\":\"no_context\"}");
    return;
  }

  Ds18Role activeSinkRole = SensorAssignments::activeSinkRole(s_ctx->config);
  Ds18RoleAssignment sinkAssignment;
  bool sinkAssigned = SensorAssignments::getAssignment(s_ctx->assignments, activeSinkRole, sinkAssignment);

  float boilerTop = NAN, boilerBottom = NAN;
  float bufferTop = NAN, bufferHigh = NAN, bufferMid = NAN, bufferBottom = NAN;
  float collector1Flow = s_ctx->sensors.collectorValid ? s_ctx->sensors.collectorC : NAN;
  float collector1Return = NAN;
  float poolTemp = NAN;

  bool valid = false;

  SensorAssignments::readByRole(s_ctx->assignments, Ds18Role::SINK_BOILER_TOP, boilerTop, valid);
  SensorAssignments::readByRole(s_ctx->assignments, Ds18Role::BOILER_BOTTOM, boilerBottom, valid);

  SensorAssignments::readByRole(s_ctx->assignments, Ds18Role::SINK_BUFFER_TOP, bufferTop, valid); 
  SensorAssignments::readByRole(s_ctx->assignments, Ds18Role::BUFFER_HIGH, bufferHigh, valid);
  SensorAssignments::readByRole(s_ctx->assignments, Ds18Role::BUFFER_MID, bufferMid, valid);
  SensorAssignments::readByRole(s_ctx->assignments, Ds18Role::BUFFER_BOTTOM, bufferBottom, valid);

  SensorAssignments::readByRole(s_ctx->assignments, Ds18Role::RETURN_COLLECTOR_1, collector1Return, valid);
  SensorAssignments::readByRole(s_ctx->assignments, Ds18Role::SWIMMINGPOOL, poolTemp, valid);

  bool boilerPresent =
    SensorAssignments::hasRole(s_ctx->assignments, Ds18Role::SINK_BOILER_TOP) ||
    SensorAssignments::hasRole(s_ctx->assignments, Ds18Role::BOILER_BOTTOM);

  bool bufferPresent =
    SensorAssignments::hasRole(s_ctx->assignments, Ds18Role::SINK_BUFFER_TOP) ||
    SensorAssignments::hasRole(s_ctx->assignments, Ds18Role::BUFFER_HIGH) ||
    SensorAssignments::hasRole(s_ctx->assignments, Ds18Role::BUFFER_MID) ||
    SensorAssignments::hasRole(s_ctx->assignments, Ds18Role::BUFFER_BOTTOM);

  bool collector1Present = s_ctx->sensors.collectorValid;

  bool poolPresent =
    SensorAssignments::hasRole(s_ctx->assignments, Ds18Role::SWIMMINGPOOL);

  uint8_t zoneValveCount = 0;
  for (uint8_t i = 0; i < RELAY_COUNT; i++) {
    if (RelayOutputs::isUsableAsZoneValve(*s_ctx, i)) {
      zoneValveCount++;
    }
  }

  String json = "{";

  json += "\"activeSinkRole\":\"" + String(SensorRoles::toKey(activeSinkRole)) + "\",";
  json += "\"activeSinkLabel\":\"" + String(SensorRoles::toLabel(activeSinkRole)) + "\",";
  json += "\"sinkAssigned\":" + String(sinkAssigned ? "true" : "false") + ",";
  json += "\"pumpOn\":" + String(s_ctx->control.relayEnable ? "true" : "false") + ",";

  json += "\"components\":{";
  json += "\"boiler\":" + String(boilerPresent ? "true" : "false") + ",";
  json += "\"buffer\":" + String(bufferPresent ? "true" : "false") + ",";
  json += "\"collector1\":" + String(collector1Present ? "true" : "false") + ",";
  json += "\"pool\":" + String(poolPresent ? "true" : "false") + ",";
  json += "\"zoneValves\":" + String(zoneValveCount > 0 ? "true" : "false");
  json += "},";

  json += "\"zoneValves\":[";
  bool firstZoneValve = true;
  for (uint8_t i = 0; i < RELAY_COUNT; i++) {
    if (!RelayOutputs::isUsableAsZoneValve(*s_ctx, i)) continue;
    if (!firstZoneValve) json += ",";
    firstZoneValve = false;
    json += "{";
    json += "\"index\":" + String(i) + ",";
    json += "\"name\":\"" + relayHardwareLabel(i) + "\",";
    json += "\"state\":" + String(RelayOutputs::get(*s_ctx, i) ? "true" : "false");
    json += "}";
  }
  json += "],";

  json += "\"heatSources\":{";
  json += "\"solar1\":" + String(s_ctx->sensors.heatSources.solarCollector1Valid ? String(s_ctx->sensors.heatSources.solarCollector1C, 2) : "null") + ",";
  json += "\"solar2\":" + String(s_ctx->sensors.heatSources.solarCollector2Valid ? String(s_ctx->sensors.heatSources.solarCollector2C, 2) : "null") + ",";
  json += "\"solar3\":" + String(s_ctx->sensors.heatSources.solarCollector3Valid ? String(s_ctx->sensors.heatSources.solarCollector3C, 2) : "null") + ",";
  json += "\"oven\":" + String(s_ctx->sensors.heatSources.altSourceOvenValid ? String(s_ctx->sensors.heatSources.altSourceOvenC, 2) : "null") + ",";
  json += "\"other\":" + String(s_ctx->sensors.heatSources.altSourceOtherValid ? String(s_ctx->sensors.heatSources.altSourceOtherC, 2) : "null");
  json += "},";

  json += "\"pumps\":[";
  bool firstPump = true;
  for (uint8_t i = 0; i < MAX_PUMPS; i++) {
    const PumpConfig& p = s_ctx->config.pumps[i];
    if (!p.enabled) continue;
    if (!firstPump) json += ",";
    firstPump = false;
    json += "{";
    json += "\"index\":" + String(i) + ",";
    json += "\"name\":\"Pumpe " + String(i + 1) + "\",";
    json += "\"state\":" + String(p.state ? "true" : "false") + ",";
    json += "\"source\":\"" + String(HeatSourceRoles::toLabel(p.sourceRole)) + "\",";
    json += "\"sink\":\"" + String(p.switchValveEnabled ? String("Umschaltventil") : String(SensorRoles::toLabel(p.sinkRole))) + "\",";
    json += "\"diff\":" + String(isnan(p.lastDiffC) ? "null" : String(p.lastDiffC, 2)) + ",";
    json += "\"pwm\":" + String(p.lastPwmPercent, 1);
    json += "}";
  }
  json += "],";

  json += "\"temperatures\":{";
  json += "\"boilerTop\":" + String(isnan(boilerTop) ? "null" : String(boilerTop, 2)) + ",";
  json += "\"boilerBottom\":" + String(isnan(boilerBottom) ? "null" : String(boilerBottom, 2)) + ",";
  json += "\"bufferTop\":" + String(isnan(bufferTop) ? "null" : String(bufferTop, 2)) + ",";
  json += "\"bufferHigh\":" + String(isnan(bufferHigh) ? "null" : String(bufferHigh, 2)) + ",";
  json += "\"bufferMid\":" + String(isnan(bufferMid) ? "null" : String(bufferMid, 2)) + ",";
  json += "\"bufferBottom\":" + String(isnan(bufferBottom) ? "null" : String(bufferBottom, 2)) + ",";
  json += "\"collector1Flow\":" + String(isnan(collector1Flow) ? "null" : String(collector1Flow, 2)) + ",";
  json += "\"collector1Return\":" + String(isnan(collector1Return) ? "null" : String(collector1Return, 2)) + ",";
  json += "\"poolTemp\":" + String(isnan(poolTemp) ? "null" : String(poolTemp, 2));
  json += "}";

  json += "}";

  server.send(200, "application/json", json);
}
void handleHeatSourcesJson() {
  if (!s_ctx) {
    server.send(500, "application/json", "{\"error\":\"no_context\"}");
    return;
  }

  String json = "{ \"assignments\": [";

  for (uint8_t i = 0; i < s_ctx->heatSourceAssignments.count; i++) {
    if (i > 0) json += ",";

    const HeatSourceAssignment& a = s_ctx->heatSourceAssignments.items[i];

    String channelKey = "ch1";
    if (a.channel == MaxChannel::CH2) channelKey = "ch2";
    else if (a.channel == MaxChannel::CH3) channelKey = "ch3";

    json += "{";
    json += "\"channel\":\"" + channelKey + "\",";
    json += "\"role\":\"" + String(HeatSourceRoles::toKey(a.role)) + "\",";
    json += "\"label\":\"" + String(HeatSourceRoles::toLabel(a.role)) + "\"";
    json += "}";
  }

  json += "]}";
  server.send(200, "application/json", json);
}
void handleMaxStatusJson() {
  if (!s_ctx) {
    server.send(500, "application/json", "{\"error\":\"no_context\"}");
    return;
  }

  String json = "{";

  json += "\"channels\":[";
  for (int i = 0; i < 3; i++) {
    if (i > 0) json += ",";

    const MaxChannelReading& r = s_ctx->maxReadings[i];
    const MaxChannelConfig& c =
      (i == 0) ? s_ctx->config.max1 :
      (i == 1) ? s_ctx->config.max2 :
                 s_ctx->config.max3;

    String channelKey = (i == 0) ? "ch1" : (i == 1) ? "ch2" : "ch3";

    json += "{";
    json += "\"channel\":\"" + channelKey + "\",";
    json += "\"label\":\"ADC" + String(i + 1) + "\",";
    json += "\"enabled\":" + String(c.enabled ? "true" : "false") + ",";
    json += "\"present\":" + String(r.present ? "true" : "false") + ",";
    json += "\"valid\":" + String(r.valid ? "true" : "false") + ",";
    json += "\"tempC\":" + String(isnan(r.tempC) ? "null" : String(r.tempC, 2)) + ",";
    json += "\"rawTempC\":" + String(isnan(r.rawTempC) ? "null" : String(r.rawTempC, 2)) + ",";
    json += "\"resistanceOhm\":" + String(isnan(r.resistanceOhm) ? "null" : String(r.resistanceOhm, 2)) + ",";
    json += "\"rawRtd\":" + String(r.rawRtd) + ",";
    json += "\"fault\":" + String(r.fault) + ",";
    json += "\"offsetC\":" + String(c.offsetC, 2) + ",";
    json += "\"calFactor\":" + String(c.calFactor, 4);
    json += "}";
  }
  json += "],";

  json += "\"activeHeatSource\":{";
  json += "\"role\":\"" + String(HeatSourceRoles::toKey(s_ctx->sensors.activeHeatSource.role)) + "\",";
  json += "\"label\":\"" + String(HeatSourceRoles::toLabel(s_ctx->sensors.activeHeatSource.role)) + "\",";
  json += "\"tempC\":" + String(isnan(s_ctx->sensors.activeHeatSource.tempC) ? "null" : String(s_ctx->sensors.activeHeatSource.tempC, 2)) + ",";
  json += "\"valid\":" + String(s_ctx->sensors.activeHeatSource.valid ? "true" : "false");
  json += "}";

  json += "}";

  server.send(200, "application/json", json);
}
void handleAssignHeatSource() {
  if (!serviceSessionValid()) {
    server.send(403, "text/plain", "Nicht erlaubt");
    return;
  }

  if (!s_ctx) {
    server.send(500, "text/plain", "Kein Kontext");
    return;
  }

  String channelArg = server.arg("channel");
  String roleArg = server.arg("role");

  MaxChannel channel = MaxChannel::CH1;
  if (channelArg.equalsIgnoreCase("ch2")) channel = MaxChannel::CH2;
  else if (channelArg.equalsIgnoreCase("ch3")) channel = MaxChannel::CH3;

  HeatSourceRole role = HeatSourceRoles::fromKey(roleArg);
  if (role == HeatSourceRole::NONE) {
    server.send(400, "text/plain", "Ungültige Wärmequellenrolle");
    return;
  }

  if (!HeatSourceAssignments::setAssignment(s_ctx->heatSourceAssignments, channel, role)) {
    server.send(400, "text/plain", "Zuordnung fehlgeschlagen");
    return;
  }

  HeatSourceStorage::saveAssignments(s_ctx->heatSourceAssignments);
  server.send(200, "text/plain", "OK");
}
void handleSaveMaxConfig() {
  if (!serviceSessionValid()) {
    server.send(403, "text/plain", "Nicht erlaubt");
    return;
  }

  if (!s_ctx) {
    server.send(500, "text/plain", "Kein Kontext");
    return;
  }

  auto applyChannel = [&](MaxChannelConfig& chCfg, const String& prefix) {
    String v;

    v = server.arg(prefix + "Enabled");
    if (v.length()) chCfg.enabled = (v.toInt() != 0);

    v = server.arg(prefix + "OffsetC");
    if (v.length()) chCfg.offsetC = v.toFloat();

    v = server.arg(prefix + "CalFactor");
    if (v.length()) chCfg.calFactor = v.toFloat();
  };

  applyChannel(s_ctx->config.max1, "max1");
  applyChannel(s_ctx->config.max2, "max2");
  applyChannel(s_ctx->config.max3, "max3");

  Storage::saveConfig(s_ctx->config);

  server.send(200, "text/plain", "OK");
}


  String relayFunctionLabel(RelayFunction f) {
    switch (f) {
      case RelayFunction::PUMP_ENABLE: return "Pump Enable";
      case RelayFunction::ZONE_VALVE: return "Zonenventil";
      case RelayFunction::HEATER_ROD: return "Heizstab";
      case RelayFunction::MIXER: return "Mischer";
      case RelayFunction::NONE:
      default: return "frei";
    }
  }

  void handleRelaysJson() {
    if (!s_ctx) {
      server.send(500, "application/json", "{\"error\":\"no_context\"}");
      return;
    }

    String json = "{";

    json += "\"relays\":[";
    for (uint8_t i = 0; i < RELAY_COUNT; i++) {
      if (i > 0) json += ",";

      const RelayOutputConfig& cfg = s_ctx->config.relays[i];
      const bool state = RelayOutputs::get(*s_ctx, i);

      json += "{";
      json += "\"type\":\"relay\",";
      json += "\"index\":" + String(i) + ",";
      json += "\"name\":\"" + relayHardwareLabel(i) + "\",";
      json += "\"enabled\":" + String(cfg.enabled ? "true" : "false") + ",";
      json += "\"function\":\"" + String(RelayOutputs::functionToKey(cfg.function)) + "\",";
      json += "\"functionLabel\":\"" + relayFunctionLabel(cfg.function) + "\",";
      json += "\"activeLow\":" + String(cfg.activeLow ? "true" : "false") + ",";
      json += "\"state\":" + String(state ? "true" : "false");
      json += "}";
    }

    json += "],\"pwmOutputs\":[";
    for (uint8_t i = 0; i < PWM_OUTPUT_COUNT; i++) {
      if (i > 0) json += ",";

      const PwmOutputConfig& cfg = s_ctx->config.pwmOutputs[i];

      json += "{";
      json += "\"type\":\"pwm\",";
      json += "\"index\":" + String(i) + ",";
      json += "\"name\":\"" + pwmOutputHardwareLabel(i) + "\",";
      json += "\"enabled\":" + String(cfg.enabled ? "true" : "false") + ",";
      json += "\"mode\":\"" + pwmOutputModeToKey(cfg.mode) + "\",";
      json += "\"modeLabel\":\"" + pwmOutputModeLabel(cfg.mode) + "\",";
      json += "\"function\":\"" + String(RelayOutputs::functionToKey(cfg.function)) + "\",";
      json += "\"functionLabel\":\"" + relayFunctionLabel(cfg.function) + "\",";
      json += "\"profile\":\"" + pwmProfileToKey(cfg.profile) + "\",";
      json += "\"percent\":" + String(PwmDriver::getDuty(i));
      json += "}";
    }

    json += "]}";
    server.send(200, "application/json", json);
  }

String outputRefKindToKey(OutputKind kind) {
  if (kind == OutputKind::RELAY) return "relay";
  if (kind == OutputKind::PWM_OUTPUT) return "pwm";
  return "none";
}

String outputRefLabel(OutputKind kind, uint8_t index) {
  if (kind == OutputKind::RELAY) return relayHardwareLabel(index);
  if (kind == OutputKind::PWM_OUTPUT) return pwmOutputHardwareLabel(index);
  return "nicht zugewiesen";
}

bool outputRefAssigned(OutputKind kind, uint8_t index) {
  return kind != OutputKind::NONE && index != PIN_UNUSED;
}

bool sameOutputRef(OutputKind aKind, uint8_t aIndex, OutputKind bKind, uint8_t bIndex) {
  return outputRefAssigned(aKind, aIndex) &&
         outputRefAssigned(bKind, bIndex) &&
         aKind == bKind &&
         aIndex == bIndex;
}

bool outputExistsAndMatchesFunction(const AppContext& ctx, OutputKind kind, uint8_t index, RelayFunction requiredFunction, bool allowPwmMode) {
  if (!outputRefAssigned(kind, index)) return true;

  if (kind == OutputKind::RELAY) {
    if (index >= RELAY_COUNT) return false;
    const RelayOutputConfig& out = ctx.config.relays[index];
    return out.enabled && out.function == requiredFunction;
  }

  if (kind == OutputKind::PWM_OUTPUT) {
    if (index >= PWM_OUTPUT_COUNT) return false;
    const PwmOutputConfig& out = ctx.config.pwmOutputs[index];
    if (!out.enabled) return false;

    if (allowPwmMode && out.mode == PwmOutputMode::PWM) return true;

    return out.mode == PwmOutputMode::SWITCH && out.function == requiredFunction;
  }

  return false;
}

bool outputIsUsedByHeatingCircuit(const AppContext& ctx, OutputKind kind, uint8_t index, int ignoreCircuitIndex, String& usage) {
  for (uint8_t i = 0; i < MAX_HEATING_CIRCUITS; i++) {
    if ((int)i == ignoreCircuitIndex) continue;
    const HeatingCircuitConfig& hk = ctx.config.heatingCircuits[i];
    if (!hk.enabled) continue;

    if (sameOutputRef(hk.mixerOpenOutput.kind, hk.mixerOpenOutput.index, kind, index)) {
      usage = "HK" + String(i + 1) + " Mischer AUF";
      return true;
    }
    if (sameOutputRef(hk.mixerCloseOutput.kind, hk.mixerCloseOutput.index, kind, index)) {
      usage = "HK" + String(i + 1) + " Mischer ZU";
      return true;
    }
    if (sameOutputRef(hk.pumpOutput.kind, hk.pumpOutput.index, kind, index)) {
      usage = "HK" + String(i + 1) + " Pumpe";
      return true;
    }
  }
  return false;
}

bool outputIsUsedByLegacyConsumers(const AppContext& ctx, OutputKind kind, uint8_t index, int ignorePumpIndex, String& usage, bool ignoreAuxHeater = false, bool ignoreOven = false) {
  if (!outputRefAssigned(kind, index)) return false;

  if (kind == OutputKind::RELAY) {
    for (uint8_t i = 0; i < MAX_PUMPS; i++) {
      if ((int)i == ignorePumpIndex) continue;
      const PumpConfig& pump = ctx.config.pumps[i];
      if (!pump.enabled) continue;

      if (pump.relayIndex == index) {
        usage = "Pumpe " + String(i + 1) + " Pump Enable";
        return true;
      }
      if (pump.switchValveEnabled && pump.switchValveRelayIndex == index) {
        usage = "Pumpe " + String(i + 1) + " Umschaltventil";
        return true;
      }
    }

    const AuxHeaterConfig& aux = ctx.config.auxHeater;
    if (!ignoreAuxHeater && aux.enabled) {
      if (aux.pumpRelay == index) { usage = "Zusatzheizung Pumpe"; return true; }
      if (aux.heaterRelay1 == index) { usage = "Zusatzheizung Heizstab 1"; return true; }
      if (aux.heaterRelay2 == index) { usage = "Zusatzheizung Heizstab 2"; return true; }
      if (aux.heaterRelay3 == index) { usage = "Zusatzheizung Heizstab 3"; return true; }
    }

    const OvenConfig& oven = ctx.config.oven;
    if (!ignoreOven && oven.enabled && oven.pumpRelay == index) {
      usage = "Ofenpumpe";
      return true;
    }
  }

  if (kind == OutputKind::PWM_OUTPUT) {
    for (uint8_t i = 0; i < MAX_PUMPS; i++) {
      if ((int)i == ignorePumpIndex) continue;
      const PumpConfig& pump = ctx.config.pumps[i];
      if (!pump.enabled) continue;
      if (pump.mode == PumpMode::PWM && pump.pwmChannel == index) {
        usage = "Pumpe " + String(i + 1) + " PWM";
        return true;
      }
    }
  }

  return false;
}

bool validateSingleOutputFreeForUse(const AppContext& ctx, OutputKind kind, uint8_t index, const String& newUsage, int ignoreCircuitIndex = -1, int ignorePumpIndex = -1, bool ignoreAuxHeater = false, bool ignoreOven = false) {
  if (!outputRefAssigned(kind, index)) return true;

  String usage;
  if (outputIsUsedByLegacyConsumers(ctx, kind, index, ignorePumpIndex, usage, ignoreAuxHeater, ignoreOven) ||
      outputIsUsedByHeatingCircuit(ctx, kind, index, ignoreCircuitIndex, usage)) {
    server.send(409, "text/plain", outputRefLabel(kind, index) + " ist bereits belegt: " + usage + ". Neue Zuordnung: " + newUsage);
    return false;
  }

  return true;
}

bool validatePwmOutputModeChange(const AppContext& ctx, uint8_t index, const PwmOutputConfig& candidate) {
  if (index >= PWM_OUTPUT_COUNT) return false;

  String usage;
  if (candidate.mode == PwmOutputMode::PWM &&
      (outputIsUsedByHeatingCircuit(ctx, OutputKind::PWM_OUTPUT, index, -1, usage))) {
    server.send(409, "text/plain", pwmOutputHardwareLabel(index) + " ist bereits als Schaltausgang belegt: " + usage);
    return false;
  }

  if (candidate.mode == PwmOutputMode::SWITCH &&
      outputIsUsedByLegacyConsumers(ctx, OutputKind::PWM_OUTPUT, index, -1, usage)) {
    server.send(409, "text/plain", pwmOutputHardwareLabel(index) + " ist bereits als PWM-Ausgang belegt: " + usage);
    return false;
  }

  if (candidate.mode == PwmOutputMode::PWM && candidate.function != RelayFunction::NONE) {
    server.send(409, "text/plain", "PO im PWM-Modus darf keine Schaltfunktion haben");
    return false;
  }

  return true;
}

bool validateRelayFunctionChange(const AppContext& ctx, uint8_t index, RelayFunction newFunction) {
  String usage;
  if (outputIsUsedByLegacyConsumers(ctx, OutputKind::RELAY, index, -1, usage) ||
      outputIsUsedByHeatingCircuit(ctx, OutputKind::RELAY, index, -1, usage)) {
    const RelayOutputConfig& current = ctx.config.relays[index];
    if (newFunction != current.function) {
      server.send(409, "text/plain", relayHardwareLabel(index) + " ist bereits belegt: " + usage + ". Funktion kann nicht geändert werden.");
      return false;
    }
  }
  return true;
}

bool validateHeatingCircuitOutputConfig(const AppContext& ctx, const HeatingCircuitConfig& candidate, int circuitIndex) {
  if (!candidate.enabled) {
    return true;
  }

  if (sameOutputRef(candidate.mixerOpenOutput.kind, candidate.mixerOpenOutput.index,
                    candidate.mixerCloseOutput.kind, candidate.mixerCloseOutput.index)) {
    server.send(409, "text/plain", "Mischer AUF und Mischer ZU dürfen nicht derselbe Ausgang sein");
    return false;
  }

  if (sameOutputRef(candidate.mixerOpenOutput.kind, candidate.mixerOpenOutput.index,
                    candidate.pumpOutput.kind, candidate.pumpOutput.index)) {
    server.send(409, "text/plain", "Mischer AUF und Heizkreispumpe dürfen nicht derselbe Ausgang sein");
    return false;
  }

  if (sameOutputRef(candidate.mixerCloseOutput.kind, candidate.mixerCloseOutput.index,
                    candidate.pumpOutput.kind, candidate.pumpOutput.index)) {
    server.send(409, "text/plain", "Mischer ZU und Heizkreispumpe dürfen nicht derselbe Ausgang sein");
    return false;
  }

  if (!outputExistsAndMatchesFunction(ctx, candidate.mixerOpenOutput.kind, candidate.mixerOpenOutput.index, RelayFunction::MIXER, false)) {
    server.send(409, "text/plain", "Mischer AUF muss ein Ausgang mit Funktion Mischer sein");
    return false;
  }

  if (!outputExistsAndMatchesFunction(ctx, candidate.mixerCloseOutput.kind, candidate.mixerCloseOutput.index, RelayFunction::MIXER, false)) {
    server.send(409, "text/plain", "Mischer ZU muss ein Ausgang mit Funktion Mischer sein");
    return false;
  }

  if (candidate.pumpMode == HeatingCircuitPumpMode::SWITCHED &&
      !outputExistsAndMatchesFunction(ctx, candidate.pumpOutput.kind, candidate.pumpOutput.index, RelayFunction::PUMP_ENABLE, false)) {
    server.send(409, "text/plain", "Schaltpumpe muss ein Ausgang mit Funktion Pump Enable sein");
    return false;
  }

  if (candidate.pumpMode == HeatingCircuitPumpMode::PWM &&
      !(candidate.pumpOutput.kind == OutputKind::PWM_OUTPUT &&
        candidate.pumpOutput.index < PWM_OUTPUT_COUNT &&
        ctx.config.pwmOutputs[candidate.pumpOutput.index].enabled &&
        ctx.config.pwmOutputs[candidate.pumpOutput.index].mode == PwmOutputMode::PWM)) {
    server.send(409, "text/plain", "PWM-Pumpe muss einen PO-Ausgang im Modus PWM verwenden");
    return false;
  }

  if (!validateSingleOutputFreeForUse(ctx, candidate.mixerOpenOutput.kind, candidate.mixerOpenOutput.index, "HK" + String(circuitIndex + 1) + " Mischer AUF", circuitIndex)) return false;
  if (!validateSingleOutputFreeForUse(ctx, candidate.mixerCloseOutput.kind, candidate.mixerCloseOutput.index, "HK" + String(circuitIndex + 1) + " Mischer ZU", circuitIndex)) return false;
  if (!validateSingleOutputFreeForUse(ctx, candidate.pumpOutput.kind, candidate.pumpOutput.index, "HK" + String(circuitIndex + 1) + " Pumpe", circuitIndex)) return false;

  return true;
}



bool validateAuxHeaterOutputConfig(const AppContext& ctx, const AuxHeaterConfig& candidate) {
  if (!candidate.enabled) {
    return true;
  }

  if (candidate.pumpRelay != PIN_UNUSED) {
    if (!outputExistsAndMatchesFunction(ctx, OutputKind::RELAY, candidate.pumpRelay, RelayFunction::PUMP_ENABLE, false)) {
      server.send(409, "text/plain", "Zusatzheizung-Pumpe muss ein Ausgang mit Funktion Pump Enable sein");
      return false;
    }
    if (!validateSingleOutputFreeForUse(ctx, OutputKind::RELAY, candidate.pumpRelay,
                                        "Zusatzheizung Pumpe", -1, -1, true, false)) {
      return false;
    }
  }

  const uint8_t heaterRelays[3] = {
    candidate.heaterRelay1,
    candidate.heaterRelay2,
    candidate.heaterRelay3
  };

  for (uint8_t i = 0; i < 3; i++) {
    const uint8_t relay = heaterRelays[i];
    if (relay == PIN_UNUSED) continue;

    if (!outputExistsAndMatchesFunction(ctx, OutputKind::RELAY, relay, RelayFunction::HEATER_ROD, false)) {
      server.send(409, "text/plain", "Heizstab Stufe " + String(i + 1) + " muss ein Ausgang mit Funktion Heizstab sein");
      return false;
    }

    for (uint8_t j = i + 1; j < 3; j++) {
      if (relay == heaterRelays[j] && heaterRelays[j] != PIN_UNUSED) {
        server.send(409, "text/plain", "Heizstab-Ausgaenge duerfen nicht doppelt verwendet werden");
        return false;
      }
    }

    if (candidate.pumpRelay != PIN_UNUSED && relay == candidate.pumpRelay) {
      server.send(409, "text/plain", "Zusatzheizung-Pumpe und Heizstab duerfen nicht derselbe Ausgang sein");
      return false;
    }

    if (!validateSingleOutputFreeForUse(ctx, OutputKind::RELAY, relay,
                                        "Zusatzheizung Heizstab " + String(i + 1),
                                        -1, -1, true, false)) {
      return false;
    }
  }

  return true;
}

bool validateOvenOutputConfig(const AppContext& ctx, const OvenConfig& candidate) {
  if (!candidate.enabled) {
    return true;
  }

  if (candidate.pumpRelay == PIN_UNUSED) {
    return true;
  }

  if (!outputExistsAndMatchesFunction(ctx, OutputKind::RELAY, candidate.pumpRelay, RelayFunction::PUMP_ENABLE, false)) {
    server.send(409, "text/plain", "Ofenpumpenrelais muss als Pump Enable konfiguriert sein");
    return false;
  }

  if (!validateSingleOutputFreeForUse(ctx, OutputKind::RELAY, candidate.pumpRelay,
                                      "Ofenpumpe", -1, -1, false, true)) {
    return false;
  }

  return true;
}

  void handleRelayConfig() {
    if (!serviceSessionValid()) {
      server.send(403, "text/plain", "Nicht erlaubt");
      return;
    }

    if (!s_ctx) {
      server.send(500, "text/plain", "Kein Kontext");
      return;
    }

    if (!server.hasArg("index")) {
      server.send(400, "text/plain", "index fehlt");
      return;
    }

    const String type = server.hasArg("type") ? server.arg("type") : "relay";
    int idx = server.arg("index").toInt();

    if (type == "pwm") {
      if (idx < 0 || idx >= PWM_OUTPUT_COUNT) {
        server.send(400, "text/plain", "ungueltiger PO-Ausgang");
        return;
      }

      PwmOutputConfig candidate = s_ctx->config.pwmOutputs[idx];

      candidate.enabled = server.arg("enabled").toInt() != 0;
      if (server.hasArg("mode")) {
        candidate.mode = pwmOutputModeFromKey(server.arg("mode"));
      }
      if (server.hasArg("function")) {
        candidate.function = RelayOutputs::functionFromKey(server.arg("function"));
      }
      if (server.hasArg("profile")) {
        candidate.profile = (server.arg("profile") == "heating") ? PwmProfile::HEATING : PwmProfile::SOLAR;
      }

      if (!candidate.enabled) {
        candidate.function = RelayFunction::NONE;
      }

      // Auf der Seite "Ausgaenge" wird nur die Faehigkeit des Ausgangs definiert.
      // Harte Doppelbelegung wird erst beim Zuordnen in Verbraucher-Menues geprueft.
      if (candidate.mode == PwmOutputMode::PWM) {
        candidate.function = RelayFunction::NONE;
      }

      s_ctx->config.pwmOutputs[idx] = candidate;
      Storage::saveConfig(s_ctx->config);
      server.send(200, "text/plain", "OK");
      return;
    }

    if (idx < 0 || idx >= RELAY_COUNT) {
      server.send(400, "text/plain", "ungueltiger index");
      return;
    }

    bool enabled = server.arg("enabled").toInt() != 0;
    bool activeLow = true;
    if (server.hasArg("activeLow")) {
      activeLow = server.arg("activeLow").toInt() != 0;
    }

    RelayFunction fn = RelayOutputs::functionFromKey(server.arg("function"));
    if (fn == RelayFunction::NONE) {
      enabled = false;
    }

    // Auf der Seite "Ausgaenge" wird nur die Faehigkeit des Ausgangs definiert.
    // Harte Doppelbelegung wird erst beim Zuordnen in Verbraucher-Menues geprueft.
    RelayOutputs::configure(*s_ctx, (uint8_t)idx, enabled, fn, activeLow);
    Storage::saveConfig(s_ctx->config);

    server.send(200, "text/plain", "OK");
  }

  void handleRelayTest() {
    if (!serviceSessionValid()) {
      server.send(403, "text/plain", "Nicht erlaubt");
      return;
    }

    if (!s_ctx) {
      server.send(500, "text/plain", "Kein Kontext");
      return;
    }

    if (!server.hasArg("index") || !server.hasArg("state")) {
      server.send(400, "text/plain", "index/state fehlt");
      return;
    }

    int idx = server.arg("index").toInt();
    if (idx < 0 || idx >= RELAY_COUNT) {
      server.send(400, "text/plain", "ungueltiger index");
      return;
    }

    bool on = server.arg("state").toInt() != 0;
    if (!RelayOutputs::set(*s_ctx, (uint8_t)idx, on)) {
      server.send(400, "text/plain", "Relais nicht schaltbar");
      return;
    }

    server.send(200, "text/plain", "OK");
  }


  String jsonFloat(float v, uint8_t decimals) {
  if (isnan(v)) return "null";
  return String(v, (unsigned int)decimals);
}

  String feedbackInputLabel(uint8_t ordinal, uint8_t gpio) {
    return String("FB") + String(ordinal) + String(" (GPIO ") + String(gpio) + String(")");
  }

  String pumpModeToKey(PumpMode mode) {
    switch (mode) {
      case PumpMode::RELAY: return "relay";
      case PumpMode::PWM: return "pwm";
      case PumpMode::OFF:
      default: return "off";
    }
  }

  PumpMode pumpModeFromKey(const String& key) {
    if (key == "relay") return PumpMode::RELAY;
    if (key == "pwm") return PumpMode::PWM;
    return PumpMode::OFF;
  }

  String pumpModeLabel(PumpMode mode) {
    switch (mode) {
      case PumpMode::RELAY: return "Umwaelzpumpe";
      case PumpMode::PWM: return "PWM-Pumpe";
      case PumpMode::OFF:
      default: return "Aus";
    }
  }

  int pumpRelayUsedBy(const AppContext& ctx, uint8_t relayIndex, int ignorePump) {
    if (relayIndex == PIN_UNUSED) return -1;
    for (uint8_t i = 0; i < MAX_PUMPS; i++) {
      if ((int)i == ignorePump) continue;
      if (ctx.config.pumps[i].relayIndex == relayIndex) return i;
    }
    return -1;
  }

  int valveRelayUsedBy(const AppContext& ctx, uint8_t relayIndex, int ignorePump) {
    if (relayIndex == PIN_UNUSED) return -1;
    for (uint8_t i = 0; i < MAX_PUMPS; i++) {
      if ((int)i == ignorePump) continue;
      if (ctx.config.pumps[i].switchValveRelayIndex == relayIndex) return i;
    }
    return -1;
  }

  int pwmChannelUsedBy(const AppContext& ctx, uint8_t pwmChannel, int ignorePump) {
    if (pwmChannel == PIN_UNUSED) return -1;
    for (uint8_t i = 0; i < MAX_PUMPS; i++) {
      if ((int)i == ignorePump) continue;
      const PumpConfig& p = ctx.config.pumps[i];
      if (p.mode == PumpMode::PWM && p.pwmChannel == pwmChannel) return i;
    }
    return -1;
  }

  
  int pwmSwitchUsedByOutput(const AppContext& ctx, uint8_t channel, int ignoreOutput) {
    if (channel == PIN_UNUSED) return -1;
    for (uint8_t i = 0; i < PWM_OUTPUT_COUNT; i++) {
      if ((int)i == ignoreOutput) continue;
      const PwmOutputConfig& out = ctx.config.pwmOutputs[i];
      if (out.enabled && out.mode == PwmOutputMode::SWITCH && i == channel) return i;
    }
    return -1;
  }

  bool pwmOutputAvailableForPump(const AppContext& ctx, uint8_t channel) {
    if (channel == PIN_UNUSED || channel >= PWM_OUTPUT_COUNT) return false;
    const PwmOutputConfig& out = ctx.config.pwmOutputs[channel];
    return out.enabled && out.mode == PwmOutputMode::PWM;
  }

int feedbackPinUsedBy(const AppContext& ctx, uint8_t feedbackPin, int ignorePump) {
    if (feedbackPin == PIN_UNUSED) return -1;
    for (uint8_t i = 0; i < MAX_PUMPS; i++) {
      if ((int)i == ignorePump) continue;
      const PumpConfig& p = ctx.config.pumps[i];
      if (p.mode == PumpMode::PWM && p.feedbackPin == feedbackPin) return i;
    }
    return -1;
  }

  void appendActiveDs18RoleOptions(String& json, const AppContext& ctx, bool& first) {
    for (uint8_t i = 0; i < ctx.assignments.count; i++) {
      const Ds18RoleAssignment& assignment = ctx.assignments.items[i];
      if (!assignment.assigned || assignment.role == Ds18Role::NONE) continue;

      if (!first) json += ",";
      first = false;

      json += "{\"key\":\"" + String(SensorRoles::toKey(assignment.role)) + "\",";
      json += "\"value\":" + String((int)assignment.role) + ",";
      json += "\"label\":\"" + String(SensorRoles::toLabel(assignment.role)) + "\"}";
    }
  }

  bool validatePumpRelayMapping(const AppContext& ctx, uint8_t pumpIndex, const PumpConfig& candidate, String& error) {
    const bool pumpActive = candidate.enabled &&
                            (candidate.mode == PumpMode::RELAY || candidate.mode == PumpMode::PWM);

    if (!pumpActive) {
      return true;
    }

    if (candidate.mode == PumpMode::RELAY) {
      if (candidate.relayIndex == PIN_UNUSED || candidate.relayIndex >= RELAY_COUNT) {
        error = "Pumpenrelais ungueltig oder nicht gesetzt";
        return false;
      }

      if (!outputExistsAndMatchesFunction(ctx, OutputKind::RELAY, candidate.relayIndex, RelayFunction::PUMP_ENABLE, false)) {
        error = "Pumpenrelais ist nicht als Pump Enable konfiguriert";
        return false;
      }

      if (!validateSingleOutputFreeForUse(ctx, OutputKind::RELAY, candidate.relayIndex,
                                          "Pumpe " + String(pumpIndex + 1) + " Pump Enable",
                                          -1, pumpIndex)) {
        return false;
      }
    }

    if (candidate.mode == PumpMode::PWM) {
      if (candidate.pwmChannel == PIN_UNUSED || candidate.pwmChannel >= PWM_OUTPUT_COUNT) {
        error = "PWM-Kanal ungueltig oder nicht gesetzt";
        return false;
      }

      if (!outputExistsAndMatchesFunction(ctx, OutputKind::PWM_OUTPUT, candidate.pwmChannel, RelayFunction::NONE, true)) {
        error = "PO-Ausgang ist nicht als PWM-Ausgang konfiguriert";
        return false;
      }

      if (!validateSingleOutputFreeForUse(ctx, OutputKind::PWM_OUTPUT, candidate.pwmChannel,
                                          "Pumpe " + String(pumpIndex + 1) + " PWM",
                                          -1, pumpIndex)) {
        return false;
      }

      if (candidate.feedbackPin != PIN_UNUSED) {
        const int feedbackUsedBy = feedbackPinUsedBy(ctx, candidate.feedbackPin, pumpIndex);
        if (feedbackUsedBy >= 0) {
          error = "Feedback-Eingang bereits von Pumpe " + String(feedbackUsedBy + 1) + " verwendet";
          return false;
        }
      }
    }

    if (candidate.switchValveEnabled && candidate.switchValveRelayIndex != PIN_UNUSED) {
      if (candidate.switchValveRelayIndex >= RELAY_COUNT) {
        error = "Ventilrelais ungueltig";
        return false;
      }

      if (!outputExistsAndMatchesFunction(ctx, OutputKind::RELAY, candidate.switchValveRelayIndex, RelayFunction::ZONE_VALVE, false)) {
        error = "Ventilrelais ist nicht als Zonenventil konfiguriert";
        return false;
      }

      if (candidate.mode == PumpMode::RELAY &&
          candidate.switchValveRelayIndex == candidate.relayIndex) {
        error = "Pumpenrelais und Ventilrelais duerfen nicht identisch sein";
        return false;
      }

      if (!validateSingleOutputFreeForUse(ctx, OutputKind::RELAY, candidate.switchValveRelayIndex,
                                          "Pumpe " + String(pumpIndex + 1) + " Umschaltventil",
                                          -1, pumpIndex)) {
        return false;
      }
    }

    return true;
  }

  void handlePumpsJson() {
    if (!s_ctx) {
      server.send(500, "application/json", "{\"error\":\"no_context\"}");
      return;
    }

    String json = "{\"pumps\":[";

    for (uint8_t i = 0; i < MAX_PUMPS; i++) {
      if (i > 0) json += ",";
      const PumpConfig& p = s_ctx->config.pumps[i];

      json += "{";
      json += "\"index\":" + String(i) + ",";
      json += "\"name\":\"Pumpe " + String(i + 1) + "\",";
      json += "\"enabled\":" + String(p.enabled ? "true" : "false") + ",";
      json += "\"mode\":\"" + pumpModeToKey(p.mode) + "\",";
      json += "\"modeLabel\":\"" + pumpModeLabel(p.mode) + "\",";
      json += "\"sourceType\":\"" + String(p.sourceType == PumpSourceType::SENSOR_ROLE ? "sensor" : "heat_source") + "\",";
      json += "\"sourceRole\":\"" + String(HeatSourceRoles::toKey(p.sourceRole)) + "\",";
      json += "\"sourceSensorRole\":\"" + String(SensorRoles::toKey(p.sourceSensorRole)) + "\",";
      json += "\"sourceLabel\":\"" + String(p.sourceType == PumpSourceType::SENSOR_ROLE ? SensorRoles::toLabel(p.sourceSensorRole) : HeatSourceRoles::toLabel(p.sourceRole)) + "\",";
      json += "\"sinkRole\":\"" + String(SensorRoles::toKey(p.sinkRole)) + "\",";
      json += "\"sinkLabel\":\"" + String(p.sinkRole == Ds18Role::NONE ? "Priorisierter Sink" : SensorRoles::toLabel(p.sinkRole)) + "\",";
      json += "\"relayIndex\":" + String(p.relayIndex) + ",";
      json += "\"pwmChannel\":" + String(p.pwmChannel) + ",";
      json += "\"feedbackPin\":" + String(p.feedbackPin) + ",";
      json += "\"targetDiff\":" + String(p.targetDiff, 2) + ",";
      json += "\"hysteresis\":" + String(p.hysteresis, 2) + ",";
      json += "\"startDiff\":" + String(p.startDiff, 2) + ",";
      json += "\"pidKp\":" + String(p.pidKp, 4) + ",";
      json += "\"pidKi\":" + String(p.pidKi, 4) + ",";
      json += "\"pidKd\":" + String(p.pidKd, 4) + ",";
      json += "\"minPwmPercent\":" + String(p.minPwmPercent, 2) + ",";
      json += "\"maxPwmPercent\":" + String(p.maxPwmPercent, 2) + ",";
      json += "\"switchValveEnabled\":" + String(p.switchValveEnabled ? "true" : "false") + ",";
      json += "\"switchValveRelayIndex\":" + String(p.switchValveRelayIndex) + ",";
      json += "\"switchValveTravelTimeMs\":" + String(p.switchValveTravelTimeMs) + ",";
      json += "\"switchValveMoving\":" + String(p.switchValveMoving ? "true" : "false") + ",";
      json += "\"switchValveStateForTargetA\":" + String(p.switchValveStateForTargetA ? "true" : "false") + ",";
      json += "\"activeTargetIndex\":" + String(p.activeTargetIndex) + ",";
      json += "\"targets\":[";
      for (uint8_t t = 0; t < PUMP_ROUTE_TARGET_COUNT; t++) {
        if (t > 0) json += ",";
        const PumpRouteTargetConfig& target = p.targets[t];
        json += "{";
        json += "\"index\":" + String(t) + ",";
        json += "\"name\":\"Ziel " + String(t == 0 ? "A" : "B") + "\",";
        json += "\"enabled\":" + String(target.enabled ? "true" : "false") + ",";
        json += "\"sinkRole\":\"" + String(SensorRoles::toKey(target.sinkRole)) + "\",";
        json += "\"sinkLabel\":\"" + String(SensorRoles::toLabel(target.sinkRole)) + "\",";
        json += "\"targetDiffOverride\":" + String(target.targetDiffOverride, 2) + ",";
        json += "\"hysteresisOverride\":" + String(target.hysteresisOverride, 2) + ",";
        json += "\"minTempC\":" + String(target.minTempC, 2) + ",";
        json += "\"maxTempC\":" + String(target.maxTempC, 2) + ",";
        json += "\"active\":" + String(target.active ? "true" : "false") + ",";
        json += "\"lastSinkC\":" + jsonFloat(target.lastSinkC, 2) + ",";
        json += "\"lastDiffC\":" + jsonFloat(target.lastDiffC, 2);
        json += "}";
      }
      json += "],";
      json += "\"state\":" + String(p.state ? "true" : "false") + ",";
      json += "\"lastSourceC\":" + jsonFloat(p.lastSourceC, 2) + ",";
      json += "\"lastSinkC\":" + jsonFloat(p.lastSinkC, 2) + ",";
      json += "\"lastDiffC\":" + jsonFloat(p.lastDiffC, 2) + ",";
      json += "\"lastPwmPercent\":" + String(p.lastPwmPercent, 1) + ",";
      json += "\"feedbackSignalPresent\":" + String(p.feedbackSignalPresent ? "true" : "false") + ",";
      json += "\"feedbackError\":" + String(p.feedbackError ? "true" : "false") + ",";
      json += "\"feedbackDutyPercent\":" + jsonFloat(p.feedbackDutyPercent, 1);
      json += "}";
    }

    json += "],\"pumpRelays\":[";
    bool first = true;
    for (uint8_t i = 0; i < RELAY_COUNT; i++) {
      const RelayOutputConfig& r = s_ctx->config.relays[i];
      if (!r.enabled || r.function != RelayFunction::PUMP_ENABLE) continue;
      if (!first) json += ",";
      first = false;
      json += "{\"index\":" + String(i) + ",\"label\":\"" + relayHardwareLabel(i) + "\",\"usedBy\":" + String(pumpRelayUsedBy(*s_ctx, i, -1)) + "}";
    }

    json += "],\"zoneValveRelays\":[";
    first = true;
    for (uint8_t i = 0; i < RELAY_COUNT; i++) {
      const RelayOutputConfig& r = s_ctx->config.relays[i];
      if (!r.enabled || r.function != RelayFunction::ZONE_VALVE) continue;
      if (!first) json += ",";
      first = false;
      json += "{\"index\":" + String(i) + ",\"label\":\"" + relayHardwareLabel(i) + "\",\"usedBy\":" + String(valveRelayUsedBy(*s_ctx, i, -1)) + "}";
    }

    json += "],\"pcaChannels\":[";
    first = true;
    for (uint8_t ch = 0; ch < PWM_OUTPUT_COUNT; ch++) {
      const PwmOutputConfig& out = s_ctx->config.pwmOutputs[ch];
      if (!out.enabled || out.mode != PwmOutputMode::PWM) continue;
      if (!first) json += ",";
      first = false;
      json += "{\"index\":" + String(ch) + ",\"label\":\"" + pwmOutputHardwareLabel(ch) + "\",\"usedBy\":" + String(pwmChannelUsedBy(*s_ctx, ch, -1)) + ",\"profile\":\"" + pwmProfileToKey(out.profile) + "\"}";
    }

    json += "],\"feedbackPins\":[";
    first = true;
    const uint8_t feedbackPins[] = {PIN_UNUSED, 35, 36, 39, 32, 33, 34};
    for (uint8_t fp = 0; fp < sizeof(feedbackPins) / sizeof(feedbackPins[0]); fp++) {
      const uint8_t pin = feedbackPins[fp];
      if (!first) json += ",";
      first = false;
      String label = (pin == PIN_UNUSED) ? String("kein Feedback") : feedbackInputLabel(fp - 1, pin);
      json += "{\"index\":" + String(pin) + ",\"label\":\"" + label + "\",\"usedBy\":" + String(feedbackPinUsedBy(*s_ctx, pin, -1)) + "}";
    }

    json += "],\"heatSources\":[";
    bool firstHs = true;
    for (int i = 0; i <= (int)HeatSourceRole::ALT_SOURCE_OTHER; i++) {
      HeatSourceRole role = (HeatSourceRole)i;
      if (!firstHs) json += ",";
      firstHs = false;
      json += "{\"key\":\"" + String(HeatSourceRoles::toKey(role)) + "\",\"value\":" + String(i) + ",\"label\":\"" + String(HeatSourceRoles::toLabel(role)) + "\"}";
    }

    json += "],\"sensorSources\":[";
    bool firstSensorSource = true;
    appendActiveDs18RoleOptions(json, *s_ctx, firstSensorSource);

    json += "],\"sinkRoles\":[";
    bool firstSink = true;
    appendActiveDs18RoleOptions(json, *s_ctx, firstSink);

    json += "]}";
    server.send(200, "application/json", json);
  }

  void handlePumpConfig() {
    if (!serviceSessionValid()) {
      server.send(403, "text/plain", "Nicht erlaubt");
      return;
    }

    if (!s_ctx) {
      server.send(500, "text/plain", "Kein Kontext");
      return;
    }

    if (!server.hasArg("index")) {
      server.send(400, "text/plain", "index fehlt");
      return;
    }

    int idx = server.arg("index").toInt();
    if (idx < 0 || idx >= MAX_PUMPS) {
      server.send(400, "text/plain", "ungueltiger index");
      return;
    }

    PumpConfig& p = s_ctx->config.pumps[idx];
    PumpConfig old = p;

    if (server.hasArg("mode")) p.mode = pumpModeFromKey(server.arg("mode"));
    p.enabled = (p.mode == PumpMode::RELAY || p.mode == PumpMode::PWM);

    if (server.hasArg("sourceSelect")) {
      String sourceSelect = server.arg("sourceSelect");
      if (sourceSelect.startsWith("heat:")) {
        p.sourceType = PumpSourceType::HEAT_SOURCE_ROLE;
        p.sourceRole = HeatSourceRoles::fromKey(sourceSelect.substring(5));
        p.sourceSensorRole = Ds18Role::NONE;
      } else if (sourceSelect.startsWith("sensor:")) {
        p.sourceType = PumpSourceType::SENSOR_ROLE;
        p.sourceSensorRole = SensorRoles::fromKey(sourceSelect.substring(7));
        p.sourceRole = HeatSourceRole::NONE;
      }
    } else {
      if (server.hasArg("sourceType")) p.sourceType = (server.arg("sourceType") == "sensor") ? PumpSourceType::SENSOR_ROLE : PumpSourceType::HEAT_SOURCE_ROLE;
      if (server.hasArg("sourceRole")) p.sourceRole = HeatSourceRoles::fromKey(server.arg("sourceRole"));
      if (server.hasArg("sourceSensorRole")) p.sourceSensorRole = SensorRoles::fromKey(server.arg("sourceSensorRole"));
    }

    if (server.hasArg("sinkRole")) p.sinkRole = SensorRoles::fromKey(server.arg("sinkRole"));

    if (server.hasArg("targetSelect")) {
      String targetSelect = server.arg("targetSelect");
      if (targetSelect.startsWith("sink:")) {
        p.switchValveEnabled = false;
        p.switchValveRelayIndex = PIN_UNUSED;
        p.sinkRole = SensorRoles::fromKey(targetSelect.substring(5));
      } else if (targetSelect.startsWith("valve:")) {
        p.switchValveEnabled = true;
        p.sinkRole = Ds18Role::NONE;
        p.switchValveRelayIndex = (uint8_t)targetSelect.substring(6).toInt();
      }
    }

    if (server.hasArg("relayIndex")) p.relayIndex = (uint8_t)server.arg("relayIndex").toInt();
    if (server.hasArg("pwmChannel")) p.pwmChannel = (uint8_t)server.arg("pwmChannel").toInt();
    if (server.hasArg("pwmProfile")) p.pwmProfile = (server.arg("pwmProfile") == "heating") ? PwmProfile::HEATING : PwmProfile::SOLAR;
    if (server.hasArg("feedbackPin")) p.feedbackPin = (uint8_t)server.arg("feedbackPin").toInt();

    if (server.hasArg("targetDiff")) p.targetDiff = server.arg("targetDiff").toFloat();
    if (server.hasArg("hysteresis")) p.hysteresis = server.arg("hysteresis").toFloat();
    if (server.hasArg("startDiff")) p.startDiff = server.arg("startDiff").toFloat();
    if (server.hasArg("pidKp")) p.pidKp = server.arg("pidKp").toFloat();
    if (server.hasArg("pidKi")) p.pidKi = server.arg("pidKi").toFloat();
    if (server.hasArg("pidKd")) p.pidKd = server.arg("pidKd").toFloat();
    if (server.hasArg("minPwmPercent")) p.minPwmPercent = server.arg("minPwmPercent").toFloat();
    if (server.hasArg("maxPwmPercent")) p.maxPwmPercent = server.arg("maxPwmPercent").toFloat();

    if (server.hasArg("switchValveEnabled")) p.switchValveEnabled = (server.arg("switchValveEnabled").toInt() != 0);
    if (server.hasArg("switchValveRelayIndex")) p.switchValveRelayIndex = (uint8_t)server.arg("switchValveRelayIndex").toInt();
    if (server.hasArg("switchValveTravelTimeMs")) p.switchValveTravelTimeMs = (uint32_t)server.arg("switchValveTravelTimeMs").toInt();
    if (server.hasArg("switchValveStateForTargetA")) p.switchValveStateForTargetA = (server.arg("switchValveStateForTargetA").toInt() != 0);

    for (uint8_t t = 0; t < PUMP_ROUTE_TARGET_COUNT; t++) {
      const String prefix = "target" + String(t) + "_";
      if (server.hasArg(prefix + "enabled")) p.targets[t].enabled = (server.arg(prefix + "enabled").toInt() != 0);
      if (server.hasArg(prefix + "sinkRole")) p.targets[t].sinkRole = SensorRoles::fromKey(server.arg(prefix + "sinkRole"));
      if (server.hasArg(prefix + "targetDiffOverride")) p.targets[t].targetDiffOverride = server.arg(prefix + "targetDiffOverride").toFloat();
      if (server.hasArg(prefix + "hysteresisOverride")) p.targets[t].hysteresisOverride = server.arg(prefix + "hysteresisOverride").toFloat();
      if (server.hasArg(prefix + "minTempC")) p.targets[t].minTempC = server.arg(prefix + "minTempC").toFloat();
      if (server.hasArg(prefix + "maxTempC")) p.targets[t].maxTempC = server.arg(prefix + "maxTempC").toFloat();
    }

String validationError;

if (!validatePumpRelayMapping(*s_ctx, (uint8_t)idx, p, validationError)) {
  p = old;
  server.send(409, "text/plain", validationError);
  return;
}

if (!OutputValidation::validateAll(*s_ctx, validationError)) {
  p = old;
  server.send(409, "text/plain", validationError);
  return;
}

Storage::saveConfig(s_ctx->config);
server.send(200, "text/plain", "OK");

  }



  void handleOvenJson() {
    if (!s_ctx) {
      server.send(500, "application/json", "{\"error\":\"no_context\"}");
      return;
    }

    const OvenConfig& cfg = s_ctx->config.oven;

    String json = "{";

    json += "\"status\":{";
    json += "\"ovenTemperatureC\":";
    json += jsonFloat(OvenControl::ovenTemperatureC(), 1);
    json += ",\"servoAngle\":";
    json += String(OvenControl::servoAngle());
    json += ",\"pumpActive\":";
    json += OvenControl::pumpActive() ? "true" : "false";
    json += ",\"state\":\"";
    json += OvenControl::stateText();
    json += "\"";
    json += ",\"userRequestedActive\":";
    json += OvenControl::userRequestedActive() ? "true" : "false";
    json += ",\"autoStarted\":";
    json += OvenControl::autoStarted() ? "true" : "false";
    json += ",\"peakTemperatureC\":";
    json += jsonFloat(OvenControl::peakTemperatureC(), 1);
    json += "}";

    json += ",\"config\":{";
    json += "\"enabled\":";
    json += cfg.enabled ? "true" : "false";
    json += ",\"pumpRelay\":";
    json += String(cfg.pumpRelay);
    json += ",\"targetSinkRole\":";
    json += String((int)cfg.targetSinkRole);
    json += ",\"autoStartEnabled\":";
    json += cfg.autoStartEnabled ? "true" : "false";
    json += ",\"autoStartTemperatureC\":";
    json += jsonFloat(cfg.autoStartTemperatureC, 1);
    json += ",\"autoStartRiseC\":";
    json += jsonFloat(cfg.autoStartRiseC, 1);
    json += ",\"autoReturnToStandbyTemperatureC\":";
    json += jsonFloat(cfg.autoReturnToStandbyTemperatureC, 1);
    json += ",\"targetOvenTemperatureC\":";
    json += jsonFloat(cfg.targetOvenTemperatureC, 1);
    json += ",\"criticalOvenTemperatureC\":";
    json += jsonFloat(cfg.criticalOvenTemperatureC, 1);
    json += ",\"pumpOnTemperatureC\":";
    json += jsonFloat(cfg.pumpOnTemperatureC, 1);
    json += ",\"pumpHysteresisC\":";
    json += jsonFloat(cfg.pumpHysteresisC, 1);
    json += ",\"pumpOnTemperatureDifferenceC\":";
    json += jsonFloat(cfg.pumpOnTemperatureDifferenceC, 1);
    json += ",\"pumpOffTemperatureDifferenceC\":";
    json += jsonFloat(cfg.pumpOffTemperatureDifferenceC, 1);
    json += ",\"pumpStopDropFromPeakC\":";
    json += jsonFloat(cfg.pumpStopDropFromPeakC, 1);
    json += ",\"servoMinimumAngle\":";
    json += String(cfg.servoMinimumAngle);
    json += ",\"servoMaximumAngle\":";
    json += String(cfg.servoMaximumAngle);
    json += ",\"servoBaseAngle\":";
    json += String(cfg.servoBaseAngle);
    json += ",\"pidKp\":";
    json += jsonFloat(cfg.pidKp, 3);
    json += ",\"pidKi\":";
    json += jsonFloat(cfg.pidKi, 3);
    json += ",\"pidKd\":";
    json += jsonFloat(cfg.pidKd, 3);
    json += "}";

    json += ",\"sinkRoles\":[";
    bool firstSink = true;
    appendActiveDs18RoleOptions(json, *s_ctx, firstSink);
    json += "]";

    json += ",\"pumpRelays\":[";
    bool firstPumpRelay = true;
    for (uint8_t i = 0; i < RELAY_COUNT; i++) {
      const RelayOutputConfig& relay = s_ctx->config.relays[i];
      if (!relay.enabled) continue;
      if (relay.function != RelayFunction::PUMP_ENABLE) continue;

      if (!firstPumpRelay) json += ",";
      firstPumpRelay = false;

      json += "{\"value\":";
      json += String(i);
      json += ",\"label\":\"";
      json += relayHardwareLabel(i);
      json += " - Pump Enable\"}";
    }
    json += "]";

    json += "}";

    server.send(200, "application/json", json);
  }

  void handleOvenSave() {
    if (!serviceSessionValid()) {
      server.send(403, "text/plain", "Nicht erlaubt");
      return;
    }

    if (!s_ctx) {
      server.send(500, "text/plain", "Kein Kontext");
      return;
    }

    OvenConfig candidate = s_ctx->config.oven;
    OvenConfig& cfg = candidate;

    if (server.hasArg("enabled"))
      cfg.enabled = (server.arg("enabled").toInt() != 0);

    if (server.hasArg("pumpRelay"))
      cfg.pumpRelay = (uint8_t)server.arg("pumpRelay").toInt();

    if (server.hasArg("targetSinkRole"))
      cfg.targetSinkRole = (Ds18Role)server.arg("targetSinkRole").toInt();

    if (server.hasArg("autoStartEnabled"))
      cfg.autoStartEnabled = (server.arg("autoStartEnabled").toInt() != 0);

    if (server.hasArg("autoStartTemperatureC"))
      cfg.autoStartTemperatureC = server.arg("autoStartTemperatureC").toFloat();

    if (server.hasArg("autoStartRiseC"))
      cfg.autoStartRiseC = server.arg("autoStartRiseC").toFloat();

    if (server.hasArg("autoReturnToStandbyTemperatureC"))
      cfg.autoReturnToStandbyTemperatureC = server.arg("autoReturnToStandbyTemperatureC").toFloat();

    if (server.hasArg("targetOvenTemperatureC"))
      cfg.targetOvenTemperatureC = server.arg("targetOvenTemperatureC").toFloat();

    if (server.hasArg("criticalOvenTemperatureC"))
      cfg.criticalOvenTemperatureC = server.arg("criticalOvenTemperatureC").toFloat();

    if (server.hasArg("pumpOnTemperatureC"))
      cfg.pumpOnTemperatureC = server.arg("pumpOnTemperatureC").toFloat();

    if (server.hasArg("pumpHysteresisC"))
      cfg.pumpHysteresisC = server.arg("pumpHysteresisC").toFloat();

    if (server.hasArg("pumpOnTemperatureDifferenceC"))
      cfg.pumpOnTemperatureDifferenceC = server.arg("pumpOnTemperatureDifferenceC").toFloat();

    if (server.hasArg("pumpOffTemperatureDifferenceC"))
      cfg.pumpOffTemperatureDifferenceC = server.arg("pumpOffTemperatureDifferenceC").toFloat();

    if (server.hasArg("pumpStopDropFromPeakC"))
      cfg.pumpStopDropFromPeakC = server.arg("pumpStopDropFromPeakC").toFloat();

    if (server.hasArg("servoMinimumAngle"))
      cfg.servoMinimumAngle = (uint8_t)server.arg("servoMinimumAngle").toInt();

    if (server.hasArg("servoMaximumAngle"))
      cfg.servoMaximumAngle = (uint8_t)server.arg("servoMaximumAngle").toInt();

    if (server.hasArg("servoBaseAngle"))
      cfg.servoBaseAngle = (uint8_t)server.arg("servoBaseAngle").toInt();

    if (server.hasArg("pidKp"))
      cfg.pidKp = server.arg("pidKp").toFloat();

    if (server.hasArg("pidKi"))
      cfg.pidKi = server.arg("pidKi").toFloat();

    if (server.hasArg("pidKd"))
      cfg.pidKd = server.arg("pidKd").toFloat();

if (!validateOvenOutputConfig(*s_ctx, candidate)) {
  return;
}

OvenConfig old = s_ctx->config.oven;

s_ctx->config.oven = candidate;

String validationError;
if (!OutputValidation::validateAll(*s_ctx, validationError)) {
  s_ctx->config.oven = old;
  server.send(409, "text/plain", validationError);
  return;
}

Storage::saveConfig(s_ctx->config);
server.send(200, "text/plain", "OK");
  }

  void handleOvenStart() {
    if (!s_ctx) {
      server.send(500, "text/plain", "Kein Kontext");
      return;
    }

    if (!s_ctx->config.oven.enabled) {
      server.send(409, "text/plain", "Ofensteuerung ist nicht aktiviert");
      return;
    }

    if (!s_ctx->sensors.heatSources.altSourceOvenValid) {
      server.send(409, "text/plain", "Keine gueltige Ofen-Waermequelle definiert");
      return;
    }

    if (SafetyManager::status().ovenOvertemperatureActive) {
      server.send(409, "text/plain", "Ofen-Safety aktiv");
      return;
    }

    OvenControl::requestStart();
    server.send(200, "text/plain", "OK");
  }

  void handleOvenStop() {
    if (!s_ctx) {
      server.send(500, "text/plain", "Kein Kontext");
      return;
    }

    OvenControl::requestStop(*s_ctx);
    server.send(200, "text/plain", "OK");
  }

  void handleTestOverviewJson() {
    if (!s_ctx) {
      server.send(500, "application/json", "{\"error\":\"no_context\"}");
      return;
    }

    String json = "{";
    json += "\"relays\":[";
    for (uint8_t i = 0; i < RELAY_COUNT; i++) {
      if (i > 0) json += ",";
      const RelayOutputConfig& cfg = s_ctx->config.relays[i];
      json += "{";
      json += "\"index\":" + String(i) + ",";
      json += "\"label\":\"" + relayHardwareLabel(i) + "\",";
      json += "\"function\":\"" + String(RelayOutputs::functionToKey(cfg.function)) + "\",";
      json += "\"activeLow\":" + String(cfg.activeLow ? "true" : "false") + ",";
      json += "\"state\":" + String(RelayOutputs::get(*s_ctx, i) ? "true" : "false");
      json += "}";
    }
    json += "],\"pcaChannels\":[";
    for (uint8_t ch = 0; ch < PWM_OUTPUT_COUNT; ch++) {
      if (ch > 0) json += ",";
      const PwmOutputConfig& out = s_ctx->config.pwmOutputs[ch];
      json += "{\"index\":" + String(ch) + ",\"label\":\"" + pwmOutputHardwareLabel(ch) + "\",\"mode\":\"" + pwmOutputModeToKey(out.mode) + "\",\"function\":\"" + String(RelayOutputs::functionToKey(out.function)) + "\",\"percent\":" + String(PwmDriver::getDuty(ch)) + "}";
    }
    json += "],\"pumps\":[";
    for (uint8_t i = 0; i < MAX_PUMPS; i++) {
      if (i > 0) json += ",";
      const PumpConfig& pump = s_ctx->config.pumps[i];
      json += "{";
      json += "\"index\":" + String(i) + ",";
      json += "\"label\":\"Pumpe " + String(i + 1) + "\",";
      json += "\"enabled\":" + String(pump.enabled ? "true" : "false") + ",";
      json += "\"mode\":\"" + pumpModeToKey(pump.mode) + "\",";
      json += "\"relayIndex\":" + String(pump.relayIndex) + ",";
      json += "\"pwmChannel\":" + String(pump.pwmChannel) + ",";
      json += "\"feedbackPin\":" + String(pump.feedbackPin) + ",";
      json += "\"feedbackSignalPresent\":" + String(pump.feedbackSignalPresent ? "true" : "false") + ",";
      json += "\"feedbackError\":" + String(pump.feedbackError ? "true" : "false") + ",";
      json += "\"feedbackDutyPercent\":" + jsonFloat(pump.feedbackDutyPercent, 1);
      json += "}";
    }
    json += "]}";
    server.send(200, "application/json", json);
  }

  void handleTestRelayRaw() {
    if (!commissioningTestAuthorized()) {
      server.send(403, "text/plain", "Testmodus nicht aktiv oder Inbetriebnahme-PIN falsch");
      return;
    }
    if (!server.hasArg("index") || !server.hasArg("state")) {
      server.send(400, "text/plain", "index/state fehlt");
      return;
    }
    int idx = server.arg("index").toInt();
    if (idx < 0 || idx >= RELAY_COUNT) {
      server.send(400, "text/plain", "ungueltiger index");
      return;
    }
    bool on = server.arg("state").toInt() != 0;
    if (!RelayOutputs::testSetRaw(*s_ctx, (uint8_t)idx, on)) {
      server.send(400, "text/plain", "Relais-Test fehlgeschlagen");
      return;
    }
    server.send(200, "text/plain", "OK");
  }

  void handleTestPcaPwm() {
    if (!commissioningTestAuthorized()) {
      server.send(403, "text/plain", "Testmodus nicht aktiv oder Inbetriebnahme-PIN falsch");
      return;
    }
    if (!server.hasArg("channel") || !server.hasArg("percent")) {
      server.send(400, "text/plain", "channel/percent fehlt");
      return;
    }
    int ch = server.arg("channel").toInt();
    int percent = server.arg("percent").toInt();
    if (ch < 0 || ch >= PWM_OUTPUT_COUNT) {
      server.send(400, "text/plain", "ungueltiger PCA-Kanal");
      return;
    }
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;

    const PwmOutputConfig& out = s_ctx->config.pwmOutputs[ch];
    if (out.mode == PwmOutputMode::SWITCH) {
      percent = (percent >= 50) ? 100 : 0;
      PwmDriver::setSwitch((uint8_t)ch, percent > 0, out.profile);
    } else {
      PwmDriver::setDuty((uint8_t)ch, (uint8_t)percent, out.profile);
    }

    Serial.print("PCA TEST Kanal ");
    Serial.print(ch);
    Serial.print(" -> ");
    Serial.print(percent);
    Serial.println(" %");
    Serial.flush();
    server.send(200, "text/plain", "OK");
  }

  void handleTestAllOff() {
    if (!commissioningTestAuthorized()) {
      server.send(403, "text/plain", "Testmodus nicht aktiv oder Inbetriebnahme-PIN falsch");
      return;
    }
    RelayOutputs::allOff(*s_ctx);
    PwmDriver::allOff();
    server.send(200, "text/plain", "OK");
  }

  void handleTestModeGet() {
    String json = "{";
    json += "\"active\":" + String(g_commissioningTestActive ? "true" : "false");
    json += ",\"message\":\"";
    json += g_commissioningTestActive ? "Testmodus aktiv - Regelung pausiert" : "Normalbetrieb";
    json += "\"}";
    server.send(200, "application/json", json);
  }

  void handleTestModeSet() {
    if (!server.hasArg("active")) {
      server.send(400, "text/plain", "active fehlt");
      return;
    }

    const bool requestedActive = server.arg("active").toInt() != 0;

    if (requestedActive) {
      if (!testSessionValid()) {
        server.send(403, "text/plain", "Inbetriebnahme-PIN falsch");
        return;
      }

      activateCommissioningTestMode();
      server.send(200, "application/json", "{\"ok\":true,\"active\":true}");
      return;
    }

    deactivateCommissioningTestMode();
    server.send(200, "application/json", "{\"ok\":true,\"active\":false}");
  }

 static void handleSafetyJson() {
  if (s_ctx == nullptr) {
    server.send(500, "application/json", "{\"error\":\"context missing\"}");
    return;
  }

  const SafetyManager::SafetyStatus& st = SafetyManager::status();
  const ConfigData& c = s_ctx->config;

  String json = "{";

  json += "\"status\":{";
  json += "\"mode\":\"";
  json += SafetyManager::modeToText(st.mode);
  json += "\",\"message\":\"";
  json += st.message;
  json += "\",\"lowestCollectorTemperatureC\":";
  json += jsonFloat(st.lowestCollectorTemperatureC, 1);
  json += ",\"highestCollectorTemperatureC\":";
  json += jsonFloat(st.highestCollectorTemperatureC, 1);
  json += ",\"highestStorageTemperatureC\":";
  json += jsonFloat(st.highestStorageTemperatureC, 1);
  json += ",\"frostStorageTemperatureC\":";
  json += jsonFloat(st.frostStorageTemperatureC, 1);
  json += ",\"ovenTemperatureC\":";
  json += jsonFloat(st.ovenTemperatureC, 1);
  json += ",\"controlledStagnation\":";
  json += st.controlledStagnation ? "true" : "false";
  json += "}";

  json += ",\"config\":{";
  json += "\"solarFluidType\":";
  json += String((int)c.solarFluidType);
  json += ",\"solarHydraulicType\":";
  json += String((int)c.solarHydraulicType);
  json += ",\"solarCollectorType\":";
  json += String((int)c.solarCollectorType);

  json += ",\"frostEnabled\":";
  json += c.frostEnabled ? "true" : "false";
  json += ",\"frostCollectorOnC\":";
  json += jsonFloat(c.frostCollectorOnC, 1);
  json += ",\"frostSafeCollectorTemperatureC\":";
  json += jsonFloat(c.frostSafeCollectorTemperatureC, 1);
  json += ",\"frostSinkMinC\":";
  json += jsonFloat(c.frostSinkMinC, 1);
  json += ",\"frostPumpPercent\":";
  json += jsonFloat(c.frostPumpPercent, 0);
  json += ",\"frostProtectionStorageRole\":";
  json += String((int)c.frostProtectionStorageRole);

  json += ",\"stagnationEnabled\":";
  json += c.stagnationEnabled ? "true" : "false";
  json += ",\"stagnationCollectorOnC\":";
  json += jsonFloat(c.stagnationCollectorOnC, 1);
  json += ",\"stagnationCollectorOffC\":";
  json += jsonFloat(c.stagnationCollectorOffC, 1);
  json += ",\"stagnationPumpPercent\":";
  json += jsonFloat(c.stagnationPumpPercent, 0);

  json += ",\"storageProtectionEnabled\":";
  json += c.storageProtectionEnabled ? "true" : "false";
  json += ",\"sinkMaxC\":";
  json += jsonFloat(c.sinkMaxC, 1);
  json += ",\"storageCriticalTemperatureC\":";
  json += jsonFloat(c.storageCriticalTemperatureC, 1);
  json += ",\"safetyNightCoolingEnabled\":";
  json += c.safetyNightCoolingEnabled ? "true" : "false";
  json += ",\"safetyNightCoolingTargetTemperatureC\":";
  json += jsonFloat(c.safetyNightCoolingTargetTemperatureC, 1);

  json += ",\"ovenProtectionEnabled\":";
  json += c.ovenProtectionEnabled ? "true" : "false";
  json += ",\"ovenOvertemperatureOnC\":";
  json += jsonFloat(c.ovenOvertemperatureOnC, 1);
  json += ",\"ovenOvertemperatureOffC\":";
  json += jsonFloat(c.ovenOvertemperatureOffC, 1);
  json += ",\"nightCoolingActive\":";
  json += st.safetyNightCoolingActive ? "true":"false";

  json += ",\"nightCoolingCollectorTemperatureC\":";
  json += jsonFloat(st.nightCoolingCollectorTemperatureC,1);

  json += ",\"nightCoolingStorageTemperatureC\":";
  json += jsonFloat(st.nightCoolingStorageTemperatureC,1);

  json += ",\"nightCoolingDeltaC\":";
  json += jsonFloat(st.nightCoolingDeltaC,1);

  json += ",\"nightCoolingPumpPercent\":";
  json += jsonFloat(st.nightCoolingPumpPercent,0);
  json += "}";

json += ",\"storageRoles\":[";

bool first = true;
for (uint8_t i = 0; i < s_ctx->assignments.count; i++) {
  const Ds18RoleAssignment& assignment = s_ctx->assignments.items[i];

  if (!assignment.assigned) continue;
  if (assignment.role == Ds18Role::NONE) continue;

  if (!first) json += ",";
  first = false;

  json += "{\"value\":";
  json += String((int)assignment.role);
  json += ",\"label\":\"";
  json += SensorRoles::toLabel(assignment.role);
  json += "\"}";
}

json += "]";

  json += "}";

  server.send(200, "application/json", json);
}

static void handleSafetySave() {
  if (s_ctx == nullptr) {
    server.send(500, "application/json", "{\"error\":\"context missing\"}");
    return;
  }

  ConfigData& c = s_ctx->config;

  c.solarFluidType = (SolarFluidType)server.arg("solarFluidType").toInt();
  c.solarHydraulicType = (SolarHydraulicType)server.arg("solarHydraulicType").toInt();
  c.solarCollectorType = (SolarCollectorType)server.arg("solarCollectorType").toInt();

  c.frostEnabled = server.arg("frostEnabled").toInt() != 0;
  c.frostCollectorOnC = server.arg("frostCollectorOnC").toFloat();
  c.frostSafeCollectorTemperatureC = server.arg("frostSafeCollectorTemperatureC").toFloat();
  c.frostSinkMinC = server.arg("frostSinkMinC").toFloat();
  c.frostPumpPercent = server.arg("frostPumpPercent").toFloat();
  c.frostProtectionStorageRole = (Ds18Role)server.arg("frostProtectionStorageRole").toInt();

  c.stagnationEnabled = server.arg("stagnationEnabled").toInt() != 0;
  c.stagnationCollectorOnC = server.arg("stagnationCollectorOnC").toFloat();
  c.stagnationCollectorOffC = server.arg("stagnationCollectorOffC").toFloat();
  c.stagnationPumpPercent = server.arg("stagnationPumpPercent").toFloat();

  c.storageProtectionEnabled = server.arg("storageProtectionEnabled").toInt() != 0;
  c.sinkMaxC = server.arg("sinkMaxC").toFloat();
  c.storageCriticalTemperatureC = server.arg("storageCriticalTemperatureC").toFloat();
  c.safetyNightCoolingEnabled = server.arg("safetyNightCoolingEnabled").toInt() != 0;
  c.safetyNightCoolingTargetTemperatureC = server.arg("safetyNightCoolingTargetTemperatureC").toFloat();

  c.ovenProtectionEnabled = server.arg("ovenProtectionEnabled").toInt() != 0;
  c.ovenOvertemperatureOnC = server.arg("ovenOvertemperatureOnC").toFloat();
  c.ovenOvertemperatureOffC = server.arg("ovenOvertemperatureOffC").toFloat();

  Storage::saveConfig(s_ctx->config);

  server.send(200, "application/json", "{\"ok\":true}");
}

} // namespace

// ================================
// HEATING CIRCUITS API HELPERS
// ================================
namespace {

  String outputKindToKey(OutputKind kind) {
    if (kind == OutputKind::RELAY) return "relay";
    if (kind == OutputKind::PWM_OUTPUT) return "pwm";
    return "none";
  }

  OutputKind outputKindFromKey(const String& key) {
    if (key == "relay") return OutputKind::RELAY;
    if (key == "pwm") return OutputKind::PWM_OUTPUT;
    return OutputKind::NONE;
  }

  void appendOutputOptionsJson(String& json) {
    json += "\"outputs\":[";
    bool first = true;
    for (uint8_t i = 0; i < RELAY_COUNT; i++) {
      const RelayOutputConfig& r = s_ctx->config.relays[i];
      if (!r.enabled) continue;
      if (!first) json += ",";
      first = false;
      json += "{\"kind\":\"relay\",\"index\":" + String(i) + ",\"label\":\"" + relayHardwareLabel(i) + "\",\"function\":\"" + String(RelayOutputs::functionToKey(r.function)) + "\",\"mode\":\"switch\"}";
    }
    for (uint8_t i = 0; i < PWM_OUTPUT_COUNT; i++) {
      const PwmOutputConfig& po = s_ctx->config.pwmOutputs[i];
      if (!po.enabled) continue;
      if (!first) json += ",";
      first = false;
      json += "{\"kind\":\"pwm\",\"index\":" + String(i) + ",\"label\":\"" + pwmOutputHardwareLabel(i) + "\",\"mode\":\"" + pwmOutputModeToKey(po.mode) + "\",\"function\":\"" + String(RelayOutputs::functionToKey(po.function)) + "\"}";
    }
    json += "]";
  }

  void appendAllAssignableDs18RolesJson(String& json, const char* fieldName) {
    json += "\"" + String(fieldName) + "\":[";
    bool first = true;
    for (int i = 0; i <= (int)Ds18Role::HK4_RETURN; i++) {
      Ds18Role role = (Ds18Role)i;
      if (!SensorRoles::isAssignable(role)) continue;
      if (!first) json += ",";
      first = false;
      json += "{\"value\":" + String((int)role) + ",\"key\":\"" + String(SensorRoles::toKey(role)) + "\",\"label\":\"" + String(SensorRoles::toLabel(role)) + "\"}";
    }
    json += "]";
  }

  void handleHeatingCircuitsJson() {
    if (!s_ctx) {
      server.send(500, "application/json", "{\"error\":\"no_context\"}");
      return;
    }

    String json = "{";
    json += "\"circuits\":[";
    for (uint8_t i = 0; i < MAX_HEATING_CIRCUITS; i++) {
      if (i > 0) json += ",";
      const HeatingCircuitConfig& cfg = s_ctx->config.heatingCircuits[i];
      const HeatingCircuitRuntime& rt = s_ctx->heatingCircuitRuntime[i];
      json += "{";
      json += "\"index\":" + String(i) + ",";
      json += "\"name\":\"HK" + String(i + 1) + "\",";
      json += "\"enabled\":" + String(cfg.enabled ? "true" : "false") + ",";
      json += "\"mixerType\":" + String((int)cfg.mixerType) + ",";
      json += "\"controlMode\":" + String((int)cfg.controlMode) + ",";
      json += "\"mixerOpenOutputKind\":\"" + outputKindToKey(cfg.mixerOpenOutput.kind) + "\",";
      json += "\"mixerOpenOutputIndex\":" + String(cfg.mixerOpenOutput.index) + ",";
      json += "\"mixerCloseOutputKind\":\"" + outputKindToKey(cfg.mixerCloseOutput.kind) + "\",";
      json += "\"mixerCloseOutputIndex\":" + String(cfg.mixerCloseOutput.index) + ",";
      json += "\"pumpMode\":" + String((int)cfg.pumpMode) + ",";
      json += "\"pumpOutputKind\":\"" + outputKindToKey(cfg.pumpOutput.kind) + "\",";
      json += "\"pumpOutputIndex\":" + String(cfg.pumpOutput.index) + ",";
      json += "\"pumpMinPercent\":" + String(cfg.pumpMinPercent) + ",";
      json += "\"pumpMaxPercent\":" + String(cfg.pumpMaxPercent) + ",";
      json += "\"flowSensorRole\":" + String((int)cfg.flowSensorRole) + ",";
      json += "\"returnSensorRole\":" + String((int)cfg.returnSensorRole) + ",";
      json += "\"roomSensorRole\":" + String((int)cfg.roomSensorRole) + ",";
      json += "\"bufferReferenceRole\":" + String((int)cfg.bufferReferenceRole) + ",";
      json += "\"outsideSensorRole\":" + String((int)cfg.outsideSensorRole) + ",";
      json += "\"fixedFlowTemperatureC\":" + String(cfg.fixedFlowTemperatureC, 2) + ",";
      json += "\"maximumFlowTemperatureC\":" + String(cfg.maximumFlowTemperatureC, 2) + ",";
      json += "\"minimumFlowTemperatureC\":" + String(cfg.minimumFlowTemperatureC, 2) + ",";
      json += "\"roomTargetTemperatureC\":" + String(cfg.roomTargetTemperatureC, 2) + ",";
      json += "\"roomInfluenceK\":" + String(cfg.roomInfluenceK, 2) + ",";
      json += "\"heatingCurveBaseC\":" + String(cfg.heatingCurveBaseC, 2) + ",";
      json += "\"heatingCurveSlope\":" + String(cfg.heatingCurveSlope, 3) + ",";
      json += "\"frostProtectionEnabled\":" + String(cfg.frostProtectionEnabled ? "true" : "false") + ",";
      json += "\"frostStartTemperatureC\":" + String(cfg.frostStartTemperatureC, 2) + ",";
      json += "\"frostTargetFlowTemperatureC\":" + String(cfg.frostTargetFlowTemperatureC, 2) + ",";
      json += "\"mixerFullTravelMs\":" + String(cfg.mixerFullTravelMs) + ",";
      json += "\"mixerPulseMs\":" + String(cfg.mixerPulseMs) + ",";
      json += "\"mixerPauseMs\":" + String(cfg.mixerPauseMs) + ",";
      json += "\"runtime\":{";
      json += "\"active\":" + String(rt.active ? "true" : "false") + ",";
      json += "\"pumpActive\":" + String(rt.pumpActive ? "true" : "false") + ",";
      json += "\"flowTemperatureC\":" + jsonFloat(rt.flowTemperatureC, 2) + ",";
      json += "\"targetFlowTemperatureC\":" + jsonFloat(rt.targetFlowTemperatureC, 2) + ",";
      json += "\"mixerPosition\":" + String(rt.estimatedMixerPositionPercent);
      json += "}";
      json += "}";
    }
    json += "],";
    appendOutputOptionsJson(json);
    json += ",";
    appendAllAssignableDs18RolesJson(json, "sensorRoles");
    json += "}";
    server.send(200, "application/json", json);
  }

  void handleHeatingCircuitSave() {
    if (!serviceSessionValid()) {
      server.send(403, "text/plain", "Nicht erlaubt");
      return;
    }
    if (!s_ctx || !server.hasArg("index")) {
      server.send(400, "text/plain", "Parameter fehlen");
      return;
    }
    int idx = server.arg("index").toInt();
    if (idx < 0 || idx >= MAX_HEATING_CIRCUITS) {
      server.send(400, "text/plain", "ungueltiger Heizkreis");
      return;
    }

    HeatingCircuitConfig candidate = s_ctx->config.heatingCircuits[idx];

    candidate.enabled = server.arg("enabled").toInt() != 0;
    if (server.hasArg("mixerType")) candidate.mixerType = (server.arg("mixerType").toInt() == 1) ? HeatingCircuitMixerType::THERMAL : HeatingCircuitMixerType::THREE_POINT;
    if (server.hasArg("controlMode")) candidate.controlMode = (server.arg("controlMode").toInt() == 1) ? HeatingCircuitControlMode::WEATHER_COMPENSATED : HeatingCircuitControlMode::FIXED_FLOW;
    if (server.hasArg("mixerOpenOutputKind")) candidate.mixerOpenOutput.kind = outputKindFromKey(server.arg("mixerOpenOutputKind"));
    if (server.hasArg("mixerOpenOutputIndex")) candidate.mixerOpenOutput.index = (uint8_t)server.arg("mixerOpenOutputIndex").toInt();
    if (server.hasArg("mixerCloseOutputKind")) candidate.mixerCloseOutput.kind = outputKindFromKey(server.arg("mixerCloseOutputKind"));
    if (server.hasArg("mixerCloseOutputIndex")) candidate.mixerCloseOutput.index = (uint8_t)server.arg("mixerCloseOutputIndex").toInt();
    if (server.hasArg("pumpMode")) candidate.pumpMode = (HeatingCircuitPumpMode)server.arg("pumpMode").toInt();
    if (server.hasArg("pumpOutputKind")) candidate.pumpOutput.kind = outputKindFromKey(server.arg("pumpOutputKind"));
    if (server.hasArg("pumpOutputIndex")) candidate.pumpOutput.index = (uint8_t)server.arg("pumpOutputIndex").toInt();
    if (server.hasArg("pumpMinPercent")) candidate.pumpMinPercent = (uint8_t)server.arg("pumpMinPercent").toInt();
    if (server.hasArg("pumpMaxPercent")) candidate.pumpMaxPercent = (uint8_t)server.arg("pumpMaxPercent").toInt();
    if (server.hasArg("flowSensorRole")) candidate.flowSensorRole = (Ds18Role)server.arg("flowSensorRole").toInt();
    if (server.hasArg("returnSensorRole")) candidate.returnSensorRole = (Ds18Role)server.arg("returnSensorRole").toInt();
    if (server.hasArg("roomSensorRole")) candidate.roomSensorRole = (Ds18Role)server.arg("roomSensorRole").toInt();
    if (server.hasArg("bufferReferenceRole")) candidate.bufferReferenceRole = (Ds18Role)server.arg("bufferReferenceRole").toInt();
    if (server.hasArg("outsideSensorRole")) candidate.outsideSensorRole = (Ds18Role)server.arg("outsideSensorRole").toInt();
    if (server.hasArg("fixedFlowTemperatureC")) candidate.fixedFlowTemperatureC = server.arg("fixedFlowTemperatureC").toFloat();
    if (server.hasArg("maximumFlowTemperatureC")) candidate.maximumFlowTemperatureC = server.arg("maximumFlowTemperatureC").toFloat();
    if (server.hasArg("minimumFlowTemperatureC")) candidate.minimumFlowTemperatureC = server.arg("minimumFlowTemperatureC").toFloat();
    if (server.hasArg("roomTargetTemperatureC")) candidate.roomTargetTemperatureC = server.arg("roomTargetTemperatureC").toFloat();
    if (server.hasArg("roomInfluenceK")) candidate.roomInfluenceK = server.arg("roomInfluenceK").toFloat();
    if (server.hasArg("heatingCurveBaseC")) candidate.heatingCurveBaseC = server.arg("heatingCurveBaseC").toFloat();
    if (server.hasArg("heatingCurveSlope")) candidate.heatingCurveSlope = server.arg("heatingCurveSlope").toFloat();
    if (server.hasArg("frostProtectionEnabled")) candidate.frostProtectionEnabled = server.arg("frostProtectionEnabled").toInt() != 0;
    if (server.hasArg("frostStartTemperatureC")) candidate.frostStartTemperatureC = server.arg("frostStartTemperatureC").toFloat();
    if (server.hasArg("frostTargetFlowTemperatureC")) candidate.frostTargetFlowTemperatureC = server.arg("frostTargetFlowTemperatureC").toFloat();
    if (server.hasArg("mixerFullTravelMs")) candidate.mixerFullTravelMs = (uint32_t)server.arg("mixerFullTravelMs").toInt();
    if (server.hasArg("mixerPulseMs")) candidate.mixerPulseMs = (uint32_t)server.arg("mixerPulseMs").toInt();
    if (server.hasArg("mixerPauseMs")) candidate.mixerPauseMs = (uint32_t)server.arg("mixerPauseMs").toInt();

if (!validateHeatingCircuitOutputConfig(*s_ctx, candidate, idx)) {
  return;
}

HeatingCircuitConfig old = s_ctx->config.heatingCircuits[idx];

s_ctx->config.heatingCircuits[idx] = candidate;

String validationError;
if (!OutputValidation::validateAll(*s_ctx, validationError)) {
  s_ctx->config.heatingCircuits[idx] = old;
  server.send(409, "text/plain", validationError);
  return;
}

Storage::saveConfig(s_ctx->config);
server.send(200, "text/plain", "OK");
  }


void handleAuxHeaterJson() {
  if (!s_ctx) {
    server.send(500, "application/json", "{\"error\":\"no_context\"}");
    return;
  }

  const AuxHeaterConfig& cfg = s_ctx->config.auxHeater;

  String json = "{";
  json += "\"config\":{";
  json += "\"enabled\":";
  json += String(cfg.enabled ? "true" : "false");
  json += ",\"minimumTemperatureC\":";
  json += String(cfg.minimumTemperatureC, 1);
  json += ",\"targetTemperatureC\":";
  json += String(cfg.targetTemperatureC, 1);
  json += ",\"hysteresisC\":";
  json += String(cfg.hysteresisC, 1);
  json += ",\"sinkRole\":\"";
  json += SensorRoles::toKey(cfg.sinkRole);
  json += "\"";
  json += ",\"pumpRelay\":";
  json += String(cfg.pumpRelay);
  json += ",\"heaterRelay1\":";
  json += String(cfg.heaterRelay1);
  json += ",\"heaterRelay2\":";
  json += String(cfg.heaterRelay2);
  json += ",\"heaterRelay3\":";
  json += String(cfg.heaterRelay3);
  json += ",\"preRunSeconds\":";
  json += String(cfg.preRunMs / 1000UL);
  json += ",\"cooldownSeconds\":";
  json += String(cfg.cooldownMs / 1000UL);
  json += "}";

  json += ",\"sinkRoles\":[";
  bool firstSink = true;
  appendActiveDs18RoleOptions(json, *s_ctx, firstSink);
  json += "]";

  json += ",\"pumpRelays\":[";
  bool firstPump = true;
  for (uint8_t i = 0; i < RELAY_COUNT; i++) {
    const RelayOutputConfig& r = s_ctx->config.relays[i];
    if (!r.enabled || r.function != RelayFunction::PUMP_ENABLE) continue;
    if (!firstPump) json += ",";
    firstPump = false;

    String usage;
    const bool busy = outputIsUsedByLegacyConsumers(*s_ctx, OutputKind::RELAY, i, -1, usage, true, false) ||
                      outputIsUsedByHeatingCircuit(*s_ctx, OutputKind::RELAY, i, -1, usage);

    json += "{\"value\":";
    json += String(i);
    json += ",\"label\":\"";
    json += relayHardwareLabel(i);
    if (busy) {
      json += " (belegt: ";
      json += usage;
      json += ")";
    }
    json += "\",\"used\":";
    json += String(busy ? "true" : "false");
    json += "}";
  }
  json += "]";

  json += ",\"heaterRelays\":[";
  bool firstHeater = true;
  for (uint8_t i = 0; i < RELAY_COUNT; i++) {
    const RelayOutputConfig& r = s_ctx->config.relays[i];
    if (!r.enabled || r.function != RelayFunction::HEATER_ROD) continue;
    if (!firstHeater) json += ",";
    firstHeater = false;

    String usage;
    const bool busy = outputIsUsedByLegacyConsumers(*s_ctx, OutputKind::RELAY, i, -1, usage, true, false) ||
                      outputIsUsedByHeatingCircuit(*s_ctx, OutputKind::RELAY, i, -1, usage);

    json += "{\"value\":";
    json += String(i);
    json += ",\"label\":\"";
    json += relayHardwareLabel(i);
    if (busy) {
      json += " (belegt: ";
      json += usage;
      json += ")";
    }
    json += "\",\"used\":";
    json += String(busy ? "true" : "false");
    json += "}";
  }
  json += "]";

  json += "}";
  server.send(200, "application/json", json);
}

void handleAuxHeaterSave() {
  if (!serviceSessionValid()) {
    server.send(403, "text/plain", "Nicht erlaubt");
    return;
  }

  if (!s_ctx) {
    server.send(500, "text/plain", "Kein Kontext");
    return;
  }

  AuxHeaterConfig candidate = s_ctx->config.auxHeater;

  if (server.hasArg("enabled")) candidate.enabled = server.arg("enabled").toInt() != 0;
  if (server.hasArg("minimumTemperatureC")) candidate.minimumTemperatureC = server.arg("minimumTemperatureC").toFloat();
  if (server.hasArg("targetTemperatureC")) candidate.targetTemperatureC = server.arg("targetTemperatureC").toFloat();
  if (server.hasArg("hysteresisC")) candidate.hysteresisC = server.arg("hysteresisC").toFloat();
  if (server.hasArg("sinkRole")) candidate.sinkRole = SensorRoles::fromKey(server.arg("sinkRole"));
  if (server.hasArg("pumpRelay")) candidate.pumpRelay = (uint8_t)server.arg("pumpRelay").toInt();
  if (server.hasArg("heaterRelay1")) candidate.heaterRelay1 = (uint8_t)server.arg("heaterRelay1").toInt();
  if (server.hasArg("heaterRelay2")) candidate.heaterRelay2 = (uint8_t)server.arg("heaterRelay2").toInt();
  if (server.hasArg("heaterRelay3")) candidate.heaterRelay3 = (uint8_t)server.arg("heaterRelay3").toInt();
  if (server.hasArg("preRunSeconds")) candidate.preRunMs = (uint32_t)server.arg("preRunSeconds").toInt() * 1000UL;
  if (server.hasArg("cooldownSeconds")) candidate.cooldownMs = (uint32_t)server.arg("cooldownSeconds").toInt() * 1000UL;

if (!validateAuxHeaterOutputConfig(*s_ctx, candidate)) {
  return;
}

AuxHeaterConfig old = s_ctx->config.auxHeater;

s_ctx->config.auxHeater = candidate;

String validationError;
if (!OutputValidation::validateAll(*s_ctx, validationError)) {
  s_ctx->config.auxHeater = old;
  server.send(409, "text/plain", validationError);
  return;
}

Storage::saveConfig(s_ctx->config);
server.send(200, "text/plain", "OK");
}


} // namespace

namespace UI {

void begin(AppContext& ctx) {
  Serial.println("UI::begin()");
  s_ctx = &ctx;

  server.on("/", handleRoot);
  server.on("/api/status", handleStatusJson);
  server.on("/api/ds18b20", handleDs18b20Json);
  server.on("/api/roles", handleRolesJson);
  server.on("/api/assignments", HTTP_GET, handleAssignmentsJson);

  server.on("/service-assign-role", HTTP_POST, handleAssignRole);
  server.on("/api/plant", HTTP_GET, handlePlantJson);
  server.on("/api/heat-sources", HTTP_GET, handleHeatSourcesJson);
  server.on("/api/max-status", HTTP_GET, handleMaxStatusJson);
  server.on("/api/relays", HTTP_GET, handleRelaysJson);
  server.on("/service-relay-config", HTTP_POST, handleRelayConfig);
  server.on("/api/pumps", HTTP_GET, handlePumpsJson);
  server.on("/service-pump-config", HTTP_POST, handlePumpConfig);
  server.on("/api/oven", HTTP_GET, handleOvenJson);
  server.on("/service-save-oven", HTTP_POST, handleOvenSave);
  server.on("/service-oven-start", HTTP_POST, handleOvenStart);
  server.on("/service-oven-stop", HTTP_POST, handleOvenStop);
  server.on("/api/test", HTTP_GET, handleTestOverviewJson);
  server.on("/api/testmode", HTTP_GET, handleTestModeGet);
  server.on("/service-testmode", HTTP_POST, handleTestModeSet);
  server.on("/service-test-relay-raw", HTTP_POST, handleTestRelayRaw);
  server.on("/service-test-pca-pwm", HTTP_POST, handleTestPcaPwm);
  server.on("/service-test-all-off", HTTP_POST, handleTestAllOff);
  server.on("/service-assign-heat-source", HTTP_POST, handleAssignHeatSource);
  server.on("/service-save-max-config", HTTP_POST, handleSaveMaxConfig);
  server.on("/api/heating-circuits", HTTP_GET, handleHeatingCircuitsJson);
  server.on("/service-heating-circuit", HTTP_POST, handleHeatingCircuitSave);
  server.on("/api/safety", HTTP_GET, handleSafetyJson);
  server.on("/service-save-safety", HTTP_POST, handleSafetySave);
  server.on("/api/aux-heater", HTTP_GET, handleAuxHeaterJson);
  server.on("/service-save-aux-heater", HTTP_POST, handleAuxHeaterSave);
  server.onNotFound(handleStatic);

  server.begin();
  Serial.println("Webserver gestartet");
}

void update() {
  server.handleClient();
}


bool commissioningTestActive() {
  return g_commissioningTestActive;
}
}