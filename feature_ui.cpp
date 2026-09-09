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
#include "feature_servo_driver.h"
#include "feature_valves.h"
#include "feature_pump_routing.h"
#include "feature_heating_circuits.h"
#include "feature_heat_sources_max31865.h"
#include "feature_alarms.h"
#include <WebServer.h>
#include <SD.h>
#include <WiFi.h>

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
  bool g_restartScheduled = false;
  uint32_t g_restartAtMs = 0;

  void scheduleRestart(uint32_t delayMs = 1200) {
    g_restartScheduled = true;
    g_restartAtMs = millis() + delayMs;
  }
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

  String jsonFloat(float v, uint8_t decimals);
  String jsonEscape(const char* value);
  String ipToString(const IPAddress& ip);
  void handleAlarmsJson();
  void handleAlarmsAck();
  void handleAlarmsClearHistory();

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

    if (pump.valveIndex != PIN_UNUSED && pump.activeTargetIndex < PUMP_ROUTE_TARGET_COUNT) {
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
    json += "\"sink\":\"" + String((p.valveIndex != PIN_UNUSED) ? String("Umschaltventil") : String(SensorRoles::toLabel(p.sinkRole))) + "\",";
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

  // Browser status must use the same authoritative runtime read path as /api/max-debug.
  HeatSourcesMax::readAllNow(*s_ctx);

  String json = "{";
  json += "\"rNominalOhm\":" + String(MAX31865_RNOMINAL, 2) + ",";
  json += "\"rRefOhm\":" + String(MAX31865_RREF, 2) + ",";
  json += "\"wireMode\":\"" + String(MAX31865_USE_3WIRE ? "3-wire" : "2/4-wire") + "\",";
  json += "\"validResistanceMinOhm\":" + String(MAX31865_VALID_R_MIN_OHM, 1) + ",";
  json += "\"validResistanceMaxOhm\":" + String(MAX31865_VALID_R_MAX_OHM, 1) + ",";
  json += "\"validTempMinC\":" + String(MAX31865_VALID_TEMP_MIN_C, 1) + ",";
  json += "\"validTempMaxC\":" + String(MAX31865_VALID_TEMP_MAX_C, 1) + ",";

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
    json += "\"tempC\":" + jsonFloat(r.tempC, 2) + ",";
    json += "\"rawTempC\":" + jsonFloat(r.rawTempC, 2) + ",";
    json += "\"resistanceOhm\":" + jsonFloat(r.resistanceOhm, 2) + ",";
    json += "\"rawRtd\":" + String(r.rawRtd) + ",";
    json += "\"rtdCode\":" + String(r.rtdCode) + ",";
    json += "\"rawFaultBit\":" + String(r.rawFaultBit ? "true" : "false") + ",";
    json += "\"fault\":" + String(r.fault) + ",";
    json += "\"status\":\"" + jsonEscape(r.status) + "\",";
    json += "\"offsetC\":" + String(c.offsetC, 2) + ",";
    json += "\"calFactor\":" + String(c.calFactor, 4);
    json += "}";
  }
  json += "],";

  json += "\"activeHeatSource\":{";
  json += "\"role\":\"" + String(HeatSourceRoles::toKey(s_ctx->sensors.activeHeatSource.role)) + "\",";
  json += "\"label\":\"" + String(HeatSourceRoles::toLabel(s_ctx->sensors.activeHeatSource.role)) + "\",";
  json += "\"tempC\":" + jsonFloat(s_ctx->sensors.activeHeatSource.tempC, 2) + ",";
  json += "\"valid\":" + String(s_ctx->sensors.activeHeatSource.valid ? "true" : "false");
  json += "}";

  json += "}";

  server.send(200, "application/json", json);
}

void handleMaxDebugJson() {
  if (!s_ctx) {
    server.send(500, "application/json", "{\"error\":\"no_context\"}");
    return;
  }

  const bool oneShot = !server.hasArg("oneshot") || server.arg("oneshot") != "0";
  server.send(200, "application/json", HeatSourcesMax::debugJson(*s_ctx, oneShot));
}

void handleNetworkJson() {
  if (!s_ctx) {
    server.send(500, "application/json", "{\"error\":\"no_context\"}");
    return;
  }

  const bool staConnected = (WiFi.status() == WL_CONNECTED);
  String json = "{";
  json += "\"apSsid\":\"" + jsonEscape(s_ctx->config.apName) + "\",";
  json += "\"apIp\":\"" + ipToString(WiFi.softAPIP()) + "\",";
  json += "\"staEnabled\":" + String(s_ctx->config.staEnabled ? "true" : "false") + ",";
  json += "\"staSsid\":\"" + jsonEscape(s_ctx->config.staSsid) + "\",";
  json += "\"staConnected\":" + String(staConnected ? "true" : "false") + ",";
  json += "\"staIp\":\"" + String(staConnected ? ipToString(WiFi.localIP()) : "") + "\",";
  json += "\"staRssi\":" + String(staConnected ? String(WiFi.RSSI()) : "null") + ",";
  json += "\"hostName\":\"" + jsonEscape(s_ctx->config.hostName) + "\",";
  json += "\"macSta\":\"" + WiFi.macAddress() + "\",";
  json += "\"macAp\":\"" + WiFi.softAPmacAddress() + "\"";
  json += "}";

  server.send(200, "application/json", json);
}

void handleNetworkSave() {
  if (!serviceSessionValid()) {
    server.send(403, "text/plain", "Nicht erlaubt");
    return;
  }

  if (!s_ctx) {
    server.send(500, "text/plain", "Kein Kontext");
    return;
  }

  s_ctx->config.staEnabled = server.hasArg("staEnabled") && server.arg("staEnabled") == "1";

  if (server.hasArg("staSsid")) {
    String v = server.arg("staSsid");
    v.trim();
    if (v.length() >= (int)sizeof(s_ctx->config.staSsid)) {
      server.send(400, "text/plain", "SSID ist zu lang");
      return;
    }
    v.toCharArray(s_ctx->config.staSsid, sizeof(s_ctx->config.staSsid));
  }

  if (server.hasArg("staPassword")) {
    String v = server.arg("staPassword");
    if (v.length() >= (int)sizeof(s_ctx->config.staPassword)) {
      server.send(400, "text/plain", "WLAN-Passwort ist zu lang");
      return;
    }
    v.toCharArray(s_ctx->config.staPassword, sizeof(s_ctx->config.staPassword));
  }

  if (server.hasArg("hostName")) {
    String v = server.arg("hostName");
    v.trim();
    if (!v.length()) v = DEFAULT_HOSTNAME;
    if (v.length() >= (int)sizeof(s_ctx->config.hostName)) {
      server.send(400, "text/plain", "Hostname ist zu lang");
      return;
    }
    v.toCharArray(s_ctx->config.hostName, sizeof(s_ctx->config.hostName));
  }

  if (!Storage::saveConfig(s_ctx->config)) {
    server.send(500, "text/plain", "Speichern fehlgeschlagen");
    return;
  }

  server.send(200, "text/plain", "OK_RESTART_REQUIRED");
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
      case RelayFunction::ALARM_BUZZER: return "Summer / Alarm";
      case RelayFunction::NONE:
      default: return "frei";
    }
  }

  bool outputUsageLabel(const AppContext& ctx, OutputKind kind, uint8_t index, String& usage);

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
      String usage;
      const bool locked = outputUsageLabel(*s_ctx, OutputKind::RELAY, i, usage);

      json += "{";
      json += "\"type\":\"relay\",";
      json += "\"index\":" + String(i) + ",";
      json += "\"name\":\"" + relayHardwareLabel(i) + "\",";
      json += "\"enabled\":" + String(cfg.enabled ? "true" : "false") + ",";
      json += "\"function\":\"" + String(RelayOutputs::functionToKey(cfg.function)) + "\",";
      json += "\"functionLabel\":\"" + relayFunctionLabel(cfg.function) + "\",";
      json += "\"activeLow\":" + String(cfg.activeLow ? "true" : "false") + ",";
      json += "\"locked\":" + String(locked ? "true" : "false") + ",";
      json += "\"usedBy\":";
      if (locked) json += "\"" + usage + "\""; else json += "null";
      json += ",\"state\":" + String(state ? "true" : "false");
      json += "}";
    }

    json += "],\"pwmOutputs\":[";
    for (uint8_t i = 0; i < PWM_OUTPUT_COUNT; i++) {
      if (i > 0) json += ",";

      const PwmOutputConfig& cfg = s_ctx->config.pwmOutputs[i];
      String usage;
      const bool locked = outputUsageLabel(*s_ctx, OutputKind::PWM_OUTPUT, i, usage);

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
      json += "\"locked\":" + String(locked ? "true" : "false") + ",";
      json += "\"usedBy\":";
      if (locked) json += "\"" + usage + "\""; else json += "null";
      json += ",\"percent\":" + String(PwmDriver::getDuty(i));
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

OutputKind outputKindFromKey(const String& key);
int valveOutputUsedBy(const AppContext& ctx, OutputKind kind, uint8_t index, int ignoreValve);

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

bool outputIsUsedByValves(const AppContext& ctx, OutputKind kind, uint8_t index, int ignoreValveIndex, String& usage) {
  const int usedBy = valveOutputUsedBy(ctx, kind, index, ignoreValveIndex);
  if (usedBy < 0) return false;
  usage = "Ventil " + String(usedBy + 1);
  return true;
}

bool outputUsageLabel(const AppContext& ctx, OutputKind kind, uint8_t index, String& usage) {
  usage = "";
  return outputIsUsedByLegacyConsumers(ctx, kind, index, -1, usage) ||
         outputIsUsedByHeatingCircuit(ctx, kind, index, -1, usage) ||
         outputIsUsedByValves(ctx, kind, index, -1, usage);
}

bool validateSingleOutputFreeForUse(const AppContext& ctx, OutputKind kind, uint8_t index, const String& newUsage, int ignoreCircuitIndex = -1, int ignorePumpIndex = -1, bool ignoreAuxHeater = false, bool ignoreOven = false, int ignoreValveIndex = -1) {
  if (!outputRefAssigned(kind, index)) return true;

  String usage;
  if (outputIsUsedByLegacyConsumers(ctx, kind, index, ignorePumpIndex, usage, ignoreAuxHeater, ignoreOven) ||
      outputIsUsedByHeatingCircuit(ctx, kind, index, ignoreCircuitIndex, usage) ||
      outputIsUsedByValves(ctx, kind, index, ignoreValveIndex, usage)) {
    server.send(409, "text/plain", outputRefLabel(kind, index) + " ist bereits belegt: " + usage + ". Neue Zuordnung: " + newUsage);
    return false;
  }

  return true;
}

bool validatePwmOutputModeChange(const AppContext& ctx, uint8_t index, const PwmOutputConfig& candidate) {
  if (index >= PWM_OUTPUT_COUNT) return false;

  String usage;
  const PwmOutputConfig& current = ctx.config.pwmOutputs[index];

  const bool usedAsSwitchOutput =
      outputIsUsedByHeatingCircuit(ctx, OutputKind::PWM_OUTPUT, index, -1, usage) ||
      outputIsUsedByValves(ctx, OutputKind::PWM_OUTPUT, index, -1, usage);

  if (usedAsSwitchOutput) {
    if (!candidate.enabled ||
        candidate.mode != PwmOutputMode::SWITCH ||
        candidate.mode != current.mode ||
        candidate.function != current.function) {
      server.send(409, "text/plain", pwmOutputHardwareLabel(index) + " ist bereits als Schaltausgang belegt: " + usage + ". Modus/Funktion kann nicht geändert werden.");
      return false;
    }
  }

  if ((!candidate.enabled || candidate.mode == PwmOutputMode::SWITCH) &&
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

bool validateRelayFunctionChange(const AppContext& ctx, uint8_t index, RelayFunction newFunction, bool newEnabled) {
  String usage;
  if (outputIsUsedByLegacyConsumers(ctx, OutputKind::RELAY, index, -1, usage) ||
      outputIsUsedByHeatingCircuit(ctx, OutputKind::RELAY, index, -1, usage) ||
      outputIsUsedByValves(ctx, OutputKind::RELAY, index, -1, usage)) {
    const RelayOutputConfig& current = ctx.config.relays[index];
    if (!newEnabled || newFunction != current.function) {
      server.send(409, "text/plain", relayHardwareLabel(index) + " ist bereits belegt: " + usage + ". Ausgang kann nicht deaktiviert und Funktion kann nicht geändert werden.");
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

      if (!validatePwmOutputModeChange(*s_ctx, (uint8_t)idx, candidate)) {
        return;
      }

      PwmOutputConfig old = s_ctx->config.pwmOutputs[idx];
      s_ctx->config.pwmOutputs[idx] = candidate;

      String validationError;
      if (!OutputValidation::validateAll(*s_ctx, validationError)) {
        s_ctx->config.pwmOutputs[idx] = old;
        server.send(409, "text/plain", validationError);
        return;
      }

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

    if (!validateRelayFunctionChange(*s_ctx, (uint8_t)idx, fn, enabled)) {
      return;
    }

    // Auf der Seite "Ausgaenge" wird nur die Faehigkeit des Ausgangs definiert.
    // Harte Doppelbelegung wird erst beim Zuordnen in Verbraucher-Menues geprueft.
    RelayOutputConfig old = s_ctx->config.relays[idx];
    RelayOutputs::configure(*s_ctx, (uint8_t)idx, enabled, fn, activeLow);

    String validationError;
    if (!OutputValidation::validateAll(*s_ctx, validationError)) {
      s_ctx->config.relays[idx] = old;
      RelayOutputs::set(*s_ctx, (uint8_t)idx, false);
      server.send(409, "text/plain", validationError);
      return;
    }

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

  String jsonEscape(const char* value) {
    String out;
    const char* p = value ? value : "";
    while (*p) {
      char c = *p++;
      if (c == '\\' || c == '"') {
        out += '\\';
        out += c;
      } else if (c == '\n') {
        out += "\\n";
      } else if (c == '\r') {
        out += "\\r";
      } else {
        out += c;
      }
    }
    return out;
  }

  String ipToString(const IPAddress& ip) {
    return String(ip[0]) + "." + String(ip[1]) + "." + String(ip[2]) + "." + String(ip[3]);
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

  int valveOutputUsedBy(const AppContext& ctx, OutputKind kind, uint8_t index, int ignoreValve) {
    if (!outputRefAssigned(kind, index)) return -1;
    for (uint8_t i = 0; i < MAX_VALVES; i++) {
      if ((int)i == ignoreValve) continue;
      const ValveConfig& v = ctx.config.valves[i];
      if (v.enabled && v.output.kind == kind && v.output.index == index) return i;
    }
    return -1;
  }

  int valveRelayUsedBy(const AppContext& ctx, uint8_t relayIndex, int ignoreValve) {
    return valveOutputUsedBy(ctx, OutputKind::RELAY, relayIndex, ignoreValve);
  }

  int pumpUsingValve(const AppContext& ctx, uint8_t valveIndex, int ignorePump = -1) {
    if (valveIndex == PIN_UNUSED) return -1;
    for (uint8_t i = 0; i < MAX_PUMPS; i++) {
      if ((int)i == ignorePump) continue;
      if (ctx.config.pumps[i].valveIndex == valveIndex) return i;
    }
    return -1;
  }

  int valveIndexForOutput(const AppContext& ctx, OutputKind kind, uint8_t index) {
    if (!outputRefAssigned(kind, index)) return -1;
    for (uint8_t i = 0; i < MAX_VALVES; i++) {
      const ValveConfig& v = ctx.config.valves[i];
      if (v.enabled && v.output.kind == kind && v.output.index == index) return i;
    }
    return -1;
  }

  int pumpUsingValveOutput(const AppContext& ctx, OutputKind kind, uint8_t index, int ignorePump = -1) {
    int valveIndex = valveIndexForOutput(ctx, kind, index);
    if (valveIndex < 0) return -1;
    return pumpUsingValve(ctx, (uint8_t)valveIndex, ignorePump);
  }

  int pumpUsingValveRelayOutput(const AppContext& ctx, uint8_t relayIndex, int ignorePump = -1) {
    return pumpUsingValveOutput(ctx, OutputKind::RELAY, relayIndex, ignorePump);
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

    if (candidate.valveIndex != PIN_UNUSED) {
      if (candidate.valveIndex >= MAX_VALVES || !Valves::isConfigured(ctx, candidate.valveIndex)) {
        error = "Ventil ist nicht aktiv oder nicht vollstaendig konfiguriert";
        return false;
      }
      const int usedBy = pumpUsingValve(ctx, candidate.valveIndex, pumpIndex);
      if (usedBy >= 0) {
        error = "Ventil wird bereits von Pumpe " + String(usedBy + 1) + " verwendet";
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
      const bool pumpValveIndexValid = (p.valveIndex != PIN_UNUSED && p.valveIndex < MAX_VALVES);
      const bool pumpValveConfigured = (pumpValveIndexValid && Valves::isConfigured(*s_ctx, p.valveIndex));
      const ValveConfig& pv = pumpValveIndexValid ? s_ctx->config.valves[p.valveIndex] : s_ctx->config.valves[0];
      String pumpValveLabel = pumpValveIndexValid
        ? String("Ventil ") + String(p.valveIndex + 1) + String(" / ") + outputRefLabel(pv.output.kind, pv.output.index)
        : String("");
      String activeTargetName = String("");
      String activeTargetSinkRole = String("none");
      String activeTargetSinkLabel = String("");
      if (p.activeTargetIndex < PUMP_ROUTE_TARGET_COUNT) {
        const PumpRouteTargetConfig& activeTarget = p.targets[p.activeTargetIndex];
        activeTargetName = String("Ziel ") + String(p.activeTargetIndex == 0 ? "A" : "B");
        activeTargetSinkRole = SensorRoles::toKey(activeTarget.sinkRole);
        activeTargetSinkLabel = SensorRoles::toLabel(activeTarget.sinkRole);
      }
      json += "\"valveIndex\":" + String(p.valveIndex) + ",";
      json += "\"hasValve\":" + String(pumpValveConfigured ? "true" : "false") + ",";
      json += "\"valveAssigned\":" + String(pumpValveIndexValid ? "true" : "false") + ",";
      json += "\"valveConfigured\":" + String(pumpValveConfigured ? "true" : "false") + ",";
      json += "\"valveLabel\":\"" + pumpValveLabel + "\",";
      json += "\"valveCurrentPosition\":\"" + String(pumpValveConfigured ? Valves::positionToKey(Valves::currentPosition(p.valveIndex)) : "-") + "\",";
      json += "\"valveTargetPosition\":\"" + String(pumpValveConfigured ? Valves::positionToKey(Valves::targetPosition(p.valveIndex)) : "-") + "\",";
      json += "\"valveMoving\":" + String((pumpValveConfigured && Valves::isMoving(p.valveIndex)) ? "true" : "false") + ",";
      json += "\"valveMoveRemainingMs\":" + String(pumpValveConfigured ? Valves::moveRemainingMs(*s_ctx, p.valveIndex) : 0) + ",";
      json += "\"valvePendingTargetIndex\":" + String(p.valvePendingTargetIndex) + ",";
      // Kompatibilitaet fuer alte pumps.js-Versionen: Werte werden aus Valve V2 abgeleitet,
      // aber nicht mehr als PumpConfig-Felder gespeichert.
      json += "\"switchValveEnabled\":" + String(pumpValveConfigured ? "true" : "false") + ",";
      json += "\"switchValveRelayIndex\":" + String((pumpValveConfigured && pv.output.kind == OutputKind::RELAY) ? pv.output.index : PIN_UNUSED) + ",";
      json += "\"switchValveTravelTimeMs\":" + String(pumpValveConfigured ? pv.travelTimeMs : 0) + ",";
      json += "\"switchValveMoving\":" + String((pumpValveConfigured && Valves::isMoving(p.valveIndex)) ? "true" : "false") + ",";
      json += "\"switchValveStateForTargetA\":" + String((pumpValveConfigured && !pv.activeHighForB) ? "true" : "false") + ",";
      json += "\"activeTargetIndex\":" + String(p.activeTargetIndex) + ",";
      json += "\"activeTargetName\":\"" + activeTargetName + "\",";
      json += "\"activeTargetSinkRole\":\"" + activeTargetSinkRole + "\",";
      json += "\"activeTargetSinkLabel\":\"" + activeTargetSinkLabel + "\",";
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
      json += "{\"index\":" + String(i) + ",\"label\":\"" + relayHardwareLabel(i) + "\",\"valveIndex\":" + String(valveIndexForOutput(*s_ctx, OutputKind::RELAY, i)) + ",\"usedBy\":" + String(pumpUsingValveRelayOutput(*s_ctx, i, -1)) + "}";
    }

    json += "],\"activeValves\":[";
    first = true;
    for (uint8_t i = 0; i < MAX_VALVES; i++) {
      if (!Valves::isConfigured(*s_ctx, i)) continue;
      const ValveConfig& v = s_ctx->config.valves[i];
      if (!first) json += ",";
      first = false;
      const int usedBy = pumpUsingValve(*s_ctx, i, -1);
      json += "{\"index\":" + String(i) + ",";
      json += "\"name\":\"Ventil " + String(i + 1) + "\",";
      json += "\"label\":\"Ventil " + String(i + 1) + " / " + outputRefLabel(v.output.kind, v.output.index) + "\",";
      json += "\"outputLabel\":\"" + outputRefLabel(v.output.kind, v.output.index) + "\",";
      json += "\"usedBy\":" + String(usedBy);
      json += "}";
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
        p.valveIndex = PIN_UNUSED;
        p.sinkRole = SensorRoles::fromKey(targetSelect.substring(5));
      } else if (targetSelect.startsWith("valve:")) {
        // Ab Valve V2 sendet die Pumpen-UI hier den valveIndex, nicht mehr den Relaisindex.
        // Fallback fuer ganz alte SD-Dateien: falls kein passendes Ventil an diesem Index existiert,
        // wird der Wert noch als Relaisindex interpretiert.
        const uint8_t requestedValveIndex = (uint8_t)targetSelect.substring(6).toInt();
        int resolvedValveIndex = -1;
        if (requestedValveIndex < MAX_VALVES && Valves::isConfigured(*s_ctx, requestedValveIndex)) {
          resolvedValveIndex = requestedValveIndex;
        } else {
          resolvedValveIndex = valveIndexForOutput(*s_ctx, OutputKind::RELAY, requestedValveIndex);
        }
        if (resolvedValveIndex < 0 || resolvedValveIndex >= MAX_VALVES || !Valves::isConfigured(*s_ctx, (uint8_t)resolvedValveIndex)) {
          p = old;
          server.send(409, "text/plain", "Ventil nicht aktiv oder nicht vollstaendig konfiguriert. Bitte Ventil auf der Seite Ventile aktivieren und Ausgang zuordnen.");
          return;
        }
        int usedBy = pumpUsingValve(*s_ctx, (uint8_t)resolvedValveIndex, idx);
        if (usedBy >= 0) {
          p = old;
          server.send(409, "text/plain", "Ventil wird bereits von Pumpe " + String(usedBy + 1) + " verwendet");
          return;
        }
        p.sinkRole = Ds18Role::NONE;
        p.valveIndex = (uint8_t)resolvedValveIndex;
      }
    }

    if (server.hasArg("relayIndex")) p.relayIndex = (uint8_t)server.arg("relayIndex").toInt();
    if (server.hasArg("pwmChannel")) p.pwmChannel = (uint8_t)server.arg("pwmChannel").toInt();
    if (server.hasArg("pwmProfile")) p.pwmProfile = (server.arg("pwmProfile") == "heating") ? PwmProfile::HEATING : PwmProfile::SOLAR;
    if (server.hasArg("feedbackPin")) p.feedbackPin = (uint8_t)server.arg("feedbackPin").toInt();
    if (server.hasArg("valveIndex")) {
      uint8_t requestedValveIndex = (uint8_t)server.arg("valveIndex").toInt();
      if (requestedValveIndex == PIN_UNUSED) {
        p.valveIndex = PIN_UNUSED;
      } else if (requestedValveIndex < MAX_VALVES && Valves::isConfigured(*s_ctx, requestedValveIndex)) {
        const int usedBy = pumpUsingValve(*s_ctx, requestedValveIndex, idx);
        if (usedBy >= 0) {
          p = old;
          server.send(409, "text/plain", "Ventil wird bereits von Pumpe " + String(usedBy + 1) + " verwendet");
          return;
        }
        p.sinkRole = Ds18Role::NONE;
        p.valveIndex = requestedValveIndex;
      } else {
        p = old;
        server.send(409, "text/plain", "Ventil nicht aktiv oder nicht vollstaendig konfiguriert");
        return;
      }
    }

    if (server.hasArg("targetDiff")) p.targetDiff = server.arg("targetDiff").toFloat();
    if (server.hasArg("hysteresis")) p.hysteresis = server.arg("hysteresis").toFloat();
    if (server.hasArg("startDiff")) p.startDiff = server.arg("startDiff").toFloat();
    if (server.hasArg("pidKp")) p.pidKp = server.arg("pidKp").toFloat();
    if (server.hasArg("pidKi")) p.pidKi = server.arg("pidKi").toFloat();
    if (server.hasArg("pidKd")) p.pidKd = server.arg("pidKd").toFloat();
    if (server.hasArg("minPwmPercent")) p.minPwmPercent = server.arg("minPwmPercent").toFloat();
    if (server.hasArg("maxPwmPercent")) p.maxPwmPercent = server.arg("maxPwmPercent").toFloat();

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

  String testOutputRefKind(const OutputRef& ref) {
    return outputRefKindToKey(ref.kind);
  }

  String testOutputRefLabel(const OutputRef& ref) {
    return outputRefLabel(ref.kind, ref.index);
  }


  String valvePositionToKey(ValvePosition pos) {
    return (pos == ValvePosition::B) ? "B" : "A";
  }

  void testSetOutputRef(AppContext& ctx, const OutputRef& ref, bool on, uint8_t percent = 100) {
    if (ref.kind == OutputKind::RELAY && ref.index < RELAY_COUNT) {
      RelayOutputs::set(ctx, ref.index, on);
      return;
    }

    if (ref.kind == OutputKind::PWM_OUTPUT && ref.index < PWM_OUTPUT_COUNT) {
      const PwmOutputConfig& po = ctx.config.pwmOutputs[ref.index];
      if (po.mode == PwmOutputMode::PWM) {
        PwmDriver::setDuty(ref.index, on ? percent : 0, po.profile);
      } else {
        PwmDriver::setSwitch(ref.index, on, po.profile);
      }
    }
  }

  void appendTestSafetyJson(String& json) {
    const SafetyManager::SafetyStatus& st = SafetyManager::status();
    json += "\"safety\":{";
    json += "\"mode\":\"" + String(SafetyManager::modeToText(st.mode)) + "\",";
    json += "\"message\":\"" + String(st.message) + "\",";
    json += "\"frostProtectionActive\":" + String(st.frostProtectionActive ? "true" : "false") + ",";
    json += "\"collectorStagnationActive\":" + String(st.collectorStagnationActive ? "true" : "false") + ",";
    json += "\"storageOvertemperatureActive\":" + String(st.storageOvertemperatureActive ? "true" : "false") + ",";
    json += "\"ovenOvertemperatureActive\":" + String(st.ovenOvertemperatureActive ? "true" : "false") + ",";
    json += "\"sensorFaultActive\":" + String(st.sensorFaultActive ? "true" : "false") + ",";
    json += "\"blockNormalPumpControl\":" + String(st.blockNormalPumpControl ? "true" : "false") + ",";
    json += "\"forceSolarHeatDump\":" + String(st.forceSolarHeatDump ? "true" : "false") + ",";
    json += "\"forceOvenPumpRun\":" + String(st.forceOvenPumpRun ? "true" : "false") + ",";
    json += "\"controlledStagnation\":" + String(st.controlledStagnation ? "true" : "false") + ",";
    json += "\"lowestCollectorTemperatureC\":" + jsonFloat(st.lowestCollectorTemperatureC, 1) + ",";
    json += "\"highestCollectorTemperatureC\":" + jsonFloat(st.highestCollectorTemperatureC, 1) + ",";
    json += "\"highestStorageTemperatureC\":" + jsonFloat(st.highestStorageTemperatureC, 1) + ",";
    json += "\"frostStorageTemperatureC\":" + jsonFloat(st.frostStorageTemperatureC, 1) + ",";
    json += "\"ovenTemperatureC\":" + jsonFloat(st.ovenTemperatureC, 1) + ",";
    json += "\"nightCoolingActive\":" + String(st.safetyNightCoolingActive ? "true" : "false") + ",";
    json += "\"nightCoolingCollectorTemperatureC\":" + jsonFloat(st.nightCoolingCollectorTemperatureC, 1) + ",";
    json += "\"nightCoolingStorageTemperatureC\":" + jsonFloat(st.nightCoolingStorageTemperatureC, 1) + ",";
    json += "\"nightCoolingDeltaC\":" + jsonFloat(st.nightCoolingDeltaC, 1) + ",";
    json += "\"nightCoolingPumpPercent\":" + jsonFloat(st.nightCoolingPumpPercent, 0);
    json += "}";
  }


  String valveOutputLabel(const OutputRef& ref) {
    return outputRefLabel(ref.kind, ref.index);
  }

  void appendValveOutputOptions(String& json, const AppContext& ctx) {
    json += "\"outputOptions\":[";
    bool first = true;

    for (uint8_t i = 0; i < RELAY_COUNT; i++) {
      const RelayOutputConfig& r = ctx.config.relays[i];
      if (!r.enabled || r.function != RelayFunction::ZONE_VALVE) continue;
      if (!first) json += ",";
      first = false;
      json += "{\"kind\":\"relay\",\"index\":" + String(i) + ",\"label\":\"" + relayHardwareLabel(i) + "\",\"usedBy\":" + String(valveRelayUsedBy(ctx, i, -1)) + "}";
    }

    for (uint8_t i = 0; i < PWM_OUTPUT_COUNT; i++) {
      const PwmOutputConfig& po = ctx.config.pwmOutputs[i];
      if (!po.enabled || po.mode != PwmOutputMode::SWITCH || po.function != RelayFunction::ZONE_VALVE) continue;
      if (!first) json += ",";
      first = false;
      json += "{\"kind\":\"pwm\",\"index\":" + String(i) + ",\"label\":\"" + pwmOutputHardwareLabel(i) + "\",\"usedBy\":" + String(valveOutputUsedBy(ctx, OutputKind::PWM_OUTPUT, i, -1)) + "}";
    }

    json += "]";
  }

  void handleValvesJson() {
    if (!s_ctx) {
      server.send(500, "application/json", "{\"error\":\"no_context\"}");
      return;
    }

    String json = "{";
    json += "\"valves\":[";
    for (uint8_t i = 0; i < MAX_VALVES; i++) {
      if (i > 0) json += ",";
      const ValveConfig& v = s_ctx->config.valves[i];
      json += "{";
      json += "\"index\":" + String(i) + ",";
      json += "\"name\":\"Ventil " + String(i + 1) + "\",";
      json += "\"enabled\":" + String(v.enabled ? "true" : "false") + ",";
      json += "\"outputKind\":\"" + outputRefKindToKey(v.output.kind) + "\",";
      json += "\"outputIndex\":" + String(v.output.index) + ",";
      json += "\"outputLabel\":\"" + valveOutputLabel(v.output) + "\",";
      json += "\"travelTimeMs\":" + String(v.travelTimeMs) + ",";
      json += "\"activeHighForB\":" + String(v.activeHighForB ? "true" : "false") + ",";
      json += "\"safetyPosition\":\"" + String(Valves::positionToKey(v.safetyPosition)) + "\",";
      json += "\"lastRequestedPosition\":\"" + String(Valves::positionToKey(v.lastRequestedPosition)) + "\",";
      json += "\"current\":\"" + String(Valves::positionToKey(Valves::currentPosition(i))) + "\",";
      json += "\"target\":\"" + String(Valves::positionToKey(Valves::targetPosition(i))) + "\",";
      json += "\"moving\":" + String(Valves::isMoving(i) ? "true" : "false") + ",";
      json += "\"moveRemainingMs\":" + String(Valves::moveRemainingMs(*s_ctx, i)) + ",";
      json += "\"usedByPump\":" + String(pumpUsingValve(*s_ctx, i));
      json += "}";
    }

    json += "],\"pumps\":[";
    for (uint8_t i = 0; i < MAX_PUMPS; i++) {
      if (i > 0) json += ",";
      const PumpConfig& p = s_ctx->config.pumps[i];
      json += "{";
      json += "\"index\":" + String(i) + ",";
      json += "\"name\":\"Pumpe " + String(i + 1) + "\",";
      json += "\"enabled\":" + String(p.enabled ? "true" : "false") + ",";
      json += "\"sourceLabel\":\"" + String(p.sourceType == PumpSourceType::SENSOR_ROLE ? SensorRoles::toLabel(p.sourceSensorRole) : HeatSourceRoles::toLabel(p.sourceRole)) + "\",";
      json += "\"valveIndex\":" + String(p.valveIndex) + ",";
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
      json += "]}";
    }

    json += "],";
    appendValveOutputOptions(json, *s_ctx);

    json += ",\"sinkRoles\":[";
    bool firstSink = true;
    appendActiveDs18RoleOptions(json, *s_ctx, firstSink);
    json += "]}";

    server.send(200, "application/json", json);
  }

  void handleValveConfig() {
    if (!serviceSessionValid()) {
      server.send(403, "text/plain", "Nicht erlaubt");
      return;
    }
    if (!s_ctx) {
      server.send(500, "text/plain", "Kein Kontext");
      return;
    }
    if (!server.hasArg("pumpIndex")) {
      server.send(400, "text/plain", "pumpIndex fehlt");
      return;
    }

    const int pumpIdx = server.arg("pumpIndex").toInt();
    if (pumpIdx < 0 || pumpIdx >= MAX_PUMPS) {
      server.send(400, "text/plain", "ungueltige Pumpe");
      return;
    }

    ConfigData oldConfig = s_ctx->config;
    PumpConfig& pump = s_ctx->config.pumps[pumpIdx];

    uint8_t valveIndex = PIN_UNUSED;
    if (server.hasArg("valveIndex")) valveIndex = (uint8_t)server.arg("valveIndex").toInt();

    if (valveIndex == PIN_UNUSED) {
      pump.valveIndex = PIN_UNUSED;
    } else {
      if (valveIndex >= MAX_VALVES) {
        server.send(400, "text/plain", "ungueltiges Ventil");
        return;
      }

      const int usedBy = pumpUsingValve(*s_ctx, valveIndex, pumpIdx);
      if (usedBy >= 0) {
        server.send(409, "text/plain", "Ventil wird bereits von Pumpe " + String(usedBy + 1) + " verwendet");
        return;
      }

      ValveConfig& valve = s_ctx->config.valves[valveIndex];
      valve.enabled = server.arg("enabled").toInt() != 0;
      valve.output.kind = outputKindFromKey(server.arg("outputKind"));
      valve.output.index = server.hasArg("outputIndex") ? (uint8_t)server.arg("outputIndex").toInt() : PIN_UNUSED;
      valve.travelTimeMs = server.hasArg("travelTimeMs") ? (uint32_t)server.arg("travelTimeMs").toInt() : valve.travelTimeMs;
      valve.activeHighForB = server.arg("activeHighForB").toInt() != 0;
      valve.safetyPosition = Valves::positionFromKey(server.arg("safetyPosition"));

      if (valve.enabled) {
        if (!outputExistsAndMatchesFunction(*s_ctx, valve.output.kind, valve.output.index, RelayFunction::ZONE_VALVE, false)) {
          s_ctx->config = oldConfig;
          server.send(409, "text/plain", "Ventilausgang ist nicht als Zonenventil konfiguriert");
          return;
        }
      }

      pump.valveIndex = valve.enabled ? valveIndex : PIN_UNUSED;
    }

    for (uint8_t t = 0; t < PUMP_ROUTE_TARGET_COUNT; t++) {
      const String prefix = "target" + String(t) + "_";
      if (server.hasArg(prefix + "enabled")) pump.targets[t].enabled = (server.arg(prefix + "enabled").toInt() != 0);
      if (server.hasArg(prefix + "sinkRole")) pump.targets[t].sinkRole = SensorRoles::fromKey(server.arg(prefix + "sinkRole"));
      if (server.hasArg(prefix + "targetDiffOverride")) pump.targets[t].targetDiffOverride = server.arg(prefix + "targetDiffOverride").toFloat();
      if (server.hasArg(prefix + "hysteresisOverride")) pump.targets[t].hysteresisOverride = server.arg(prefix + "hysteresisOverride").toFloat();
      if (server.hasArg(prefix + "minTempC")) pump.targets[t].minTempC = server.arg(prefix + "minTempC").toFloat();
      if (server.hasArg(prefix + "maxTempC")) pump.targets[t].maxTempC = server.arg(prefix + "maxTempC").toFloat();
    }

    String validationError;
    if (!OutputValidation::validateAll(*s_ctx, validationError)) {
      s_ctx->config = oldConfig;
      server.send(409, "text/plain", validationError);
      return;
    }

    Storage::saveConfig(s_ctx->config);
    server.send(200, "text/plain", "OK");
  }


  bool validRouteTargetIndex(uint8_t targetIndex) {
    return targetIndex < PUMP_ROUTE_TARGET_COUNT;
  }

  const char* routeTargetName(uint8_t targetIndex) {
    if (targetIndex == 0) return "Ziel A";
    if (targetIndex == 1) return "Ziel B";
    return "-";
  }

  ValvePosition routeTargetValvePosition(uint8_t targetIndex) {
    return (targetIndex == 1) ? ValvePosition::B : ValvePosition::A;
  }

  bool pumpHasConfiguredValve(const AppContext& ctx, const PumpConfig& pump) {
    return pump.valveIndex != PIN_UNUSED && pump.valveIndex < MAX_VALVES && Valves::isConfigured(ctx, pump.valveIndex);
  }

  void forcePumpOutputOffForTest(AppContext& ctx, PumpConfig& pump) {
    if (pump.relayIndex != PIN_UNUSED && pump.relayIndex < RELAY_COUNT) {
      RelayOutputs::set(ctx, pump.relayIndex, false);
    }

    if (pump.mode == PumpMode::PWM && pump.pwmChannel != PIN_UNUSED && pump.pwmChannel < PWM_OUTPUT_COUNT) {
      PwmDriver::setDuty(pump.pwmChannel, 0, pump.pwmProfile);
    }

    pump.state = false;
    pump.lastPwmPercent = 0.0f;
  }

  void forcePumpOutputOnForTest(AppContext& ctx, PumpConfig& pump, uint8_t percent) {
    if (pump.relayIndex != PIN_UNUSED && pump.relayIndex < RELAY_COUNT) {
      RelayOutputs::set(ctx, pump.relayIndex, true);
    }

    if (pump.mode == PumpMode::PWM && pump.pwmChannel != PIN_UNUSED && pump.pwmChannel < PWM_OUTPUT_COUNT) {
      if (percent > 100) percent = 100;
      PwmDriver::setDuty(pump.pwmChannel, percent, pump.pwmProfile);
      pump.lastPwmPercent = percent;
    } else {
      pump.lastPwmPercent = 100.0f;
    }

    pump.state = true;
  }

  void updatePumpRouteTestRuntime(AppContext& ctx) {
    Valves::process(ctx);

    for (uint8_t i = 0; i < MAX_PUMPS; i++) {
      PumpConfig& pump = ctx.config.pumps[i];
      if (!pumpHasConfiguredValve(ctx, pump)) continue;

      if (Valves::isMoving(pump.valveIndex)) {
        forcePumpOutputOffForTest(ctx, pump);
        continue;
      }

      if (validRouteTargetIndex(pump.valvePendingTargetIndex)) {
        const uint8_t targetIndex = pump.valvePendingTargetIndex;
        pump.activeTargetIndex = targetIndex;
        pump.valvePendingTargetIndex = PIN_UNUSED;
        for (uint8_t t = 0; t < PUMP_ROUTE_TARGET_COUNT; t++) {
          pump.targets[t].active = (t == targetIndex);
        }
      }
    }
  }

  void handleTestPumpRoute() {
    if (!commissioningTestAuthorized()) {
      server.send(403, "text/plain", "Testmodus nicht aktiv oder Inbetriebnahme-PIN falsch");
      return;
    }
    if (!s_ctx) {
      server.send(500, "text/plain", "Kein Kontext");
      return;
    }
    if (!server.hasArg("pumpIndex") || !server.hasArg("targetIndex")) {
      server.send(400, "text/plain", "pumpIndex/targetIndex fehlt");
      return;
    }

    const int pumpIdx = server.arg("pumpIndex").toInt();
    const int targetIdx = server.arg("targetIndex").toInt();
    if (pumpIdx < 0 || pumpIdx >= MAX_PUMPS || targetIdx < 0 || targetIdx >= PUMP_ROUTE_TARGET_COUNT) {
      server.send(400, "text/plain", "ungueltige Pumpe oder ungueltiges Ziel");
      return;
    }

    PumpConfig& pump = s_ctx->config.pumps[pumpIdx];
    if (!pumpHasConfiguredValve(*s_ctx, pump)) {
      server.send(409, "text/plain", "Pumpe hat kein aktives Ventil als Ziel");
      return;
    }

    PumpRouteTargetConfig& target = pump.targets[targetIdx];
    if (!target.enabled || target.sinkRole == Ds18Role::NONE) {
      server.send(409, "text/plain", "Ziel ist nicht aktiv oder hat keinen Sensor");
      return;
    }

    forcePumpOutputOffForTest(*s_ctx, pump);
    pump.valvePendingTargetIndex = (uint8_t)targetIdx;
    for (uint8_t t = 0; t < PUMP_ROUTE_TARGET_COUNT; t++) {
      pump.targets[t].active = false;
    }

    if (!Valves::requestPosition(*s_ctx, pump.valveIndex, routeTargetValvePosition((uint8_t)targetIdx))) {
      pump.valvePendingTargetIndex = PIN_UNUSED;
      server.send(409, "text/plain", "Ventil konnte nicht auf das Ziel gefahren werden");
      return;
    }

    updatePumpRouteTestRuntime(*s_ctx);
    server.send(200, "text/plain", String(routeTargetName((uint8_t)targetIdx)) + " angefordert");
  }

  void handleTestPumpOutput() {
    if (!commissioningTestAuthorized()) {
      server.send(403, "text/plain", "Testmodus nicht aktiv oder Inbetriebnahme-PIN falsch");
      return;
    }
    if (!s_ctx) {
      server.send(500, "text/plain", "Kein Kontext");
      return;
    }
    if (!server.hasArg("pumpIndex") || !server.hasArg("state")) {
      server.send(400, "text/plain", "pumpIndex/state fehlt");
      return;
    }

    const int pumpIdx = server.arg("pumpIndex").toInt();
    if (pumpIdx < 0 || pumpIdx >= MAX_PUMPS) {
      server.send(400, "text/plain", "ungueltige Pumpe");
      return;
    }

    PumpConfig& pump = s_ctx->config.pumps[pumpIdx];
    const bool on = server.arg("state").toInt() != 0;
    int percent = server.hasArg("percent") ? server.arg("percent").toInt() : 100;
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;

    if (!on) {
      forcePumpOutputOffForTest(*s_ctx, pump);
      server.send(200, "text/plain", "OK");
      return;
    }

    if (!pump.enabled) {
      server.send(409, "text/plain", "Pumpe ist nicht aktiviert");
      return;
    }

    if (pump.relayIndex == PIN_UNUSED || pump.relayIndex >= RELAY_COUNT ||
        !RelayOutputs::isUsableAsPumpEnable(*s_ctx, pump.relayIndex)) {
      server.send(409, "text/plain", "Pumpenrelais ist nicht als Pump Enable konfiguriert");
      return;
    }

    if (pump.mode == PumpMode::PWM) {
      if (pump.pwmChannel == PIN_UNUSED || pump.pwmChannel >= PWM_OUTPUT_COUNT) {
        server.send(409, "text/plain", "PWM-Ausgang ist nicht konfiguriert");
        return;
      }
    }

    if (pumpHasConfiguredValve(*s_ctx, pump)) {
      updatePumpRouteTestRuntime(*s_ctx);
      if (Valves::isMoving(pump.valveIndex)) {
        forcePumpOutputOffForTest(*s_ctx, pump);
        server.send(409, "text/plain", "Ventil faehrt noch - Pumpenausgang bleibt gesperrt");
        return;
      }
      if (!validRouteTargetIndex(pump.activeTargetIndex)) {
        server.send(409, "text/plain", "Kein aktives Routing-Ziel. Zuerst Ziel A oder B anfordern.");
        return;
      }
    }

    forcePumpOutputOnForTest(*s_ctx, pump, (uint8_t)percent);
    server.send(200, "text/plain", "OK");
  }

  void handleTestOverviewJson() {
    if (!s_ctx) {
      server.send(500, "application/json", "{\"error\":\"no_context\"}");
      return;
    }

    if (g_commissioningTestActive) {
      updatePumpRouteTestRuntime(*s_ctx);
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
      json += "{";
      json += "\"index\":" + String(ch) + ",";
      json += "\"label\":\"" + pwmOutputHardwareLabel(ch) + "\",";
      json += "\"mode\":\"" + pwmOutputModeToKey(out.mode) + "\",";
      json += "\"modeLabel\":\"" + pwmOutputModeLabel(out.mode) + "\",";
      json += "\"function\":\"" + String(RelayOutputs::functionToKey(out.function)) + "\",";
      json += "\"profile\":\"" + pwmProfileToKey(out.profile) + "\",";
      json += "\"enabled\":" + String(out.enabled ? "true" : "false") + ",";
      json += "\"percent\":" + String(PwmDriver::getDuty(ch));
      json += "}";
    }

    json += "],\"servo\":{";
    json += "\"available\":true,";
    json += "\"label\":\"Ofenklappen-Servo\",";
    json += "\"pin\":" + String(OVEN_SERVO_PIN) + ",";
    json += "\"initialized\":" + String(ServoDriver::initialized() ? "true" : "false") + ",";
    json += "\"angle\":" + String(ServoDriver::angle());
    json += "}";

    json += ",\"pumps\":[";
    for (uint8_t i = 0; i < MAX_PUMPS; i++) {
      if (i > 0) json += ",";
      const PumpConfig& pump = s_ctx->config.pumps[i];
      const bool hasConfiguredValve = pumpHasConfiguredValve(*s_ctx, pump);
      String configuredTargetLabel = String(pump.sinkRole == Ds18Role::NONE ? "Priorisierter Sink" : SensorRoles::toLabel(pump.sinkRole));
      if (hasConfiguredValve) {
        configuredTargetLabel = String("Ventil ") + String(pump.valveIndex + 1) + String(" / ") + testOutputRefLabel(Valves::outputRef(*s_ctx, pump.valveIndex));
      }

      String activeTargetName = String("-");
      String activeTargetLabel = String("-");
      if (validRouteTargetIndex(pump.activeTargetIndex)) {
        const PumpRouteTargetConfig& activeTarget = pump.targets[pump.activeTargetIndex];
        activeTargetName = routeTargetName(pump.activeTargetIndex);
        activeTargetLabel = SensorRoles::toLabel(activeTarget.sinkRole);
      }

      json += "{";
      json += "\"index\":" + String(i) + ",";
      json += "\"label\":\"Pumpe " + String(i + 1) + "\",";
      json += "\"enabled\":" + String(pump.enabled ? "true" : "false") + ",";
      json += "\"mode\":\"" + pumpModeToKey(pump.mode) + "\",";
      json += "\"sourceType\":\"" + String(pump.sourceType == PumpSourceType::SENSOR_ROLE ? "sensor" : "heat_source") + "\",";
      json += "\"sourceLabel\":\"" + String(pump.sourceType == PumpSourceType::SENSOR_ROLE ? SensorRoles::toLabel(pump.sourceSensorRole) : HeatSourceRoles::toLabel(pump.sourceRole)) + "\",";
      json += "\"sinkLabel\":\"" + configuredTargetLabel + "\",";
      json += "\"targetLabel\":\"" + configuredTargetLabel + "\",";
      json += "\"relayIndex\":" + String(pump.relayIndex) + ",";
      json += "\"pwmChannel\":" + String(pump.pwmChannel) + ",";
      json += "\"feedbackPin\":" + String(pump.feedbackPin) + ",";
      json += "\"state\":" + String(pump.state ? "true" : "false") + ",";
      json += "\"lastSourceC\":" + jsonFloat(pump.lastSourceC, 1) + ",";
      json += "\"lastSinkC\":" + jsonFloat(pump.lastSinkC, 1) + ",";
      json += "\"lastDiffC\":" + jsonFloat(pump.lastDiffC, 1) + ",";
      json += "\"lastPwmPercent\":" + jsonFloat(pump.lastPwmPercent, 0) + ",";
      json += "\"feedbackSignalPresent\":" + String(pump.feedbackSignalPresent ? "true" : "false") + ",";
      json += "\"feedbackError\":" + String(pump.feedbackError ? "true" : "false") + ",";
      json += "\"feedbackDutyPercent\":" + jsonFloat(pump.feedbackDutyPercent, 1) + ",";
      json += "\"valveAssigned\":" + String(hasConfiguredValve ? "true" : "false") + ",";
      json += "\"valveIndex\":" + String(pump.valveIndex) + ",";
      json += "\"valveLabel\":\"" + String(hasConfiguredValve ? configuredTargetLabel : "") + "\",";
      json += "\"valveCurrentPosition\":\"" + String(hasConfiguredValve ? Valves::positionToKey(Valves::currentPosition(pump.valveIndex)) : "-") + "\",";
      json += "\"valveTargetPosition\":\"" + String(hasConfiguredValve ? Valves::positionToKey(Valves::targetPosition(pump.valveIndex)) : "-") + "\",";
      json += "\"valveMoving\":" + String((hasConfiguredValve && Valves::isMoving(pump.valveIndex)) ? "true" : "false") + ",";
      json += "\"valveMoveRemainingMs\":" + String(hasConfiguredValve ? Valves::moveRemainingMs(*s_ctx, pump.valveIndex) : 0) + ",";
      json += "\"valvePendingTargetIndex\":" + String(pump.valvePendingTargetIndex) + ",";
      json += "\"activeTargetIndex\":" + String(pump.activeTargetIndex) + ",";
      json += "\"activeTargetName\":\"" + activeTargetName + "\",";
      json += "\"activeTargetLabel\":\"" + activeTargetLabel + "\",";
      json += "\"targets\":[";
      for (uint8_t t = 0; t < PUMP_ROUTE_TARGET_COUNT; t++) {
        if (t > 0) json += ",";
        const PumpRouteTargetConfig& target = pump.targets[t];
        json += "{";
        json += "\"index\":" + String(t) + ",";
        json += "\"name\":\"" + String(routeTargetName(t)) + "\",";
        json += "\"enabled\":" + String(target.enabled ? "true" : "false") + ",";
        json += "\"sinkRole\":\"" + String(SensorRoles::toKey(target.sinkRole)) + "\",";
        json += "\"sinkLabel\":\"" + String(SensorRoles::toLabel(target.sinkRole)) + "\",";
        json += "\"active\":" + String(target.active ? "true" : "false") + ",";
        json += "\"lastSinkC\":" + jsonFloat(target.lastSinkC, 1) + ",";
        json += "\"lastDiffC\":" + jsonFloat(target.lastDiffC, 1);
        json += "}";
      }
      json += "]";
      json += "}";
    }


    json += "],\"valves\":[";
    for (uint8_t i = 0; i < MAX_VALVES; i++) {
      if (i > 0) json += ",";
      const ValveConfig& valve = s_ctx->config.valves[i];
      json += "{";
      json += "\"index\":" + String(i) + ",";
      json += "\"label\":\"Ventil " + String(i + 1) + "\",";
      json += "\"enabled\":" + String(valve.enabled ? "true" : "false") + ",";
      json += "\"outputKind\":\"" + testOutputRefKind(valve.output) + "\",";
      json += "\"outputIndex\":" + String(valve.output.index) + ",";
      json += "\"outputLabel\":\"" + testOutputRefLabel(valve.output) + "\",";
      json += "\"travelTimeMs\":" + String(valve.travelTimeMs) + ",";
      json += "\"current\":\"" + valvePositionToKey(Valves::currentPosition(i)) + "\",";
      json += "\"target\":\"" + valvePositionToKey(Valves::targetPosition(i)) + "\",";
      json += "\"moving\":" + String(Valves::isMoving(i) ? "true" : "false") + ",";
      json += "\"safetyPosition\":\"" + valvePositionToKey(valve.safetyPosition) + "\"";
      json += "}";
    }

    json += "],\"heatingCircuits\":[";
    for (uint8_t i = 0; i < MAX_HEATING_CIRCUITS; i++) {
      if (i > 0) json += ",";
      const HeatingCircuitConfig& cfg = s_ctx->config.heatingCircuits[i];
      const HeatingCircuitRuntime& rt = s_ctx->heatingCircuitRuntime[i];
      json += "{";
      json += "\"index\":" + String(i) + ",";
      json += "\"label\":\"HK" + String(i + 1) + "\",";
      json += "\"enabled\":" + String(cfg.enabled ? "true" : "false") + ",";
      json += "\"mixerOpenLabel\":\"" + testOutputRefLabel(cfg.mixerOpenOutput) + "\",";
      json += "\"mixerCloseLabel\":\"" + testOutputRefLabel(cfg.mixerCloseOutput) + "\",";
      json += "\"pumpLabel\":\"" + testOutputRefLabel(cfg.pumpOutput) + "\",";
      json += "\"pumpMode\":" + String((int)cfg.pumpMode) + ",";
      json += "\"active\":" + String(rt.active ? "true" : "false") + ",";
      json += "\"pumpActive\":" + String(rt.pumpActive ? "true" : "false") + ",";
      json += "\"opening\":" + String(rt.opening ? "true" : "false") + ",";
      json += "\"closing\":" + String(rt.closing ? "true" : "false") + ",";
      json += "\"flowTemperatureC\":" + jsonFloat(rt.flowTemperatureC, 1) + ",";
      json += "\"returnTemperatureC\":" + jsonFloat(rt.returnTemperatureC, 1) + ",";
      json += "\"roomTemperatureC\":" + jsonFloat(rt.roomTemperatureC, 1) + ",";
      json += "\"outsideTemperatureC\":" + jsonFloat(rt.outsideTemperatureC, 1) + ",";
      json += "\"targetFlowTemperatureC\":" + jsonFloat(rt.targetFlowTemperatureC, 1) + ",";
      json += "\"mixerPosition\":" + String(rt.estimatedMixerPositionPercent);
      json += "}";
    }

    json += "],\"sensors\":{\"ds18b20\":[";
    for (uint8_t i = 0; i < s_ctx->ds18b20.count; i++) {
      if (i > 0) json += ",";
      const Ds18b20DeviceInfo& d = s_ctx->ds18b20.devices[i];
      json += "{";
      json += "\"index\":" + String(i) + ",";
      json += "\"address\":\"" + String(d.addressText) + "\",";
      json += "\"role\":\"" + String(SensorRoles::toLabel(d.role)) + "\",";
      json += "\"temperatureC\":" + jsonFloat(d.lastTempC, 1) + ",";
      json += "\"valid\":" + String(d.lastValid ? "true" : "false");
      json += "}";
    }
    json += "],\"max\":[";
    for (uint8_t i = 0; i < 3; i++) {
      if (i > 0) json += ",";
      const MaxChannelReading& r = s_ctx->maxReadings[i];
      json += "{";
      json += "\"index\":" + String(i) + ",";
      json += "\"present\":" + String(r.present ? "true" : "false") + ",";
      json += "\"valid\":" + String(r.valid ? "true" : "false") + ",";
      json += "\"temperatureC\":" + jsonFloat(r.tempC, 1) + ",";
      json += "\"resistanceOhm\":" + jsonFloat(r.resistanceOhm, 1) + ",";
      json += "\"fault\":" + String(r.fault);
      json += "}";
    }
    json += "]}";

    json += ",";
    appendTestSafetyJson(json);

    json += ",\"alarms\":";
    Alarms::appendJson(json);

    json += ",\"diagnostics\":{";
    json += "\"faultActive\":" + String(s_ctx->diag.faultActive ? "true" : "false") + ",";
    json += "\"faultText\":\"" + jsonEscape(s_ctx->diag.faultText) + "\",";
    json += "\"bootCount\":" + String(s_ctx->diag.bootCount) + ",";
    json += "\"sensorErrorCount\":" + String(s_ctx->diag.sensorErrorCount) + ",";
    json += "\"sdErrorCount\":" + String(s_ctx->diag.sdErrorCount) + ",";
    json += "\"faultCount\":" + String(s_ctx->diag.faultCount) + ",";
    json += "\"maxErrorCount\":" + String(s_ctx->diag.maxErrorCount) + ",";
    json += "\"sdAvailable\":" + String(s_ctx->sdAvailable ? "true" : "false") + ",";
    json += "\"freeHeap\":" + String(ESP.getFreeHeap()) + ",";
    json += "\"uptimeMs\":" + String(millis());
    json += "}";

    String validationError;
    const bool outputValid = OutputValidation::validateAll(*s_ctx, validationError);
    json += ",\"outputValidation\":{";
    json += "\"ok\":" + String(outputValid ? "true" : "false") + ",";
    json += "\"message\":\"" + validationError + "\"";
    json += "}";

    json += "}";
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

  void handleTestServo() {
    if (!commissioningTestAuthorized()) {
      server.send(403, "text/plain", "Testmodus nicht aktiv oder Inbetriebnahme-PIN falsch");
      return;
    }

    if (!server.hasArg("angle")) {
      server.send(400, "text/plain", "angle fehlt");
      return;
    }

    int angle = server.arg("angle").toInt();
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;

    const uint8_t actual = ServoDriver::setAngle((uint8_t)angle);

    Serial.print("SERVO TEST GPIO");
    Serial.print(OVEN_SERVO_PIN);
    Serial.print(" -> ");
    Serial.print(actual);
    Serial.println(" Grad");
    Serial.flush();

    server.send(200, "text/plain", "OK");
  }


  void handleTestValvePosition() {
    if (!commissioningTestAuthorized()) {
      server.send(403, "text/plain", "Testmodus nicht aktiv oder Inbetriebnahme-PIN falsch");
      return;
    }
    if (!server.hasArg("index") || !server.hasArg("position")) {
      server.send(400, "text/plain", "index/position fehlt");
      return;
    }

    int idx = server.arg("index").toInt();
    if (idx < 0 || idx >= MAX_VALVES) {
      server.send(400, "text/plain", "ungueltiges Ventil");
      return;
    }

    ValvePosition pos = (server.arg("position") == "B") ? ValvePosition::B : ValvePosition::A;
    if (!Valves::requestPosition(*s_ctx, (uint8_t)idx, pos)) {
      const ValveConfig& valve = s_ctx->config.valves[idx];
      if (!valve.enabled) server.send(400, "text/plain", "Ventil ist nicht aktiviert");
      else if (valve.output.kind == OutputKind::NONE || valve.output.index == PIN_UNUSED) server.send(400, "text/plain", "Ventil hat keinen Ausgang");
      else server.send(400, "text/plain", "Ventil konnte nicht gefahren werden");
      return;
    }

    server.send(200, "text/plain", "OK");
  }

  void handleTestHeatingCircuitOutput() {
    if (!commissioningTestAuthorized()) {
      server.send(403, "text/plain", "Testmodus nicht aktiv oder Inbetriebnahme-PIN falsch");
      return;
    }
    if (!server.hasArg("index") || !server.hasArg("output") || !server.hasArg("state")) {
      server.send(400, "text/plain", "index/output/state fehlt");
      return;
    }

    int idx = server.arg("index").toInt();
    if (idx < 0 || idx >= MAX_HEATING_CIRCUITS) {
      server.send(400, "text/plain", "ungueltiger Heizkreis");
      return;
    }

    const HeatingCircuitConfig& cfg = s_ctx->config.heatingCircuits[idx];
    const String output = server.arg("output");
    const bool on = server.arg("state").toInt() != 0;
    uint8_t percent = 100;
    if (server.hasArg("percent")) {
      int p = server.arg("percent").toInt();
      if (p < 0) p = 0;
      if (p > 100) p = 100;
      percent = (uint8_t)p;
    }

    if (output == "open") {
      testSetOutputRef(*s_ctx, cfg.mixerCloseOutput, false, 0);
      testSetOutputRef(*s_ctx, cfg.mixerOpenOutput, on, percent);
    } else if (output == "close") {
      testSetOutputRef(*s_ctx, cfg.mixerOpenOutput, false, 0);
      testSetOutputRef(*s_ctx, cfg.mixerCloseOutput, on, percent);
    } else if (output == "pump") {
      testSetOutputRef(*s_ctx, cfg.pumpOutput, on, percent);
    } else {
      server.send(400, "text/plain", "ungueltiger Ausgang");
      return;
    }

    server.send(200, "text/plain", "OK");
  }

  void allOutputsOffSafe() {
    if (!s_ctx) return;
    RelayOutputs::allOff(*s_ctx);
    PwmDriver::allOff();
    ServoDriver::close();
    Valves::allOff(*s_ctx);
    HeatingCircuits::allOff(*s_ctx);
  }

  void handleTestAllOff() {
    if (!commissioningTestAuthorized()) {
      server.send(403, "text/plain", "Testmodus nicht aktiv oder Inbetriebnahme-PIN falsch");
      return;
    }
    allOutputsOffSafe();
    server.send(200, "text/plain", "OK");
  }


  void handleAlarmsJson() {
    String json;
    Alarms::appendJson(json);
    server.send(200, "application/json", json);
  }

  void handleAlarmsAck() {
    if (!testSessionValid()) {
      server.send(403, "text/plain", "Service-PIN falsch oder fehlt");
      return;
    }

    uint8_t count = 0;
    if (server.hasArg("id") && server.arg("id").length() > 0) {
      if (!Alarms::acknowledge(server.arg("id").c_str())) {
        server.send(409, "text/plain", "Alarm nicht gefunden oder kritischer aktiver Alarm ist nicht quittierbar");
        return;
      }
      count = 1;
    } else {
      count = Alarms::acknowledgeAllNonCritical();
    }

    server.send(200, "text/plain", "Quittiert: " + String(count));
  }

  void handleAlarmsClearHistory() {
    if (!testSessionValid()) {
      server.send(403, "text/plain", "Service-PIN falsch oder fehlt");
      return;
    }

    Alarms::clearHistory();
    Alarms::recordInfo("alarm_history_cleared", "Alarmhistorie wurde geloescht");
    server.send(200, "text/plain", "Alarmhistorie geloescht");
  }

  void handleFactoryReset() {
    if (!testSessionValid()) {
      server.send(403, "text/plain", "Service-PIN falsch oder fehlt");
      return;
    }

    if (!s_ctx) {
      server.send(500, "text/plain", "Kein Kontext");
      return;
    }

    if (!server.hasArg("confirm") || server.arg("confirm") != "RESET") {
      server.send(400, "text/plain", "Bestaetigung fehlt. Zum Zuruecksetzen confirm=RESET senden.");
      return;
    }

    const bool clearSensorLinks = server.hasArg("clearSensors") && server.arg("clearSensors") == "1";

    allOutputsOffSafe();
    deactivateCommissioningTestMode();

    Storage::resetConfig(s_ctx->config);
    Storage::resetDiagnostics(s_ctx->diag);
    Storage::resetMaintenance(s_ctx->maintenance);
    Alarms::resetAll();
    Alarms::recordInfo("factory_reset", clearSensorLinks
      ? "Werkseinstellungen ausgefuehrt, Sensor- und Waermequellen-Zuordnung geloescht"
      : "Werkseinstellungen ausgefuehrt, Sensor- und Waermequellen-Zuordnung erhalten");

    if (clearSensorLinks) {
      Storage::resetSensorAssignments(s_ctx->assignments);
      HeatSourceAssignments::clearTable(s_ctx->heatSourceAssignments);
    }

    const bool okConfig = Storage::saveConfig(s_ctx->config);
    const bool okDiagnostics = Storage::saveDiagnostics(s_ctx->diag);
    const bool okMaintenance = Storage::saveMaintenance(s_ctx->maintenance);
    const bool okSensors = clearSensorLinks ? Storage::saveSensorAssignments(s_ctx->assignments) : true;
    const bool okHeatSources = clearSensorLinks ? HeatSourceStorage::saveAssignments(s_ctx->heatSourceAssignments) : true;

    if (!okConfig || !okDiagnostics || !okMaintenance || !okSensors || !okHeatSources) {
      server.send(500, "text/plain", "Zuruecksetzen teilweise fehlgeschlagen. SD-Karte und Dateien pruefen.");
      return;
    }

    server.send(200, "text/plain", clearSensorLinks
      ? "Reset erfolgreich. Sensor- und Waermequellen-Zuordnung wurden geloescht. Steuerung startet neu..."
      : "Reset erfolgreich. Sensor- und Waermequellen-Zuordnung bleiben erhalten. Steuerung startet neu...");
    scheduleRestart();
  }

  void handleServiceRestart() {
    if (!testSessionValid()) {
      server.send(403, "text/plain", "Service-PIN falsch oder fehlt");
      return;
    }

    allOutputsOffSafe();
    deactivateCommissioningTestMode();
    server.send(200, "text/plain", "Neustart wird ausgefuehrt...");
    scheduleRestart();
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
    String k = key;
    k.trim();
    k.toLowerCase();

    if (k == "relay" || k == "r" || k == "relais") return OutputKind::RELAY;
    if (k == "pwm" || k == "po" || k == "pwm_output") return OutputKind::PWM_OUTPUT;
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
  server.on("/api/max-debug", HTTP_GET, handleMaxDebugJson);
  server.on("/api/network", HTTP_GET, handleNetworkJson);
  server.on("/service-save-network", HTTP_POST, handleNetworkSave);
  server.on("/api/relays", HTTP_GET, handleRelaysJson);
  server.on("/service-relay-config", HTTP_POST, handleRelayConfig);
  server.on("/api/pumps", HTTP_GET, handlePumpsJson);
  server.on("/service-pump-config", HTTP_POST, handlePumpConfig);
  server.on("/api/valves", HTTP_GET, handleValvesJson);
  server.on("/service-valve-config", HTTP_POST, handleValveConfig);
  server.on("/api/oven", HTTP_GET, handleOvenJson);
  server.on("/service-save-oven", HTTP_POST, handleOvenSave);
  server.on("/service-oven-start", HTTP_POST, handleOvenStart);
  server.on("/service-oven-stop", HTTP_POST, handleOvenStop);
  server.on("/api/test", HTTP_GET, handleTestOverviewJson);
  server.on("/api/alarms", HTTP_GET, handleAlarmsJson);
  server.on("/service-alarms-ack", HTTP_POST, handleAlarmsAck);
  server.on("/service-alarms-clear-history", HTTP_POST, handleAlarmsClearHistory);
  server.on("/api/testmode", HTTP_GET, handleTestModeGet);
  server.on("/service-testmode", HTTP_POST, handleTestModeSet);
  server.on("/service-test-relay-raw", HTTP_POST, handleTestRelayRaw);
  server.on("/service-test-pca-pwm", HTTP_POST, handleTestPcaPwm);
  server.on("/service-test-servo", HTTP_POST, handleTestServo);
  server.on("/service-test-valve", HTTP_POST, handleTestValvePosition);
  server.on("/service-test-pump-route", HTTP_POST, handleTestPumpRoute);
  server.on("/service-test-pump-output", HTTP_POST, handleTestPumpOutput);
  server.on("/service-test-heating-circuit-output", HTTP_POST, handleTestHeatingCircuitOutput);
  server.on("/service-test-all-off", HTTP_POST, handleTestAllOff);
  server.on("/service-factory-reset", HTTP_POST, handleFactoryReset);
  server.on("/service-restart", HTTP_POST, handleServiceRestart);
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
  if (g_restartScheduled && (int32_t)(millis() - g_restartAtMs) >= 0) {
    Serial.println("Geplanter Neustart wird ausgefuehrt...");
    delay(50);
    ESP.restart();
  }
}


bool commissioningTestActive() {
  return g_commissioningTestActive;
}
}