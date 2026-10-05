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
#include "feature_pumps.h"
#include "feature_heating_circuits.h"
#include "feature_heat_sources_max31865.h"
#include "feature_alarms.h"
#include "feature_aux_heater.h"
#include "feature_energy_meter.h"
#include "feature_mqtt.h"
#include "feature_time.h"
#include "feature_forecast.h"
#include "feature_ml_optimizer.h"
#include "feature_fluid_properties.h"
#include "feature_history.h"
#include <WebServer.h>
#include <SD.h>
#include <WiFi.h>
#include <stdio.h>
#include <stdlib.h>

#include "feature_build_flags.h"
#include "feature_runtime_diag.h"
#include "feature_sensor_ring.h"
#include "feature_ui_bridge.h"
static WebServer server(80);

namespace {
String pwmOutputHardwareLabel(uint8_t index) {
  return "PO-" + String(index);
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

  File g_sdUploadFile;
  String g_sdUploadTargetPath;
  String g_sdUploadTempPath;
  String g_sdUploadError;
  bool g_sdUploadCommitted = false;

  void allOutputsOffSafe();

  void activateCommissioningTestMode() {
    g_commissioningTestActive = true;
    g_commissioningTestStartedMs = millis();
    s_commissioningTestUntilMs = millis() + COMMISSIONING_TEST_TIMEOUT_MS;

    DBG_PRINTLN("INBETRIEBNAHME TESTMODUS AKTIV - REGELUNG PAUSIERT");
    DBG_FLUSH();
  }

  void deactivateCommissioningTestMode() {
    // Wichtig: Nur der echte aktive Testmodus darf beim Beenden Ausgänge abschalten.
    // Einige Browser/Seiten senden beim Verlassen vorsorglich active=0. Wenn der
    // Testmodus dabei gar nicht aktiv war, darf das den Automatikbetrieb nicht stören.
    if (!g_commissioningTestActive) {
      g_commissioningTestStartedMs = 0;
      s_commissioningTestUntilMs = 0;
      return;
    }

    allOutputsOffSafe();

    // Die Zusatzheizung cached ihre zuletzt gesetzten Sollzustaende. Da der
    // Testmodus Ausgaenge direkt ueberschreiben darf, muss dieser Cache beim
    // Ruecksprung in den Automatikbetrieb verworfen werden. Sonst kann die
    // Logik z. B. weiterhin "Pumpe EIN" glauben, obwohl das Test-All-Off den
    // realen Ausgang gerade ausgeschaltet hat.
    AuxHeater::resumeAfterTestMode();

    g_commissioningTestActive = false;
    g_commissioningTestStartedMs = 0;
    s_commissioningTestUntilMs = 0;

    DBG_PRINTLN("INBETRIEBNAHME TESTMODUS AUS - TESTAUSGAENGE AUS, AUTOMATIK NEU BEWERTET");
    DBG_FLUSH();
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
  String wifiStatusToText(wl_status_t status);
  void handleAlarmsJson();
  void handleAlarmsAck();
  void handleAlarmsClearHistory();

  String contentType(const String& path) {
    if (path.endsWith(".html")) return "text/html";
    if (path.endsWith(".css")) return "text/css";
    if (path.endsWith(".js")) return "application/javascript";
    if (path.endsWith(".json")) return "application/json";
    if (path.endsWith(".csv")) return "text/csv";
    if (path.endsWith(".svg")) return "image/svg+xml";
    return "text/plain";
  }

  bool serveSdFile(const String& path) {
    if (!SD.exists(path)) return false;

    File file = SD.open(path);
    if (!file) return false;

    // Phase 1D: the complete static UI, including HTML, is cacheable on the
    // client. UI updates use the versioned URL convention (?v=<UI_VERSION>).
    // Dynamic APIs explicitly use no-store in their handlers.
    server.sendHeader("Cache-Control", "public, max-age=31536000, immutable");
    server.sendHeader("X-SolarCtrl-UI-Version", String(UIBridge::UI_VERSION));
    server.streamFile(file, contentType(path));
    file.close();
    return true;
  }

  void handleRoot() {
    DBG_PRINTLN("HTTP GET ROOT");

    // The root itself is only a tiny version selector and is intentionally not
    // cached. The actual HTML remains long-term cached under its versioned URL.
    // This prevents an immutable old /index.html from trapping the browser on
    // an obsolete UI after a firmware/UI update.
    if (SD.exists("/index.html") || SD.exists("/www/index.html")) {
      const String target = String("/index.html?v=") + String(UIBridge::UI_VERSION);
      server.sendHeader("Cache-Control", "no-store");
      server.sendHeader("Location", target);
      server.send(302, "text/plain", "");
      return;
    }

    server.send(404, "text/plain", "Keine index.html gefunden");
  }

  void handleStatic() {
    String path = server.uri();

    if (serveSdFile(path)) return;
    if (serveSdFile("/www" + path)) return;

    server.send(404, "text/plain", "Datei nicht gefunden");
  }

  bool sdWebPathAllowed(const String& rawPath) {
    String path = rawPath;
    path.trim();
    if (!path.startsWith("/")) return false;
    if (path.indexOf("..") >= 0 || path.indexOf('\\') >= 0) return false;
    if (path.length() < 2 || path.length() > 120) return false;

    // Konfiguration, Runtime und Logs bleiben absichtlich ausserhalb des Browser-Dateimanagers.
    if (path == "/config" || path.startsWith("/config/")) return false;
    if (path == "/runtime" || path.startsWith("/runtime/")) return false;
    if (path == "/logs" || path.startsWith("/logs/")) return false;

    String lower = path;
    lower.toLowerCase();
    return lower.endsWith(".html") || lower.endsWith(".js") || lower.endsWith(".css") ||
           lower.endsWith(".svg") || lower.endsWith(".png") || lower.endsWith(".jpg") ||
           lower.endsWith(".jpeg") || lower.endsWith(".webp") || lower.endsWith(".ico") ||
           lower.endsWith(".txt");
  }

  bool ensureSdParentDirectory(const String& filePath) {
    int slash = filePath.lastIndexOf('/');
    if (slash <= 0) return true;
    String parent = filePath.substring(0, slash);
    if (parent.length() == 0 || parent == "/") return true;
    if (SD.exists(parent)) return true;

    String current;
    int pos = 1;
    while (pos < parent.length()) {
      int next = parent.indexOf('/', pos);
      String part = next < 0 ? parent.substring(pos) : parent.substring(pos, next);
      if (part.length()) {
        current += "/" + part;
        if (!SD.exists(current) && !SD.mkdir(current)) return false;
      }
      if (next < 0) break;
      pos = next + 1;
    }
    return true;
  }

  bool sdWebDirectoryAllowed(const String& rawPath) {
    String path = rawPath;
    path.trim();
    if (!path.startsWith("/")) return false;
    if (path.indexOf("..") >= 0 || path.indexOf('\\') >= 0) return false;
    if (path == "/config" || path.startsWith("/config/")) return false;
    if (path == "/runtime" || path.startsWith("/runtime/")) return false;
    if (path == "/logs" || path.startsWith("/logs/")) return false;
    if (path.startsWith("/.")) return false;
    return true;
  }

  // Service-Files is an on-demand maintenance view.  Walk the SD tree only
  // when that page explicitly asks for it.  A hard entry/depth limit prevents
  // a malformed or unexpectedly large card from monopolising the sync server.
  static constexpr uint16_t SD_WEB_TREE_MAX_FILES = 256;
  static constexpr uint8_t SD_WEB_TREE_MAX_DEPTH = 8;

  void appendSdWebTreeJson(String& json,
                           const String& dirPath,
                           bool& first,
                           uint16_t& fileCount,
                           bool& truncated,
                           uint8_t depth) {
    if (truncated || depth > SD_WEB_TREE_MAX_DEPTH || !sdWebDirectoryAllowed(dirPath)) return;

    File dir = SD.open(dirPath.c_str());
    if (!dir || !dir.isDirectory()) {
      if (dir) dir.close();
      return;
    }

    File entry = dir.openNextFile();
    while (entry) {
      String name = String(entry.name());
      String fullPath;
      if (name.startsWith("/")) {
        fullPath = name;
      } else if (dirPath == "/") {
        fullPath = "/" + name;
      } else {
        fullPath = dirPath + "/" + name;
      }

      if (entry.isDirectory()) {
        entry.close();
        if (sdWebDirectoryAllowed(fullPath)) {
          appendSdWebTreeJson(json, fullPath, first, fileCount, truncated, depth + 1);
        }
      } else {
        const uint32_t size = (uint32_t)entry.size();
        entry.close();
        if (sdWebPathAllowed(fullPath)) {
          if (fileCount >= SD_WEB_TREE_MAX_FILES) {
            truncated = true;
            break;
          }
          if (!first) json += ",";
          first = false;
          json += "{\"path\":\"" + jsonEscape(fullPath.c_str()) + "\",";
          json += "\"size\":" + String(size) + "}";
          fileCount++;
        }
      }

      if (truncated) break;
      entry = dir.openNextFile();
    }
    dir.close();
  }

  void handleSdFilesJson() {
    if (!testSessionValid()) {
      server.send(403, "application/json", "{\"error\":\"service_pin\"}");
      return;
    }
    if (!s_ctx || !s_ctx->sdAvailable) {
      server.send(503, "application/json", "{\"error\":\"sd_unavailable\"}");
      return;
    }

    String json;
    json.reserve(4096);
    json = "{\"files\":[";
    bool first = true;
    bool truncated = false;
    uint16_t fileCount = 0;
    appendSdWebTreeJson(json, "/", first, fileCount, truncated, 0);
    json += "],\"count\":" + String(fileCount) + ",\"truncated\":";
    json += truncated ? "true" : "false";
    json += "}";
    server.send(200, "application/json", json);
  }

  void handleSdDownload() {
    if (!testSessionValid()) {
      server.send(403, "text/plain", "Service-PIN falsch oder fehlt");
      return;
    }
    if (!server.hasArg("path")) {
      server.send(400, "text/plain", "path fehlt");
      return;
    }

    const String path = server.arg("path");
    if (!sdWebPathAllowed(path)) {
      server.send(403, "text/plain", "Pfad ist fuer den Web-Dateimanager nicht freigegeben");
      return;
    }
    File file = SD.open(path, FILE_READ);
    if (!file || file.isDirectory()) {
      if (file) file.close();
      server.send(404, "text/plain", "Datei nicht gefunden");
      return;
    }

    String filename = path.substring(path.lastIndexOf('/') + 1);
    server.sendHeader("Content-Disposition", "attachment; filename=\"" + filename + "\"");
    server.streamFile(file, contentType(path));
    file.close();
  }

  void resetSdUploadState() {
    if (g_sdUploadFile) g_sdUploadFile.close();
    g_sdUploadTargetPath = "";
    g_sdUploadTempPath = "";
    g_sdUploadError = "";
    g_sdUploadCommitted = false;
  }

  void handleSdUploadStream() {
    HTTPUpload& upload = server.upload();

    if (upload.status == UPLOAD_FILE_START) {
      resetSdUploadState();
      if (!testSessionValid()) {
        g_sdUploadError = "Service-PIN falsch oder fehlt";
        return;
      }
      if (!server.hasArg("path")) {
        g_sdUploadError = "path fehlt";
        return;
      }

      g_sdUploadTargetPath = server.arg("path");
      if (!sdWebPathAllowed(g_sdUploadTargetPath)) {
        g_sdUploadError = "Pfad ist fuer den Web-Dateimanager nicht freigegeben";
        return;
      }
      if (!ensureSdParentDirectory(g_sdUploadTargetPath)) {
        g_sdUploadError = "Zielordner konnte nicht angelegt werden";
        return;
      }

      g_sdUploadTempPath = g_sdUploadTargetPath + ".upload.tmp";
      if (SD.exists(g_sdUploadTempPath)) SD.remove(g_sdUploadTempPath);
      g_sdUploadFile = SD.open(g_sdUploadTempPath, FILE_WRITE);
      if (!g_sdUploadFile) g_sdUploadError = "Temporaere Datei konnte nicht angelegt werden";
      return;
    }

    if (upload.status == UPLOAD_FILE_WRITE) {
      if (g_sdUploadError.length() || !g_sdUploadFile) return;
      const size_t written = g_sdUploadFile.write(upload.buf, upload.currentSize);
      if (written != upload.currentSize) g_sdUploadError = "Schreibfehler auf SD-Karte";
      return;
    }

    if (upload.status == UPLOAD_FILE_END) {
      if (g_sdUploadFile) {
        g_sdUploadFile.flush();
        g_sdUploadFile.close();
      }
      if (g_sdUploadError.length()) return;

      const String backup = g_sdUploadTargetPath + ".bak";
      if (SD.exists(backup)) SD.remove(backup);
      const bool hadOriginal = SD.exists(g_sdUploadTargetPath);
      if (hadOriginal && !SD.rename(g_sdUploadTargetPath, backup)) {
        g_sdUploadError = "Alte Datei konnte nicht gesichert werden";
        return;
      }
      if (!SD.rename(g_sdUploadTempPath, g_sdUploadTargetPath)) {
        if (hadOriginal && SD.exists(backup)) SD.rename(backup, g_sdUploadTargetPath);
        g_sdUploadError = "Neue Datei konnte nicht aktiviert werden";
        return;
      }

      g_sdUploadCommitted = true;
      return;
    }

    if (upload.status == UPLOAD_FILE_ABORTED) {
      if (g_sdUploadFile) g_sdUploadFile.close();
      if (g_sdUploadTempPath.length() && SD.exists(g_sdUploadTempPath)) SD.remove(g_sdUploadTempPath);
      g_sdUploadError = "Upload abgebrochen";
    }
  }

  void handleSdUploadComplete() {
    if (!g_sdUploadCommitted) {
      const String message = g_sdUploadError.length() ? g_sdUploadError : "Upload nicht abgeschlossen";
      if (g_sdUploadTempPath.length() && SD.exists(g_sdUploadTempPath)) SD.remove(g_sdUploadTempPath);
      server.send(500, "text/plain", message);
      resetSdUploadState();
      return;
    }

    const String resultPath = g_sdUploadTargetPath;
    resetSdUploadState();
    server.send(200, "application/json", "{\"ok\":true,\"path\":\"" + jsonEscape(resultPath.c_str()) + "\"}");
  }

  void handleSdDelete() {
    if (!testSessionValid()) {
      server.send(403, "text/plain", "Service-PIN falsch oder fehlt");
      return;
    }
    if (!server.hasArg("path")) {
      server.send(400, "text/plain", "path fehlt");
      return;
    }

    const String path = server.arg("path");
    if (!sdWebPathAllowed(path)) {
      server.send(403, "text/plain", "Pfad ist fuer den Web-Dateimanager nicht freigegeben");
      return;
    }
    if (!SD.exists(path)) {
      server.send(404, "text/plain", "Datei nicht gefunden");
      return;
    }
    if (!SD.remove(path)) {
      server.send(500, "text/plain", "Datei konnte nicht geloescht werden");
      return;
    }
    server.send(200, "text/plain", "OK");
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
  json += ",\"ovenEnabled\":" + String(s_ctx->config.oven.enabled ? "true" : "false");
  json += ",\"ovenActive\":" + String(OvenControl::active() ? "true" : "false");
  json += ",\"ovenSafety\":" + String(SafetyManager::status().ovenOvertemperatureActive ? "true" : "false");
  json += ",\"ovenState\":\"" + String(OvenControl::stateText()) + "\"";
  json += ",\"ovenTemperatureC\":" + jsonFloat(OvenControl::ovenTemperatureC(), 1);
  json += ",\"ovenServoAngle\":" + String(OvenControl::servoAngle());
  json += ",\"ovenServoOpeningPercent\":" + String(OvenControl::servoOpeningPercent());
  json += ",\"ovenPumpActive\":" + String(OvenControl::pumpActive() ? "true" : "false");
  json += ",\"energyMeterEnabled\":" + String(s_ctx->config.energyMeter.enabled ? "true" : "false");
  json += ",\"energyTotalKWh\":" + String(s_ctx->energyMeter.totalEnergyKWh, 3);
  json += ",\"energyPowerKw\":" + String(s_ctx->energyMeter.thermalPowerKw, 3);
  json += ",\"energyFlowLMin\":" + String(s_ctx->energyMeter.flowLitersPerMinute, 2);
  json += ",\"energyVolumeLiters\":" + String(s_ctx->energyMeter.totalVolumeLiters, 1);
  json += ",\"mqttEnabled\":" + String(s_ctx->config.mqttEnabled ? "true" : "false");
  json += ",\"mqttConnected\":" + String(MqttBridge::connected() ? "true" : "false");
  json += ",\"alarmActiveCount\":" + String(Alarms::activeCount());
  json += ",\"alarmCritical\":" + String(Alarms::hasCriticalActive() ? "true" : "false");
  json += ",\"alarmMessage\":\"" + jsonEscape(Alarms::currentMessage()) + "\"";
  json += ",\"buzzerMuted\":" + String(Alarms::buzzerMuted() ? "true" : "false");
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

  const bool doScan = server.hasArg("scan") && server.arg("scan") == "1";
  if (doScan) {
    SinkSensor::scanBus(s_ctx->ds18b20);
  }

  String json = "{";
  json += "\"count\":" + String(s_ctx->ds18b20.count) + ",";
  json += "\"max\":" + String(MAX_DS18B20_SENSORS) + ",";
  json += "\"scanned\":" + String(doScan ? "true" : "false") + ",";
  json += "\"devices\": [";

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

    if (!SensorRoles::isAssignable(role)) {
      server.send(400, "text/plain", "Ungueltige oder fuer DS18 nicht erlaubte Rolle");
      return;
    }

    // Rolle "none" bedeutet: diesen Sensor aus der Rollentabelle entfernen.
    // Gleichzeitig erlauben wir das Umlegen eines Sensors von z.B. Puffer Top
    // auf HK1 Vorlauf ohne vorherigen Werksreset.
    Ds18Role oldRoleForAddress = Ds18Role::NONE;
    for (uint8_t i = 0; i < s_ctx->assignments.count; i++) {
      const Ds18RoleAssignment& existing = s_ctx->assignments.items[i];
      if (!existing.assigned) continue;
      if (String(existing.addressText) == address) {
        oldRoleForAddress = existing.role;
        break;
      }
    }

    if (role == Ds18Role::NONE) {
      if (oldRoleForAddress != Ds18Role::NONE) {
        SensorAssignments::removeRole(s_ctx->assignments, oldRoleForAddress);
      }
      Storage::saveSensorAssignments(s_ctx->assignments);
      server.send(200, "text/plain", "OK");
      return;
    }

    for (uint8_t i = 0; i < s_ctx->assignments.count; i++) {
      const Ds18RoleAssignment& existing = s_ctx->assignments.items[i];
      if (!existing.assigned) continue;

      if (existing.role == role && String(existing.addressText) != address) {
        server.send(409, "text/plain", "Diese Sensorrolle ist bereits einem anderen DS18B20 zugeordnet");
        return;
      }
    }

    if (oldRoleForAddress != Ds18Role::NONE && oldRoleForAddress != role) {
      SensorAssignments::removeRole(s_ctx->assignments, oldRoleForAddress);
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
  bool readRoleTemp(Ds18Role role, float& tempC, bool& valid) {
  return SensorAssignments::readByRole(
    s_ctx->ds18b20,
    s_ctx->assignments,
    role,
    tempC,
    valid
  );
}

  bool maxChannelEnabled(MaxChannel channel) {
    if (!s_ctx) return false;
    switch (channel) {
      case MaxChannel::CH1: return s_ctx->config.max1.enabled;
      case MaxChannel::CH2: return s_ctx->config.max2.enabled;
      case MaxChannel::CH3: return s_ctx->config.max3.enabled;
      case MaxChannel::CH4: return s_ctx->config.max4.enabled;
      default: return false;
    }
  }

  bool roleBelongsToStorage(Ds18Role role, bool buffer) {
    if (buffer) {
      return role == Ds18Role::SINK_BUFFER_TOP ||
             role == Ds18Role::BUFFER_HIGH ||
             role == Ds18Role::BUFFER_MID ||
             role == Ds18Role::BUFFER_BOTTOM;
    }
    return role == Ds18Role::SINK_BOILER_TOP ||
           role == Ds18Role::BOILER_BOTTOM;
  }

  float configuredStorageMaximumC(bool buffer) {
    float maximumC = 0.0f;

    // Die fuer Pumpenrouten eingestellte Maximaltemperatur beschreibt die
    // beabsichtigte obere Ladetemperatur des jeweiligen Speichers. Wenn
    // mehrere Routen denselben Speicher bedienen, verwenden wir die hoechste
    // konfigurierte Grenze fuer die Farbdarstellung.
    for (uint8_t i = 0; i < MAX_PUMPS; i++) {
      const PumpConfig& pump = s_ctx->config.pumps[i];
      if (!pump.enabled || pump.mode == PumpMode::OFF) continue;

      for (uint8_t t = 0; t < PUMP_ROUTE_TARGET_COUNT; t++) {
        const PumpRouteTargetConfig& target = pump.targets[t];
        if (!target.enabled || !roleBelongsToStorage(target.sinkRole, buffer)) continue;
        if (target.maxTempC > maximumC) maximumC = target.maxTempC;
      }
    }

    // Rueckfall fuer Anlagen ohne explizite Routen-Maximaltemperatur.
    if (maximumC <= 10.0f) maximumC = s_ctx->config.sinkMaxC;
    if (maximumC <= 10.0f) maximumC = 70.0f;
    return maximumC;
  }


void handleUiVersionJson() {
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json",
              String("{\"uiVersion\":") + String(UIBridge::UI_VERSION) +
              ",\"binaryProtocol\":" + String(UIBridge::PROTOCOL_VERSION) + "}");
}


void sendQueuedControlCommand(UIBridge::ControlAction action, int32_t valueA = 0, int32_t valueB = 0) {
  UIBridge::ControlCommand command;
  command.type = UIBridge::CommandType::USER_COMMAND;
  command.action = (uint16_t)action;
  command.valueA = valueA;
  command.valueB = valueB;

  if (!UIBridge::submitCommand(command)) {
    server.sendHeader("Cache-Control", "no-store");
    server.send(503, "application/json", "{\"ok\":false,\"error\":\"command_queue_full\"}");
    return;
  }

  char body[96];
  snprintf(body, sizeof(body), "{\"ok\":true,\"queued\":true,\"commandId\":%lu}", (unsigned long)command.id);
  server.sendHeader("Cache-Control", "no-store");
  server.send(202, "application/json", body);
}

void handleQueuedBuzzerMute() {
  bool mute = true;
  if (server.hasArg("state")) {
    String v = server.arg("state");
    v.toLowerCase();
    mute = !(v == "0" || v == "off" || v == "false");
  }
  sendQueuedControlCommand(UIBridge::ControlAction::BUZZER_MUTE, mute ? 1 : 0);
}

void handleQueuedOvenStart() {
  sendQueuedControlCommand(UIBridge::ControlAction::OVEN_START);
}

void handleQueuedOvenStop() {
  sendQueuedControlCommand(UIBridge::ControlAction::OVEN_STOP);
}

void handleUiCommandStatus() {
  if (!server.hasArg("id")) {
    server.sendHeader("Cache-Control", "no-store");
    server.send(400, "application/json", "{\"error\":\"id_missing\"}");
    return;
  }

  const uint32_t id = (uint32_t)strtoul(server.arg("id").c_str(), nullptr, 10);
  UIBridge::CommandResult result;
  if (!UIBridge::getCommandResult(id, result)) {
    server.sendHeader("Cache-Control", "no-store");
    server.send(404, "application/json", "{\"error\":\"command_unknown\"}");
    return;
  }

  const char* state = "unknown";
  switch (result.state) {
    case UIBridge::CommandState::PENDING: state = "pending"; break;
    case UIBridge::CommandState::APPLIED: state = "applied"; break;
    case UIBridge::CommandState::REJECTED: state = "rejected"; break;
    default: break;
  }

  char body[192];
  snprintf(body, sizeof(body),
           "{\"commandId\":%lu,\"state\":\"%s\",\"status\":%u,\"message\":\"%s\"}",
           (unsigned long)result.id, state, (unsigned)result.httpStatus, result.message);
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", body);
}

void handlePlantLiveBinary() {
  // Fixed stack buffer; no JSON String and no heap allocation for the payload.
  uint8_t packet[512];
  const size_t len = UIBridge::encodePlantLiveV2(packet, sizeof(packet));
  if (!len) {
    server.sendHeader("Cache-Control", "no-store");
    server.send(503, "text/plain", "snapshot unavailable");
    return;
  }

  server.sendHeader("Cache-Control", "no-store");
  server.sendHeader("X-SolarCtrl-Binary-Protocol", String(UIBridge::PROTOCOL_VERSION));
  server.setContentLength(len);
  server.send(200, "application/octet-stream", "");
  server.sendContent((const char*)packet, len);
}

void handlePlantLiveJson() {
  if (!s_ctx) {
    server.send(500, "application/json", "{\"error\":\"no_context\"}");
    return;
  }

  // Schneller Live-Endpunkt: nur Werte, die sich waehrend des Betriebs
  // aendern und von der Startseite tatsaechlich verwendet werden.
  float boilerTop = NAN, boilerBottom = NAN;
  float bufferTop = NAN, bufferHigh = NAN, bufferMid = NAN, bufferBottom = NAN;
  float poolTemp = NAN, outsideTemp = NAN;
  bool valid = false;

  SensorAssignments::readByRole(s_ctx->ds18b20, s_ctx->assignments, Ds18Role::SINK_BOILER_TOP, boilerTop, valid);
  SensorAssignments::readByRole(s_ctx->ds18b20, s_ctx->assignments, Ds18Role::BOILER_BOTTOM, boilerBottom, valid);
  SensorAssignments::readByRole(s_ctx->ds18b20, s_ctx->assignments, Ds18Role::SINK_BUFFER_TOP, bufferTop, valid);
  SensorAssignments::readByRole(s_ctx->ds18b20, s_ctx->assignments, Ds18Role::BUFFER_HIGH, bufferHigh, valid);
  SensorAssignments::readByRole(s_ctx->ds18b20, s_ctx->assignments, Ds18Role::BUFFER_MID, bufferMid, valid);
  SensorAssignments::readByRole(s_ctx->ds18b20, s_ctx->assignments, Ds18Role::BUFFER_BOTTOM, bufferBottom, valid);
  SensorAssignments::readByRole(s_ctx->ds18b20, s_ctx->assignments, Ds18Role::SWIMMINGPOOL, poolTemp, valid);
  SensorAssignments::readByRole(s_ctx->ds18b20, s_ctx->assignments, Ds18Role::OUTSIDE_TEMPERATURE, outsideTemp, valid);

  String json;
  json.reserve(2350);
  json = "{";

  json += "\"temperatures\":{";
  json += "\"boilerTop\":" + jsonFloat(boilerTop, 1) + ",";
  json += "\"boilerBottom\":" + jsonFloat(boilerBottom, 1) + ",";
  json += "\"bufferTop\":" + jsonFloat(bufferTop, 1) + ",";
  json += "\"bufferHigh\":" + jsonFloat(bufferHigh, 1) + ",";
  json += "\"bufferMid\":" + jsonFloat(bufferMid, 1) + ",";
  json += "\"bufferBottom\":" + jsonFloat(bufferBottom, 1) + ",";
  json += "\"pool\":" + jsonFloat(poolTemp, 1) + ",";
  json += "\"outside\":" + jsonFloat(outsideTemp, 1);
  json += "},";

  json += "\"collectors\":[";
  bool first = true;
  const HeatSourceRole collectorRoles[] = {
    HeatSourceRole::SOLAR_COLLECTOR_1,
    HeatSourceRole::SOLAR_COLLECTOR_2,
    HeatSourceRole::SOLAR_COLLECTOR_3
  };
  for (uint8_t i = 0; i < 3; i++) {
    if (!HeatSourceAssignments::hasRole(s_ctx->heatSourceAssignments, collectorRoles[i])) continue;
    if (!first) json += ",";
    first = false;

    float temp = NAN;
    bool tempValid = false;
    switch (i) {
      case 0: temp = s_ctx->sensors.heatSources.solarCollector1C; tempValid = s_ctx->sensors.heatSources.solarCollector1Valid; break;
      case 1: temp = s_ctx->sensors.heatSources.solarCollector2C; tempValid = s_ctx->sensors.heatSources.solarCollector2Valid; break;
      case 2: temp = s_ctx->sensors.heatSources.solarCollector3C; tempValid = s_ctx->sensors.heatSources.solarCollector3Valid; break;
    }

    json += "{\"index\":" + String(i) +
            ",\"valid\":" + String(tempValid ? "true" : "false") +
            ",\"tempC\":" + String(tempValid ? String(temp, 1) : "null") + "}";
  }
  json += "],";

  json += "\"heatSources\":{";
  json += "\"otherC\":" + String(s_ctx->sensors.heatSources.altSourceOtherValid
      ? String(s_ctx->sensors.heatSources.altSourceOtherC, 1) : "null");
  json += "},";

  json += "\"pumps\":[";
  first = true;
  for (uint8_t i = 0; i < MAX_PUMPS; i++) {
    const PumpConfig& pump = s_ctx->config.pumps[i];
    if (!pump.enabled || pump.mode == PumpMode::OFF) continue;
    if (!first) json += ",";
    first = false;
    json += "{";
    json += "\"index\":" + String(i) + ",";
    json += "\"active\":" + String(pump.state ? "true" : "false") + ",";
    json += "\"pwm\":" + String(pump.lastPwmPercent, 0) + ",";
    json += "\"activeTargetIndex\":" + String(pump.activeTargetIndex);
    json += "}";
  }
  json += "],";

  json += "\"energy\":{";
  json += "\"enabled\":" + String(s_ctx->config.energyMeter.enabled ? "true" : "false") + ",";
  json += "\"totalKWh\":" + String(s_ctx->energyMeter.totalEnergyKWh, 3) + ",";
  json += "\"powerKw\":" + String(s_ctx->energyMeter.thermalPowerKw, 3) + ",";
  json += "\"flowLMin\":" + String(s_ctx->energyMeter.flowLitersPerMinute, 2);
  json += "},";

  json += "\"oven\":{";
  json += "\"active\":" + String(OvenControl::active() ? "true" : "false") + ",";
  json += "\"state\":\"" + String(OvenControl::stateText()) + "\",";
  json += "\"temperatureC\":" + jsonFloat(OvenControl::ovenTemperatureC(), 1) + ",";
  json += "\"temperatureValid\":" + String(!isnan(OvenControl::ovenTemperatureC()) ? "true" : "false") + ",";
  json += "\"servoOpeningPercent\":" + String(OvenControl::servoOpeningPercent());
  json += "},";

  json += "\"auxHeater\":{";
  json += "\"released\":" + String(s_ctx->config.auxHeater.userReleaseEnabled ? "true" : "false") + ",";
  json += "\"state\":\"" + String(AuxHeater::stateText()) + "\",";
  json += "\"pumpActive\":" + String(AuxHeater::pumpActive() ? "true" : "false") + ",";
  json += "\"activeStages\":" + String(AuxHeater::activeStageCount());
  json += "},";

  json += "\"heatingCircuits\":[";
  first = true;
  for (uint8_t i = 0; i < MAX_HEATING_CIRCUITS; i++) {
    const HeatingCircuitConfig& cfg = s_ctx->config.heatingCircuits[i];
    if (!cfg.enabled) continue;
    const HeatingCircuitRuntime& rt = s_ctx->heatingCircuitRuntime[i];
    if (!first) json += ",";
    first = false;
    json += "{";
    json += "\"index\":" + String(i) + ",";
    json += "\"active\":" + String(rt.active ? "true" : "false") + ",";
    json += "\"pumpActive\":" + String(rt.pumpActive ? "true" : "false") + ",";
    json += "\"pumpPercent\":" + String(rt.pumpPercent) + ",";
    json += "\"flowC\":" + jsonFloat(rt.flowTemperatureC, 1) + ",";
    json += "\"returnC\":" + jsonFloat(rt.returnTemperatureC, 1) + ",";
    json += "\"targetC\":" + jsonFloat(rt.targetFlowTemperatureC, 1) + ",";
    json += "\"roomC\":" + jsonFloat(rt.roomTemperatureC, 1) + ",";
    json += "\"effectiveRoomTargetC\":" + jsonFloat(rt.effectiveRoomTargetTemperatureC, 1) + ",";
    json += "\"nightSetbackActive\":" + String(rt.nightSetbackActive ? "true" : "false");
    json += "}";
  }
  json += "],";

  json += "\"system\":{";
  json += "\"uptimeMs\":" + String(millis()) + ",";
  json += "\"alarmActiveCount\":" + String(Alarms::activeCount()) + ",";
  json += "\"alarmCritical\":" + String(Alarms::hasCriticalActive() ? "true" : "false") + ",";
  json += "\"alarmMessage\":\"" + jsonEscape(Alarms::currentMessage()) + "\",";
  json += "\"buzzerMuted\":" + String(Alarms::buzzerMuted() ? "true" : "false");
  json += "}";

  json += "}";
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", json);
}

void handlePlantJson() {
  if (!s_ctx) {
    server.send(500, "application/json", "{\"error\":\"no_context\"}");
    return;
  }

  const bool boilerPresent =
    SensorAssignments::hasRole(s_ctx->assignments, Ds18Role::SINK_BOILER_TOP) ||
    SensorAssignments::hasRole(s_ctx->assignments, Ds18Role::BOILER_BOTTOM);
  const bool bufferPresent =
    SensorAssignments::hasRole(s_ctx->assignments, Ds18Role::SINK_BUFFER_TOP) ||
    SensorAssignments::hasRole(s_ctx->assignments, Ds18Role::BUFFER_HIGH) ||
    SensorAssignments::hasRole(s_ctx->assignments, Ds18Role::BUFFER_MID) ||
    SensorAssignments::hasRole(s_ctx->assignments, Ds18Role::BUFFER_BOTTOM);
  const bool poolPresent =
    SensorAssignments::hasRole(s_ctx->assignments, Ds18Role::SWIMMINGPOOL);

  const HeatSourceRole collectorRoles[] = {
    HeatSourceRole::SOLAR_COLLECTOR_1,
    HeatSourceRole::SOLAR_COLLECTOR_2,
    HeatSourceRole::SOLAR_COLLECTOR_3
  };

  String json;
  json.reserve(3900);
  json = "{";

  json += "\"layout\":{";
  json += "\"boiler\":" + String(boilerPresent ? "true" : "false") + ",";
  json += "\"buffer\":" + String(bufferPresent ? "true" : "false") + ",";
  json += "\"pool\":" + String(poolPresent ? "true" : "false") + ",";
  json += "\"otherSource\":" + String(HeatSourceAssignments::hasRole(
      s_ctx->heatSourceAssignments, HeatSourceRole::ALT_SOURCE_OTHER) ? "true" : "false") + ",";
  json += "\"oven\":" + String(s_ctx->config.oven.enabled ? "true" : "false") + ",";
  json += "\"auxHeater\":" + String(s_ctx->config.auxHeater.enabled ? "true" : "false") + ",";
  json += "\"energyMeter\":" + String(s_ctx->config.energyMeter.enabled ? "true" : "false") + ",";

  json += "\"collectors\":[";
  bool first = true;
  for (uint8_t i = 0; i < 3; i++) {
    const HeatSourceRole role = collectorRoles[i];
    if (!HeatSourceAssignments::hasRole(s_ctx->heatSourceAssignments, role)) continue;
    if (!first) json += ",";
    first = false;
    json += "{\"index\":" + String(i) +
            ",\"key\":\"" + String(HeatSourceRoles::toKey(role)) +
            "\",\"label\":\"" + String(HeatSourceRoles::toLabel(role)) + "\"}";
  }
  json += "],";

  json += "\"heatingCircuits\":[";
  first = true;
  for (uint8_t i = 0; i < MAX_HEATING_CIRCUITS; i++) {
    if (!s_ctx->config.heatingCircuits[i].enabled) continue;
    if (!first) json += ",";
    first = false;
    json += String(i);
  }
  json += "]";
  json += "},";

  json += "\"storageColorScale\":{";
  json += "\"minC\":10.0,";
  json += "\"bufferMaxC\":" + String(configuredStorageMaximumC(true), 1) + ",";
  json += "\"boilerMaxC\":" + String(configuredStorageMaximumC(false), 1);
  json += "},";

  json += "\"collectors\":[";
  first = true;
  for (uint8_t i = 0; i < 3; i++) {
    const HeatSourceRole role = collectorRoles[i];
    HeatSourceAssignment assignment;
    const bool assigned =
      HeatSourceAssignments::getByRole(s_ctx->heatSourceAssignments, role, assignment);
    if (!assigned) continue;
    if (!first) json += ",";
    first = false;
    json += "{";
    json += "\"index\":" + String(i) + ",";
    json += "\"assigned\":true,";
    json += "\"sensorEnabled\":" + String(maxChannelEnabled(assignment.channel) ? "true" : "false");
    json += "}";
  }
  json += "],";

  json += "\"pumps\":[";
  first = true;
  for (uint8_t i = 0; i < MAX_PUMPS; i++) {
    const PumpConfig& pump = s_ctx->config.pumps[i];
    if (!pump.enabled || pump.mode == PumpMode::OFF) continue;
    if (!first) json += ",";
    first = false;
    json += "{";
    json += "\"index\":" + String(i) + ",";
    json += "\"mode\":" + String((int)pump.mode) + ",";
    json += "\"sourceType\":" + String((int)pump.sourceType) + ",";
    json += "\"sourceRole\":\"" + String(HeatSourceRoles::toKey(pump.sourceRole)) + "\",";
    json += "\"sourceSensorRole\":\"" + String(SensorRoles::toKey(pump.sourceSensorRole)) + "\",";
    json += "\"sinkRole\":\"" + String(SensorRoles::toKey(pump.sinkRole)) + "\",";
    json += "\"valveIndex\":" + String(pump.valveIndex) + ",";
    json += "\"targets\":[";
    bool firstTarget = true;
    for (uint8_t t = 0; t < PUMP_ROUTE_TARGET_COUNT; t++) {
      if (!pump.targets[t].enabled || pump.targets[t].sinkRole == Ds18Role::NONE) continue;
      if (!firstTarget) json += ",";
      firstTarget = false;
      json += "{\"index\":" + String(t) +
              ",\"sinkRole\":\"" + String(SensorRoles::toKey(pump.targets[t].sinkRole)) + "\"}";
    }
    json += "]";
    json += "}";
  }
  json += "],";

  json += "\"auxHeater\":{";
  json += "\"sinkRole\":\"" + String(SensorRoles::toKey(s_ctx->config.auxHeater.sinkRole)) + "\"";
  json += "},";

  json += "\"heatingCircuits\":[";
  first = true;
  for (uint8_t i = 0; i < MAX_HEATING_CIRCUITS; i++) {
    const HeatingCircuitConfig& cfg = s_ctx->config.heatingCircuits[i];
    if (!cfg.enabled) continue;
    if (!first) json += ",";
    first = false;
    json += "{";
    json += "\"index\":" + String(i) + ",";
    json += "\"roomControlEnabled\":" + String(cfg.roomControlEnabled ? "true" : "false") + ",";
    json += "\"roomTargetC\":" + String(cfg.roomTargetTemperatureC, 1) + ",";
    json += "\"nightSetbackEnabled\":" + String(cfg.nightSetbackEnabled ? "true" : "false") + ",";
    json += "\"bufferReferenceRole\":\"" + String(SensorRoles::toKey(cfg.bufferReferenceRole)) + "\"";
    json += "}";
  }
  json += "],";

  const Forecast::Summary forecastSummary = Forecast::summary();
  json += "\"system\":{";
  json += "\"mqttEnabled\":" + String(s_ctx->config.mqttEnabled ? "true" : "false") + ",";
  json += "\"mqttConnected\":" + String(MqttBridge::connected() ? "true" : "false") + ",";
  json += "\"wifiConnected\":" + String(WiFi.status() == WL_CONNECTED ? "true" : "false") + ",";
  json += "\"forecastEnabled\":" + String(s_ctx->config.forecastEnabled ? "true" : "false") + ",";
  json += "\"forecastValid\":" + String(forecastSummary.valid ? "true" : "false") + ",";
  json += "\"todayRemainingKWhM2\":" +
          (isnan(forecastSummary.todayRemainingKWhM2) ? String("null") : String(forecastSummary.todayRemainingKWhM2, 3)) + ",";
  json += "\"tomorrowKWhM2\":" +
          (isnan(forecastSummary.tomorrowKWhM2) ? String("null") : String(forecastSummary.tomorrowKWhM2, 3)) + ",";
  json += "\"tomorrowSunshineHours\":" +
          (isnan(forecastSummary.tomorrowSunshineHours) ? String("null") : String(forecastSummary.tomorrowSunshineHours, 2)) + ",";
  json += "\"timeValid\":" + String(TimeService::valid() ? "true" : "false") + ",";
  json += "\"dateText\":\"" + jsonEscape(TimeService::dateText().c_str()) + "\",";
  json += "\"timeText\":\"" + jsonEscape(TimeService::timeText().c_str()) + "\"";
  json += "},";

  json += "\"ml\":" + MlOptimizer::json(*s_ctx);
  json += "}";

  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", json);
}

int pumpUsingFeedbackGpio(const AppContext& ctx, uint8_t gpio) {
  if (gpio == PIN_UNUSED) return -1;
  for (uint8_t i = 0; i < MAX_PUMPS; i++) {
    const PumpConfig& pump = ctx.config.pumps[i];
    if (!pump.enabled || pump.mode != PumpMode::PWM) continue;
    if (pump.feedbackPin == gpio) return i;
  }
  return -1;
}

bool energyMeterUsesFeedbackGpio(const AppContext& ctx, uint8_t gpio) {
  if (!ctx.config.energyMeter.enabled) return false;
  const uint8_t configuredGpio = EnergyMeter::gpioForFeedbackInput(ctx.config.energyMeter.feedbackInputIndex);
  return configuredGpio != PIN_UNUSED && configuredGpio == gpio;
}

void handleEnergyMeterJson() {
  if (!s_ctx) {
    server.send(500, "application/json", "{\"error\":\"no_context\"}");
    return;
  }

  const EnergyMeterConfig& cfg = s_ctx->config.energyMeter;
  const EnergyMeterRuntime& rt = s_ctx->energyMeter;
  String json = "{";
  json += "\"enabled\":" + String(cfg.enabled ? "true" : "false") + ",";
  json += "\"feedbackInputIndex\":" + String(cfg.feedbackInputIndex) + ",";
  json += "\"feedbackGpio\":" + String(EnergyMeter::gpioForFeedbackInput(cfg.feedbackInputIndex)) + ",";
  json += "\"pulsesPerLiter\":" + String(cfg.pulsesPerLiter, 4) + ",";
  json += "\"frequencyFactorHzPerLMin\":" + String(cfg.pulsesPerLiter / 60.0f, 5) + ",";
  json += "\"flowSensorRole\":\"" + String(SensorRoles::toKey(cfg.flowSensorRole)) + "\",";
  json += "\"returnSensorRole\":\"" + String(SensorRoles::toKey(cfg.returnSensorRole)) + "\",";
  json += "\"energyFactorWhPerLiterK\":" + String(EnergyMeter::effectiveEnergyFactorWhPerLiterK(s_ctx->config, rt.temperatureValid ? 0.5f*(rt.flowTemperatureC+rt.returnTemperatureC) : 40.0f), 5) + ",";
  json += "\"energyFactorSource\":\"plantDataFluidModel\",";
  json += "\"logIntervalMs\":" + String(cfg.logIntervalMs) + ",";
  json += "\"csvPath\":\"" + String(FILE_ENERGY_METER_LOG) + "\",";
  json += "\"runtime\":{";
  json += "\"feedbackAttached\":" + String(rt.feedbackAttached ? "true" : "false") + ",";
  json += "\"temperatureValid\":" + String(rt.temperatureValid ? "true" : "false") + ",";
  json += "\"totalPulses\":" + String(rt.totalPulses) + ",";
  json += "\"totalVolumeLiters\":" + String(rt.totalVolumeLiters, 3) + ",";
  json += "\"totalEnergyKWh\":" + String(rt.totalEnergyKWh, 5) + ",";
  json += "\"flowLitersPerMinute\":" + String(rt.flowLitersPerMinute, 3) + ",";
  json += "\"flowTemperatureC\":" + jsonFloat(rt.flowTemperatureC, 2) + ",";
  json += "\"returnTemperatureC\":" + jsonFloat(rt.returnTemperatureC, 2) + ",";
  json += "\"deltaTemperatureK\":" + jsonFloat(rt.deltaTemperatureK, 2) + ",";
  json += "\"thermalPowerKw\":" + String(rt.thermalPowerKw, 3);
  json += "},";

  json += "\"feedbackInputs\":[";
  for (uint8_t i = 0; i < FEEDBACK_INPUT_COUNT; i++) {
    if (i > 0) json += ",";
    const uint8_t gpio = FEEDBACK_INPUT_PINS[i];
    const int usedByPump = pumpUsingFeedbackGpio(*s_ctx, gpio);
    json += "{";
    json += "\"index\":" + String(i) + ",";
    json += "\"gpio\":" + String(gpio) + ",";
    json += "\"label\":\"FB" + String(i) + " (GPIO " + String(gpio) + ")\",";
    json += "\"usedByPump\":" + String(usedByPump);
    json += "}";
  }
  json += "],";

  json += "\"sensorRoles\":[";
  bool first = true;
  for (int i = 1; i <= (int)Ds18Role::HK4_RETURN; i++) {
    const Ds18Role role = (Ds18Role)i;
    if (!first) json += ",";
    first = false;
    json += "{";
    json += "\"key\":\"" + String(SensorRoles::toKey(role)) + "\",";
    json += "\"label\":\"" + String(SensorRoles::toLabel(role)) + "\",";
    json += "\"assigned\":" + String(SensorAssignments::hasRole(s_ctx->assignments, role) ? "true" : "false");
    json += "}";
  }
  json += "]";
  json += "}";
  server.send(200, "application/json", json);
}

void handleEnergyMeterSave() {
  if (!s_ctx) {
    server.send(500, "text/plain", "Kein Kontext");
    return;
  }

  EnergyMeterConfig candidate = s_ctx->config.energyMeter;
  candidate.enabled = server.hasArg("enabled") && server.arg("enabled").toInt() != 0;
  if (server.hasArg("feedbackInputIndex")) {
    const int idx = server.arg("feedbackInputIndex").toInt();
    candidate.feedbackInputIndex = (idx >= 0 && idx < FEEDBACK_INPUT_COUNT) ? (uint8_t)idx : PIN_UNUSED;
  }
  if (server.hasArg("pulsesPerLiter")) candidate.pulsesPerLiter = server.arg("pulsesPerLiter").toFloat();
  if (server.hasArg("flowSensorRole")) candidate.flowSensorRole = SensorRoles::fromKey(server.arg("flowSensorRole"));
  if (server.hasArg("returnSensorRole")) candidate.returnSensorRole = SensorRoles::fromKey(server.arg("returnSensorRole"));

  if (candidate.enabled) {
    if (candidate.feedbackInputIndex == PIN_UNUSED || candidate.feedbackInputIndex >= FEEDBACK_INPUT_COUNT) {
      server.send(400, "text/plain", "Feedback-Eingang fuer Flowmeter fehlt");
      return;
    }
    if (candidate.pulsesPerLiter <= 0.0f || candidate.pulsesPerLiter > 1000000.0f) {
      server.send(400, "text/plain", "Impulse pro Liter muessen groesser 0 sein");
      return;
    }
    if (candidate.flowSensorRole == Ds18Role::NONE || candidate.returnSensorRole == Ds18Role::NONE ||
        candidate.flowSensorRole == candidate.returnSensorRole) {
      server.send(400, "text/plain", "Vorlauf- und Ruecklaufsensor muessen gesetzt und verschieden sein");
      return;
    }
    if (!SensorAssignments::hasRole(s_ctx->assignments, candidate.flowSensorRole) ||
        !SensorAssignments::hasRole(s_ctx->assignments, candidate.returnSensorRole)) {
      server.send(409, "text/plain", "Vorlauf- oder Ruecklaufsensor ist aktuell keinem DS18B20 zugeordnet");
      return;
    }
    const uint8_t gpio = EnergyMeter::gpioForFeedbackInput(candidate.feedbackInputIndex);
    const int usedByPump = pumpUsingFeedbackGpio(*s_ctx, gpio);
    if (usedByPump >= 0) {
      server.send(409, "text/plain", "Feedback-Eingang wird bereits von Pumpe " + String(usedByPump + 1) + " verwendet");
      return;
    }
  }

  s_ctx->config.energyMeter = candidate;
  if (!Storage::saveConfig(s_ctx->config)) {
    server.send(500, "text/plain", "Energiezaehler konnte nicht gespeichert werden");
    return;
  }
  EnergyMeter::reconfigure(*s_ctx);
  server.send(200, "text/plain", "OK");
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
    else if (a.channel == MaxChannel::CH4) channelKey = "ch4";

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

  // Snapshot-only endpoint: never trigger a MAX31865 conversion from HTTP.
  // The controller FSM owns sensor timing; the browser only reads its latest
  // completed values and therefore cannot stall regulation for ~80 ms/channel.

  String json = "{";
  json += "\"rNominalOhm\":" + String(MAX31865_RNOMINAL, 2) + ",";
  json += "\"rRefOhm\":" + String(MAX31865_RREF, 2) + ",";
  json += "\"wireMode\":\"" + String(MAX31865_USE_3WIRE ? "3-wire" : "2/4-wire") + "\",";
  json += "\"validResistanceMinOhm\":" + String(MAX31865_VALID_R_MIN_OHM, 1) + ",";
  json += "\"validResistanceMaxOhm\":" + String(MAX31865_VALID_R_MAX_OHM, 1) + ",";
  json += "\"validTempMinC\":" + String(MAX31865_VALID_TEMP_MIN_C, 1) + ",";
  json += "\"validTempMaxC\":" + String(MAX31865_VALID_TEMP_MAX_C, 1) + ",";

  json += "\"channels\":[";
  for (int i = 0; i < MAX_MAX31865_CHANNELS; i++) {
    if (i > 0) json += ",";

    const MaxChannelReading& r = s_ctx->maxReadings[i];
    const MaxChannelConfig& c =
      (i == 0) ? s_ctx->config.max1 :
      (i == 1) ? s_ctx->config.max2 :
      (i == 2) ? s_ctx->config.max3 :
                 s_ctx->config.max4;

    String channelKey = (i == 0) ? "ch1" : (i == 1) ? "ch2" : (i == 2) ? "ch3" : "ch4";

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
    json += "\"lastUpdateMs\":" + String(r.lastUpdateMs) + ",";
    json += "\"ageMs\":" + String(r.lastUpdateMs ? (uint32_t)(millis() - r.lastUpdateMs) : 0) + ",";
    json += "\"sequence\":" + String(r.sequence) + ",";
    json += "\"cycleDone\":" + String(s_ctx->maxRuntime[i].cycleDone ? "true" : "false") + ",";
    json += "\"fsmStep\":" + String((int)s_ctx->maxRuntime[i].step) + ",";
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

#if SOLARCTRL_MAX_DIAGNOSTICS
void handleMaxDebugJson() {
  if (!s_ctx) {
    server.send(500, "application/json", "{\"error\":\"no_context\"}");
    return;
  }

  const bool oneShot = !server.hasArg("oneshot") || server.arg("oneshot") != "0";
  server.send(200, "application/json", HeatSourcesMax::debugJson(*s_ctx, oneShot));
}


void handleMaxFaultTraceJson() {
  if (!s_ctx) {
    server.send(500, "application/json", "{\"error\":\"no_context\"}");
    return;
  }

  server.send(200, "application/json", HeatSourcesMax::faultTraceJson(*s_ctx));
}


void handleMaxSequenceDiagJson() {
  if (!s_ctx) {
    server.send(500, "application/json", "{\"error\":\"no_context\"}");
    return;
  }

  server.send(200, "application/json", HeatSourcesMax::sequenceDiagnosticJson(*s_ctx));
}

#endif

void handleNetworkJson() {
  if (!s_ctx) {
    server.send(500, "application/json", "{\"error\":\"no_context\"}");
    return;
  }

  const wl_status_t staStatus = WiFi.status();
  const bool staConnected = (staStatus == WL_CONNECTED);
  const bool apActive = s_ctx->networkApActive || !(WiFi.softAPIP()[0] == 0 && WiFi.softAPIP()[1] == 0 && WiFi.softAPIP()[2] == 0 && WiFi.softAPIP()[3] == 0);
  String json = "{";
  json += "\"apSsid\":\"" + jsonEscape(s_ctx->config.apName) + "\",";
  json += "\"apActive\":" + String(apActive ? "true" : "false") + ",";
  json += "\"apIp\":\"" + ipToString(WiFi.softAPIP()) + "\",";
  json += "\"wifiMode\":" + String((int)WiFi.getMode()) + ",";
  json += "\"staEnabled\":" + String(s_ctx->config.staEnabled ? "true" : "false") + ",";
  json += "\"staConfigured\":" + String(s_ctx->networkStaConfigured ? "true" : "false") + ",";
  json += "\"staSsid\":\"" + jsonEscape(s_ctx->config.staSsid) + "\",";
  json += "\"staConnected\":" + String(staConnected ? "true" : "false") + ",";
  json += "\"staStatus\":" + String((int)staStatus) + ",";
  json += "\"staStatusText\":\"" + jsonEscape(wifiStatusToText(staStatus).c_str()) + "\",";
  json += "\"staIp\":\"" + String(staConnected ? ipToString(WiFi.localIP()) : "") + "\",";
  json += "\"staRssi\":" + String(staConnected ? String(WiFi.RSSI()) : "null") + ",";
  json += "\"staEverConnected\":" + String(s_ctx->networkStaEverConnected ? "true" : "false") + ",";
  json += "\"staReconnectActive\":" + String(s_ctx->networkStaConnectAttemptActive ? "true" : "false") + ",";
  json += "\"lastStaReconnectAttemptMs\":" + String(s_ctx->networkLastStaReconnectAttemptMs) + ",";
  json += "\"staDisconnectedSinceMs\":" + String(s_ctx->networkStaDisconnectedSinceMs) + ",";
  json += "\"hostName\":\"" + jsonEscape(s_ctx->config.hostName) + "\",";
  json += "\"mqttEnabled\":" + String(s_ctx->config.mqttEnabled ? "true" : "false") + ",";
  json += "\"mqttHost\":\"" + jsonEscape(s_ctx->config.mqttHost) + "\",";
  json += "\"mqttPort\":" + String(s_ctx->config.mqttPort) + ",";
  json += "\"mqttUser\":\"" + jsonEscape(s_ctx->config.mqttUser) + "\",";
  json += "\"mqttPasswordSet\":" + String(s_ctx->config.mqttPassword[0] ? "true" : "false") + ",";
  json += "\"mqttClientId\":\"" + jsonEscape(s_ctx->config.mqttClientId) + "\",";
  json += "\"mqttBaseTopic\":\"" + jsonEscape(s_ctx->config.mqttBaseTopic) + "\",";
  json += "\"mqttDiscoveryEnabled\":" + String(s_ctx->config.mqttDiscoveryEnabled ? "true" : "false") + ",";
  json += "\"mqttPublishIntervalMs\":" + String(s_ctx->config.mqttPublishIntervalMs) + ",";
  json += "\"mqttConnected\":" + String(MqttBridge::connected() ? "true" : "false") + ",";
  json += "\"mqttConnecting\":" + String(MqttBridge::connecting() ? "true" : "false") + ",";
  json += "\"mqttLastError\":\"" + jsonEscape(MqttBridge::lastError()) + "\",";
  json += "\"mqttLastConnectAttemptMs\":" + String(MqttBridge::lastConnectAttemptMs()) + ",";
  json += "\"ntpEnabled\":" + String(s_ctx->config.ntpEnabled ? "true" : "false") + ",";
  json += "\"ntpServer\":\"" + jsonEscape(s_ctx->config.ntpServer) + "\",";
  json += "\"ntpStatus\":\"" + String(TimeService::statusText()) + "\",";
  json += "\"ntpLastSyncEpoch\":" + String(TimeService::lastSyncEpoch()) + ",";
  json += "\"timeValid\":" + String(TimeService::valid() ? "true" : "false") + ",";
  json += "\"dateText\":\"" + jsonEscape(TimeService::dateText().c_str()) + "\",";
  json += "\"timeText\":\"" + jsonEscape(TimeService::timeText().c_str()) + "\",";
  json += "\"forecastEnabled\":" + String(s_ctx->config.forecastEnabled ? "true" : "false") + ",";
  json += "\"forecastLatitude\":" + String(s_ctx->config.forecastLatitude, 5) + ",";
  json += "\"forecastLongitude\":" + String(s_ctx->config.forecastLongitude, 5) + ",";
  json += "\"forecastIntervalMin\":" + String(s_ctx->config.forecastIntervalMs / 60000UL) + ",";
  json += "\"forecastAuxDelayEnabled\":" + String(s_ctx->config.forecastAuxDelayEnabled ? "true" : "false") + ",";
  json += "\"forecastAuxMaxWaitHours\":" + String(s_ctx->config.forecastAuxMaxWaitMs / 3600000.0f, 1) + ",";
  json += "\"forecastSolarPriorityEnabled\":" + String(s_ctx->config.forecastSolarPriorityEnabled ? "true" : "false") + ",";
  json += "\"forecast\":" + Forecast::json(*s_ctx) + ",";
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

  if (server.hasArg("staEnabled")) s_ctx->config.staEnabled = server.arg("staEnabled") == "1";

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

  if (server.hasArg("mqttEnabled")) s_ctx->config.mqttEnabled = server.arg("mqttEnabled") == "1";
  if (server.hasArg("mqttHost")) {
    String v = server.arg("mqttHost"); v.trim();
    if (v.length() >= (int)sizeof(s_ctx->config.mqttHost)) { server.send(400, "text/plain", "MQTT Broker/Host ist zu lang"); return; }
    v.toCharArray(s_ctx->config.mqttHost, sizeof(s_ctx->config.mqttHost));
  }
  if (server.hasArg("mqttPort")) {
    const int port = server.arg("mqttPort").toInt();
    if (port < 1 || port > 65535) { server.send(400, "text/plain", "MQTT Port ungueltig"); return; }
    s_ctx->config.mqttPort = (uint16_t)port;
  }
  if (server.hasArg("mqttUser")) {
    String v = server.arg("mqttUser"); v.trim();
    if (v.length() >= (int)sizeof(s_ctx->config.mqttUser)) { server.send(400, "text/plain", "MQTT Benutzername ist zu lang"); return; }
    v.toCharArray(s_ctx->config.mqttUser, sizeof(s_ctx->config.mqttUser));
  }
  if (server.hasArg("mqttPasswordClear") && server.arg("mqttPasswordClear") == "1") {
    s_ctx->config.mqttPassword[0] = '\0';
  } else if (server.hasArg("mqttPassword") && server.arg("mqttPassword").length() > 0) {
    String v = server.arg("mqttPassword");
    if (v.length() >= (int)sizeof(s_ctx->config.mqttPassword)) { server.send(400, "text/plain", "MQTT Passwort ist zu lang"); return; }
    v.toCharArray(s_ctx->config.mqttPassword, sizeof(s_ctx->config.mqttPassword));
  }
  if (server.hasArg("mqttClientId")) {
    String v = server.arg("mqttClientId"); v.trim(); if (!v.length()) v = "SolarCtrl";
    if (v.length() >= (int)sizeof(s_ctx->config.mqttClientId)) { server.send(400, "text/plain", "MQTT Client-ID ist zu lang"); return; }
    v.toCharArray(s_ctx->config.mqttClientId, sizeof(s_ctx->config.mqttClientId));
  }
  if (server.hasArg("mqttBaseTopic")) {
    String v = server.arg("mqttBaseTopic"); v.trim(); while (v.endsWith("/")) v.remove(v.length() - 1); if (!v.length()) v = "haus/solarctrl";
    if (v.length() >= (int)sizeof(s_ctx->config.mqttBaseTopic)) { server.send(400, "text/plain", "MQTT Basis-Topic ist zu lang"); return; }
    v.toCharArray(s_ctx->config.mqttBaseTopic, sizeof(s_ctx->config.mqttBaseTopic));
  }
  if (server.hasArg("mqttDiscoveryEnabled")) s_ctx->config.mqttDiscoveryEnabled = server.arg("mqttDiscoveryEnabled") == "1";
  if (server.hasArg("mqttPublishIntervalS")) {
    const long sec = server.arg("mqttPublishIntervalS").toInt();
    if (sec < 2 || sec > 300) { server.send(400, "text/plain", "MQTT Intervall muss 2..300 s sein"); return; }
    s_ctx->config.mqttPublishIntervalMs = (uint32_t)sec * 1000UL;
  }

  if (server.hasArg("ntpEnabled")) s_ctx->config.ntpEnabled = server.arg("ntpEnabled") == "1";
  if (server.hasArg("ntpServer")) { String v=server.arg("ntpServer"); v.trim(); if(!v.length()) v="pool.ntp.org"; if(v.length() >= (int)sizeof(s_ctx->config.ntpServer)){server.send(400,"text/plain","NTP Server ist zu lang");return;} v.toCharArray(s_ctx->config.ntpServer,sizeof(s_ctx->config.ntpServer)); }
  if (server.hasArg("forecastEnabled")) s_ctx->config.forecastEnabled = server.arg("forecastEnabled") == "1";
  if (server.hasArg("forecastLatitude")) { float v=server.arg("forecastLatitude").toFloat(); if(v < -90.0f || v > 90.0f){server.send(400,"text/plain","Breitengrad ungueltig");return;} s_ctx->config.forecastLatitude=v; }
  if (server.hasArg("forecastLongitude")) { float v=server.arg("forecastLongitude").toFloat(); if(v < -180.0f || v > 180.0f){server.send(400,"text/plain","Laengengrad ungueltig");return;} s_ctx->config.forecastLongitude=v; }
  if (server.hasArg("forecastIntervalMin")) { long m=server.arg("forecastIntervalMin").toInt(); if(m<15||m>180){server.send(400,"text/plain","Forecast Intervall muss 15..180 min sein");return;} s_ctx->config.forecastIntervalMs=(uint32_t)m*60000UL; }
  if (server.hasArg("forecastAuxDelayEnabled")) s_ctx->config.forecastAuxDelayEnabled = server.arg("forecastAuxDelayEnabled") == "1";
  if (server.hasArg("forecastAuxMaxWaitHours")) { float h=server.arg("forecastAuxMaxWaitHours").toFloat(); if(h<0.25f||h>24.0f){server.send(400,"text/plain","Forecast Wartezeit muss 0.25..24 h sein");return;} s_ctx->config.forecastAuxMaxWaitMs=(uint32_t)(h*3600000.0f); }
  if (server.hasArg("forecastSolarPriorityEnabled")) s_ctx->config.forecastSolarPriorityEnabled = server.arg("forecastSolarPriorityEnabled") == "1";

  if (!Storage::saveConfig(s_ctx->config)) {
    server.send(500, "text/plain", "Speichern fehlgeschlagen");
    return;
  }

  MqttBridge::forceReconnect();
  TimeService::forceSync();
  Forecast::forceRefresh();
  server.send(200, "text/plain", "OK");
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
  else if (channelArg.equalsIgnoreCase("ch4")) channel = MaxChannel::CH4;

  HeatSourceRole role = HeatSourceRoles::fromKey(roleArg);
  if (role == HeatSourceRole::NONE) {
    // "nicht zugewiesen" ist eine gueltige Konfiguration. Vorhandene
    // Kanalzuordnung entfernen, damit deaktivierte ADCs keine alte Rolle behalten.
    HeatSourceAssignments::removeByChannel(s_ctx->heatSourceAssignments, channel);
    HeatSourceStorage::saveAssignments(s_ctx->heatSourceAssignments);
    HeatSourceAssignments::resolveHeatSources(*s_ctx);
    server.send(200, "text/plain", "OK");
    return;
  }

  if (!HeatSourceAssignments::setAssignment(s_ctx->heatSourceAssignments, channel, role)) {
    server.send(400, "text/plain", "Zuordnung fehlgeschlagen");
    return;
  }

  HeatSourceStorage::saveAssignments(s_ctx->heatSourceAssignments);
  HeatSourceAssignments::resolveHeatSources(*s_ctx);
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
  applyChannel(s_ctx->config.max4, "max4");

  // Deaktivierte MAX-Kanaele duerfen keine aktive Sensorrolle behalten.
  bool assignmentsChanged = false;
  if (!s_ctx->config.max1.enabled) assignmentsChanged |= HeatSourceAssignments::removeByChannel(s_ctx->heatSourceAssignments, MaxChannel::CH1);
  if (!s_ctx->config.max2.enabled) assignmentsChanged |= HeatSourceAssignments::removeByChannel(s_ctx->heatSourceAssignments, MaxChannel::CH2);
  if (!s_ctx->config.max3.enabled) assignmentsChanged |= HeatSourceAssignments::removeByChannel(s_ctx->heatSourceAssignments, MaxChannel::CH3);
  if (!s_ctx->config.max4.enabled) assignmentsChanged |= HeatSourceAssignments::removeByChannel(s_ctx->heatSourceAssignments, MaxChannel::CH4);

  Storage::saveConfig(s_ctx->config);
  if (assignmentsChanged) HeatSourceStorage::saveAssignments(s_ctx->heatSourceAssignments);
  HeatSourceAssignments::resolveHeatSources(*s_ctx);

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

uint8_t clampToUint8(int value, int minimum, int maximum) {
  if (value < minimum) return (uint8_t)minimum;
  if (value > maximum) return (uint8_t)maximum;
  return (uint8_t)value;
}

String outputRefValue(OutputKind kind, uint8_t index) {
  if (!outputRefAssigned(kind, index)) return "none:255";
  return outputRefKindToKey(kind) + ":" + String(index);
}

Ds18Role heatingCircuitFlowRole(uint8_t circuitIndex) {
  switch (circuitIndex) {
    case 0: return Ds18Role::HK1_FLOW;
    case 1: return Ds18Role::HK2_FLOW;
    case 2: return Ds18Role::HK3_FLOW;
    case 3: return Ds18Role::HK4_FLOW;
    default: return Ds18Role::NONE;
  }
}

Ds18Role heatingCircuitReturnRole(uint8_t circuitIndex) {
  switch (circuitIndex) {
    case 0: return Ds18Role::HK1_RETURN;
    case 1: return Ds18Role::HK2_RETURN;
    case 2: return Ds18Role::HK3_RETURN;
    case 3: return Ds18Role::HK4_RETURN;
    default: return Ds18Role::NONE;
  }
}

int8_t heatingCircuitRouteIndex(const PumpConfig& pump) {
  if (pump.sourceType != PumpSourceType::SENSOR_ROLE) return -1;
  for (uint8_t i = 0; i < MAX_HEATING_CIRCUITS; i++) {
    if (pump.sourceSensorRole == heatingCircuitFlowRole(i) &&
        pump.sinkRole == heatingCircuitReturnRole(i)) {
      return (int8_t)i;
    }
  }
  return -1;
}

int8_t partialHeatingCircuitRouteIndex(const PumpConfig& pump) {
  if (pump.sourceType != PumpSourceType::SENSOR_ROLE) return -1;

  for (uint8_t i = 0; i < MAX_HEATING_CIRCUITS; i++) {
    if (pump.sourceSensorRole == heatingCircuitFlowRole(i) ||
        pump.sinkRole == heatingCircuitReturnRole(i)) {
      return (int8_t)i;
    }
  }
  return -1;
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
    // Heizkreis-Pumpen werden ab jetzt zentral im Pumpenmenue belegt.
    // Alte hk.pumpOutput-Felder bleiben nur zur Migration/Freigabe erhalten
    // und duerfen keine Ausgangsbelegung mehr blockieren.
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
      if (sameOutputRef(aux.pumpOutput.kind, aux.pumpOutput.index, kind, index)) { usage = "Zusatzheizung Pumpe"; return true; }
      if (sameOutputRef(aux.heaterOutput1.kind, aux.heaterOutput1.index, kind, index)) { usage = "Zusatzheizung Heizstab 1"; return true; }
      if (sameOutputRef(aux.heaterOutput2.kind, aux.heaterOutput2.index, kind, index)) { usage = "Zusatzheizung Heizstab 2"; return true; }
      if (sameOutputRef(aux.heaterOutput3.kind, aux.heaterOutput3.index, kind, index)) { usage = "Zusatzheizung Heizstab 3"; return true; }
    }

    // Ofenpumpe wird nicht mehr direkt im Ofenmodul belegt, sondern als
    // normale Pumpe mit Quelle "Ofen" im Pumpenmenue.
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

    const AuxHeaterConfig& aux = ctx.config.auxHeater;
    if (!ignoreAuxHeater && aux.enabled) {
      if (sameOutputRef(aux.pumpOutput.kind, aux.pumpOutput.index, kind, index)) { usage = "Zusatzheizung Pumpe"; return true; }
      if (sameOutputRef(aux.heaterOutput1.kind, aux.heaterOutput1.index, kind, index)) { usage = "Zusatzheizung Heizstab 1"; return true; }
      if (sameOutputRef(aux.heaterOutput2.kind, aux.heaterOutput2.index, kind, index)) { usage = "Zusatzheizung Heizstab 2"; return true; }
      if (sameOutputRef(aux.heaterOutput3.kind, aux.heaterOutput3.index, kind, index)) { usage = "Zusatzheizung Heizstab 3"; return true; }
    }

    // Ofenpumpe wird nicht mehr direkt im Ofenmodul belegt, sondern als
    // normale Pumpe mit Quelle "Ofen" im Pumpenmenue.
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


  String wifiStatusToText(wl_status_t status) {
    switch (status) {
      case WL_CONNECTED: return "verbunden";
      case WL_NO_SSID_AVAIL: return "SSID nicht gefunden";
      case WL_CONNECT_FAILED: return "Verbindung fehlgeschlagen";
      case WL_CONNECTION_LOST: return "Verbindung verloren";
      case WL_DISCONNECTED: return "getrennt";
      case WL_IDLE_STATUS: return "wartet";
      default: return String("Status ") + String((int)status);
    }
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
        if (energyMeterUsesFeedbackGpio(ctx, candidate.feedbackPin)) {
          error = "Feedback-Eingang wird vom Energiezaehler verwendet";
          return false;
        }
      }
    }

    if (candidate.enabled && candidate.mode != PumpMode::OFF &&
        candidate.sourceType == PumpSourceType::SENSOR_ROLE) {
      const int8_t fullHk = heatingCircuitRouteIndex(candidate);
      const int8_t partialHk = partialHeatingCircuitRouteIndex(candidate);

      if (partialHk >= 0 && fullHk < 0) {
        error = "Heizkreis-Pumpe muss Quelle HKx Vorlauf und Ziel HKx Ruecklauf desselben Heizkreises verwenden";
        return false;
      }

      if (fullHk >= 0) {
        for (uint8_t i = 0; i < MAX_PUMPS; i++) {
          if (i == pumpIndex) continue;
          const PumpConfig& other = ctx.config.pumps[i];
          if (!other.enabled || other.mode == PumpMode::OFF) continue;
          if (heatingCircuitRouteIndex(other) == fullHk) {
            error = "Es darf nur eine Pumpe fuer HK" + String((int)fullHk + 1) + " konfiguriert sein";
            return false;
          }
        }
      }
    }

    if (candidate.enabled && candidate.mode != PumpMode::OFF &&
        candidate.sourceType == PumpSourceType::HEAT_SOURCE_ROLE &&
        candidate.sourceRole == HeatSourceRole::ALT_SOURCE_OVEN) {
      for (uint8_t i = 0; i < MAX_PUMPS; i++) {
        if (i == pumpIndex) continue;
        const PumpConfig& other = ctx.config.pumps[i];
        if (!other.enabled || other.mode == PumpMode::OFF) continue;
        if (other.sourceType == PumpSourceType::HEAT_SOURCE_ROLE &&
            other.sourceRole == HeatSourceRole::ALT_SOURCE_OVEN) {
          error = "Es darf nur eine Pumpe mit Quelle Ofen konfiguriert sein";
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
      json += "\"pwmProfile\":\"" + pwmProfileToKey(p.pwmProfile) + "\",";
      json += "\"feedbackPin\":" + String(p.feedbackPin) + ",";
      json += "\"targetDiff\":" + String(p.targetDiff, 2) + ",";
      json += "\"hysteresis\":" + String(p.hysteresis, 2) + ",";
      json += "\"startDiff\":" + String(p.startDiff, 2) + ",";
      json += "\"pidKp\":" + String(p.pidKp, 4) + ",";
      json += "\"pidKi\":" + String(p.pidKi, 4) + ",";
      json += "\"pidKd\":" + String(p.pidKd, 4) + ",";
      json += "\"mlStartDiffMinC\":" + String(p.mlStartDiffMinC, 2) + ",";
      json += "\"mlStartDiffMaxC\":" + String(p.mlStartDiffMaxC, 2) + ",";
      json += "\"mlTargetDiffMinC\":" + String(p.mlTargetDiffMinC, 2) + ",";
      json += "\"mlTargetDiffMaxC\":" + String(p.mlTargetDiffMaxC, 2) + ",";
      json += "\"mlHysteresisMinC\":" + String(p.mlHysteresisMinC, 2) + ",";
      json += "\"mlHysteresisMaxC\":" + String(p.mlHysteresisMaxC, 2) + ",";
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
    json += "{\"index\":255,\"label\":\"kein Feedback\",\"usedBy\":-1,\"energyMeterUsed\":false}";
    first = false;
    for (uint8_t fp = 0; fp < FEEDBACK_INPUT_COUNT; fp++) {
      const uint8_t pin = FEEDBACK_INPUT_PINS[fp];
      json += ",";
      const String label = feedbackInputLabel(fp, pin);
      json += "{\"index\":" + String(pin) + ",\"label\":\"" + label + "\",\"usedBy\":" + String(feedbackPinUsedBy(*s_ctx, pin, -1)) + ",\"energyMeterUsed\":" + String(energyMeterUsesFeedbackGpio(*s_ctx, pin) ? "true" : "false") + "}";
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

    if (server.hasArg("mlStartDiffMinC")) p.mlStartDiffMinC = server.arg("mlStartDiffMinC").toFloat();
    if (server.hasArg("mlStartDiffMaxC")) p.mlStartDiffMaxC = server.arg("mlStartDiffMaxC").toFloat();
    if (server.hasArg("mlTargetDiffMinC")) p.mlTargetDiffMinC = server.arg("mlTargetDiffMinC").toFloat();
    if (server.hasArg("mlTargetDiffMaxC")) p.mlTargetDiffMaxC = server.arg("mlTargetDiffMaxC").toFloat();
    if (server.hasArg("mlHysteresisMinC")) p.mlHysteresisMinC = server.arg("mlHysteresisMinC").toFloat();
    if (server.hasArg("mlHysteresisMaxC")) p.mlHysteresisMaxC = server.arg("mlHysteresisMaxC").toFloat();

    const bool startBoundsValid =
      p.mlStartDiffMinC <= 0.0f ||
      p.mlStartDiffMaxC <= 0.0f ||
      p.mlStartDiffMaxC >= p.mlStartDiffMinC;
    const bool targetBoundsValid =
      p.mlTargetDiffMinC <= 0.0f ||
      p.mlTargetDiffMaxC <= 0.0f ||
      p.mlTargetDiffMaxC >= p.mlTargetDiffMinC;
    const bool hysteresisBoundsValid =
      p.mlHysteresisMinC <= 0.0f ||
      p.mlHysteresisMaxC <= 0.0f ||
      p.mlHysteresisMaxC >= p.mlHysteresisMinC;

    if (!startBoundsValid || !targetBoundsValid || !hysteresisBoundsValid) {
      p = old;
      server.send(400, "text/plain", "ML-Grenzen ungueltig: Max muss >= Min sein");
      return;
    }

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
    const int8_t ovenPumpIndex = Pumps::configuredOvenPumpIndex(*s_ctx);
    const PumpConfig* ovenPump = (ovenPumpIndex >= 0) ? &s_ctx->config.pumps[(uint8_t)ovenPumpIndex] : nullptr;

    String ovenPumpLabel;
    if (ovenPump) {
      ovenPumpLabel = "Pumpe " + String((int)ovenPumpIndex + 1) + " / " + pumpModeLabel(ovenPump->mode);
      if (ovenPump->mode == PumpMode::PWM) {
        ovenPumpLabel += " / Enable ";
        ovenPumpLabel += relayHardwareLabel(ovenPump->relayIndex);
        ovenPumpLabel += " / PWM ";
        ovenPumpLabel += pwmOutputHardwareLabel(ovenPump->pwmChannel);
      } else {
        ovenPumpLabel += " / ";
        ovenPumpLabel += relayHardwareLabel(ovenPump->relayIndex);
      }
    }

    String json = "{";

    json += "\"status\":{";
    json += "\"ovenTemperatureC\":";
    json += jsonFloat(OvenControl::ovenTemperatureC(), 1);
    json += ",\"servoAngle\":";
    json += String(OvenControl::servoAngle());
    json += ",\"servoOpeningPercent\":";
    json += String(OvenControl::servoOpeningPercent());
    json += ",\"targetTemperatureReached\":";
    json += (OvenControl::targetTemperatureReached() ? "true" : "false");
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
    json += ",\"ovenPumpConfigured\":";
    json += ovenPump ? "true" : "false";
    json += ",\"ovenPumpIndex\":";
    json += ovenPump ? String(ovenPumpIndex) : String(-1);
    json += ",\"ovenPumpLabel\":";
    if (ovenPump) json += "\"" + jsonEscape(ovenPumpLabel.c_str()) + "\"";
    else json += "null";
    json += ",\"ovenPumpPwmPercent\":";
    json += ovenPump ? jsonFloat(ovenPump->lastPwmPercent, 1) : String("null");
    json += "}";

    json += ",\"config\":{";
    json += "\"enabled\":";
    json += cfg.enabled ? "true" : "false";
    json += ",\"ovenPumpIndex\":";
    json += ovenPump ? String(ovenPumpIndex) : String(-1);
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
    json += ",\"servoClosedAngle\":";
    json += String(cfg.servoClosedAngle);
    json += ",\"servoOpenAngle\":";
    json += String(cfg.servoOpenAngle);
    json += ",\"servoStandbyOpeningPercent\":";
    json += String(cfg.servoStandbyOpeningPercent);
    json += ",\"servoStartOpeningPercent\":";
    json += String(cfg.servoStartOpeningPercent);
    json += ",\"servoMinimumOpeningPercent\":";
    json += String(cfg.servoMinimumOpeningPercent);
    json += ",\"servoMaximumOpeningPercent\":";
    json += String(cfg.servoMaximumOpeningPercent);
    json += ",\"servoStepPercent\":";
    json += String(cfg.servoStepPercent);
    json += ",\"servoDeadbandC\":";
    json += jsonFloat(cfg.servoDeadbandC, 1);
    json += ",\"burnoutVentMinutes\":";
    json += String(cfg.burnoutVentMinutes);
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

    if (server.hasArg("servoClosedAngle"))
      cfg.servoClosedAngle = (uint8_t)server.arg("servoClosedAngle").toInt();

    if (server.hasArg("servoOpenAngle"))
      cfg.servoOpenAngle = (uint8_t)server.arg("servoOpenAngle").toInt();

    if (server.hasArg("servoStandbyOpeningPercent"))
      cfg.servoStandbyOpeningPercent = (uint8_t)server.arg("servoStandbyOpeningPercent").toInt();

    if (server.hasArg("servoStartOpeningPercent"))
      cfg.servoStartOpeningPercent = (uint8_t)server.arg("servoStartOpeningPercent").toInt();

    if (server.hasArg("servoMinimumOpeningPercent"))
      cfg.servoMinimumOpeningPercent = (uint8_t)server.arg("servoMinimumOpeningPercent").toInt();

    if (server.hasArg("servoMaximumOpeningPercent"))
      cfg.servoMaximumOpeningPercent = (uint8_t)server.arg("servoMaximumOpeningPercent").toInt();

    if (server.hasArg("servoStepPercent"))
      cfg.servoStepPercent = (uint8_t)server.arg("servoStepPercent").toInt();

    if (server.hasArg("servoDeadbandC"))
      cfg.servoDeadbandC = server.arg("servoDeadbandC").toFloat();

    if (server.hasArg("burnoutVentMinutes"))
      cfg.burnoutVentMinutes = (uint16_t)server.arg("burnoutVentMinutes").toInt();

    cfg.servoStandbyOpeningPercent = clampToUint8(cfg.servoStandbyOpeningPercent, 0, 100);
    cfg.servoStartOpeningPercent = clampToUint8(cfg.servoStartOpeningPercent, 0, 100);
    cfg.servoMinimumOpeningPercent = clampToUint8(cfg.servoMinimumOpeningPercent, 0, 100);
    cfg.servoMaximumOpeningPercent = clampToUint8(cfg.servoMaximumOpeningPercent, 0, 100);
    cfg.servoStepPercent = clampToUint8(cfg.servoStepPercent, 1, 100);
    cfg.servoClosedAngle = clampToUint8(cfg.servoClosedAngle, 0, 180);
    cfg.servoOpenAngle = clampToUint8(cfg.servoOpenAngle, 0, 180);

    if (server.hasArg("pidKp"))
      cfg.pidKp = server.arg("pidKp").toFloat();

    if (server.hasArg("pidKi"))
      cfg.pidKi = server.arg("pidKi").toFloat();

    if (server.hasArg("pidKd"))
      cfg.pidKd = server.arg("pidKd").toFloat();

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
    json += "\"storageMaximumActive\":" + String(st.storageMaximumActive ? "true" : "false") + ",";
    json += "\"storageCriticalOvertemperatureActive\":" + String(st.storageCriticalOvertemperatureActive ? "true" : "false") + ",";
    json += "\"storageForcedCoolingActive\":" + String(st.storageForcedCoolingActive ? "true" : "false") + ",";
    json += "\"storageForcedCoolingPumpCount\":" + String(st.storageForcedCoolingPumpCount) + ",";
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

  void handleTestServoJson() {
    String json = "{\"servo\":{";
    json += "\"available\":true,";
    json += "\"label\":\"Ofenklappen-Servo\",";
    json += "\"pin\":" + String(OVEN_SERVO_PIN) + ",";
    json += "\"initialized\":" + String(ServoDriver::initialized() ? "true" : "false") + ",";
    json += "\"angle\":" + String(ServoDriver::angle());
    json += "}}";
    server.send(200, "application/json", json);
  }

  void handleTestSensorsJson() {
    if (!s_ctx) { server.send(500, "application/json", "{\"error\":\"no_context\"}"); return; }

    String json = "{\"sensors\":{\"ds18b20\":[";
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
    for (uint8_t i = 0; i < MAX_MAX31865_CHANNELS; i++) {
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
    json += "]}}";
    server.send(200, "application/json", json);
  }

  void handleTestSafetyJson() {
    String json = "{";
    appendTestSafetyJson(json);
    json += "}";
    server.send(200, "application/json", json);
  }

  void handleTestDiagnosticsJson() {
    if (!s_ctx) { server.send(500, "application/json", "{\"error\":\"no_context\"}"); return; }

    String json = "{\"diagnostics\":{";
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
    json += "},";

    String validationError;
    const bool outputValid = OutputValidation::validateAll(*s_ctx, validationError);
    json += "\"outputValidation\":{";
    json += "\"ok\":" + String(outputValid ? "true" : "false") + ",";
    json += "\"message\":\"" + jsonEscape(validationError.c_str()) + "\"";
    json += "}}";
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

    DBG_PRINT("PCA TEST Kanal ");
    DBG_PRINT(ch);
    DBG_PRINT(" -> ");
    DBG_PRINT(percent);
    DBG_PRINTLN(" %");
    DBG_FLUSH();
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

    DBG_PRINT("SERVO TEST GPIO");
    DBG_PRINT(OVEN_SERVO_PIN);
    DBG_PRINT(" -> ");
    DBG_PRINT(actual);
    DBG_PRINTLN(" Grad");
    DBG_FLUSH();

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
      server.send(409, "text/plain", "Heizkreispumpe wird zentral im Pumpenmenue konfiguriert. Bitte die Pumpe dort oder im Pumpen/Routing-Test testen.");
      return;
    } else {
      server.send(400, "text/plain", "ungueltiger Ausgang");
      return;
    }

    server.send(200, "text/plain", "OK");
  }

  void allOutputsOffSafe() {
    if (!s_ctx) return;

    // Zuerst die Modul-Runtimes in einen eindeutig AUS-Zustand bringen.
    HeatingCircuits::allOff(*s_ctx);
    Pumps::allOff(*s_ctx);
    Valves::allOff(*s_ctx);

    // Danach die Hardware global absichern. PCA-Schaltausgaenge muessen dabei
    // ihr jeweiliges Profil beruecksichtigen; ein fixes SOLAR-Profil waere fuer
    // HEATING-Ausgaenge elektrisch nicht zwingend AUS.
    RelayOutputs::allOff(*s_ctx);
    PwmDriver::allOff(s_ctx->config);

    // Der Servo darf nicht auf den elektrischen Winkel 0 Grad gezwungen werden.
    // Verwendet wird die konfigurierte Standby-/Safety-Oeffnung, z. B. 0 %
    // Oeffnung -> 45 Grad bei der aktuellen Ofenkalibrierung.
    OvenControl::applyStandbyServoPosition(*s_ctx);
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

  void handleBuzzerMute() {
    if (!s_ctx) { server.send(500, "text/plain", "Kein Kontext"); return; }
    bool mute = true;
    if (server.hasArg("state")) {
      String v = server.arg("state"); v.toLowerCase();
      mute = !(v == "0" || v == "off" || v == "false");
    }
    Alarms::setBuzzerMuted(mute);
    server.send(200, "application/json", String("{\"muted\":") + (Alarms::buzzerMuted() ? "true" : "false") + "}");
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
    Storage::resetEnergyMeterRuntime(s_ctx->energyMeter);
    Storage::saveEnergyMeterRuntime(s_ctx->energyMeter);
    EnergyMeter::reconfigure(*s_ctx);
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
  json += ",\"collectorApertureM2\":"; json += jsonFloat(c.collectorApertureM2, 3);
  json += ",\"collectorTiltDeg\":"; json += jsonFloat(c.collectorTiltDeg, 1);
  json += ",\"collectorAzimuthDeg\":"; json += jsonFloat(c.collectorAzimuthDeg, 1);

  json += ",\"frostEnabled\":";
  json += c.frostEnabled ? "true" : "false";
  json += ",\"frostCollectorOnC\":";
  json += jsonFloat(c.frostCollectorOnC, 1);
  json += ",\"frostCollectorOffC\":";
  json += jsonFloat(c.frostCollectorOffC, 1);
  json += ",\"frostSafeCollectorTemperatureC\":";
  json += jsonFloat(c.frostSafeCollectorTemperatureC, 1);
  json += ",\"frostSinkMinC\":";
  json += jsonFloat(c.frostSinkMinC, 1);
  json += ",\"frostPumpPercent\":";
  json += jsonFloat(c.frostPumpPercent, 0);
  json += ",\"frostProtectionStorageRole\":";
  json += String((int)c.frostProtectionStorageRole);
  json += ",\"frostRequiredProtectionC\":"; json += jsonFloat(c.frostRequiredProtectionC, 1);
  json += ",\"frostProtectionReserveK\":"; json += jsonFloat(c.frostProtectionReserveK, 1);

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

  // Physikalische Solaranlagendaten werden auf /plant-data.html gepflegt.
  // Alte Clients duerfen sie weiterhin mitsenden, fehlende Felder duerfen sie aber niemals ueberschreiben.
  if (server.hasArg("solarFluidType")) c.solarFluidType = (SolarFluidType)server.arg("solarFluidType").toInt();
  if (server.hasArg("solarHydraulicType")) c.solarHydraulicType = (SolarHydraulicType)server.arg("solarHydraulicType").toInt();
  if (server.hasArg("solarCollectorType")) c.solarCollectorType = (SolarCollectorType)server.arg("solarCollectorType").toInt();
  if (server.hasArg("collectorApertureM2")) c.collectorApertureM2 = constrain(server.arg("collectorApertureM2").toFloat(), 0.1f, 100.0f);
  if (server.hasArg("collectorTiltDeg")) c.collectorTiltDeg = constrain(server.arg("collectorTiltDeg").toFloat(), 0.0f, 90.0f);
  if (server.hasArg("collectorAzimuthDeg")) { float a=server.arg("collectorAzimuthDeg").toFloat(); while(a<0)a+=360.0f; while(a>=360)a-=360.0f; c.collectorAzimuthDeg=a; }

  if (server.hasArg("frostEnabled")) c.frostEnabled = server.arg("frostEnabled").toInt() != 0;
  if (server.hasArg("frostCollectorOnC")) c.frostCollectorOnC = server.arg("frostCollectorOnC").toFloat();
  if (server.hasArg("frostCollectorOffC")) c.frostCollectorOffC = server.arg("frostCollectorOffC").toFloat();
  if (server.hasArg("frostSafeCollectorTemperatureC")) c.frostSafeCollectorTemperatureC = server.arg("frostSafeCollectorTemperatureC").toFloat();
  if (server.hasArg("frostSinkMinC")) c.frostSinkMinC = server.arg("frostSinkMinC").toFloat();
  if (server.hasArg("frostPumpPercent")) c.frostPumpPercent = server.arg("frostPumpPercent").toFloat();
  if (server.hasArg("frostProtectionStorageRole")) c.frostProtectionStorageRole = (Ds18Role)server.arg("frostProtectionStorageRole").toInt();
  if (server.hasArg("frostRequiredProtectionC")) c.frostRequiredProtectionC = constrain(server.arg("frostRequiredProtectionC").toFloat(), -60.0f, 5.0f);
  if (server.hasArg("frostProtectionReserveK")) c.frostProtectionReserveK = constrain(server.arg("frostProtectionReserveK").toFloat(), 0.0f, 20.0f);

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

      String usage;
      const bool locked = outputUsageLabel(*s_ctx, OutputKind::RELAY, i, usage);

      if (!first) json += ",";
      first = false;
      json += "{\"kind\":\"relay\",\"index\":" + String(i) + ",\"label\":\"" + relayHardwareLabel(i) + "\",\"function\":\"" + String(RelayOutputs::functionToKey(r.function)) + "\",\"mode\":\"switch\"";
      json += ",\"locked\":" + String(locked ? "true" : "false");
      json += ",\"usedBy\":";
      if (locked) json += "\"" + jsonEscape(usage.c_str()) + "\""; else json += "null";
      json += "}";
    }
    for (uint8_t i = 0; i < PWM_OUTPUT_COUNT; i++) {
      const PwmOutputConfig& po = s_ctx->config.pwmOutputs[i];
      if (!po.enabled) continue;

      String usage;
      const bool locked = outputUsageLabel(*s_ctx, OutputKind::PWM_OUTPUT, i, usage);

      if (!first) json += ",";
      first = false;
      json += "{\"kind\":\"pwm\",\"index\":" + String(i) + ",\"label\":\"" + pwmOutputHardwareLabel(i) + "\",\"mode\":\"" + pwmOutputModeToKey(po.mode) + "\",\"function\":\"" + String(RelayOutputs::functionToKey(po.function)) + "\"";
      json += ",\"locked\":" + String(locked ? "true" : "false");
      json += ",\"usedBy\":";
      if (locked) json += "\"" + jsonEscape(usage.c_str()) + "\""; else json += "null";
      json += "}";
    }
    json += "]";
  }


  String heatingCircuitSensorTemperatureOnly(Ds18Role role) {
    if (role == Ds18Role::NONE) return String("-");

    float tempC = NAN;
    bool valid = false;
    if (s_ctx && SensorAssignments::readByRole(s_ctx->ds18b20, s_ctx->assignments, role, tempC, valid) && valid && !isnan(tempC)) {
      return String(tempC, 1) + " °C";
    }

    return String("-");
  }


  String heatingCircuitSensorSummary(Ds18Role role) {
    if (role == Ds18Role::NONE) return String("-");

    String label = SensorRoles::toLabel(role);
    float tempC = NAN;
    bool valid = false;
    if (s_ctx && SensorAssignments::readByRole(s_ctx->ds18b20, s_ctx->assignments, role, tempC, valid) && valid && !isnan(tempC)) {
      label += ": ";
      label += String(tempC, 1);
      label += " °C";
      return label;
    }

    if (s_ctx && SensorAssignments::hasRole(s_ctx->assignments, role)) {
      label += ": ungueltig";
    } else {
      label += ": nicht zugeordnet";
    }
    return label;
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
      json += "\"pumpTargetDeltaC\":" + String(cfg.pumpTargetDeltaC, 2) + ",";
      json += "\"pumpFullDeltaC\":" + String(cfg.pumpFullDeltaC, 2) + ",";
      const int8_t hkPumpIndex = Pumps::configuredHeatingCircuitPumpIndex(*s_ctx, i);
      json += "\"detectedPumpIndex\":" + String(hkPumpIndex) + ",";
      if (hkPumpIndex >= 0) {
        const PumpConfig& hp = s_ctx->config.pumps[(uint8_t)hkPumpIndex];
        String hpLabel = "Pumpe " + String((int)hkPumpIndex + 1) + " / " + pumpModeLabel(hp.mode);
        hpLabel += " / " + String(SensorRoles::toLabel(hp.sourceSensorRole));
        hpLabel += " -> " + String(SensorRoles::toLabel(hp.sinkRole));
        hpLabel += " / Enable " + relayHardwareLabel(hp.relayIndex);
        if (hp.mode == PumpMode::PWM) {
          hpLabel += " / PWM " + pwmOutputHardwareLabel(hp.pwmChannel);
          hpLabel += " / " + String(hp.minPwmPercent, 0) + "-" + String(hp.maxPwmPercent, 0) + "%";
        }
        json += "\"detectedPumpLabel\":\"" + jsonEscape(hpLabel.c_str()) + "\",";
      } else {
        json += "\"detectedPumpLabel\":\"Keine Heizkreispumpe im Pumpenmenue gefunden\",";
      }
      const Ds18Role effectiveFlowRole = HeatingCircuits::effectiveFlowSensorRole(i, cfg);
      const Ds18Role effectiveReturnRole = HeatingCircuits::effectiveReturnSensorRole(i, cfg);
      const Ds18Role effectiveRoomRole = HeatingCircuits::effectiveRoomSensorRole(i, cfg);
      const Ds18Role effectiveOutsideRole = HeatingCircuits::effectiveOutsideSensorRole(cfg);
      json += "\"flowSensorRole\":" + String((int)effectiveFlowRole) + ",";
      json += "\"returnSensorRole\":" + String((int)effectiveReturnRole) + ",";
      json += "\"configuredFlowSensorRole\":" + String((int)cfg.flowSensorRole) + ",";
      json += "\"configuredReturnSensorRole\":" + String((int)cfg.returnSensorRole) + ",";
      json += "\"roomSensorRole\":" + String((int)effectiveRoomRole) + ",";
      json += "\"configuredRoomSensorRole\":" + String((int)cfg.roomSensorRole) + ",";
      json += "\"bufferReferenceRole\":" + String((int)cfg.bufferReferenceRole) + ",";
      json += "\"outsideSensorRole\":" + String((int)effectiveOutsideRole) + ",";
      json += "\"configuredOutsideSensorRole\":" + String((int)cfg.outsideSensorRole) + ",";
      json += "\"autoFlowSensorLabel\":\"" + jsonEscape(heatingCircuitSensorTemperatureOnly(effectiveFlowRole).c_str()) + "\",";
      json += "\"autoReturnSensorLabel\":\"" + jsonEscape(heatingCircuitSensorTemperatureOnly(effectiveReturnRole).c_str()) + "\",";
      json += "\"autoRoomSensorLabel\":\"" + jsonEscape(heatingCircuitSensorSummary(effectiveRoomRole).c_str()) + "\",";
      json += "\"autoOutsideSensorLabel\":\"" + jsonEscape(heatingCircuitSensorTemperatureOnly(effectiveOutsideRole).c_str()) + "\",";
      json += "\"roomControlEnabled\":" + String(cfg.roomControlEnabled ? "true" : "false") + ",";
      json += "\"fixedFlowTemperatureC\":" + String(cfg.fixedFlowTemperatureC, 2) + ",";
      json += "\"maximumFlowTemperatureC\":" + String(cfg.maximumFlowTemperatureC, 2) + ",";
      json += "\"minimumFlowTemperatureC\":" + String(cfg.minimumFlowTemperatureC, 2) + ",";
      json += "\"roomTargetTemperatureC\":" + String(cfg.roomTargetTemperatureC, 2) + ",";
      json += "\"roomInfluenceK\":" + String(cfg.roomInfluenceK, 2) + ",";
      json += "\"nightSetbackEnabled\":" + String(cfg.nightSetbackEnabled ? "true" : "false") + ",";
      json += "\"nightSetbackStartMinute\":" + String(cfg.nightSetbackStartMinute) + ",";
      json += "\"nightSetbackEndMinute\":" + String(cfg.nightSetbackEndMinute) + ",";
      json += "\"nightSetbackK\":" + String(cfg.nightSetbackK, 2) + ",";
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
      json += "\"returnTemperatureC\":" + jsonFloat(rt.returnTemperatureC, 2) + ",";
      json += "\"roomTemperatureC\":" + jsonFloat(rt.roomTemperatureC, 2) + ",";
      json += "\"outsideTemperatureC\":" + jsonFloat(rt.outsideTemperatureC, 2) + ",";
      json += "\"targetFlowTemperatureC\":" + jsonFloat(rt.targetFlowTemperatureC, 2) + ",";
      json += "\"effectiveRoomTargetTemperatureC\":" + jsonFloat(rt.effectiveRoomTargetTemperatureC, 2) + ",";
      json += "\"nightSetbackActive\":" + String(rt.nightSetbackActive ? "true" : "false") + ",";
      json += "\"spreadTemperatureC\":" + jsonFloat(rt.spreadTemperatureC, 2) + ",";
      json += "\"pumpPercent\":" + String(rt.pumpPercent) + ",";
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
    if (server.hasArg("pumpTargetDeltaC")) candidate.pumpTargetDeltaC = server.arg("pumpTargetDeltaC").toFloat();
    if (server.hasArg("pumpFullDeltaC")) candidate.pumpFullDeltaC = server.arg("pumpFullDeltaC").toFloat();
    if (server.hasArg("flowSensorRole")) candidate.flowSensorRole = (Ds18Role)server.arg("flowSensorRole").toInt();
    if (server.hasArg("returnSensorRole")) candidate.returnSensorRole = (Ds18Role)server.arg("returnSensorRole").toInt();
    if (server.hasArg("roomSensorRole")) candidate.roomSensorRole = (Ds18Role)server.arg("roomSensorRole").toInt();
    if (server.hasArg("bufferReferenceRole")) candidate.bufferReferenceRole = (Ds18Role)server.arg("bufferReferenceRole").toInt();
    if (server.hasArg("outsideSensorRole")) candidate.outsideSensorRole = (Ds18Role)server.arg("outsideSensorRole").toInt();
    if (server.hasArg("fixedFlowTemperatureC")) candidate.fixedFlowTemperatureC = server.arg("fixedFlowTemperatureC").toFloat();
    if (server.hasArg("maximumFlowTemperatureC")) candidate.maximumFlowTemperatureC = server.arg("maximumFlowTemperatureC").toFloat();
    if (server.hasArg("minimumFlowTemperatureC")) candidate.minimumFlowTemperatureC = server.arg("minimumFlowTemperatureC").toFloat();
    candidate.roomControlEnabled = server.hasArg("roomControlEnabled") && server.arg("roomControlEnabled").toInt() != 0;
    if (server.hasArg("roomTargetTemperatureC")) candidate.roomTargetTemperatureC = server.arg("roomTargetTemperatureC").toFloat();
    if (server.hasArg("roomInfluenceK")) candidate.roomInfluenceK = server.arg("roomInfluenceK").toFloat();
    candidate.nightSetbackEnabled = server.hasArg("nightSetbackEnabled") && server.arg("nightSetbackEnabled").toInt() != 0;
    if (server.hasArg("nightSetbackStartMinute")) candidate.nightSetbackStartMinute = (uint16_t)constrain(server.arg("nightSetbackStartMinute").toInt(), 0, 1439);
    if (server.hasArg("nightSetbackEndMinute")) candidate.nightSetbackEndMinute = (uint16_t)constrain(server.arg("nightSetbackEndMinute").toInt(), 0, 1439);
    if (server.hasArg("nightSetbackK")) candidate.nightSetbackK = constrain(server.arg("nightSetbackK").toFloat(), 0.0f, 10.0f);
    candidate.roomTargetTemperatureC = constrain(candidate.roomTargetTemperatureC, 16.0f, 30.0f);
    if (server.hasArg("heatingCurveBaseC")) candidate.heatingCurveBaseC = server.arg("heatingCurveBaseC").toFloat();
    if (server.hasArg("heatingCurveSlope")) candidate.heatingCurveSlope = server.arg("heatingCurveSlope").toFloat();
    if (server.hasArg("frostProtectionEnabled")) candidate.frostProtectionEnabled = server.arg("frostProtectionEnabled").toInt() != 0;
    if (server.hasArg("frostStartTemperatureC")) candidate.frostStartTemperatureC = server.arg("frostStartTemperatureC").toFloat();
    if (server.hasArg("frostTargetFlowTemperatureC")) candidate.frostTargetFlowTemperatureC = server.arg("frostTargetFlowTemperatureC").toFloat();
    if (server.hasArg("mixerFullTravelMs")) candidate.mixerFullTravelMs = (uint32_t)server.arg("mixerFullTravelMs").toInt();
    if (server.hasArg("mixerPulseMs")) candidate.mixerPulseMs = (uint32_t)server.arg("mixerPulseMs").toInt();
    if (server.hasArg("mixerPauseMs")) candidate.mixerPauseMs = (uint32_t)server.arg("mixerPauseMs").toInt();

if (candidate.pumpTargetDeltaC < 0.5f) candidate.pumpTargetDeltaC = 0.5f;
if (candidate.pumpFullDeltaC <= candidate.pumpTargetDeltaC) candidate.pumpFullDeltaC = candidate.pumpTargetDeltaC + 1.0f;

HeatingCircuitConfig old = s_ctx->config.heatingCircuits[idx];

s_ctx->config.heatingCircuits[idx] = candidate;

String validationError;
if (!OutputValidation::validateAll(*s_ctx, validationError)) {
  s_ctx->config.heatingCircuits[idx] = old;
  server.send(409, "text/plain", validationError);
  return;
}

HeatingCircuits::releaseRemovedOutputs(*s_ctx, old, candidate);
HeatingCircuits::resetMixerFsm(*s_ctx, (uint8_t)idx);
s_ctx->heatingCircuitRuntime[idx].pumpActive = false;

if (!Storage::saveConfig(s_ctx->config)) {
  s_ctx->config.heatingCircuits[idx] = old;
  server.send(500, "text/plain", "Konfiguration konnte nicht auf SD gespeichert werden");
  return;
}
server.send(200, "text/plain", "OK");
  }

void handleHeatingCircuitRoomTarget() {
  if (!s_ctx || !server.hasArg("index") || !server.hasArg("targetC")) {
    server.send(400, "text/plain", "Parameter fehlen");
    return;
  }
  const int idx = server.arg("index").toInt();
  const float targetC = server.arg("targetC").toFloat();
  if (idx < 0 || idx >= MAX_HEATING_CIRCUITS || targetC < 16.0f || targetC > 30.0f) {
    server.send(400, "text/plain", "Raum-Soll muss 16.0..30.0 C sein");
    return;
  }
  HeatingCircuitConfig& cfg = s_ctx->config.heatingCircuits[idx];
  if (!cfg.enabled || !cfg.roomControlEnabled) {
    server.send(409, "text/plain", "Raumregelung fuer diesen Heizkreis ist nicht aktiv");
    return;
  }
  const float oldTarget = cfg.roomTargetTemperatureC;
  cfg.roomTargetTemperatureC = targetC;
  if (!Storage::saveConfig(s_ctx->config)) {
    cfg.roomTargetTemperatureC = oldTarget;
    server.send(500, "text/plain", "Raum-Soll konnte nicht gespeichert werden");
    return;
  }
  server.send(200, "application/json", String("{\"targetC\":") + String(cfg.roomTargetTemperatureC, 1) + "}");
}

void handleManualTimeSet() {
  if (!serviceSessionValid()) {
    server.send(403, "text/plain", "Nicht erlaubt");
    return;
  }
  if (!s_ctx || !server.hasArg("date") || !server.hasArg("time")) {
    server.send(400, "text/plain", "Datum oder Uhrzeit fehlt");
    return;
  }
  int year=0, month=0, day=0, hour=0, minute=0;
  if (sscanf(server.arg("date").c_str(), "%d-%d-%d", &year, &month, &day) != 3 ||
      sscanf(server.arg("time").c_str(), "%d:%d", &hour, &minute) != 2 ||
      !TimeService::setManualLocalTime(*s_ctx, year, month, day, hour, minute, 0)) {
    server.send(400, "text/plain", "Ungueltiges Datum oder Uhrzeit");
    return;
  }
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
  json += ",\"released\":";
  json += String(cfg.userReleaseEnabled ? "true" : "false");
  json += ",\"minimumTemperatureC\":";
  json += String(cfg.minimumTemperatureC, 1);
  json += ",\"targetTemperatureC\":";
  json += String(cfg.targetTemperatureC, 1);
  json += ",\"hysteresisC\":";
  json += String(cfg.hysteresisC, 1);
  json += ",\"sinkRole\":\"";
  json += SensorRoles::toKey(cfg.sinkRole);
  json += "\"";
  const OutputRef& activePumpOutput = cfg.pumpOutput;

  json += ",\"pumpOutputKind\":\"";
  json += outputRefKindToKey(activePumpOutput.kind);
  json += "\",\"pumpOutputIndex\":";
  json += String(activePumpOutput.index);
  json += ",\"pumpOutputValue\":\"";
  json += outputRefValue(activePumpOutput.kind, activePumpOutput.index);
  json += "\"";

  json += ",\"heaterOutput1Kind\":\"";
  json += outputRefKindToKey(cfg.heaterOutput1.kind);
  json += "\",\"heaterOutput1Index\":";
  json += String(cfg.heaterOutput1.index);
  json += ",\"heaterOutput1Value\":\"";
  json += outputRefValue(cfg.heaterOutput1.kind, cfg.heaterOutput1.index);
  json += "\"";

  json += ",\"heaterOutput2Kind\":\"";
  json += outputRefKindToKey(cfg.heaterOutput2.kind);
  json += "\",\"heaterOutput2Index\":";
  json += String(cfg.heaterOutput2.index);
  json += ",\"heaterOutput2Value\":\"";
  json += outputRefValue(cfg.heaterOutput2.kind, cfg.heaterOutput2.index);
  json += "\"";

  json += ",\"heaterOutput3Kind\":\"";
  json += outputRefKindToKey(cfg.heaterOutput3.kind);
  json += "\",\"heaterOutput3Index\":";
  json += String(cfg.heaterOutput3.index);
  json += ",\"heaterOutput3Value\":\"";
  json += outputRefValue(cfg.heaterOutput3.kind, cfg.heaterOutput3.index);
  json += "\"";

  json += ",\"preRunSeconds\":";
  json += String(cfg.preRunMs / 1000UL);
  json += ",\"cooldownSeconds\":";
  json += String(cfg.cooldownMs / 1000UL);
  json += "}";

  json += ",\"sinkRoles\":[";
  bool firstSink = true;
  appendActiveDs18RoleOptions(json, *s_ctx, firstSink);
  json += "]";

  json += ",\"pumpOutputs\":[";
  bool firstPump = true;

  for (uint8_t i = 0; i < RELAY_COUNT; i++) {
    const RelayOutputConfig& r = s_ctx->config.relays[i];
    if (!r.enabled || r.function != RelayFunction::PUMP_ENABLE) continue;
    if (!firstPump) json += ",";
    firstPump = false;

    String usage;
    const bool busy = outputIsUsedByLegacyConsumers(*s_ctx, OutputKind::RELAY, i, -1, usage, true, false) ||
                      outputIsUsedByHeatingCircuit(*s_ctx, OutputKind::RELAY, i, -1, usage) ||
                      outputIsUsedByValves(*s_ctx, OutputKind::RELAY, i, -1, usage);

    json += "{\"value\":\"";
    json += outputRefValue(OutputKind::RELAY, i);
    json += "\",\"kind\":\"relay\",\"index\":";
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

  for (uint8_t i = 0; i < PWM_OUTPUT_COUNT; i++) {
    const PwmOutputConfig& po = s_ctx->config.pwmOutputs[i];
    if (!po.enabled || po.mode != PwmOutputMode::SWITCH || po.function != RelayFunction::PUMP_ENABLE) continue;
    if (!firstPump) json += ",";
    firstPump = false;

    String usage;
    const bool busy = outputIsUsedByLegacyConsumers(*s_ctx, OutputKind::PWM_OUTPUT, i, -1, usage, true, false) ||
                      outputIsUsedByHeatingCircuit(*s_ctx, OutputKind::PWM_OUTPUT, i, -1, usage) ||
                      outputIsUsedByValves(*s_ctx, OutputKind::PWM_OUTPUT, i, -1, usage);

    json += "{\"value\":\"";
    json += outputRefValue(OutputKind::PWM_OUTPUT, i);
    json += "\",\"kind\":\"pwm\",\"index\":";
    json += String(i);
    json += ",\"label\":\"";
    json += pwmOutputHardwareLabel(i);
    json += " / PO-Schaltausgang";
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

  json += ",\"heaterOutputs\":[";
  bool firstHeater = true;

  for (uint8_t i = 0; i < RELAY_COUNT; i++) {
    const RelayOutputConfig& r = s_ctx->config.relays[i];
    if (!r.enabled || r.function != RelayFunction::HEATER_ROD) continue;
    if (!firstHeater) json += ",";
    firstHeater = false;

    String usage;
    const bool busy = outputIsUsedByLegacyConsumers(*s_ctx, OutputKind::RELAY, i, -1, usage, true, false) ||
                      outputIsUsedByHeatingCircuit(*s_ctx, OutputKind::RELAY, i, -1, usage) ||
                      outputIsUsedByValves(*s_ctx, OutputKind::RELAY, i, -1, usage);

    json += "{\"value\":\"";
    json += outputRefValue(OutputKind::RELAY, i);
    json += "\",\"kind\":\"relay\",\"index\":";
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

  for (uint8_t i = 0; i < PWM_OUTPUT_COUNT; i++) {
    const PwmOutputConfig& po = s_ctx->config.pwmOutputs[i];
    if (!po.enabled || po.mode != PwmOutputMode::SWITCH || po.function != RelayFunction::HEATER_ROD) continue;
    if (!firstHeater) json += ",";
    firstHeater = false;

    String usage;
    const bool busy = outputIsUsedByLegacyConsumers(*s_ctx, OutputKind::PWM_OUTPUT, i, -1, usage, true, false) ||
                      outputIsUsedByHeatingCircuit(*s_ctx, OutputKind::PWM_OUTPUT, i, -1, usage) ||
                      outputIsUsedByValves(*s_ctx, OutputKind::PWM_OUTPUT, i, -1, usage);

    json += "{\"value\":\"";
    json += outputRefValue(OutputKind::PWM_OUTPUT, i);
    json += "\",\"kind\":\"pwm\",\"index\":";
    json += String(i);
    json += ",\"label\":\"";
    json += pwmOutputHardwareLabel(i);
    json += " / PO-Schaltausgang";
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
  if (server.hasArg("sinkRole")) {
    const String sinkArg = server.arg("sinkRole");
    candidate.sinkRole = (sinkArg == "none" || sinkArg == "255") ? Ds18Role::NONE : SensorRoles::fromKey(sinkArg);
  }

  auto readOutputRefArg = [](const String& kindArg, const String& indexArg) -> OutputRef {
    OutputRef ref;
    ref.kind = outputKindFromKey(kindArg);
    ref.index = (uint8_t)indexArg.toInt();
    if (ref.kind == OutputKind::NONE || ref.index == PIN_UNUSED) {
      ref.kind = OutputKind::NONE;
      ref.index = PIN_UNUSED;
    }
    return ref;
  };

  auto readLegacyRelayRef = [](const String& arg) -> OutputRef {
    const uint8_t index = (uint8_t)arg.toInt();
    if (index == PIN_UNUSED) return OutputRef{};
    return OutputRef{OutputKind::RELAY, index};
  };

  if (server.hasArg("pumpOutputKind") && server.hasArg("pumpOutputIndex")) {
    candidate.pumpOutput = readOutputRefArg(server.arg("pumpOutputKind"), server.arg("pumpOutputIndex"));
  } else if (server.hasArg("pumpRelay")) {
    candidate.pumpOutput = readLegacyRelayRef(server.arg("pumpRelay"));
  }

  if (server.hasArg("heaterOutput1Kind") && server.hasArg("heaterOutput1Index")) {
    candidate.heaterOutput1 = readOutputRefArg(server.arg("heaterOutput1Kind"), server.arg("heaterOutput1Index"));
  } else if (server.hasArg("heaterRelay1")) {
    candidate.heaterOutput1 = readLegacyRelayRef(server.arg("heaterRelay1"));
  }

  if (server.hasArg("heaterOutput2Kind") && server.hasArg("heaterOutput2Index")) {
    candidate.heaterOutput2 = readOutputRefArg(server.arg("heaterOutput2Kind"), server.arg("heaterOutput2Index"));
  } else if (server.hasArg("heaterRelay2")) {
    candidate.heaterOutput2 = readLegacyRelayRef(server.arg("heaterRelay2"));
  }

  if (server.hasArg("heaterOutput3Kind") && server.hasArg("heaterOutput3Index")) {
    candidate.heaterOutput3 = readOutputRefArg(server.arg("heaterOutput3Kind"), server.arg("heaterOutput3Index"));
  } else if (server.hasArg("heaterRelay3")) {
    candidate.heaterOutput3 = readLegacyRelayRef(server.arg("heaterRelay3"));
  }

  if (server.hasArg("preRunSeconds")) candidate.preRunMs = (uint32_t)server.arg("preRunSeconds").toInt() * 1000UL;
  if (server.hasArg("cooldownSeconds")) candidate.cooldownMs = (uint32_t)server.arg("cooldownSeconds").toInt() * 1000UL;

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


void handleAuxHeaterRelease() {
  if (!s_ctx) {
    server.send(500, "text/plain", "Kein Kontext");
    return;
  }
  if (!s_ctx->config.auxHeater.enabled) {
    server.send(409, "text/plain", "Zusatzheizung ist nicht aktiviert");
    return;
  }
  if (!server.hasArg("state")) {
    server.send(400, "text/plain", "state fehlt");
    return;
  }

  const bool release = server.arg("state").toInt() != 0;
  const bool oldRelease = s_ctx->config.auxHeater.userReleaseEnabled;
  s_ctx->config.auxHeater.userReleaseEnabled = release;

  if (!Storage::saveConfig(s_ctx->config)) {
    s_ctx->config.auxHeater.userReleaseEnabled = oldRelease;
    server.send(500, "text/plain", "Freigabe konnte nicht gespeichert werden");
    return;
  }

  server.send(200, "application/json", String("{\"released\":") + (release ? "true" : "false") + "}");
}

static void handlePlantDataJson() {
  if (!s_ctx) { server.send(500, "application/json", "{\"error\":\"no_context\"}"); return; }
  const ConfigData& c = s_ctx->config;
  const float meanFluidC = s_ctx->energyMeter.temperatureValid
      ? 0.5f * (s_ctx->energyMeter.flowTemperatureC + s_ctx->energyMeter.returnTemperatureC) : 40.0f;
  const FluidProperties::Result fluid = FluidProperties::evaluate(c, meanFluidC);
  String j="{";
  j += "\"solarFluidType\":" + String((int)c.solarFluidType) + ",";
  j += "\"glycolType\":" + String((int)c.glycolType) + ",";
  j += "\"glycolMeasuredFreezeProtectionC\":" + String(c.glycolMeasuredFreezeProtectionC,1) + ",";
  j += "\"solarHydraulicType\":" + String((int)c.solarHydraulicType) + ",";
  j += "\"solarCollectorType\":" + String((int)c.solarCollectorType) + ",";
  j += "\"collectorApertureM2\":" + String(c.collectorApertureM2,3) + ",";
  j += "\"collectorTiltDeg\":" + String(c.collectorTiltDeg,1) + ",";
  j += "\"collectorAzimuthDeg\":" + String(c.collectorAzimuthDeg,1) + ",";
  j += "\"bufferVolumeLiters\":" + String(c.bufferVolumeLiters,1) + ",";
  j += "\"boilerVolumeLiters\":" + String(c.boilerVolumeLiters,1) + ",";
  j += "\"mlMode\":" + String((int)c.mlMode) + ",";
  j += "\"glycolConcentrationPercent\":" + String(fluid.glycolConcentrationPercent,1) + ",";
  j += "\"waterConcentrationPercent\":" + String(fluid.waterConcentrationPercent,1) + ",";
  j += "\"fluidHeatCapacityWhPerLiterK\":" + String(fluid.volumetricHeatCapacityWhPerLiterK,4) + ",";
  j += "\"freezePointInRange\":" + String(fluid.freezePointInRange?"true":"false") + ",";
  j += "\"frostRequiredProtectionC\":" + String(c.frostRequiredProtectionC,1) + ",";
  j += "\"frostProtectionReserveK\":" + String(c.frostProtectionReserveK,1) + ",";
  j += "\"frostSafetyCompatible\":" + String(fluid.safetyCompatible?"true":"false") + ",";
  j += "\"frostSafetyMarginMet\":" + String(fluid.safetyMarginMet?"true":"false") + ",";
  j += "\"heatingCircuits\":[";
  for(uint8_t i=0;i<MAX_HEATING_CIRCUITS;i++){
    if(i)j+=","; j+="{\"index\":"+String(i)+",\"enabled\":"+String(c.heatingCircuits[i].enabled?"true":"false")+",\"areaM2\":"+String(c.heatingCircuitAreaM2[i],1)+",\"emitterType\":"+String((int)c.heatingCircuitEmitterType[i])+"}";
  }
  j += "],\"ml\":" + MlOptimizer::json(*s_ctx) + "}";
  server.send(200,"application/json",j);
}

static void handlePlantDataSave() {
  if (!s_ctx) { server.send(500,"text/plain","context missing"); return; }
  if (!serviceSessionValid()) { server.send(403,"text/plain","Service nicht freigegeben"); return; }

  // Erst in eine Kandidatenkopie schreiben. Erst wenn die SD-Datei erfolgreich
  // geschrieben wurde, wird die laufende Konfiguration ersetzt. Dadurch kann ein
  // SD-Schreibfehler keine nur im RAM scheinbar gespeicherte Anlagendatei erzeugen.
  ConfigData candidate=s_ctx->config;
  ConfigData& c=candidate;
  if(server.hasArg("solarFluidType")){int v=server.arg("solarFluidType").toInt();c.solarFluidType=v==1?SolarFluidType::WATER:SolarFluidType::GLYCOL;}
  if(server.hasArg("glycolType")){c.glycolType=server.arg("glycolType").toInt()==1?GlycolType::ETHYLENE:GlycolType::PROPYLENE;}
  if(server.hasArg("glycolMeasuredFreezeProtectionC"))c.glycolMeasuredFreezeProtectionC=constrain(server.arg("glycolMeasuredFreezeProtectionC").toFloat(),-60.0f,0.0f);
  if(server.hasArg("solarHydraulicType"))c.solarHydraulicType=server.arg("solarHydraulicType").toInt()==1?SolarHydraulicType::DRAINBACK:SolarHydraulicType::CLOSED_PRESSURIZED;
  if(server.hasArg("solarCollectorType"))c.solarCollectorType=server.arg("solarCollectorType").toInt()==1?SolarCollectorType::EVACUATED_TUBE:SolarCollectorType::FLAT_PLATE;
  if(server.hasArg("collectorApertureM2"))c.collectorApertureM2=constrain(server.arg("collectorApertureM2").toFloat(),0.1f,100.0f);
  if(server.hasArg("collectorTiltDeg"))c.collectorTiltDeg=constrain(server.arg("collectorTiltDeg").toFloat(),0.0f,90.0f);
  if(server.hasArg("collectorAzimuthDeg")){float a=server.arg("collectorAzimuthDeg").toFloat();while(a<0)a+=360;while(a>=360)a-=360;c.collectorAzimuthDeg=a;}
  if(server.hasArg("bufferVolumeLiters"))c.bufferVolumeLiters=constrain(server.arg("bufferVolumeLiters").toFloat(),0.0f,20000.0f);
  if(server.hasArg("boilerVolumeLiters"))c.boilerVolumeLiters=constrain(server.arg("boilerVolumeLiters").toFloat(),0.0f,5000.0f);
  if(server.hasArg("mlMode")){int m=server.arg("mlMode").toInt();c.mlMode=m==2?MlMode::AUTOMATIC:(m==1?MlMode::LEARN_ONLY:MlMode::OFF);}
  for(uint8_t i=0;i<MAX_HEATING_CIRCUITS;i++){
    String ak="heatingCircuitAreaM2_"+String(i), ek="heatingCircuitEmitterType_"+String(i);
    if(server.hasArg(ak))c.heatingCircuitAreaM2[i]=constrain(server.arg(ak).toFloat(),0.0f,1000.0f);
    if(server.hasArg(ek)){int e=server.arg(ek).toInt();c.heatingCircuitEmitterType[i]=(e>=0&&e<=3)?(HeatingEmitterType)e:HeatingEmitterType::UNKNOWN;}
  }

  if(!Storage::saveConfig(candidate)) {
    server.send(500,"application/json","{\"ok\":false,\"error\":\"config_write_failed\"}");
    return;
  }
  s_ctx->config=candidate;
  Forecast::forceRefresh();
  server.send(200,"application/json","{\"ok\":true}");
}


static void handleHistoryConfigJson() {
  if (!s_ctx) { server.send(500,"application/json","{\"error\":\"no_context\"}"); return; }
  server.send(200,"application/json",History::configJson(*s_ctx));
}

static void handleHistoryConfigSave() {
  if (!s_ctx) { server.send(500,"text/plain","context missing"); return; }
  if (!serviceSessionValid()) { server.send(403,"text/plain","Service nicht freigegeben"); return; }
  if(server.hasArg("enabled")) History::setEnabled(server.arg("enabled").toInt()!=0);
  if(server.hasArg("sensorId") && server.hasArg("show")) History::setDisplayEnabled(server.arg("sensorId").c_str(),server.arg("show").toInt()!=0);
  if(!History::savePreferences()) { server.send(500,"application/json","{\"ok\":false,\"error\":\"history_config_write_failed\"}"); return; }
  server.send(200,"application/json","{\"ok\":true}");
}

static void handleHistoryJson() {
  if (!s_ctx) { server.send(500,"application/json","{\"error\":\"no_context\"}"); return; }
  History::handleHistoryRequest(*s_ctx,server);
}

static void handleMlReset() {
  if (!s_ctx) { server.send(500,"text/plain","context missing"); return; }
  if (!serviceSessionValid()) { server.send(403,"text/plain","Service nicht freigegeben"); return; }
  MlOptimizer::resetLearning(*s_ctx);
  server.send(200,"application/json","{\"ok\":true}");
}

static void handleMlV2Json() {
  if (!s_ctx) {
    server.send(500, "application/json", "{\"error\":\"no_context\"}");
    return;
  }

  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", MlOptimizer::json(*s_ctx));
}


} // namespace

namespace UI {

void handleForecastJson() {
  if (!s_ctx) { server.send(500, "application/json", "{\"error\":\"no_context\"}"); return; }
  server.send(200, "application/json", Forecast::json(*s_ctx));
}

void begin(AppContext& ctx) {
  DBG_PRINTLN("UI::begin()");
  s_ctx = &ctx;

  server.on("/", handleRoot);
  server.on("/api/status", handleStatusJson);
  server.on("/api/ds18b20", handleDs18b20Json);
  server.on("/api/roles", handleRolesJson);
  server.on("/api/assignments", HTTP_GET, handleAssignmentsJson);
  server.on("/service-assign-role", HTTP_POST, handleAssignRole);
  server.on("/api/plant", HTTP_GET, handlePlantJson);
  server.on("/api/plant-overview", HTTP_GET, handlePlantJson);
  server.on("/api/plant-live", HTTP_GET, handlePlantLiveJson);
  server.on("/api/plant-live.bin", HTTP_GET, handlePlantLiveBinary);
  server.on("/api/ui-version", HTTP_GET, handleUiVersionJson);
  server.on("/api/ui-command-status", HTTP_GET, handleUiCommandStatus);
  server.on("/api/heat-sources", HTTP_GET, handleHeatSourcesJson);
  server.on("/api/max-status", HTTP_GET, handleMaxStatusJson);
#if SOLARCTRL_MAX_DIAGNOSTICS
  server.on("/api/max-debug", HTTP_GET, handleMaxDebugJson);
  server.on("/api/max-fault-trace", HTTP_GET, handleMaxFaultTraceJson);
  server.on("/api/max-sequence-diag", HTTP_GET, handleMaxSequenceDiagJson);
#endif
  server.on("/api/network", HTTP_GET, handleNetworkJson);
  server.on("/api/forecast", HTTP_GET, handleForecastJson);
  server.on("/api/plant-data", HTTP_GET, handlePlantDataJson);
  server.on("/api/ml-v2", HTTP_GET, handleMlV2Json);
  server.on("/service-save-plant-data", HTTP_POST, handlePlantDataSave);
  server.on("/service-reset-ml", HTTP_POST, handleMlReset);
  server.on("/api/history-config", HTTP_GET, handleHistoryConfigJson);
  server.on("/service-save-history-config", HTTP_POST, handleHistoryConfigSave);
  server.on("/api/history", HTTP_GET, handleHistoryJson);
  server.on("/service-save-network", HTTP_POST, handleNetworkSave);
  server.on("/service-set-manual-time", HTTP_POST, handleManualTimeSet);
  server.on("/api/relays", HTTP_GET, handleRelaysJson);
  server.on("/service-relay-config", HTTP_POST, handleRelayConfig);
  server.on("/api/pumps", HTTP_GET, handlePumpsJson);
  server.on("/service-pump-config", HTTP_POST, handlePumpConfig);
  server.on("/api/valves", HTTP_GET, handleValvesJson);
  server.on("/service-valve-config", HTTP_POST, handleValveConfig);
  server.on("/api/oven", HTTP_GET, handleOvenJson);
  server.on("/service-save-oven", HTTP_POST, handleOvenSave);
  server.on("/service-oven-start", HTTP_POST, handleQueuedOvenStart);
  server.on("/service-oven-stop", HTTP_POST, handleQueuedOvenStop);
  server.on("/api/test/servo", HTTP_GET, handleTestServoJson);
  server.on("/api/test/sensors", HTTP_GET, handleTestSensorsJson);
  server.on("/api/test/safety", HTTP_GET, handleTestSafetyJson);
  server.on("/api/test/diagnostics", HTTP_GET, handleTestDiagnosticsJson);
  server.on("/api/alarms", HTTP_GET, handleAlarmsJson);
  server.on("/service-alarms-ack", HTTP_POST, handleAlarmsAck);
  server.on("/service-alarm-buzzer-mute", HTTP_POST, handleQueuedBuzzerMute);
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
  server.on("/api/runtime-diag", HTTP_GET, []() { server.send(200, "application/json", RuntimeDiag::json()); });
  server.on("/api/sensor-ring", HTTP_GET, []() { server.send(200, "application/json", SensorRing::json()); });
  server.on("/api/sd/files", HTTP_GET, handleSdFilesJson);
  server.on("/api/sd/download", HTTP_GET, handleSdDownload);
  server.on("/api/sd/upload", HTTP_POST, handleSdUploadComplete, handleSdUploadStream);
  // Legacy/dev-tool compatible upload alias. Uses the exact same PIN, path,
  // temp-file and .bak commit logic as /api/sd/upload.
  server.on("/sd-upload-direct", HTTP_POST, handleSdUploadComplete, handleSdUploadStream);
  server.on("/api/sd/delete", HTTP_POST, handleSdDelete);
  server.on("/service-factory-reset", HTTP_POST, handleFactoryReset);
  server.on("/service-restart", HTTP_POST, handleServiceRestart);
  server.on("/service-assign-heat-source", HTTP_POST, handleAssignHeatSource);
  server.on("/service-save-max-config", HTTP_POST, handleSaveMaxConfig);
  server.on("/api/heating-circuits", HTTP_GET, handleHeatingCircuitsJson);
  server.on("/service-heating-circuit", HTTP_POST, handleHeatingCircuitSave);
  server.on("/api/heating-circuit-room-target", HTTP_POST, handleHeatingCircuitRoomTarget);
  server.on("/api/safety", HTTP_GET, handleSafetyJson);
  server.on("/service-save-safety", HTTP_POST, handleSafetySave);
  server.on("/api/aux-heater", HTTP_GET, handleAuxHeaterJson);
  server.on("/service-save-aux-heater", HTTP_POST, handleAuxHeaterSave);
  server.on("/api/aux-heater-release", HTTP_POST, handleAuxHeaterRelease);
  server.on("/api/energy-meter", HTTP_GET, handleEnergyMeterJson);
  server.on("/service-save-energy-meter", HTTP_POST, handleEnergyMeterSave);
  server.onNotFound(handleStatic);

  server.begin();
  DBG_PRINTLN("Webserver gestartet");
}

void update() {
  server.handleClient();
  if (g_restartScheduled && (int32_t)(millis() - g_restartAtMs) >= 0) {
    DBG_PRINTLN("Geplanter Neustart wird ausgefuehrt...");
    DBG_FLUSH();
    ESP.restart();
  }
}


bool commissioningTestActive() {
  return g_commissioningTestActive;
}
}
