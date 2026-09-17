#include "app_fsm.h"
#include "config.h"
#include "feature_valves.h"
#include "feature_heat_source_storage.h"
#include "feature_heat_sources_max31865.h"
#include "feature_io_expander_pcf8574.h"
#include "feature_pumps.h"
#include "feature_pwm_pca9685.h"
#include "feature_relay_outputs.h"
#include "feature_sdcard.h"
#include "feature_sensor_assignments.h"
#include "feature_sink_ds18b20.h"
#include "feature_storage.h"
#include "feature_ui.h"
#include "feature_safety_manager.h"
#include "feature_aux_heater.h"
#include "feature_oven.h"
#include "feature_heating_circuits.h"
#include "feature_alarms.h"
#include "feature_output_validation.h"
#include "feature_energy_meter.h"

#include <Arduino.h>
#include <WiFi.h>
#include <string.h>
#include <stdio.h>

namespace {
  void copyFaultText(char* dst, size_t dstSize, const char* src) {
    if (!dst || dstSize == 0) return;
    strncpy(dst, src ? src : "", dstSize - 1);
    dst[dstSize - 1] = '\0';
  }

  void forceAllPumpsOff(AppContext& ctx) {
    Pumps::allOff(ctx);
    ctx.control.relayEnable = false;
    ctx.control.pwmPercent = 0;
    ctx.control.diffC = NAN;
    ctx.control.protectionMode = ProtectionMode::NONE;
  }

  static constexpr uint32_t WIFI_STA_RECONNECT_INTERVAL_MS = 30000UL;
  static constexpr uint32_t WIFI_AP_HEALTH_INTERVAL_MS = 10000UL;

  uint32_t g_lastSoftApCheckMs = 0;

  bool validSoftApIp() {
    IPAddress ip = WiFi.softAPIP();
    return !(ip[0] == 0 && ip[1] == 0 && ip[2] == 0 && ip[3] == 0);
  }
}

AppFSM::AppFSM() {
  memset(&ctx_, 0, sizeof(ctx_));
  ctx_.state = SystemState::BOOT;
  ctx_.stateEnteredAtMs = millis();
}

void AppFSM::begin() {
  Serial.begin(SERIAL_BAUDRATE);
  delay(50);

  ctx_.state = SystemState::BOOT;
  ctx_.stateEnteredAtMs = millis();
  ctx_.lastSampleAtMs = 0;
  ctx_.lastRuntimeSaveAtMs = millis();
  Alarms::begin();
  Alarms::recordInfo("system_boot", "Steuerung gestartet");
}

void AppFSM::loop() {
  switch (ctx_.state) {
    case SystemState::BOOT:
      changeState(SystemState::INIT_HW);
      break;

    case SystemState::INIT_HW:
      stateInitHw();
      break;

    case SystemState::INIT_SD:
      stateInitSd();
      break;

    case SystemState::INIT_STORAGE:
      stateInitStorage();
      break;

    case SystemState::LOAD_CONFIG:
      stateLoadConfig();
      break;

    case SystemState::INIT_NETWORK:
      stateInitNetwork();
      break;

    case SystemState::INIT_UI:
      stateInitUi();
      break;

    case SystemState::INIT_SENSORS:
      stateInitSensors();
      break;

    case SystemState::SELF_TEST:
      stateSelfTest();
      break;

    case SystemState::IDLE:
      stateIdle();
      break;

    case SystemState::READ_SENSORS:
      stateReadSensors();
      break;

    case SystemState::VALIDATE_SENSORS:
      stateValidateSensors();
      break;

    case SystemState::COMPUTE_CONTROL:
      stateComputeControl();
      break;

    case SystemState::APPLY_OUTPUTS:
      stateApplyOutputs();
      break;

    case SystemState::UPDATE_RUNTIME:
      stateUpdateRuntime();
      break;

    case SystemState::FAULT:
      stateFault();
      break;
  }

  if (ctx_.networkInitialized) {
    serviceNetwork();
  }
  Alarms::process(ctx_);
}

void AppFSM::changeState(SystemState next) {
  ctx_.state = next;
  ctx_.stateEnteredAtMs = millis();
}

void AppFSM::stateInitHw() {
  Serial.println("INIT_HW gestartet");

  // I2C-Hardware initialisieren, aber fehlende Erweiterungsbausteine blockieren den AP nicht.
  bool relayOk = RelayOutputs::begin(ctx_);
  Serial.print("Relais-PCF: ");
  Serial.println(relayOk ? "OK" : "FEHLER/NICHT GEFUNDEN");

  // PCA9685 und Ofen-Servo werden erst nach LOAD_CONFIG initialisiert.
  // Beide benoetigen die gespeicherten Profil-/Kalibrierwerte, damit es beim
  // Booten keinen kurzen falschen Ausgangspegel bzw. Servo-Winkel gibt.
  HeatingCircuits::begin(ctx_);
  changeState(SystemState::INIT_SD);
}

void AppFSM::stateInitSd() {
  ctx_.sdAvailable = SDCard::begin();

  Alarms::setPersistentHistoryEnabled(ctx_.sdAvailable);

  if (!ctx_.sdAvailable) {
    ctx_.diag.sdErrorCount++;
    copyFaultText(ctx_.diag.faultText, sizeof(ctx_.diag.faultText), "SD Initialisierung fehlgeschlagen");
    Alarms::raise("sd_init_failed", Alarms::Severity::WARNING, "SD Initialisierung fehlgeschlagen", true);
  } else {
    Alarms::clear("sd_init_failed");
    Alarms::recordInfo("sd_ready", "SD-Karte bereit; Alarmhistorie aktiv");
  }

  changeState(SystemState::INIT_STORAGE);
}

void AppFSM::stateInitStorage() {
  Storage::begin(ctx_.sdAvailable);
  Valves::begin(ctx_);
  changeState(SystemState::LOAD_CONFIG);
}

void AppFSM::stateLoadConfig() {
  Storage::loadConfig(ctx_.config);
  Storage::loadDiagnostics(ctx_.diag);
  Storage::loadMaintenance(ctx_.maintenance);
  Storage::loadEnergyMeterRuntime(ctx_.energyMeter);
  Storage::loadSensorAssignments(ctx_.assignments);
  HeatSourceStorage::loadAssignments(ctx_.heatSourceAssignments);

  // Profil-/kalibrierungsabhaengige Hardware erst jetzt initialisieren,
  // nachdem die SD-Konfiguration geladen wurde.
  bool pwmOk = PwmDriver::begin();
  Serial.print("PCA9685: ");
  Serial.println(pwmOk ? "OK" : "FEHLER/NICHT GEFUNDEN");
  if (pwmOk) {
    PwmDriver::allOff(ctx_.config);
  }

  OvenControl::begin(ctx_);
  EnergyMeter::begin(ctx_);

  ctx_.diag.bootCount++;
  Storage::saveDiagnostics(ctx_.diag);

  changeState(SystemState::INIT_NETWORK);
}

void AppFSM::ensureSoftAp() {
  const uint32_t now = millis();
  if (g_lastSoftApCheckMs != 0 && (uint32_t)(now - g_lastSoftApCheckMs) < WIFI_AP_HEALTH_INTERVAL_MS) {
    return;
  }
  g_lastSoftApCheckMs = now;

  auto mode = WiFi.getMode();
  ctx_.networkWifiMode = static_cast<uint8_t>(mode);

  if (mode != WIFI_AP_STA) {
    Serial.println("WLAN-Modus wurde korrigiert: WIFI_AP_STA");
    WiFi.mode(WIFI_AP_STA);
  }

  const char* ssid = ctx_.config.apName[0] ? ctx_.config.apName : DEFAULT_AP_SSID;
  const char* pass = ctx_.config.apPassword[0] ? ctx_.config.apPassword : DEFAULT_AP_PASSWORD;

  if (!validSoftApIp()) {
    Serial.println("SoftAP nicht aktiv - starte SoftAP erneut");
    WiFi.softAP(ssid, pass);
  }

  ctx_.networkApActive = validSoftApIp();
}

void AppFSM::startStaConnect() {
  if (!ctx_.config.staEnabled || !ctx_.config.staSsid[0]) {
    ctx_.networkStaConfigured = false;
    ctx_.networkStaConnectAttemptActive = false;
    return;
  }

  ensureSoftAp();

  const char* hostName = ctx_.config.hostName[0] ? ctx_.config.hostName : DEFAULT_HOSTNAME;
  WiFi.setHostname(hostName);

  Serial.print("Starte nicht-blockierenden WLAN-Client-Verbindungsversuch: ");
  Serial.println(ctx_.config.staSsid);

  if (ctx_.config.staPassword[0]) {
    WiFi.begin(ctx_.config.staSsid, ctx_.config.staPassword);
  } else {
    WiFi.begin(ctx_.config.staSsid);
  }

  ctx_.networkStaConfigured = true;
  ctx_.networkStaConnectAttemptActive = true;
  ctx_.networkLastStaReconnectAttemptMs = millis();
}

void AppFSM::serviceNetwork() {
  ensureSoftAp();

  const bool staConfigured = ctx_.config.staEnabled && ctx_.config.staSsid[0];
  ctx_.networkStaConfigured = staConfigured;
  ctx_.networkWifiMode = static_cast<uint8_t>(WiFi.getMode());
  ctx_.networkStaStatus = static_cast<uint8_t>(WiFi.status());

  if (!staConfigured) {
    ctx_.networkStaConnected = false;
    ctx_.networkStaConnectAttemptActive = false;
    ctx_.networkStaRssi = 0;
    ctx_.networkStaDisconnectedSinceMs = 0;
    Alarms::clear("wifi_sta_disconnected");
    return;
  }

  const uint32_t now = millis();
  const bool connected = (WiFi.status() == WL_CONNECTED);
  ctx_.networkStaConnected = connected;

  if (connected) {
    ctx_.networkStaEverConnected = true;
    ctx_.networkStaConnectAttemptActive = false;
    ctx_.networkStaRssi = WiFi.RSSI();
    ctx_.networkStaDisconnectedSinceMs = 0;
    if (ctx_.networkStaConnectedSinceMs == 0) ctx_.networkStaConnectedSinceMs = now;
    Alarms::clear("wifi_sta_disconnected");
    return;
  }

  ctx_.networkStaConnectedSinceMs = 0;
  ctx_.networkStaRssi = 0;
  if (ctx_.networkStaDisconnectedSinceMs == 0) ctx_.networkStaDisconnectedSinceMs = now;

  Alarms::raise("wifi_sta_disconnected", Alarms::Severity::WARNING, "WLAN-Client ist nicht verbunden; SoftAP bleibt aktiv", true);

  if (ctx_.networkLastStaReconnectAttemptMs == 0 ||
      (uint32_t)(now - ctx_.networkLastStaReconnectAttemptMs) >= WIFI_STA_RECONNECT_INTERVAL_MS) {
    startStaConnect();
  }
}

void AppFSM::stateInitNetwork() {
  Serial.println("INIT_NETWORK gestartet");

  WiFi.persistent(false);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(false);
  WiFi.mode(WIFI_AP_STA);

  const char* ssid = ctx_.config.apName[0] ? ctx_.config.apName : DEFAULT_AP_SSID;
  const char* pass = ctx_.config.apPassword[0] ? ctx_.config.apPassword : DEFAULT_AP_PASSWORD;
  const char* hostName = ctx_.config.hostName[0] ? ctx_.config.hostName : DEFAULT_HOSTNAME;

  WiFi.setHostname(hostName);

  Serial.print("Starte AP mit SSID: ");
  Serial.println(ssid);

  bool ok = WiFi.softAP(ssid, pass);

  Serial.print("softAP Ergebnis: ");
  Serial.println(ok ? "OK" : "FEHLER");

  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  ctx_.networkInitialized = true;
  ctx_.networkApActive = validSoftApIp();
  g_lastSoftApCheckMs = 0;
  ensureSoftAp();

  if (ctx_.config.staEnabled && ctx_.config.staSsid[0]) {
    startStaConnect();
    Serial.println("WLAN-Client-Verbindung laeuft nicht-blockierend im Hintergrund.");
  } else {
    Serial.println("WLAN-Client deaktiviert. Nur SoftAP aktiv.");
  }

  changeState(SystemState::INIT_UI);
}

void AppFSM::stateInitUi() {
  Serial.println("INIT_UI gestartet");
  UI::begin(ctx_);
  changeState(SystemState::INIT_SENSORS);
}

void AppFSM::stateInitSensors() {
  
  Serial.println("INIT_SENSORS gestartet");
  Serial.flush();
  
  Serial.println("SinkSensor::begin...");
  Serial.flush();
  bool sinkBusOk = SinkSensor::begin();
  Serial.println(sinkBusOk ? "SinkSensor OK" : "SinkSensor FEHLER");
  Serial.flush();

  Serial.println("HeatSourcesMax::begin...");
  Serial.flush();
  bool heatSourcesOk = HeatSourcesMax::begin(ctx_);
  Serial.println(heatSourcesOk ? "HeatSourcesMax OK" : "HeatSourcesMax FEHLER");
  Serial.flush();

  Serial.println("SinkSensor::scanBus...");
  Serial.flush();
  SinkSensor::scanBus(ctx_.ds18b20);
  Serial.println("scanBus fertig");
  Serial.flush();

  if (!sinkBusOk) {
    copyFaultText(
      ctx_.diag.faultText,
      sizeof(ctx_.diag.faultText),
      "DS18B20 Initialisierung fehlgeschlagen"
    );
    Alarms::raise("ds18b20_init_failed", Alarms::Severity::CRITICAL, "DS18B20 Initialisierung fehlgeschlagen", false);
    changeState(SystemState::FAULT);
    return;
  }

  if (!heatSourcesOk) {
    copyFaultText(
      ctx_.diag.faultText,
      sizeof(ctx_.diag.faultText),
      "MAX31865 Initialisierung fehlgeschlagen"
    );
    Alarms::raise("max31865_init_failed", Alarms::Severity::CRITICAL, "MAX31865 Initialisierung fehlgeschlagen", false);
    changeState(SystemState::FAULT);
    return;
  }

  Serial.println("SensorAssignments::resolveAssignments...");
  Serial.flush();

  if (!SensorAssignments::resolveAssignments(ctx_.ds18b20, ctx_.config, ctx_.assignments)) {
    if (ctx_.ds18b20.count == 0) {
      copyFaultText(
        ctx_.diag.faultText,
        sizeof(ctx_.diag.faultText),
        "Kein DS18B20 gefunden"
      );
      Alarms::raise("ds18b20_not_found", Alarms::Severity::CRITICAL, "Kein DS18B20 gefunden", false);
    } else {
      copyFaultText(
        ctx_.diag.faultText,
        sizeof(ctx_.diag.faultText),
        "DS18B20 Rollen-Zuordnung erforderlich"
      );
      Alarms::raise("ds18b20_role_assignment_required", Alarms::Severity::CRITICAL, "DS18B20 Rollen-Zuordnung erforderlich", false);
    }

    Serial.println("SensorAssignments FEHLER");
    Serial.flush();

    changeState(SystemState::FAULT);
    return;
  }

  Serial.println("SensorAssignments OK");
  Serial.flush();
  Alarms::clear("ds18b20_init_failed");
  Alarms::clear("ds18b20_not_found");
  Alarms::clear("ds18b20_role_assignment_required");
  Alarms::clear("max31865_init_failed");

  Serial.println("Storage::saveSensorAssignments...");
  Serial.flush();
  Storage::saveSensorAssignments(ctx_.assignments);
  Serial.println("SensorAssignments gespeichert");
  Serial.flush();

  Serial.println("RelayOutputs::begin...");
  Serial.flush();
  RelayOutputs::begin(ctx_);
  RelayOutputs::allOff(ctx_);
  Serial.println("RelayOutputs init fertig");
  Serial.flush();

  Serial.println("Pumps::begin...");
  Serial.flush();
  Pumps::begin(ctx_);
  Serial.println("Pumps init fertig");
  Serial.flush();

  Serial.println("INIT_SENSORS fertig -> SELF_TEST");
  Serial.flush();
  SafetyManager::begin(ctx_);

  changeState(SystemState::SELF_TEST);
}

void AppFSM::stateSelfTest() {
  HeatSourcesMax::startCycle(ctx_);

  while (!HeatSourcesMax::cycleComplete(ctx_)) {
    HeatSourcesMax::process(ctx_);
    yield();
  }

  EnergyMeter::process(ctx_);

  Ds18Role sinkRole = SensorAssignments::activeSinkRole(ctx_.config);
  SensorAssignments::readByRole(
    ctx_.assignments,
    sinkRole,
    ctx_.sensors.sinkC,
    ctx_.sensors.sinkValid
  );

  Serial.print("SELF_TEST activeHeatSource.role: ");
  Serial.println((int)ctx_.sensors.activeHeatSource.role);

  Serial.print("SELF_TEST activeHeatSource.tempC: ");
  Serial.print(ctx_.sensors.activeHeatSource.tempC);
  Serial.print(" | valid: ");
  Serial.println(ctx_.sensors.activeHeatSource.valid ? "JA" : "NEIN");

  Serial.print("SELF_TEST sinkC: ");
  Serial.print(ctx_.sensors.sinkC);
  Serial.print(" | valid: ");
  Serial.println(ctx_.sensors.sinkValid ? "JA" : "NEIN");

  if (!ctx_.sensors.activeHeatSource.valid || !ctx_.sensors.sinkValid) {
    ctx_.diag.sensorErrorCount++;
    ctx_.diag.faultActive = true;
    copyFaultText(ctx_.diag.faultText, sizeof(ctx_.diag.faultText), "Waermequelle oder Sink beim Start ungueltig");
    Alarms::raise("startup_source_or_sink_invalid", Alarms::Severity::CRITICAL, "Waermequelle oder Ziel-Sensor beim Start ungueltig", false);
    changeState(SystemState::FAULT);
    return;
  }

  ctx_.diag.faultActive = false;
  ctx_.diag.faultText[0] = '\0';
  Alarms::clear("startup_source_or_sink_invalid");
  Alarms::clear("source_or_sink_invalid");

  changeState(SystemState::IDLE);
}

void AppFSM::stateIdle() {
  const uint32_t now = millis();

  if ((uint32_t)(now - ctx_.lastSampleAtMs) >= ctx_.config.sampleIntervalMs) {
    changeState(SystemState::READ_SENSORS);
    return;
  }

  if ((uint32_t)(now - ctx_.lastRuntimeSaveAtMs) >= ctx_.config.runtimeSaveIntervalMs) {
    changeState(SystemState::UPDATE_RUNTIME);
    return;
  }
}

void AppFSM::stateReadSensors() {
  HeatSourcesMax::startCycle(ctx_);
  ctx_.lastSampleAtMs = millis();
  changeState(SystemState::VALIDATE_SENSORS);
}

void AppFSM::stateValidateSensors() {
  HeatSourcesMax::process(ctx_);

  if (!HeatSourcesMax::cycleComplete(ctx_)) {
    return;
  }

  EnergyMeter::process(ctx_);

  Ds18Role sinkRole = SensorAssignments::activeSinkRole(ctx_.config);
  SensorAssignments::readByRole(
    ctx_.assignments,
    sinkRole,
    ctx_.sensors.sinkC,
    ctx_.sensors.sinkValid
  );

  Serial.print("READ activeHeatSource.tempC: ");
  Serial.print(ctx_.sensors.activeHeatSource.tempC);
  Serial.print(" | valid: ");
  Serial.println(ctx_.sensors.activeHeatSource.valid ? "JA" : "NEIN");

  Serial.print("READ sinkC: ");
  Serial.print(ctx_.sensors.sinkC);
  Serial.print(" | valid: ");
  Serial.println(ctx_.sensors.sinkValid ? "JA" : "NEIN");

  if (!ctx_.sensors.activeHeatSource.valid || !ctx_.sensors.sinkValid) {
    ctx_.diag.sensorErrorCount++;
    ctx_.diag.faultActive = true;
    copyFaultText(ctx_.diag.faultText, sizeof(ctx_.diag.faultText), "Waermequelle oder Sink ungueltig");
    Alarms::raise("source_or_sink_invalid", Alarms::Severity::CRITICAL, "Waermequelle oder Ziel-Sensor ungueltig", false);
    changeState(SystemState::FAULT);
    return;
  }

  ctx_.diag.faultActive = false;
  ctx_.diag.faultText[0] = '\0';
  Alarms::clear("source_or_sink_invalid");
  Alarms::clear("startup_source_or_sink_invalid");

  changeState(SystemState::COMPUTE_CONTROL);
}

void AppFSM::stateComputeControl() {
  if (ctx_.commissioning.active) {

  Serial.println("TESTMODUS AKTIV - Regelung pausiert");
  Serial.flush();

  return;
}
  if (ctx_.sensors.activeHeatSource.valid && ctx_.sensors.sinkValid) {
    ctx_.control.diffC = ctx_.sensors.activeHeatSource.tempC - ctx_.sensors.sinkC;
  } else {
    ctx_.control.diffC = NAN;
  }
  if (UI::commissioningTestActive()) {
    Serial.println("INBETRIEBNAHME TESTMODUS: normale Ausgangslogik pausiert");
    Serial.flush();
    changeState(SystemState::IDLE);
    return;
  }

  SafetyManager::evaluate(ctx_);
  const auto& safetyStatus = SafetyManager::status();

  // Safety hat Vorrang
  if (safetyStatus.blockNormalPumpControl) {
    
    Serial.print("SAFETY ACTIVE: ");
    Serial.println(safetyStatus.message);
    Serial.flush();

    // Beispiel:
    // Hier später:
   SafetyManager::applyOutputs(ctx_);

    // Normale Pumpenlogik blockieren
    changeState(SystemState::IDLE);
    return;
  }

  Valves::process(ctx_);  
  AuxHeater::process(ctx_);
  OvenControl::process(ctx_);
  Pumps::process(ctx_);
  HeatingCircuits::process(ctx_);


  ctx_.diag.lastCollectorC = ctx_.sensors.activeHeatSource.tempC;
  ctx_.diag.lastSinkC = ctx_.sensors.sinkC;
  ctx_.diag.lastDiffC = ctx_.control.diffC;
  ctx_.diag.lastProtectionMode = static_cast<uint8_t>(ctx_.control.protectionMode);

  changeState(SystemState::APPLY_OUTPUTS);
}

void AppFSM::stateApplyOutputs() {
  // Die neue Pumpenarchitektur schaltet direkt in Pumps::process().

  changeState(SystemState::IDLE);
}

void AppFSM::stateUpdateRuntime() {
  ctx_.lastRuntimeSaveAtMs = millis();

  if (ctx_.config.staEnabled && ctx_.config.staSsid[0]) {
    if (WiFi.status() == WL_CONNECTED) {
      Alarms::clear("wifi_sta_disconnected");
    } else {
      Alarms::raise("wifi_sta_disconnected", Alarms::Severity::WARNING, "WLAN-Client ist nicht verbunden; SoftAP bleibt aktiv", true);
    }
  } else {
    Alarms::clear("wifi_sta_disconnected");
  }

  String outputValidationError;
  if (!OutputValidation::validateAll(ctx_, outputValidationError)) {
    Alarms::raise("output_config_invalid", Alarms::Severity::CRITICAL, outputValidationError.c_str(), false);
  } else {
    Alarms::clear("output_config_invalid");
  }

  const auto& safetyStatus = SafetyManager::status();
  if (safetyStatus.collectorStagnationActive) {
    Alarms::raise("safety_collector_stagnation", Alarms::Severity::WARNING, "Kollektor-Stagnationsschutz aktiv", true);
  } else {
    Alarms::clear("safety_collector_stagnation");
  }

  if (safetyStatus.storageOvertemperatureActive) {
    Alarms::raise("safety_storage_overtemperature", Alarms::Severity::CRITICAL, "Speicher-Uebertemperatur aktiv", false);
  } else {
    Alarms::clear("safety_storage_overtemperature");
  }

  if (safetyStatus.ovenOvertemperatureActive) {
    Alarms::raise("safety_oven_overtemperature", Alarms::Severity::CRITICAL, "Ofen-Uebertemperatur aktiv", false);
  } else {
    Alarms::clear("safety_oven_overtemperature");
  }

  for (uint8_t i = 0; i < MAX_PUMPS; i++) {
    char alarmId[40];
    snprintf(alarmId, sizeof(alarmId), "pump_%u_feedback_error", (unsigned)(i + 1));
    const PumpConfig& pump = ctx_.config.pumps[i];
    if (pump.enabled && pump.feedbackError) {
      char msg[96];
      snprintf(msg, sizeof(msg), "Pumpe %u: Feedbacksignal ungueltig", (unsigned)(i + 1));
      Alarms::raise(alarmId, Alarms::Severity::WARNING, msg, true);
    } else {
      Alarms::clear(alarmId);
    }
  }

  Storage::saveDiagnostics(ctx_.diag);
  Storage::saveMaintenance(ctx_.maintenance);
  EnergyMeter::saveRuntime(ctx_);

  changeState(SystemState::IDLE);
}

void AppFSM::stateFault() {
  if (UI::commissioningTestActive()) {
    // Manueller Testmodus hat Vorrang vor dem Fault-All-Off,
    // damit Relais-/Pumpen-/Ventiltests nicht sofort wieder überschrieben werden.
    if ((uint32_t)(millis() - ctx_.stateEnteredAtMs) >= FAULT_RETRY_INTERVAL_MS) {
      changeState(SystemState::READ_SENSORS);
    }
    return;
  }

  forceAllPumpsOff(ctx_);
  RelayOutputs::allOff(ctx_);
  HeatingCircuits::allOff(ctx_);

  if ((uint32_t)(millis() - ctx_.stateEnteredAtMs) >= FAULT_RETRY_INTERVAL_MS) {
    changeState(SystemState::READ_SENSORS);
  }
}
