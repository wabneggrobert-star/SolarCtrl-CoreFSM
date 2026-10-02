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
#include "feature_sensor_roles.h"
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
#include "feature_mqtt.h"
#include "feature_time.h"
#include "feature_forecast.h"
#include "feature_ml_optimizer.h"
#include "feature_fluid_properties.h"
#include "feature_history.h"

#include <Arduino.h>
#include <WiFi.h>
#include <string.h>
#include <stdio.h>

#include "feature_build_flags.h"
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

  // Fast-Control V1:
  // Zeitkritische, bereits millis()-basierte Zustandsautomaten laufen mit 10 Hz.
  // Die eigentliche temperaturbasierte Pumpen-PID-Regelung bleibt dagegen
  // absichtlich an einem abgeschlossenen Sensorsnapshot haengen.
  static constexpr uint32_t FAST_CONTROL_INTERVAL_MS = 100UL;

  uint32_t g_lastSoftApCheckMs = 0;

  bool validSoftApIp() {
    IPAddress ip = WiFi.softAPIP();
    return !(ip[0] == 0 && ip[1] == 0 && ip[2] == 0 && ip[3] == 0);
  }

  // Die zentrale FSM darf keinen festen Top-Sensor mehr als einzigen Sink erzwingen.
  // Die Pumpen-/Routinglogik arbeitet bereits mit frei konfigurierten Speicherrollen.
  // Fuer den globalen Plausibilitaetscheck wird deshalb zuerst ein tatsaechlich
  // konfiguriertes Pumpenziel gesucht. Nur wenn keines vorhanden ist, folgen die
  // Legacy-Prioritaet und zuletzt irgendein gueltig zugewiesener Speichersensor.
  bool readConfiguredStorageSink(AppContext& ctx, Ds18Role& role, float& tempC, bool& valid) {
    role = Ds18Role::NONE;
    tempC = NAN;
    valid = false;

    auto tryRole = [&](Ds18Role candidate) -> bool {
      if (!SensorRoles::isSinkRole(candidate)) return false;
      float candidateC = NAN;
      bool candidateValid = false;
      if (!SensorAssignments::readByRole(ctx.ds18b20, ctx.assignments, candidate, candidateC, candidateValid)) return false;
      if (!candidateValid || isnan(candidateC)) return false;
      role = candidate;
      tempC = candidateC;
      valid = true;
      return true;
    };

    // 1) Reale Ziele aktivierter Pumpen haben Vorrang.
    for (uint8_t p = 0; p < MAX_PUMPS; p++) {
      const PumpConfig& pump = ctx.config.pumps[p];
      if (!pump.enabled || pump.mode == PumpMode::OFF) continue;

      for (uint8_t t = 0; t < PUMP_ROUTE_TARGET_COUNT; t++) {
        const PumpRouteTargetConfig& target = pump.targets[t];
        if (target.enabled && tryRole(target.sinkRole)) return true;
      }

      if (tryRole(pump.sinkRole)) return true;
    }

    // 2) Rueckwaertskompatibilitaet fuer alte Ein-Sink-Konfigurationen.
    if (tryRole(SensorAssignments::activeSinkRole(ctx.config))) return true;

    // 3) Falls noch kein Pumpenziel konfiguriert ist, akzeptiere jeden gueltig
    //    zugewiesenen Speicher-Sink. Das verhindert einen falschen globalen Fault
    //    nur wegen fehlendem Boiler-Top/Puffer-Top.
    static const Ds18Role storageRoles[] = {
      Ds18Role::SINK_BOILER_TOP,
      Ds18Role::BOILER_BOTTOM,
      Ds18Role::SINK_BUFFER_TOP,
      Ds18Role::BUFFER_HIGH,
      Ds18Role::BUFFER_MID,
      Ds18Role::BUFFER_BOTTOM
    };

    for (Ds18Role candidate : storageRoles) {
      if (tryRole(candidate)) return true;
    }

    return false;
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
  selfTestSensorCycleStarted_ = false;
  lastFastControlAtMs_ = 0;
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

  // Zeitkritische Aktor-/Timerlogik ist vom langsamen Sensorsample getrennt.
  // Der Scheduler arbeitet auch waehrend DS18B20/MAX31865 auf ihre Conversion
  // warten. Safety bleibt dabei immer vorrangig.
  runFastControlScheduler();

  if (ctx_.networkInitialized) {
    serviceNetwork();
    TimeService::process(ctx_);
    Forecast::process(ctx_);
    MqttBridge::process(ctx_);
  }
  // ML ist ein optionaler Overlay-Layer. Es darf weder Safety noch die normale
  // FSM blockieren; bei fehlenden Daten bleibt es automatisch passiv.
  MlOptimizer::process(ctx_, ctx_.commissioning.active || UI::commissioningTestActive());
  // History ist optional und laeuft nur in einer freien IDLE-Phase. Bei aktivem
  // Safety-Eingriff wird bewusst nicht auf SD geschrieben, damit Schutz und
  // normale Regelung immer Vorrang vor Logging haben.
  if (ctx_.state == SystemState::IDLE && !SafetyManager::status().blockNormalPumpControl) {
    History::process(ctx_);
  }
  Alarms::process(ctx_);
}

void AppFSM::runFastControlScheduler() {
  // Erst nach erfolgreichem Sensor-Selftest aktivieren. In Boot/Init/SELF_TEST
  // und FAULT bleibt die bestehende FSM alleiniger Besitzer der Ausgaenge.
  const bool runtimeState =
    ctx_.state == SystemState::IDLE ||
    ctx_.state == SystemState::READ_SENSORS ||
    ctx_.state == SystemState::VALIDATE_SENSORS ||
    ctx_.state == SystemState::COMPUTE_CONTROL ||
    ctx_.state == SystemState::APPLY_OUTPUTS ||
    ctx_.state == SystemState::UPDATE_RUNTIME;

  if (!runtimeState) return;

  const uint32_t now = millis();
  if (lastFastControlAtMs_ != 0 &&
      (uint32_t)(now - lastFastControlAtMs_) < FAST_CONTROL_INTERVAL_MS) {
    return;
  }
  lastFastControlAtMs_ = now;

  // Safety wird mit 10 Hz gegen den jeweils letzten gueltigen Sensorsnapshot
  // bewertet. Neue Temperaturen kommen weiterhin nur aus abgeschlossenen
  // Sensorzyklen, aber Safety-Ausgaenge/Timer muessen nicht auf den naechsten
  // 2-s-Control-Zyklus warten.
  SafetyManager::evaluate(ctx_);
  const auto& safetyStatus = SafetyManager::status();

  if (safetyStatus.blockNormalPumpControl) {
    // Alte Safety-Mischerauftraege duerfen keinen neuen Safety-Zyklus ueberleben.
    HeatingCircuits::prepareSafetyCycle(ctx_);
    SafetyManager::applyOutputs(ctx_);

    // safetyHeatDump() setzt ggf. in applyOutputs() einen expliziten
    // OPEN/CLOSE-Auftrag. Die eigentliche Puls-/Pause-FSM laeuft danach hier.
    HeatingCircuits::processFast(ctx_, true);

    Pumps::processFast(ctx_);
    return;
  }

  // In beiden Testmodi darf die normale Automatik keine Testausgaenge
  // ueberschreiben. Safety wurde absichtlich bereits davor ausgewertet.
  if (ctx_.commissioning.active || UI::commissioningTestActive()) {
    Pumps::processFast(ctx_);
    return;
  }

  // Diese Module besitzen bereits eigene millis()-basierte Zeitlogik und sind
  // deshalb fuer den 100-ms-Takt geeignet.
  Valves::process(ctx_);
  AuxHeater::process(ctx_);
  OvenControl::process(ctx_);

  // Pumpenfeedback ist jetzt interruptbasiert und darf schnell ausgewertet.
  // Die eigentliche Solar-/Differenz-/PID-Regelung bleibt in Pumps::process().
  Pumps::processFast(ctx_);

  // Heizkreis V2:
  // Der langsame HeatingCircuits::process()-Pfad berechnet weiterhin nur mit
  // neuen Temperatursnapshots die gewuenschte Richtung. Die physische
  // PULSE->WAIT-Mischer-FSM darf dagegen mit 10 Hz laufen.
  HeatingCircuits::processFast(ctx_, false);
}

void AppFSM::changeState(SystemState next) {
  ctx_.state = next;
  ctx_.stateEnteredAtMs = millis();
}

void AppFSM::stateInitHw() {
  DBG_PRINTLN("INIT_HW gestartet");

  // I2C-Hardware initialisieren, aber fehlende Erweiterungsbausteine blockieren den AP nicht.
  bool relayOk = RelayOutputs::begin(ctx_);
  DBG_PRINT("Relais-PCF: ");
  DBG_PRINTLN(relayOk ? "OK" : "FEHLER/NICHT GEFUNDEN");

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
  DBG_PRINT("PCA9685: ");
  DBG_PRINTLN(pwmOk ? "OK" : "FEHLER/NICHT GEFUNDEN");
  if (pwmOk) {
    PwmDriver::allOff(ctx_.config);
  }

  OvenControl::begin(ctx_);
  EnergyMeter::begin(ctx_);
  TimeService::begin(ctx_);
  Forecast::begin(ctx_);
  MlOptimizer::begin(ctx_);
  History::begin(ctx_);

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
    DBG_PRINTLN("WLAN-Modus wurde korrigiert: WIFI_AP_STA");
    WiFi.mode(WIFI_AP_STA);
  }

  const char* ssid = ctx_.config.apName[0] ? ctx_.config.apName : DEFAULT_AP_SSID;
  const char* pass = ctx_.config.apPassword[0] ? ctx_.config.apPassword : DEFAULT_AP_PASSWORD;

  if (!validSoftApIp()) {
    DBG_PRINTLN("SoftAP nicht aktiv - starte SoftAP erneut");
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

  DBG_PRINT("Starte nicht-blockierenden WLAN-Client-Verbindungsversuch: ");
  DBG_PRINTLN(ctx_.config.staSsid);

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
  DBG_PRINTLN("INIT_NETWORK gestartet");

  WiFi.persistent(false);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(false);
  WiFi.mode(WIFI_AP_STA);

  const char* ssid = ctx_.config.apName[0] ? ctx_.config.apName : DEFAULT_AP_SSID;
  const char* pass = ctx_.config.apPassword[0] ? ctx_.config.apPassword : DEFAULT_AP_PASSWORD;
  const char* hostName = ctx_.config.hostName[0] ? ctx_.config.hostName : DEFAULT_HOSTNAME;

  WiFi.setHostname(hostName);

  DBG_PRINT("Starte AP mit SSID: ");
  DBG_PRINTLN(ssid);

  bool ok = WiFi.softAP(ssid, pass);

  DBG_PRINT("softAP Ergebnis: ");
  DBG_PRINTLN(ok ? "OK" : "FEHLER");

  DBG_PRINT("AP IP: ");
  DBG_PRINTLN(WiFi.softAPIP());

  ctx_.networkInitialized = true;
  ctx_.networkApActive = validSoftApIp();
  g_lastSoftApCheckMs = 0;
  ensureSoftAp();

  if (ctx_.config.staEnabled && ctx_.config.staSsid[0]) {
    startStaConnect();
    DBG_PRINTLN("WLAN-Client-Verbindung laeuft nicht-blockierend im Hintergrund.");
  } else {
    DBG_PRINTLN("WLAN-Client deaktiviert. Nur SoftAP aktiv.");
  }

  // MQTT ist rein additiv. Fehlender Broker/WLAN darf die Regelung nie blockieren.
  MqttBridge::begin(ctx_);

  changeState(SystemState::INIT_UI);
}

void AppFSM::stateInitUi() {
  DBG_PRINTLN("INIT_UI gestartet");
  UI::begin(ctx_);
  changeState(SystemState::INIT_SENSORS);
}

void AppFSM::stateInitSensors() {
  
  DBG_PRINTLN("INIT_SENSORS gestartet");
  DBG_FLUSH();
  
  DBG_PRINTLN("SinkSensor::begin...");
  DBG_FLUSH();
  bool sinkBusOk = SinkSensor::begin();
  DBG_PRINTLN(sinkBusOk ? "SinkSensor OK" : "SinkSensor FEHLER");
  DBG_FLUSH();

  DBG_PRINTLN("HeatSourcesMax::begin...");
  DBG_FLUSH();
  bool heatSourcesOk = HeatSourcesMax::begin(ctx_);
  DBG_PRINTLN(heatSourcesOk ? "HeatSourcesMax OK" : "HeatSourcesMax FEHLER");
  DBG_FLUSH();

  DBG_PRINTLN("SinkSensor::scanBus...");
  DBG_FLUSH();
  SinkSensor::scanBus(ctx_.ds18b20);
  DBG_PRINTLN("scanBus fertig");
  DBG_FLUSH();

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

  DBG_PRINTLN("SensorAssignments::resolveAssignments...");
  DBG_FLUSH();

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

    DBG_PRINTLN("SensorAssignments FEHLER");
    DBG_FLUSH();

    changeState(SystemState::FAULT);
    return;
  }

  DBG_PRINTLN("SensorAssignments OK");
  DBG_FLUSH();
  Alarms::clear("ds18b20_init_failed");
  Alarms::clear("ds18b20_not_found");
  Alarms::clear("ds18b20_role_assignment_required");
  Alarms::clear("max31865_init_failed");

  DBG_PRINTLN("Storage::saveSensorAssignments...");
  DBG_FLUSH();
  Storage::saveSensorAssignments(ctx_.assignments);
  DBG_PRINTLN("SensorAssignments gespeichert");
  DBG_FLUSH();

  DBG_PRINTLN("RelayOutputs::begin...");
  DBG_FLUSH();
  RelayOutputs::begin(ctx_);
  RelayOutputs::allOff(ctx_);
  DBG_PRINTLN("RelayOutputs init fertig");
  DBG_FLUSH();

  DBG_PRINTLN("Pumps::begin...");
  DBG_FLUSH();
  Pumps::begin(ctx_);
  DBG_PRINTLN("Pumps init fertig");
  DBG_FLUSH();

  DBG_PRINTLN("INIT_SENSORS fertig -> SELF_TEST");
  DBG_FLUSH();
  SafetyManager::begin(ctx_);

  changeState(SystemState::SELF_TEST);
}

void AppFSM::stateSelfTest() {
  // Beide Sensorwelten starten denselben Self-Test-Zyklus. Der DS18B20-Pfad
  // wartet nicht mehr blockierend auf die Conversion, sondern kehrt sofort zurueck.
  if (!selfTestSensorCycleStarted_) {
    SinkSensor::startCycle(ctx_.ds18b20);
    HeatSourcesMax::startCycle(ctx_);
    selfTestSensorCycleStarted_ = true;
    return;
  }

  SinkSensor::process(ctx_.ds18b20);
  HeatSourcesMax::process(ctx_);

  if (!SinkSensor::cycleComplete() || !HeatSourcesMax::cycleComplete(ctx_)) {
    return;
  }

  selfTestSensorCycleStarted_ = false;
  EnergyMeter::process(ctx_);

  Ds18Role sinkRole = Ds18Role::NONE;
  readConfiguredStorageSink(
    ctx_,
    sinkRole,
    ctx_.sensors.sinkC,
    ctx_.sensors.sinkValid
  );

  DBG_PRINT("SELF_TEST activeHeatSource.role: ");
  DBG_PRINTLN((int)ctx_.sensors.activeHeatSource.role);

  DBG_PRINT("SELF_TEST activeHeatSource.tempC: ");
  DBG_PRINT(ctx_.sensors.activeHeatSource.tempC);
  DBG_PRINT(" | valid: ");
  DBG_PRINTLN(ctx_.sensors.activeHeatSource.valid ? "JA" : "NEIN");

  DBG_PRINT("SELF_TEST sinkC: ");
  DBG_PRINT(ctx_.sensors.sinkC);
  DBG_PRINT(" | valid: ");
  DBG_PRINTLN(ctx_.sensors.sinkValid ? "JA" : "NEIN");

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
  // Beide Messzyklen werden gemeinsam angestossen. DS18B20 und MAX31865
  // laufen ab hier als nicht-blockierende FSMs im Hintergrund.
  SinkSensor::startCycle(ctx_.ds18b20);
  HeatSourcesMax::startCycle(ctx_);
  ctx_.lastSampleAtMs = millis();
  changeState(SystemState::VALIDATE_SENSORS);
}

void AppFSM::stateValidateSensors() {
  SinkSensor::process(ctx_.ds18b20);
  HeatSourcesMax::process(ctx_);

  if (!SinkSensor::cycleComplete() || !HeatSourcesMax::cycleComplete(ctx_)) {
    return;
  }

  EnergyMeter::process(ctx_);

  Ds18Role sinkRole = Ds18Role::NONE;
  readConfiguredStorageSink(
    ctx_,
    sinkRole,
    ctx_.sensors.sinkC,
    ctx_.sensors.sinkValid
  );

  DBG_PRINT("READ activeHeatSource.tempC: ");
  DBG_PRINT(ctx_.sensors.activeHeatSource.tempC);
  DBG_PRINT(" | valid: ");
  DBG_PRINTLN(ctx_.sensors.activeHeatSource.valid ? "JA" : "NEIN");

  DBG_PRINT("READ sinkC: ");
  DBG_PRINT(ctx_.sensors.sinkC);
  DBG_PRINT(" | valid: ");
  DBG_PRINTLN(ctx_.sensors.sinkValid ? "JA" : "NEIN");

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
  if (ctx_.sensors.activeHeatSource.valid && ctx_.sensors.sinkValid) {
    ctx_.control.diffC = ctx_.sensors.activeHeatSource.tempC - ctx_.sensors.sinkC;
  } else {
    ctx_.control.diffC = NAN;
  }

  // Safety wird auch im Service-/Inbetriebnahme-Test ausgewertet. Ein manueller
  // Test darf einen echten kritischen Speicherzustand nicht maskieren.
  SafetyManager::evaluate(ctx_);
  const auto& safetyStatus = SafetyManager::status();

  if (safetyStatus.blockNormalPumpControl) {
    DBG_PRINT("SAFETY ACTIVE: ");
    DBG_PRINTLN(safetyStatus.message);
    DBG_FLUSH();

    SafetyManager::applyOutputs(ctx_);
    changeState(SystemState::IDLE);
    return;
  }

  if (ctx_.commissioning.active) {
    DBG_PRINTLN("TESTMODUS AKTIV - Regelung pausiert");
    DBG_FLUSH();
    return;
  }

  if (UI::commissioningTestActive()) {
    DBG_PRINTLN("INBETRIEBNAHME TESTMODUS: normale Ausgangslogik pausiert");
    DBG_FLUSH();
    changeState(SystemState::IDLE);
    return;
  }

  // Sensorsnapshot-getriebene Regelung:
  // Pumps::process() enthaelt die temperaturbasierte Differenz-/PID-Regelung
  // und darf deshalb exakt einmal pro neuem Sensorsnapshot laufen.
  Pumps::process(ctx_);

  // Heizkreis-Sollwerte und Pumpenanforderung bleiben snapshot-getrieben.
  // HeatingCircuits::process() setzt nur den Mischer-Richtungsauftrag; die
  // physische Puls-/Pause-FSM laeuft im 100-ms-Fast-Control.
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

  // Plausibilitaetswarnung zwischen gemessenem Glykol-Frostschutz und Safety-Auslegung.
  // Reine Warnung: sie blockiert weder Grundregelung noch Safety-Ausgaenge.
  if (ctx_.config.solarFluidType == SolarFluidType::GLYCOL) {
    const FluidProperties::Result fluid = FluidProperties::evaluate(ctx_.config, 40.0f);
    if (!fluid.safetyCompatible) {
      Alarms::raise("glycol_freeze_protection_mismatch", Alarms::Severity::WARNING,
                    "Gemessener Glykol-Frostschutz reicht fuer die Safety-Auslegung nicht aus", true);
    } else {
      Alarms::clear("glycol_freeze_protection_mismatch");
    }
  } else {
    Alarms::clear("glycol_freeze_protection_mismatch");
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

  // Speicheralarme werden direkt im schnellen Safety-Zyklus synchronisiert,
  // damit die kritische Temperatur nicht vom Runtime-Save-Intervall abhaengt.
  if (!safetyStatus.storageCriticalOvertemperatureActive) {
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
