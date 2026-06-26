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

#include <Arduino.h>
#include <WiFi.h>
#include <string.h>

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

  bool pwmOk = PwmDriver::begin();
  Serial.print("PCA9685: ");
  Serial.println(pwmOk ? "OK" : "FEHLER/NICHT GEFUNDEN");
  OvenControl::begin(ctx_);
  HeatingCircuits::begin(ctx_);
  changeState(SystemState::INIT_SD);
}

void AppFSM::stateInitSd() {
  ctx_.sdAvailable = SDCard::begin();

  if (!ctx_.sdAvailable) {
    ctx_.diag.sdErrorCount++;
    copyFaultText(ctx_.diag.faultText, sizeof(ctx_.diag.faultText), "SD Initialisierung fehlgeschlagen");
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
  Storage::loadSensorAssignments(ctx_.assignments);
  HeatSourceStorage::loadAssignments(ctx_.heatSourceAssignments);

  ctx_.diag.bootCount++;
  Storage::saveDiagnostics(ctx_.diag);

  changeState(SystemState::INIT_NETWORK);
}

void AppFSM::stateInitNetwork() {
  Serial.println("INIT_NETWORK gestartet");

  WiFi.mode(WIFI_AP);

  const char* ssid = ctx_.config.apName[0] ? ctx_.config.apName : DEFAULT_AP_SSID;
  const char* pass = ctx_.config.apPassword[0] ? ctx_.config.apPassword : DEFAULT_AP_PASSWORD;

  Serial.print("Starte AP mit SSID: ");
  Serial.println(ssid);

  bool ok = WiFi.softAP(ssid, pass);

  Serial.print("softAP Ergebnis: ");
  Serial.println(ok ? "OK" : "FEHLER");

  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

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
    changeState(SystemState::FAULT);
    return;
  }

  if (!heatSourcesOk) {
    copyFaultText(
      ctx_.diag.faultText,
      sizeof(ctx_.diag.faultText),
      "MAX31865 Initialisierung fehlgeschlagen"
    );
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
    } else {
      copyFaultText(
        ctx_.diag.faultText,
        sizeof(ctx_.diag.faultText),
        "DS18B20 Rollen-Zuordnung erforderlich"
      );
    }

    Serial.println("SensorAssignments FEHLER");
    Serial.flush();

    changeState(SystemState::FAULT);
    return;
  }

  Serial.println("SensorAssignments OK");
  Serial.flush();

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
    changeState(SystemState::FAULT);
    return;
  }

  ctx_.diag.faultActive = false;
  ctx_.diag.faultText[0] = '\0';

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
    changeState(SystemState::FAULT);
    return;
  }

  ctx_.diag.faultActive = false;
  ctx_.diag.faultText[0] = '\0';

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

  Storage::saveDiagnostics(ctx_.diag);
  Storage::saveMaintenance(ctx_.maintenance);

  changeState(SystemState::IDLE);
}

void AppFSM::stateFault() {
  forceAllPumpsOff(ctx_);
  RelayOutputs::allOff(ctx_);
  HeatingCircuits::allOff(ctx_);

  if ((uint32_t)(millis() - ctx_.stateEnteredAtMs) >= FAULT_RETRY_INTERVAL_MS) {
    changeState(SystemState::READ_SENSORS);
  }
}
