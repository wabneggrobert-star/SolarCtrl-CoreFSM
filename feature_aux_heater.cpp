#include "feature_aux_heater.h"
#include "feature_relay_outputs.h"
#include "feature_pwm_pca9685.h"
#include "feature_safety_manager.h"
#include "feature_sensor_assignments.h"

#include <Arduino.h>
#include <math.h>

namespace {

enum class AuxState : uint8_t {
  IDLE,
  PRE_RUN,
  HEATING,
  COOLDOWN
};

AuxState g_state = AuxState::IDLE;
uint32_t g_stateStartedMs = 0;

// Heizstab-Stufen duerfen beim Hochregeln nicht gleichzeitig zuschalten.
// Abregeln darf dagegen sofort erfolgen, damit kurz vor Zieltemperatur
// keine unnoetige Leistung mehr eingebracht wird.
static constexpr uint32_t AUX_STAGE_STEP_DELAY_MS = 10000UL;

uint8_t g_activeStageCount = 0;
uint32_t g_lastStageChangeMs = 0;

// Der Reglerlauf erfolgt nur alle paar Sekunden. Wiederholtes Schreiben derselben
// PO-/Relais-Schaltstellung kann bei manchen Ausgangstreibern wie ein Takten wirken.
// Deshalb werden die logischen Zusatzheizungs-Ausgaenge gecached und nur bei echter
// Sollwertaenderung geschrieben.
OutputRef g_lastPumpRef;
bool g_pumpApplied = false;
bool g_pumpDesired = false;

OutputRef g_lastStageRef[3];
bool g_stageApplied[3] = {false, false, false};
bool g_stageDesired[3] = {false, false, false};

bool validRelay(uint8_t relayIndex) {
  return relayIndex != PIN_UNUSED && relayIndex < RELAY_COUNT;
}

void setRelaySafe(AppContext& ctx, uint8_t relayIndex, bool on) {
  if (!validRelay(relayIndex)) return;
  RelayOutputs::set(ctx, relayIndex, on);
}

bool outputAssigned(const OutputRef& ref) {
  return ref.kind != OutputKind::NONE && ref.index != PIN_UNUSED;
}

bool sameOutputRef(const OutputRef& a, const OutputRef& b) {
  return a.kind == b.kind && a.index == b.index;
}

OutputRef stageOutput(const AuxHeaterConfig& cfg, uint8_t stageIndex) {
  switch (stageIndex) {
    case 0: return outputAssigned(cfg.heaterOutput1) ? cfg.heaterOutput1 : OutputRef{OutputKind::RELAY, cfg.heaterRelay1};
    case 1: return outputAssigned(cfg.heaterOutput2) ? cfg.heaterOutput2 : OutputRef{OutputKind::RELAY, cfg.heaterRelay2};
    case 2: return outputAssigned(cfg.heaterOutput3) ? cfg.heaterOutput3 : OutputRef{OutputKind::RELAY, cfg.heaterRelay3};
    default: return OutputRef{};
  }
}

void setOutputSafe(AppContext& ctx, const OutputRef& ref, bool on) {
  if (!outputAssigned(ref)) return;

  if (ref.kind == OutputKind::RELAY) {
    setRelaySafe(ctx, ref.index, on);
    return;
  }

  if (ref.kind == OutputKind::PWM_OUTPUT && ref.index < PWM_OUTPUT_COUNT) {
    const PwmOutputConfig& po = ctx.config.pwmOutputs[ref.index];
    if (!po.enabled) return;
    PwmDriver::setSwitch(ref.index, on, po.profile);
  }
}

void applyCachedOutput(AppContext& ctx, const OutputRef& desiredRef, bool desiredOn,
                       OutputRef& lastRef, bool& applied, bool& lastDesired) {
  // Wurde die Ausgangszuordnung im Webmenue geaendert, alten Ausgang sicher freigeben.
  if (applied && !sameOutputRef(desiredRef, lastRef)) {
    setOutputSafe(ctx, lastRef, false);
    applied = false;
    lastDesired = false;
  }

  if (!outputAssigned(desiredRef)) {
    lastRef = desiredRef;
    applied = false;
    lastDesired = false;
    return;
  }

  if (!applied || lastDesired != desiredOn) {
    setOutputSafe(ctx, desiredRef, desiredOn);
    lastRef = desiredRef;
    lastDesired = desiredOn;
    applied = true;
  }
}

void resetOutputCache() {
  g_lastPumpRef = OutputRef{};
  g_pumpApplied = false;
  g_pumpDesired = false;

  for (uint8_t i = 0; i < 3; i++) {
    g_lastStageRef[i] = OutputRef{};
    g_stageApplied[i] = false;
    g_stageDesired[i] = false;
  }
}

OutputRef pumpOutput(const AuxHeaterConfig& cfg) {
  return outputAssigned(cfg.pumpOutput) ? cfg.pumpOutput : OutputRef{OutputKind::RELAY, cfg.pumpRelay};
}

void setPump(AppContext& ctx, bool on) {
  applyCachedOutput(ctx, pumpOutput(ctx.config.auxHeater), on,
                    g_lastPumpRef, g_pumpApplied, g_pumpDesired);
}

uint8_t availableStageCount(const AuxHeaterConfig& cfg) {
  uint8_t count = 0;
  for (uint8_t i = 0; i < 3; i++) {
    if (outputAssigned(stageOutput(cfg, i))) count++;
  }
  return count;
}

void setConfiguredHeaterStages(AppContext& ctx, uint8_t requestedCount) {
  const AuxHeaterConfig& cfg = ctx.config.auxHeater;
  uint8_t configuredSeen = 0;

  for (uint8_t i = 0; i < 3; i++) {
    const OutputRef out = stageOutput(cfg, i);
    bool desiredOn = false;

    if (outputAssigned(out)) {
      configuredSeen++;
      desiredOn = configuredSeen <= requestedCount;
    }

    applyCachedOutput(ctx, out, desiredOn,
                      g_lastStageRef[i], g_stageApplied[i], g_stageDesired[i]);
  }
}

void setHeaterStageCount(AppContext& ctx, uint8_t stageCount) {
  const uint8_t maxStages = availableStageCount(ctx.config.auxHeater);
  if (stageCount > maxStages) stageCount = maxStages;

  setConfiguredHeaterStages(ctx, stageCount);
  g_activeStageCount = stageCount;
}

void heatersOff(AppContext& ctx) {
  setHeaterStageCount(ctx, 0);
}

float auxHeaterStopTemperatureC(const AuxHeaterConfig& cfg) {
  // Einschaltbedingung: unter Mindesttemperatur.
  // Abschaltbedingung: Zieltemperatur + Hysterese.
  // Beispiel: Minimum 35 C, Ziel 55 C, Hysterese 2 K -> Start <35 C, Stop >=57 C.
  return cfg.targetTemperatureC + max(0.0f, cfg.hysteresisC);
}

uint8_t requestedStageCount(const AuxHeaterConfig& cfg, float currentTemperatureC) {
  const uint8_t maxStages = availableStageCount(cfg);
  if (maxStages == 0) return 0;

  const float h = max(0.1f, cfg.hysteresisC);
  const float stopTemperatureC = auxHeaterStopTemperatureC(cfg);

  // Geheizt wird nicht nur bis Zieltemperatur, sondern bis Ziel + Hysterese.
  if (currentTemperatureC >= stopTemperatureC) return 0;

  // Bei Zieltemperatur bis Ziel+Hysterese nur noch eine Stufe halten.
  // Beispiel 55..57 C = 1 Stufe.
  if (currentTemperatureC >= cfg.targetTemperatureC) {
    return 1;
  }

  // Knapp unter Zieltemperatur Leistung reduzieren.
  // Beispiel bei 55 C Ziel und 2 K Hysterese: 51..55 C = maximal 2 Stufen.
  if (currentTemperatureC >= (cfg.targetTemperatureC - (2.0f * h))) {
    return maxStages >= 2 ? 2 : 1;
  }

  // Deutlich unter Ziel: alle verfuegbaren Stufen anfordern,
  // das Zuschalten erfolgt weiterhin zeitversetzt.
  return maxStages;
}

void applyStageRequest(AppContext& ctx, uint8_t requestedCount, uint32_t now) {
  const uint8_t maxStages = availableStageCount(ctx.config.auxHeater);
  if (requestedCount > maxStages) requestedCount = maxStages;

  // Abregeln sofort, damit kurz vor Zieltemperatur nicht weiter mit voller Leistung geheizt wird.
  if (requestedCount < g_activeStageCount) {
    setHeaterStageCount(ctx, requestedCount);
    g_lastStageChangeMs = now;
    return;
  }

  // Erste Stufe sofort einschalten. Weitere Stufen nur mit Abstand zuschalten.
  if (requestedCount > g_activeStageCount) {
    if (g_activeStageCount == 0 || (uint32_t)(now - g_lastStageChangeMs) >= AUX_STAGE_STEP_DELAY_MS) {
      setHeaterStageCount(ctx, g_activeStageCount + 1);
      g_lastStageChangeMs = now;
      return;
    }
  }

  // Keine zyklische Neuschreibung gleicher Ausgangszustaende.
  // Die Ausgaenge werden nur bei echter Aenderung geschrieben.
  setConfiguredHeaterStages(ctx, g_activeStageCount);
}

bool readTargetTemperature(AppContext& ctx, float& temperatureC) {
  temperatureC = NAN;

  if (ctx.config.auxHeater.sinkRole == Ds18Role::NONE) {
    return false;
  }

  bool valid = false;
  if (!SensorAssignments::readByRole(
        ctx.assignments,
        ctx.config.auxHeater.sinkRole,
        temperatureC,
        valid)) {
    return false;
  }

  return valid && !isnan(temperatureC);
}

bool safetyBlocksAuxHeater() {
  const SafetyManager::SafetyStatus& safety = SafetyManager::status();

  if (safety.sensorFaultActive) return true;
  if (safety.storageOvertemperatureActive) return true;
  if (safety.collectorStagnationActive) return true;
  if (safety.frostProtectionActive) return true;
  if (safety.controlledStagnation) return true;

  return false;
}

void enterCooldown(AppContext& ctx) {
  heatersOff(ctx);
  setPump(ctx, true);
  g_state = AuxState::COOLDOWN;
  g_stateStartedMs = millis();
}

} // namespace

namespace AuxHeater {

void begin(AppContext&) {
  g_state = AuxState::IDLE;
  g_stateStartedMs = 0;
  g_activeStageCount = 0;
  g_lastStageChangeMs = 0;
  resetOutputCache();
}

void allOff(AppContext& ctx) {
  heatersOff(ctx);
  setPump(ctx, false);
  g_state = AuxState::IDLE;
  g_stateStartedMs = 0;
  g_activeStageCount = 0;
  g_lastStageChangeMs = 0;
}

void resumeAfterTestMode() {
  // Der Testmodus darf die Hardware direkt schalten. Dadurch kann der lokale
  // Ausgangs-Cache der Zusatzheizung nach dem Verlassen des Testmodus von der
  // realen PCA-/Relaislage abweichen. Nur den Cache verwerfen, den logischen
  // Heizzyklus aber erhalten: PRE_RUN/HEATING/COOLDOWN darf nach dem Testmodus
  // dort weiterlaufen, wo er vorher war.
  resetOutputCache();

  Serial.println("AUX HEATER: Testmodus beendet - Ausgangs-Cache verworfen");
  Serial.flush();
}

bool heatingActive() {
  return g_state == AuxState::HEATING;
}

bool cooldownActive() {
  return g_state == AuxState::COOLDOWN;
}

void process(AppContext& ctx) {
  AuxHeaterConfig& cfg = ctx.config.auxHeater;

  if (!cfg.enabled) {
    allOff(ctx);
    return;
  }

  if (safetyBlocksAuxHeater()) {
    allOff(ctx);
    return;
  }

  float targetTemperatureC = NAN;
  if (!readTargetTemperature(ctx, targetTemperatureC)) {
    allOff(ctx);
    return;
  }

  const uint32_t now = millis();

  switch (g_state) {

    case AuxState::IDLE:
      heatersOff(ctx);
      setPump(ctx, false);

      if (targetTemperatureC < cfg.minimumTemperatureC) {
        setPump(ctx, true);
        g_state = AuxState::PRE_RUN;
        g_stateStartedMs = now;
        Serial.println("AUX HEATER: Pumpenvorlauf gestartet");
      }
      break;

    case AuxState::PRE_RUN:
      heatersOff(ctx);
      setPump(ctx, true);

      // Falls der Speicher waehrend des Pumpenvorlaufs bereits durch eine andere
      // Waermequelle auf Ziel + Hysterese steigt, wird die Zusatzheizung abgebrochen.
      if (targetTemperatureC >= auxHeaterStopTemperatureC(cfg)) {
        setPump(ctx, false);
        g_state = AuxState::IDLE;
        g_stateStartedMs = now;
        break;
      }

      if ((uint32_t)(now - g_stateStartedMs) >= cfg.preRunMs) {
        g_state = AuxState::HEATING;
        g_stateStartedMs = now;
        g_activeStageCount = 0;
        g_lastStageChangeMs = 0;
        Serial.println("AUX HEATER: Heizbetrieb gestartet");
      }
      break;

    case AuxState::HEATING: {
      setPump(ctx, true);

      const uint8_t requestedStages = requestedStageCount(cfg, targetTemperatureC);

      if (requestedStages == 0) {
        Serial.println("AUX HEATER: Zieltemperatur erreicht, Nachlauf gestartet");
        enterCooldown(ctx);
        break;
      }

      applyStageRequest(ctx, requestedStages, now);
      break;
    }

    case AuxState::COOLDOWN:
      heatersOff(ctx);
      setPump(ctx, true);

      if ((uint32_t)(now - g_stateStartedMs) >= cfg.cooldownMs) {
        setPump(ctx, false);
        g_state = AuxState::IDLE;
        g_stateStartedMs = now;
        Serial.println("AUX HEATER: Nachlauf beendet");
      }
      break;
  }
}

} // namespace AuxHeater
