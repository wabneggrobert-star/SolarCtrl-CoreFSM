#include "feature_aux_heater.h"
#include "feature_relay_outputs.h"
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

bool validRelay(uint8_t relayIndex) {
  return relayIndex != PIN_UNUSED && relayIndex < RELAY_COUNT;
}

void setRelaySafe(AppContext& ctx, uint8_t relayIndex, bool on) {
  if (!validRelay(relayIndex)) return;
  RelayOutputs::set(ctx, relayIndex, on);
}

void setPump(AppContext& ctx, bool on) {
  setRelaySafe(ctx, ctx.config.auxHeater.pumpRelay, on);
}

void setHeaterStage(AppContext& ctx, uint8_t stageCount) {
  const AuxHeaterConfig& cfg = ctx.config.auxHeater;
  setRelaySafe(ctx, cfg.heaterRelay1, stageCount >= 1);
  setRelaySafe(ctx, cfg.heaterRelay2, stageCount >= 2);
  setRelaySafe(ctx, cfg.heaterRelay3, stageCount >= 3);
}

void heatersOff(AppContext& ctx) {
  setHeaterStage(ctx, 0);
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
}

void allOff(AppContext& ctx) {
  heatersOff(ctx);
  setPump(ctx, false);
  g_state = AuxState::IDLE;
  g_stateStartedMs = 0;
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

      if (targetTemperatureC <= cfg.minimumTemperatureC) {
        setPump(ctx, true);
        g_state = AuxState::PRE_RUN;
        g_stateStartedMs = now;
        Serial.println("AUX HEATER: Pumpenvorlauf gestartet");
      }
      break;

    case AuxState::PRE_RUN:
      heatersOff(ctx);
      setPump(ctx, true);

      if ((uint32_t)(now - g_stateStartedMs) >= cfg.preRunMs) {
        g_state = AuxState::HEATING;
        g_stateStartedMs = now;
        Serial.println("AUX HEATER: Heizbetrieb gestartet");
      }
      break;

    case AuxState::HEATING: {
      setPump(ctx, true);

      const float differenceToTargetC = cfg.targetTemperatureC - targetTemperatureC;

      if (differenceToTargetC <= 0.0f) {
        Serial.println("AUX HEATER: Zieltemperatur erreicht, Nachlauf gestartet");
        enterCooldown(ctx);
        break;
      }

      if (differenceToTargetC > (cfg.hysteresisC * 2.0f)) {
        setHeaterStage(ctx, 3);
      } else if (differenceToTargetC > cfg.hysteresisC) {
        setHeaterStage(ctx, 2);
      } else {
        setHeaterStage(ctx, 1);
      }
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
