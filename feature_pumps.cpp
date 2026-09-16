#include "feature_pumps.h"

#include "config.h"
#include "feature_energy_conflicts.h"
#include "feature_pump_routing.h"
#include "feature_relay_outputs.h"
#include "feature_pwm_pca9685.h"
#include "feature_sensor_assignments.h"
#include "feature_valves.h"

#include <Arduino.h>
#include <math.h>

namespace {
  struct PidState {
    float integral = 0.0f;
    float lastError = 0.0f;
  };

  PidState pid[MAX_PUMPS];

  float clampFloat(float v, float minV, float maxV) {
    if (v < minV) return minV;
    if (v > maxV) return maxV;
    return v;
  }

  bool validPumpIndex(uint8_t i) {
    return i < MAX_PUMPS;
  }

  bool validPcaChannel(uint8_t channel) {
    return channel != PIN_UNUSED && channel < 16;
  }

  bool pwmOutputAvailableForPump(const AppContext& ctx, uint8_t channel) {
    if (channel == PIN_UNUSED || channel >= PWM_OUTPUT_COUNT) return false;
    const PwmOutputConfig& out = ctx.config.pwmOutputs[channel];
    return out.enabled && out.mode == PwmOutputMode::PWM;
  }

  uint8_t defaultPcaChannelFor(uint8_t pumpIndex) {
    if (pumpIndex >= MAX_PUMPS) return PIN_UNUSED;
    return PUMP_PWM_CHANNELS[pumpIndex];
  }

uint8_t defaultFeedbackPinFor(uint8_t pumpIndex) {

  if (pumpIndex >= FEEDBACK_INPUT_COUNT) {
    return PIN_UNUSED;
  }

  return pumpIndex;
}

  void setPwmPercent(const PumpConfig& pump, float percent) {
    // Wichtig: Das Pumpenmodul darf nur PCA-Kanaele anfassen, die wirklich
    // als PWM-Pumpenausgang dieser Pumpe verwendet werden. Deaktivierte Pumpen
    // und Relaispumpen behalten teilweise Default-pwmChannel-Werte. Ohne diese
    // Abgrenzung wuerden sie fremde PO-Schaltausgaenge, z. B. Zusatzheizung
    // Heizstab PO1-PO3, bei jedem Reglerlauf wieder auf 0 % setzen.
    if (pump.mode != PumpMode::PWM) return;
    if (!validPcaChannel(pump.pwmChannel)) return;
    percent = clampFloat(percent, 0.0f, 100.0f);
    PwmDriver::setDuty(pump.pwmChannel, static_cast<uint8_t>(percent), pump.pwmProfile);
  }

  void resetPumpRuntime(PumpConfig& p) {
    p.state = false;
    p.lastSourceC = NAN;
    p.lastSinkC = NAN;
    p.lastDiffC = NAN;
    p.lastPwmPercent = 0.0f;
    p.feedbackSignalPresent = false;
    p.feedbackError = false;
    p.feedbackDutyPercent = NAN;
    p.feedbackLastCheckedMs = 0;
  }

  float computePID(uint8_t i, PumpConfig& cfg, float error) {
    PidState& s = pid[i];

    s.integral += error * cfg.pidKi;
    s.integral = clampFloat(s.integral, -100.0f, 100.0f);

    float derivative = (error - s.lastError) * cfg.pidKd;
    s.lastError = error;

    float out = cfg.pidKp * error + s.integral + derivative;
    return clampFloat(out, 0.0f, 100.0f);
  }

  bool heatSourceTempByRole(const SensorSnapshot& sensors, HeatSourceRole role, float& tempC, bool& valid) {
    tempC = NAN;
    valid = false;

    switch (role) {
      case HeatSourceRole::SOLAR_COLLECTOR_1:
        tempC = sensors.heatSources.solarCollector1C;
        valid = sensors.heatSources.solarCollector1Valid;
        return true;

      case HeatSourceRole::SOLAR_COLLECTOR_2:
        tempC = sensors.heatSources.solarCollector2C;
        valid = sensors.heatSources.solarCollector2Valid;
        return true;

      case HeatSourceRole::SOLAR_COLLECTOR_3:
        tempC = sensors.heatSources.solarCollector3C;
        valid = sensors.heatSources.solarCollector3Valid;
        return true;

      case HeatSourceRole::ALT_SOURCE_OVEN:
        tempC = sensors.heatSources.altSourceOvenC;
        valid = sensors.heatSources.altSourceOvenValid;
        return true;

      case HeatSourceRole::ALT_SOURCE_OTHER:
        tempC = sensors.heatSources.altSourceOtherC;
        valid = sensors.heatSources.altSourceOtherValid;
        return true;

      case HeatSourceRole::NONE:
      default:
        return false;
    }
  }

  bool sensorSourceTempByRole(const AppContext& ctx, Ds18Role role, float& temperatureC, bool& valid) {
    temperatureC = NAN;
    valid = false;

    if (role == Ds18Role::NONE) {
      return false;
    }

    SensorAssignments::readByRole(ctx.assignments, role, temperatureC, valid);
    return valid && !isnan(temperatureC);
  }

  bool pumpSourceTemp(const AppContext& ctx, const PumpConfig& p, float& temperatureC, bool& valid) {
    if (p.sourceType == PumpSourceType::SENSOR_ROLE) {
      return sensorSourceTempByRole(ctx, p.sourceSensorRole, temperatureC, valid);
    }

    if (p.sourceRole != HeatSourceRole::NONE) {
      return heatSourceTempByRole(ctx.sensors, p.sourceRole, temperatureC, valid);
    }

    temperatureC = ctx.sensors.activeHeatSource.tempC;
    valid = ctx.sensors.activeHeatSource.valid;
    return true;
  }


  bool isOvenPump(const PumpConfig& pump) {
    return pump.sourceType == PumpSourceType::HEAT_SOURCE_ROLE &&
           pump.sourceRole == HeatSourceRole::ALT_SOURCE_OVEN;
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

  int8_t heatingCircuitIndexFromPumpRoute(const PumpConfig& pump) {
    if (pump.sourceType != PumpSourceType::SENSOR_ROLE) return -1;
    for (uint8_t i = 0; i < MAX_HEATING_CIRCUITS; i++) {
      if (pump.sourceSensorRole == heatingCircuitFlowRole(i) &&
          pump.sinkRole == heatingCircuitReturnRole(i)) {
        return (int8_t)i;
      }
    }
    return -1;
  }

  bool isHeatingCircuitPump(const PumpConfig& pump) {
    return heatingCircuitIndexFromPumpRoute(pump) >= 0;
  }

  bool pumpEnableRelayReady(const AppContext& ctx, const PumpConfig& pump) {
    return pump.relayIndex != PIN_UNUSED &&
           pump.relayIndex < RELAY_COUNT &&
           RelayOutputs::isUsableAsPumpEnable(ctx, pump.relayIndex);
  }

  float computeOvenPwmPercent(const PumpConfig& pump, float ovenTemperatureC, float targetTemperatureC, bool targetValid) {
    const float minPwm = clampFloat(pump.minPwmPercent, 0.0f, 100.0f);
    const float maxPwm = clampFloat(pump.maxPwmPercent, minPwm, 100.0f);

    if (!targetValid || isnan(ovenTemperatureC) || isnan(targetTemperatureC)) {
      return maxPwm;
    }

    const float deltaC = ovenTemperatureC - targetTemperatureC;
    const float startC = pump.startDiff > 0.1f ? pump.startDiff : 3.0f;
    const float fullC = pump.targetDiff > startC ? pump.targetDiff : (startC + 5.0f);

    if (deltaC <= startC) return minPwm;
    if (deltaC >= fullC) return maxPwm;

    const float factor = (deltaC - startC) / (fullC - startC);
    return minPwm + (maxPwm - minPwm) * factor;
  }

  bool isSolarCollectorSource(HeatSourceRole role) {
    return role == HeatSourceRole::SOLAR_COLLECTOR_1 ||
           role == HeatSourceRole::SOLAR_COLLECTOR_2 ||
           role == HeatSourceRole::SOLAR_COLLECTOR_3;
  }

  bool isSolarCollectorPump(const PumpConfig& pump) {
    return pump.sourceType == PumpSourceType::HEAT_SOURCE_ROLE &&
           isSolarCollectorSource(pump.sourceRole);
  }

  void stopPumpZeroPercent(AppContext& ctx, uint8_t i, PumpConfig& pump) {
    if (!validPumpIndex(i)) return;

    pid[i] = {};
    pump.state = false;
    pump.lastPwmPercent = 0.0f;

    if (pump.relayIndex != PIN_UNUSED) {
      RelayOutputs::set(ctx, pump.relayIndex, false);
    }

    // Wichtig: Bei Ventilfahrt und Nachtkuehlung muss die Pumpe
    // wirklich AUS bleiben. Deshalb hier immer 0 %, auch wenn das
    // normale Sicherheitsprofil einer Heizungspumpe 100 % waere.
    setPwmPercent(pump, 0.0f);
  }

  bool resolveNightCoolingSink(const AppContext& ctx, const PumpConfig& pump, Ds18Role& sinkRole, bool& hasValve) {
    sinkRole = Ds18Role::NONE;
    hasValve = false;

    if (!isSolarCollectorPump(pump)) {
      return false;
    }

    if (pump.valveIndex != PIN_UNUSED && Valves::isConfigured(ctx, pump.valveIndex)) {
      hasValve = true;

      // Nachtkuehlung entlaedt bei Pumpen mit Umschaltventil bewusst Ziel B.
      const PumpRouteTargetConfig& targetB = pump.targets[1];
      if (!targetB.enabled || targetB.sinkRole == Ds18Role::NONE) {
        return false;
      }

      sinkRole = targetB.sinkRole;
      return true;
    }

    if (pump.sinkRole == Ds18Role::NONE) {
      return false;
    }

    sinkRole = pump.sinkRole;
    return true;
  }

  bool ensureNightCoolingValveTargetB(AppContext& ctx, uint8_t pumpIndex, PumpConfig& pump) {
    if (pump.valveIndex == PIN_UNUSED || !Valves::isConfigured(ctx, pump.valveIndex)) {
      return true;
    }

    const uint8_t targetIndex = 1; // Ziel B

    if (Valves::currentPosition(pump.valveIndex) == ValvePosition::B && !Valves::isMoving(pump.valveIndex)) {
      pump.activeTargetIndex = targetIndex;
      pump.valvePendingTargetIndex = PIN_UNUSED;
      return true;
    }

    if (!Valves::requestPosition(ctx, pump.valveIndex, ValvePosition::B)) {
      return false;
    }

    pump.valvePendingTargetIndex = targetIndex;

    if (Valves::isMoving(pump.valveIndex)) {
      stopPumpZeroPercent(ctx, pumpIndex, pump);

      Serial.print("NACHTKUEHLUNG PUMPE ");
      Serial.print(pumpIndex + 1);
      Serial.println(": Ventil V2 faehrt auf Ziel B - Pumpe bleibt AUS");
      Serial.flush();
      return false;
    }

    pump.activeTargetIndex = targetIndex;
    pump.valvePendingTargetIndex = PIN_UNUSED;
    return true;
  }


  void forcePumpOff(AppContext& ctx, uint8_t i) {
    if (!validPumpIndex(i)) return;

    PumpConfig& p = ctx.config.pumps[i];
    resetPumpRuntime(p);
    pid[i] = {};

    if (p.relayIndex != PIN_UNUSED) {
      RelayOutputs::set(ctx, p.relayIndex, false);
    }

    float safetyPercent = 0.0f;

    if (p.mode == PumpMode::PWM) {
      if (p.pwmProfile == PwmProfile::HEATING) {
        safetyPercent = 100.0f;
      }
      }

      setPwmPercent(p, safetyPercent);
}

  void updatePumpFeedback(PumpConfig& p) {
    if (p.mode != PumpMode::PWM || p.feedbackPin == PIN_UNUSED || !p.state || p.lastPwmPercent < 10.0f) {
      p.feedbackSignalPresent = false;
      p.feedbackError = false;
      p.feedbackDutyPercent = NAN;
      return;
    }

    const uint32_t now = millis();
    if ((uint32_t)(now - p.feedbackLastCheckedMs) < 2000UL) {
      return;
    }
    p.feedbackLastCheckedMs = now;

    // Feedback der PWM-Pumpe: ca. 75 Hz, Status ueber Duty Cycle.
    // Timeout 20 ms deckt eine Periode bei 75 Hz ab, blockiert aber nicht dauerhaft.
    const unsigned long highUs = pulseIn(p.feedbackPin, HIGH, 20000UL);
    const unsigned long lowUs  = pulseIn(p.feedbackPin, LOW,  20000UL);

    if (highUs == 0 || lowUs == 0) {
      p.feedbackSignalPresent = false;
      p.feedbackError = true;
      p.feedbackDutyPercent = NAN;
      return;
    }

    const float periodUs = (float)highUs + (float)lowUs;
    if (periodUs <= 0.0f) {
      p.feedbackSignalPresent = false;
      p.feedbackError = true;
      p.feedbackDutyPercent = NAN;
      return;
    }

    p.feedbackDutyPercent = ((float)highUs * 100.0f) / periodUs;
    p.feedbackSignalPresent = true;
    p.feedbackError = false;
  }

  void printPumpSwitch(uint8_t index, bool on, const PumpConfig& p) {
    Serial.print("PUMPE ");
    Serial.print(index + 1);
    Serial.print(" -> ");
    Serial.print(on ? "EIN" : "AUS");
    Serial.print(" | Relais=");
    Serial.print(p.relayIndex);
    Serial.print(" | Temperaturdifferenz=");
    Serial.print(p.lastDiffC);
    Serial.print(" | PWM=");
    Serial.print(p.lastPwmPercent);

    if (p.valveIndex != PIN_UNUSED) {
      Serial.print(" | Ventil V2=");
      Serial.print(p.valveIndex);
      Serial.print(" | Ziel=");
      if (p.activeTargetIndex == 0) Serial.print("A");
      else if (p.activeTargetIndex == 1) Serial.print("B");
      else Serial.print("-");
    }

    Serial.println();
    Serial.flush();
  }
}

namespace Pumps {

void begin(AppContext& ctx) {
  for (uint8_t i = 0; i < MAX_PUMPS; i++) {
    PumpConfig& p = ctx.config.pumps[i];

    if (p.pwmChannel == PIN_UNUSED) {
      p.pwmChannel = defaultPcaChannelFor(i);
    }

    if (p.feedbackPin == PIN_UNUSED) {
      p.feedbackPin = defaultFeedbackPinFor(i);
    }

    if (p.feedbackPin != PIN_UNUSED) {
      pinMode(p.feedbackPin, INPUT);
    }

    pid[i] = {};
    resetPumpRuntime(p);

    setPwmPercent(p, 0.0f);

    if (p.relayIndex != PIN_UNUSED) {
      RelayOutputs::set(ctx, p.relayIndex, false);
    }
  }

  PumpRouting::begin(ctx);
  EnergyConflicts::begin(ctx);
}

void allOff(AppContext& ctx) {
  for (uint8_t i = 0; i < MAX_PUMPS; i++) {
    forcePumpOff(ctx, i);
  }

  ctx.control.relayEnable = false;
  ctx.control.pwmPercent = 0;
}

void process(AppContext& ctx) {
  bool anyRelayOn = false;
  float maxPwm = 0.0f;
  bool firstDiff = true;

  EnergyConflicts::clear(ctx);

  for (uint8_t i = 0; i < MAX_PUMPS; i++) {
    PumpConfig& p = ctx.config.pumps[i];

    const bool wasOn = p.state;
    bool relayOn = false;
    float pwmPercent = 0.0f;

    if (!p.enabled || p.mode == PumpMode::OFF) {
      forcePumpOff(ctx, i);
      continue;
    }

    // Spezialpumpen werden zwar im Pumpenmenue konfiguriert, aber nicht durch
    // die freie Differenz-/Routinglogik geregelt. Das jeweilige Modul fordert
    // die Pumpe an, das Pumpenmodul setzt nur die zentrale Hardware-Konfig.
    if (isOvenPump(p) || isHeatingCircuitPump(p)) {
      continue;
    }

    if (p.relayIndex == PIN_UNUSED || !RelayOutputs::isUsableAsPumpEnable(ctx, p.relayIndex)) {
      forcePumpOff(ctx, i);
      continue;
    }

    float sourceC = NAN;
    bool sourceValid = false;
    pumpSourceTemp(ctx, p, sourceC, sourceValid);
    p.lastSourceC = sourceC;

    PumpRouting::RouteResult route = PumpRouting::resolve(ctx, i, sourceC, sourceValid);

    if (!route.active || !route.sinkValid || isnan(route.diffC)) {
      forcePumpOff(ctx, i);
      continue;
    }

    p.lastSinkC = route.sinkC;
    p.lastDiffC = route.diffC;
    p.lastPwmPercent = 0.0f;

    // Umschaltventil faehrt noch: Pumpe muss sicher AUS bleiben,
    // Routing-Ziel und Ventilbewegung duerfen aber NICHT geloescht werden.
    if (route.valveMoving) {
      p.state = false;
      pid[i] = {};
      RelayOutputs::set(ctx, p.relayIndex, false);
      setPwmPercent(p, 0.0f);

      Serial.print("PUMPE ");
      Serial.print(i + 1);
      Serial.print(" wartet auf Umschaltventil | Verbleibende Umschaltzeit ms=");
      Serial.println(route.valveMoveRemainingMs);
      Serial.flush();
      continue;
    }

    if (firstDiff) {
      ctx.control.diffC = p.lastDiffC;
      firstDiff = false;
    }

    const float effectiveTargetDiff = isnan(route.targetDiff) ? p.targetDiff : route.targetDiff;
    const float effectiveHysteresis = isnan(route.hysteresis) ? p.hysteresis : route.hysteresis;

    if (p.mode == PumpMode::RELAY) {
      if (!p.state && p.lastDiffC >= (effectiveTargetDiff + effectiveHysteresis)) {
        p.state = true;
      } else if (p.state && p.lastDiffC <= (effectiveTargetDiff - effectiveHysteresis)) {
        p.state = false;
      }

      relayOn = p.state;
    }

    if (p.mode == PumpMode::PWM) {
      if (p.lastDiffC < p.startDiff) {
        p.state = false;
        pid[i] = {};
        relayOn = false;
        pwmPercent = 0.0f;
      } else {
        p.state = true;
        relayOn = true;

        // Ziel: Temperaturdifferenz halten. Ist die Temperaturdifferenz groesser als Ziel, wird die Pumpenleistung erhoeht.
        float error = p.lastDiffC - effectiveTargetDiff;
        pwmPercent = computePID(i, p, error);
        pwmPercent = clampFloat(pwmPercent, p.minPwmPercent, p.maxPwmPercent);
      }
    }

    if (relayOn) {
      const OutputRef valveOutput = route.hasValve ? route.valveOutput : OutputRef{};

      if (!EnergyConflicts::canActivateRoute(
            ctx,
            i,
            p.sourceRole,
            route.sinkRole,
            p.relayIndex,
            valveOutput
          )) {
        forcePumpOff(ctx, i);
        if (wasOn) {
          printPumpSwitch(i, false, p);
        }
        continue;
      }

      EnergyConflicts::reserveRoute(
        ctx,
        i,
        p.sourceRole,
        route.sinkRole,
        p.relayIndex,
        valveOutput
      );
    }

    RelayOutputs::set(ctx, p.relayIndex, relayOn);

    if (relayOn && p.mode == PumpMode::PWM) {
      setPwmPercent(p, pwmPercent);
      p.lastPwmPercent = pwmPercent;
      updatePumpFeedback(p);
    } else {
      setPwmPercent(p, 0.0f);
      p.lastPwmPercent = 0.0f;
      updatePumpFeedback(p);
    }

    if (wasOn != relayOn) {
      printPumpSwitch(i, relayOn, p);
    }

    anyRelayOn = anyRelayOn || relayOn;
    if (p.lastPwmPercent > maxPwm) {
      maxPwm = p.lastPwmPercent;
    }
  }

  ctx.control.relayEnable = anyRelayOn;
  ctx.control.pwmPercent = (uint8_t)clampFloat(maxPwm, 0.0f, 100.0f);
}

int8_t configuredOvenPumpIndex(const AppContext& ctx) {
  int8_t found = -1;

  for (uint8_t i = 0; i < MAX_PUMPS; i++) {
    const PumpConfig& pump = ctx.config.pumps[i];
    if (!pump.enabled || pump.mode == PumpMode::OFF || !isOvenPump(pump)) continue;

    if (found >= 0) {
      // Mehrfachkonfiguration ist eine Validierungsaufgabe. Fuer die Runtime
      // verwenden wir deterministisch die erste aktive Ofenpumpe.
      continue;
    }
    found = (int8_t)i;
  }

  return found;
}

int8_t configuredHeatingCircuitPumpIndex(const AppContext& ctx, uint8_t circuitIndex) {
  if (circuitIndex >= MAX_HEATING_CIRCUITS) return -1;

  int8_t found = -1;
  for (uint8_t i = 0; i < MAX_PUMPS; i++) {
    const PumpConfig& pump = ctx.config.pumps[i];
    if (!pump.enabled || pump.mode == PumpMode::OFF) continue;
    if (heatingCircuitIndexFromPumpRoute(pump) != (int8_t)circuitIndex) continue;

    if (found >= 0) {
      // Mehrfachkonfiguration wird in der Validierung abgefangen.
      continue;
    }
    found = (int8_t)i;
  }
  return found;
}

bool applyHeatingCircuitPumpRequest(AppContext& ctx, uint8_t circuitIndex, bool run, uint8_t percent, float flowTemperatureC, float returnTemperatureC, bool returnValid) {
  const int8_t pumpIndex = configuredHeatingCircuitPumpIndex(ctx, circuitIndex);
  if (pumpIndex < 0) {
    return false;
  }

  PumpConfig& pump = ctx.config.pumps[(uint8_t)pumpIndex];
  const bool wasOn = pump.state;

  if (!pumpEnableRelayReady(ctx, pump)) {
    forcePumpOff(ctx, (uint8_t)pumpIndex);
    return false;
  }

  if (!run) {
    forcePumpOff(ctx, (uint8_t)pumpIndex);
    if (wasOn) {
      printPumpSwitch((uint8_t)pumpIndex, false, pump);
    }
    return true;
  }

  percent = (uint8_t)clampFloat(percent, pump.minPwmPercent, pump.maxPwmPercent);

  if (pump.mode == PumpMode::PWM) {
    if (!validPcaChannel(pump.pwmChannel) || !pwmOutputAvailableForPump(ctx, pump.pwmChannel)) {
      forcePumpOff(ctx, (uint8_t)pumpIndex);
      return false;
    }
  }

  pump.state = true;
  pump.lastSourceC = flowTemperatureC;
  pump.lastSinkC = returnValid ? returnTemperatureC : NAN;
  pump.lastDiffC = (returnValid && !isnan(flowTemperatureC) && !isnan(returnTemperatureC))
                    ? flowTemperatureC - returnTemperatureC
                    : NAN;
  pump.lastPwmPercent = (pump.mode == PumpMode::PWM) ? percent : 0.0f;

  RelayOutputs::set(ctx, pump.relayIndex, true);

  if (pump.mode == PumpMode::PWM) {
    setPwmPercent(pump, percent);
  }

  updatePumpFeedback(pump);

  if (!wasOn) {
    printPumpSwitch((uint8_t)pumpIndex, true, pump);
  }

  return true;
}

bool applyOvenPumpRequest(AppContext& ctx, bool run, float ovenTemperatureC, float targetTemperatureC, bool targetValid) {
  const int8_t pumpIndex = configuredOvenPumpIndex(ctx);
  if (pumpIndex < 0) {
    return false;
  }

  PumpConfig& pump = ctx.config.pumps[(uint8_t)pumpIndex];
  const bool wasOn = pump.state;

  if (!pumpEnableRelayReady(ctx, pump)) {
    forcePumpOff(ctx, (uint8_t)pumpIndex);
    return false;
  }

  if (!run) {
    forcePumpOff(ctx, (uint8_t)pumpIndex);
    if (wasOn) {
      printPumpSwitch((uint8_t)pumpIndex, false, pump);
    }
    return true;
  }

  float pwmPercent = 0.0f;
  if (pump.mode == PumpMode::PWM) {
    if (!validPcaChannel(pump.pwmChannel) || !pwmOutputAvailableForPump(ctx, pump.pwmChannel)) {
      forcePumpOff(ctx, (uint8_t)pumpIndex);
      return false;
    }
    pwmPercent = computeOvenPwmPercent(pump, ovenTemperatureC, targetTemperatureC, targetValid);
  }

  pump.state = true;
  pump.lastSourceC = ovenTemperatureC;
  pump.lastSinkC = targetValid ? targetTemperatureC : NAN;
  pump.lastDiffC = (targetValid && !isnan(ovenTemperatureC) && !isnan(targetTemperatureC))
                    ? ovenTemperatureC - targetTemperatureC
                    : NAN;
  pump.lastPwmPercent = (pump.mode == PumpMode::PWM) ? pwmPercent : 0.0f;

  RelayOutputs::set(ctx, pump.relayIndex, true);

  if (pump.mode == PumpMode::PWM) {
    setPwmPercent(pump, pwmPercent);
  }

  updatePumpFeedback(pump);

  if (!wasOn) {
    printPumpSwitch((uint8_t)pumpIndex, true, pump);
  }

  return true;
}

void safetyAllOff(AppContext& ctx) {
  Pumps::allOff(ctx);
}

void safetyAllOffExceptOven(AppContext& ctx) {
  for (uint8_t i = 0; i < MAX_PUMPS; i++) {
    PumpConfig& pump = ctx.config.pumps[i];

    if (pump.sourceType == PumpSourceType::HEAT_SOURCE_ROLE &&
        pump.sourceRole == HeatSourceRole::ALT_SOURCE_OVEN) {
      continue;
    }

    forcePumpOff(ctx, i);
  }

  ctx.control.relayEnable = false;
  ctx.control.pwmPercent = 0;
}

bool safetyForceRunForSource(AppContext& ctx, HeatSourceRole sourceRole, float pwmPercent) {
  bool anyStarted = false;

  for (uint8_t i = 0; i < MAX_PUMPS; i++) {
    PumpConfig& pump = ctx.config.pumps[i];

    if (!pump.enabled) continue;
    if (pump.mode == PumpMode::OFF) continue;
    if (pump.sourceType != PumpSourceType::HEAT_SOURCE_ROLE) continue;
    if (pump.sourceRole != sourceRole) continue;
    if (pump.relayIndex == PIN_UNUSED) continue;
    if (!RelayOutputs::isUsableAsPumpEnable(ctx, pump.relayIndex)) continue;

    float sourceTemperatureC = NAN;
    bool sourceValid = false;
    pumpSourceTemp(ctx, pump, sourceTemperatureC, sourceValid);

    PumpRouting::RouteResult route = PumpRouting::resolve(ctx, i, sourceTemperatureC, sourceValid);

    if (!route.active || !route.sinkValid) {
      forcePumpOff(ctx, i);
      continue;
    }

    pump.lastSourceC = sourceTemperatureC;
    pump.lastSinkC = route.sinkC;
    pump.lastDiffC = route.diffC;

    if (route.valveMoving) {
      RelayOutputs::set(ctx, pump.relayIndex, false);
      setPwmPercent(pump, 0.0f);
      pump.state = false;
      pump.lastPwmPercent = 0.0f;
      continue;
    }

    RelayOutputs::set(ctx, pump.relayIndex, true);

    pump.state = true;
    pump.lastPwmPercent = pwmPercent;

    if (pump.mode == PumpMode::PWM && pump.pwmChannel != PIN_UNUSED) {
      setPwmPercent(pump, pwmPercent);
    }

    Serial.print("SAFETY PUMPE ");
    Serial.print(i + 1);
    Serial.print(" ERZWUNGEN | Quelle=");
    Serial.print((int)sourceRole);
    Serial.print(" | PWM=");
    Serial.println(pwmPercent);
    Serial.flush();

    anyStarted = true;
  }

  return anyStarted;
}


bool safetyForceNightCooling(AppContext& ctx, float pwmPercent) {
  bool anyHandled = false;
  bool anyPumpOn = false;
  float maxPwm = 0.0f;

  pwmPercent = clampFloat(pwmPercent, 0.0f, 100.0f);

  for (uint8_t i = 0; i < MAX_PUMPS; i++) {
    PumpConfig& pump = ctx.config.pumps[i];

    if (!pump.enabled) continue;
    if (pump.mode == PumpMode::OFF) continue;
    if (!isSolarCollectorPump(pump)) continue;

    // Safety: Nachtkuehlung darf ausschliesslich Solarkollektor-Pumpen verwenden.
    if (pump.relayIndex == PIN_UNUSED ||
        !RelayOutputs::isUsableAsPumpEnable(ctx, pump.relayIndex)) {
      stopPumpZeroPercent(ctx, i, pump);
      continue;
    }

    Ds18Role sinkRole = Ds18Role::NONE;
    bool hasValve = false;
    if (!resolveNightCoolingSink(ctx, pump, sinkRole, hasValve)) {
      stopPumpZeroPercent(ctx, i, pump);
      continue;
    }

    float collectorC = NAN;
    bool collectorValid = false;
    pumpSourceTemp(ctx, pump, collectorC, collectorValid);

    float sinkC = NAN;
    bool sinkValid = false;
    sensorSourceTempByRole(ctx, sinkRole, sinkC, sinkValid);

    pump.lastSourceC = collectorC;
    pump.lastSinkC = sinkC;
    pump.lastPwmPercent = 0.0f;

    if (!collectorValid || !sinkValid || isnan(collectorC) || isnan(sinkC)) {
      stopPumpZeroPercent(ctx, i, pump);
      continue;
    }

    const float coolingDeltaC = sinkC - collectorC;
    pump.lastDiffC = coolingDeltaC;

    // Energieeffizient: nur kuehlen, wenn der Kollektor wirklich deutlich kaelter ist.
    if (collectorC > 25.0f ||
        coolingDeltaC < 8.0f ||
        sinkC <= ctx.config.safetyNightCoolingTargetTemperatureC) {
      stopPumpZeroPercent(ctx, i, pump);
      continue;
    }

    const OutputRef valveOutput = hasValve ? Valves::outputRef(ctx, pump.valveIndex) : OutputRef{};

    if (!EnergyConflicts::canActivateRoute(
          ctx,
          i,
          pump.sourceRole,
          sinkRole,
          pump.relayIndex,
          valveOutput
        )) {
      stopPumpZeroPercent(ctx, i, pump);
      continue;
    }

    // Bei Umschaltventil wird immer Ziel B verwendet.
    // Waehrend der Ventilfahrt bleibt die Pumpe zwangsweise AUS.
    if (hasValve && !ensureNightCoolingValveTargetB(ctx, i, pump)) {
      anyHandled = true;
      continue;
    }

    EnergyConflicts::reserveRoute(
      ctx,
      i,
      pump.sourceRole,
      sinkRole,
      pump.relayIndex,
      valveOutput
    );

    RelayOutputs::set(ctx, pump.relayIndex, true);

    pump.state = true;
    pump.lastPwmPercent = pwmPercent;

    if (pump.mode == PumpMode::PWM && pump.pwmChannel != PIN_UNUSED) {
      setPwmPercent(pump, pwmPercent);
    } else {
      setPwmPercent(pump, 0.0f);
    }

    Serial.print("SAFETY NACHTKUEHLUNG P");
    Serial.print(i + 1);
    Serial.print(" | Quelle=");
    Serial.print((int)pump.sourceRole);
    Serial.print(" | Ziel=");
    Serial.print((int)sinkRole);
    Serial.print(hasValve ? " (Ventil Ziel B)" : " (Direktziel)");
    Serial.print(" | Kollektor=");
    Serial.print(collectorC);
    Serial.print(" C | Speicher=");
    Serial.print(sinkC);
    Serial.print(" C | Delta=");
    Serial.print(coolingDeltaC);
    Serial.print(" K | PWM=");
    Serial.println(pwmPercent);
    Serial.flush();

    anyHandled = true;
    anyPumpOn = true;
    if (pwmPercent > maxPwm) {
      maxPwm = pwmPercent;
    }
  }

  ctx.control.relayEnable = anyPumpOn;
  ctx.control.pwmPercent = (uint8_t)clampFloat(maxPwm, 0.0f, 100.0f);

  return anyHandled;
}

bool safetyForceHeatDumpForSource(AppContext& ctx, HeatSourceRole sourceRole, float pwmPercent) {
  bool anyStarted = false;

  for (uint8_t i = 0; i < MAX_PUMPS; i++) {
    PumpConfig& pump = ctx.config.pumps[i];

    if (!pump.enabled) continue;
    if (pump.mode == PumpMode::OFF) continue;
    if (pump.sourceType != PumpSourceType::HEAT_SOURCE_ROLE) continue;
    if (pump.sourceRole != sourceRole) continue;
    if (pump.relayIndex == PIN_UNUSED) continue;
    if (!RelayOutputs::isUsableAsPumpEnable(ctx, pump.relayIndex)) continue;

    float sourceTemperatureC = NAN;
    bool sourceValid = false;
    pumpSourceTemp(ctx, pump, sourceTemperatureC, sourceValid);

    PumpRouting::RouteResult route = PumpRouting::resolveHeatDumpCoolestTarget(ctx, i, sourceTemperatureC, sourceValid);

    if (!route.active || !route.sinkValid) {
      forcePumpOff(ctx, i);
      continue;
    }

    pump.lastSourceC = sourceTemperatureC;
    pump.lastSinkC = route.sinkC;
    pump.lastDiffC = route.diffC;

    if (route.valveMoving) {
      RelayOutputs::set(ctx, pump.relayIndex, false);
      setPwmPercent(pump, 0.0f);
      pump.state = false;
      pump.lastPwmPercent = 0.0f;
      continue;
    }

    RelayOutputs::set(ctx, pump.relayIndex, true);
    pump.state = true;
    pump.lastPwmPercent = pwmPercent;

    if (pump.mode == PumpMode::PWM && pump.pwmChannel != PIN_UNUSED) {
      setPwmPercent(pump, pwmPercent);
    }

    Serial.print("SAFETY WAERMEABLEITUNG P");
    Serial.print(i + 1);
    Serial.print(" | Zieltemperatur=");
    Serial.print(route.sinkC);
    Serial.print(" | Temperaturdifferenz=");
    Serial.print(route.diffC);
    Serial.print(" | PWM=");
    Serial.println(pwmPercent);
    Serial.flush();

    anyStarted = true;
  }

  return anyStarted;
}



}
