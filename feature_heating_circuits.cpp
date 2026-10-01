#include "feature_heating_circuits.h"

#include "feature_sensor_assignments.h"
#include "feature_relay_outputs.h"
#include "feature_pwm_pca9685.h"
#include "feature_alarms.h"
#include "feature_pumps.h"
#include "feature_ml_optimizer.h"
#include "feature_time.h"

#include <Arduino.h>
#include <math.h>

namespace {

bool validOutput(const OutputRef& ref) {
  if (ref.kind == OutputKind::RELAY) return ref.index < RELAY_COUNT;
  if (ref.kind == OutputKind::PWM_OUTPUT) return ref.index < PWM_OUTPUT_COUNT;
  return false;
}

void setOutput(AppContext& ctx, const OutputRef& ref, bool on) {
  if (!validOutput(ref)) return;
  if (ref.kind == OutputKind::RELAY) {
    RelayOutputs::set(ctx, ref.index, on);
    return;
  }
  const PwmOutputConfig& po = ctx.config.pwmOutputs[ref.index];
  if (po.mode == PwmOutputMode::SWITCH) {
    PwmDriver::setSwitch(ref.index, on, po.profile);
  }
}

void setPump(AppContext& ctx, const HeatingCircuitConfig& cfg, bool on, uint8_t percent) {
  if (cfg.pumpMode == HeatingCircuitPumpMode::NONE) return;
  if (!validOutput(cfg.pumpOutput)) return;

  if (cfg.pumpOutput.kind == OutputKind::RELAY) {
    RelayOutputs::set(ctx, cfg.pumpOutput.index, on);
    return;
  }

  const PwmOutputConfig& po = ctx.config.pwmOutputs[cfg.pumpOutput.index];
  if (cfg.pumpMode == HeatingCircuitPumpMode::PWM && po.mode == PwmOutputMode::PWM) {
    PwmDriver::setDuty(cfg.pumpOutput.index, on ? percent : 0, po.profile);
  } else if (po.mode == PwmOutputMode::SWITCH) {
    PwmDriver::setSwitch(cfg.pumpOutput.index, on, po.profile);
  }
}

bool readDs18(AppContext& ctx, Ds18Role role, float& value, bool& valid) {
  value = NAN;
  valid = false;
  if (role == Ds18Role::NONE) return false;
  return SensorAssignments::readByRole(ctx.assignments, role, value, valid) && valid && !isnan(value);
}

float clampFloat(float value, float minValue, float maxValue) {
  if (value < minValue) return minValue;
  if (value > maxValue) return maxValue;
  return value;
}

Ds18Role fallbackOutsideRole(const HeatingCircuitConfig& cfg) {
  return cfg.outsideSensorRole == Ds18Role::NONE ? Ds18Role::OUTSIDE_TEMPERATURE : cfg.outsideSensorRole;
}

Ds18Role fallbackRoomRole(uint8_t circuitIndex, const HeatingCircuitConfig& cfg) {
  if (cfg.roomSensorRole != Ds18Role::NONE) return cfg.roomSensorRole;
  switch (circuitIndex) {
    case 0: return Ds18Role::ROOM_1;
    case 1: return Ds18Role::ROOM_2;
    case 2: return Ds18Role::ROOM_3;
    case 3: return Ds18Role::ROOM_4;
    default: return Ds18Role::NONE;
  }
}

bool nightSetbackIsActive(const HeatingCircuitConfig& cfg) {
  if (!cfg.nightSetbackEnabled) return false;
  uint16_t nowMinute = 0;
  if (!TimeService::localMinutesOfDay(nowMinute)) return false;

  const uint16_t start = cfg.nightSetbackStartMinute > 1439 ? 1439 : cfg.nightSetbackStartMinute;
  const uint16_t end = cfg.nightSetbackEndMinute > 1439 ? 1439 : cfg.nightSetbackEndMinute;
  if (start == end) return true;
  if (start < end) return nowMinute >= start && nowMinute < end;
  return nowMinute >= start || nowMinute < end;
}

float effectiveRoomTarget(const HeatingCircuitConfig& cfg, bool nightActive) {
  // Bei deaktivierter Nachtabsenkung bleibt die bestehende Regelung exakt erhalten.
  if (!nightActive) return cfg.roomTargetTemperatureC;
  const float target = cfg.roomTargetTemperatureC - clampFloat(cfg.nightSetbackK, 0.0f, 10.0f);
  return clampFloat(target, 16.0f, 30.0f);
}

float targetFlowTemperature(AppContext& ctx, uint8_t circuitIndex, const HeatingCircuitConfig& cfg, HeatingCircuitRuntime& rt) {
  float target = cfg.fixedFlowTemperatureC;

  rt.outsideTemperatureC = NAN;
  rt.roomTemperatureC = NAN;
  rt.nightSetbackActive = nightSetbackIsActive(cfg);
  rt.effectiveRoomTargetTemperatureC = effectiveRoomTarget(cfg, rt.nightSetbackActive);

  if (cfg.controlMode == HeatingCircuitControlMode::WEATHER_COMPENSATED) {
    float outsideC = NAN;
    bool outsideValid = false;
    if (readDs18(ctx, fallbackOutsideRole(cfg), outsideC, outsideValid)) {
      rt.outsideTemperatureC = outsideC;
      target = cfg.heatingCurveBaseC + cfg.heatingCurveSlope * (20.0f - outsideC);
    }
  }

  if (cfg.roomControlEnabled) {
    float roomC = NAN;
    bool roomValid = false;
    if (readDs18(ctx, fallbackRoomRole(circuitIndex, cfg), roomC, roomValid)) {
      rt.roomTemperatureC = roomC;
      target += (rt.effectiveRoomTargetTemperatureC - roomC) * cfg.roomInfluenceK;
    }
  }

  target = clampFloat(target, cfg.minimumFlowTemperatureC, cfg.maximumFlowTemperatureC);
  return target;
}

void stopMixer(AppContext& ctx, const HeatingCircuitConfig& cfg, HeatingCircuitRuntime& rt) {
  setOutput(ctx, cfg.mixerOpenOutput, false);
  setOutput(ctx, cfg.mixerCloseOutput, false);
  rt.opening = false;
  rt.closing = false;
}

void pulseMixer(AppContext& ctx, const HeatingCircuitConfig& cfg, HeatingCircuitRuntime& rt, int direction) {
  const uint32_t now = millis();
  if ((uint32_t)(now - rt.lastMixerActionMs) < cfg.mixerPauseMs) return;

  stopMixer(ctx, cfg, rt);

  if (direction > 0) {
    setOutput(ctx, cfg.mixerOpenOutput, true);
    rt.opening = true;
    rt.estimatedMixerPositionPercent += 2;
  } else if (direction < 0) {
    setOutput(ctx, cfg.mixerCloseOutput, true);
    rt.closing = true;
    rt.estimatedMixerPositionPercent -= 2;
  }

  if (rt.estimatedMixerPositionPercent < 0) rt.estimatedMixerPositionPercent = 0;
  if (rt.estimatedMixerPositionPercent > 100) rt.estimatedMixerPositionPercent = 100;

  rt.lastMixerActionMs = now;
}

bool sameOutput(const OutputRef& a, const OutputRef& b) {
  return a.kind == b.kind && a.index == b.index;
}

bool usesOutput(const HeatingCircuitConfig& cfg, const OutputRef& ref) {
  if (!validOutput(ref)) return false;
  if (sameOutput(cfg.mixerOpenOutput, ref)) return true;
  if (sameOutput(cfg.mixerCloseOutput, ref)) return true;
  if (cfg.pumpMode != HeatingCircuitPumpMode::NONE && sameOutput(cfg.pumpOutput, ref)) return true;
  return false;
}

void releaseSingleOutput(AppContext& ctx, const HeatingCircuitConfig& ownerCfg, const OutputRef& ref, bool asPump) {
  if (!validOutput(ref)) return;
  if (asPump) {
    HeatingCircuitConfig tmp = ownerCfg;
    tmp.pumpOutput = ref;
    setPump(ctx, tmp, false, 0);
  } else {
    setOutput(ctx, ref, false);
  }
}

String flowAlarmId(uint8_t index) {
  return "heating_circuit_" + String(index + 1) + "_flow_sensor_invalid";
}

String flowAlarmMessage(uint8_t index, Ds18Role role) {
  String msg = "Heizkreis ";
  msg += String(index + 1);
  msg += ": Vorlauffuehler fehlt oder ist ungueltig";
  if (role == Ds18Role::NONE) msg += " (keine Rolle konfiguriert)";
  return msg;
}

} // namespace

namespace HeatingCircuits {

void begin(AppContext& ctx) {
  for (uint8_t i = 0; i < MAX_HEATING_CIRCUITS; i++) {
    ctx.heatingCircuitRuntime[i] = HeatingCircuitRuntime{};
  }
}

Ds18Role defaultFlowSensorRole(uint8_t circuitIndex) {
  switch (circuitIndex) {
    case 0: return Ds18Role::HK1_FLOW;
    case 1: return Ds18Role::HK2_FLOW;
    case 2: return Ds18Role::HK3_FLOW;
    case 3: return Ds18Role::HK4_FLOW;
    default: return Ds18Role::NONE;
  }
}

Ds18Role defaultReturnSensorRole(uint8_t circuitIndex) {
  switch (circuitIndex) {
    case 0: return Ds18Role::HK1_RETURN;
    case 1: return Ds18Role::HK2_RETURN;
    case 2: return Ds18Role::HK3_RETURN;
    case 3: return Ds18Role::HK4_RETURN;
    default: return Ds18Role::NONE;
  }
}

Ds18Role defaultRoomSensorRole(uint8_t circuitIndex) {
  switch (circuitIndex) {
    case 0: return Ds18Role::ROOM_1;
    case 1: return Ds18Role::ROOM_2;
    case 2: return Ds18Role::ROOM_3;
    case 3: return Ds18Role::ROOM_4;
    default: return Ds18Role::NONE;
  }
}

Ds18Role effectiveFlowSensorRole(uint8_t circuitIndex, const HeatingCircuitConfig& cfg) {
  return cfg.flowSensorRole == Ds18Role::NONE ? defaultFlowSensorRole(circuitIndex) : cfg.flowSensorRole;
}

Ds18Role effectiveReturnSensorRole(uint8_t circuitIndex, const HeatingCircuitConfig& cfg) {
  return cfg.returnSensorRole == Ds18Role::NONE ? defaultReturnSensorRole(circuitIndex) : cfg.returnSensorRole;
}

Ds18Role effectiveRoomSensorRole(uint8_t circuitIndex, const HeatingCircuitConfig& cfg) {
  return cfg.roomSensorRole == Ds18Role::NONE ? defaultRoomSensorRole(circuitIndex) : cfg.roomSensorRole;
}

Ds18Role effectiveOutsideSensorRole(const HeatingCircuitConfig& cfg) {
  return cfg.outsideSensorRole == Ds18Role::NONE ? Ds18Role::OUTSIDE_TEMPERATURE : cfg.outsideSensorRole;
}

void releaseOutputs(AppContext& ctx, const HeatingCircuitConfig& cfg) {
  setOutput(ctx, cfg.mixerOpenOutput, false);
  setOutput(ctx, cfg.mixerCloseOutput, false);
  setPump(ctx, cfg, false, 0);
}

void releaseRemovedOutputs(AppContext& ctx, const HeatingCircuitConfig& oldCfg, const HeatingCircuitConfig& newCfg) {
  if (validOutput(oldCfg.mixerOpenOutput) && !usesOutput(newCfg, oldCfg.mixerOpenOutput)) {
    releaseSingleOutput(ctx, oldCfg, oldCfg.mixerOpenOutput, false);
  }
  if (validOutput(oldCfg.mixerCloseOutput) && !usesOutput(newCfg, oldCfg.mixerCloseOutput)) {
    releaseSingleOutput(ctx, oldCfg, oldCfg.mixerCloseOutput, false);
  }
  if (oldCfg.pumpMode != HeatingCircuitPumpMode::NONE &&
      validOutput(oldCfg.pumpOutput) &&
      !usesOutput(newCfg, oldCfg.pumpOutput)) {
    releaseSingleOutput(ctx, oldCfg, oldCfg.pumpOutput, true);
  }
}

void allOff(AppContext& ctx) {
  for (uint8_t i = 0; i < MAX_HEATING_CIRCUITS; i++) {
    HeatingCircuitConfig& cfg = ctx.config.heatingCircuits[i];
    HeatingCircuitRuntime& rt = ctx.heatingCircuitRuntime[i];
    stopMixer(ctx, cfg, rt);
    Pumps::applyHeatingCircuitPumpRequest(ctx, i, false, 0, NAN, NAN, false);
    // Legacy-Freigabe: falls in alten Configs noch ein Heizkreis-Pumpenausgang
    // eingetragen ist, wird er bei allOff weiterhin sicher ausgeschaltet.
    setPump(ctx, cfg, false, 0);
    rt.pumpActive = false;
  }
}


uint8_t safetyHeatDump(AppContext& ctx, float criticalStorageTemperatureC, float minimumDeltaC) {
  uint8_t running = 0;
  minimumDeltaC = minimumDeltaC < 1.0f ? 1.0f : minimumDeltaC;

  for (uint8_t i = 0; i < MAX_HEATING_CIRCUITS; i++) {
    HeatingCircuitConfig& cfg = ctx.config.heatingCircuits[i];
    HeatingCircuitRuntime& rt = ctx.heatingCircuitRuntime[i];

    if (!cfg.enabled || cfg.bufferReferenceRole == Ds18Role::NONE) {
      stopMixer(ctx, cfg, rt);
      Pumps::applyHeatingCircuitPumpRequest(ctx, i, false, 0, NAN, NAN, false);
      continue;
    }

    float storageC = NAN; bool storageValid = false;
    float flowC = NAN; bool flowValid = false;
    float returnC = NAN; bool returnValid = false;

    readDs18(ctx, cfg.bufferReferenceRole, storageC, storageValid);
    readDs18(ctx, effectiveFlowSensorRole(i, cfg), flowC, flowValid);
    readDs18(ctx, effectiveReturnSensorRole(i, cfg), returnC, returnValid);

    if (!storageValid || !flowValid || !returnValid ||
        storageC < criticalStorageTemperatureC ||
        (storageC - returnC) < minimumDeltaC ||
        Pumps::configuredHeatingCircuitPumpIndex(ctx, i) < 0) {
      stopMixer(ctx, cfg, rt);
      Pumps::applyHeatingCircuitPumpRequest(ctx, i, false, 0, flowC, returnC, returnValid);
      continue;
    }

    rt.active = true;
    rt.flowTemperatureC = flowC;
    rt.returnTemperatureC = returnC;
    rt.spreadTemperatureC = flowC - returnC;
    rt.targetFlowTemperatureC = cfg.maximumFlowTemperatureC;

    // Hard safety boundary: never deliberately circulate an over-temperature
    // floor/radiator supply. Close the mixer first; pump remains off until the
    // measured flow is back at or below the configured maximum.
    if (flowC > cfg.maximumFlowTemperatureC) {
      pulseMixer(ctx, cfg, rt, -1);
      Pumps::applyHeatingCircuitPumpRequest(ctx, i, false, 0, flowC, returnC, true);
      rt.pumpActive = false;
      rt.pumpPercent = 0;
      continue;
    }

    const float marginC = 1.5f;
    if (flowC < cfg.maximumFlowTemperatureC - marginC) {
      pulseMixer(ctx, cfg, rt, +1);
    } else {
      stopMixer(ctx, cfg, rt);
    }

    const int8_t pumpIndex = Pumps::configuredHeatingCircuitPumpIndex(ctx, i);
    const PumpConfig& pumpCfg = ctx.config.pumps[(uint8_t)pumpIndex];
    const uint8_t percent = (uint8_t)roundf(clampFloat(pumpCfg.maxPwmPercent, pumpCfg.minPwmPercent, 100.0f));
    const bool applied = Pumps::applyHeatingCircuitPumpRequest(ctx, i, true, percent, flowC, returnC, true);
    rt.pumpActive = applied;
    rt.pumpPercent = applied ? percent : 0;
    if (applied) {
      running++;
      Serial.print("SAFETY HK-WAERMEABLEITUNG HK"); Serial.print(i + 1);
      Serial.print(" | Speicher="); Serial.print(storageC);
      Serial.print(" C | VL="); Serial.print(flowC);
      Serial.print(" C | RL="); Serial.print(returnC);
      Serial.print(" C | MaxVL="); Serial.println(cfg.maximumFlowTemperatureC);
      Serial.flush();
    }
  }

  return running;
}

void process(AppContext& ctx) {
  for (uint8_t i = 0; i < MAX_HEATING_CIRCUITS; i++) {
    HeatingCircuitConfig& cfg = ctx.config.heatingCircuits[i];
    HeatingCircuitRuntime& rt = ctx.heatingCircuitRuntime[i];

    if (!cfg.enabled) {
      stopMixer(ctx, cfg, rt);
      Pumps::applyHeatingCircuitPumpRequest(ctx, i, false, 0, NAN, NAN, false);
      setPump(ctx, cfg, false, 0);
      rt.active = false;
      rt.pumpActive = false;
      rt.flowTemperatureC = NAN;
      rt.returnTemperatureC = NAN;
      rt.roomTemperatureC = NAN;
      rt.outsideTemperatureC = NAN;
      rt.targetFlowTemperatureC = NAN;
      rt.effectiveRoomTargetTemperatureC = NAN;
      rt.nightSetbackActive = false;
      rt.spreadTemperatureC = NAN;
      rt.pumpPercent = 0;
      Alarms::clear(flowAlarmId(i).c_str());
      continue;
    }

    const Ds18Role flowRole = effectiveFlowSensorRole(i, cfg);
    const Ds18Role returnRole = effectiveReturnSensorRole(i, cfg);

    float flowC = NAN;
    bool flowValid = false;
    if (!readDs18(ctx, flowRole, flowC, flowValid)) {
      // Automatikbetrieb ist ohne gueltigen Vorlauffuehler nicht sicher.
      // Der Inbetriebnahme-Testmodus bleibt davon unberuehrt, weil die normale
      // Regelung in app_fsm.cpp waehrend des Testmodus pausiert wird.
      stopMixer(ctx, cfg, rt);
      Pumps::applyHeatingCircuitPumpRequest(ctx, i, false, 0, NAN, NAN, false);
      setPump(ctx, cfg, false, 0);
      rt.active = false;
      rt.pumpActive = false;
      rt.flowTemperatureC = NAN;
      rt.returnTemperatureC = NAN;
      rt.roomTemperatureC = NAN;
      rt.outsideTemperatureC = NAN;
      rt.targetFlowTemperatureC = NAN;
      rt.effectiveRoomTargetTemperatureC = NAN;
      rt.nightSetbackActive = false;
      rt.spreadTemperatureC = NAN;
      rt.pumpPercent = 0;
      const String msg = flowAlarmMessage(i, flowRole);
      Alarms::raise(flowAlarmId(i).c_str(), Alarms::Severity::CRITICAL, msg.c_str(), false);
      continue;
    }

    Alarms::clear(flowAlarmId(i).c_str());
    rt.flowTemperatureC = flowC;

    float returnC = NAN;
    bool returnValid = false;
    if (readDs18(ctx, returnRole, returnC, returnValid)) {
      rt.returnTemperatureC = returnC;
      rt.spreadTemperatureC = flowC - returnC;
    } else {
      rt.returnTemperatureC = NAN;
      rt.spreadTemperatureC = NAN;
    }

    float targetC = targetFlowTemperature(ctx, i, cfg, rt);
    // ML darf den errechneten Vorlauf-Sollwert nur als Overlay und nur innerhalb
    // der bereits konfigurierten Min-/Max-Grenzen verschieben.
    targetC = MlOptimizer::adjustHeatingTarget(ctx, i, targetC, rt.roomTemperatureC, rt.outsideTemperatureC);

    // Frostschutz ueber Aussentemperatur oder Raum/Vorlauf.
    bool frostActive = false;
    if (cfg.frostProtectionEnabled) {
      float outsideC = NAN;
      bool outsideValid = false;
      if (readDs18(ctx, fallbackOutsideRole(cfg), outsideC, outsideValid) && outsideC <= cfg.frostStartTemperatureC) {
        frostActive = true;
      }
      if (flowC <= cfg.frostStartTemperatureC) frostActive = true;
    }

    if (frostActive && targetC < cfg.frostTargetFlowTemperatureC) {
      targetC = cfg.frostTargetFlowTemperatureC;
    }

    rt.targetFlowTemperatureC = targetC;
    rt.active = true;

    const float error = targetC - flowC;
    const float deadband = (cfg.mixerType == HeatingCircuitMixerType::THERMAL) ? 1.5f : 0.7f;

    if (error > deadband) {
      pulseMixer(ctx, cfg, rt, +1);
    } else if (error < -deadband) {
      pulseMixer(ctx, cfg, rt, -1);
    } else {
      stopMixer(ctx, cfg, rt);
    }

    const bool pumpConfigured = Pumps::configuredHeatingCircuitPumpIndex(ctx, i) >= 0;
    bool pumpOn = pumpConfigured && (frostActive || targetC > cfg.minimumFlowTemperatureC);
    uint8_t pumpPercent = 0;

    if (pumpOn) {
      const int8_t pumpIndex = Pumps::configuredHeatingCircuitPumpIndex(ctx, i);
      const PumpConfig& pumpCfg = ctx.config.pumps[(uint8_t)pumpIndex];
      const float minPwm = clampFloat(pumpCfg.minPwmPercent, 0.0f, 100.0f);
      const float maxPwm = clampFloat(pumpCfg.maxPwmPercent, minPwm, 100.0f);
      float computedPercent = minPwm;

      if (pumpCfg.mode == PumpMode::PWM && !isnan(rt.spreadTemperatureC)) {
        const float targetDelta = cfg.pumpTargetDeltaC > 0.1f ? cfg.pumpTargetDeltaC : 5.0f;
        const float fullDelta = cfg.pumpFullDeltaC > targetDelta ? cfg.pumpFullDeltaC : (targetDelta + 5.0f);

        if (rt.spreadTemperatureC <= targetDelta) {
          computedPercent = minPwm;
        } else if (rt.spreadTemperatureC >= fullDelta) {
          computedPercent = maxPwm;
        } else {
          const float factor = (rt.spreadTemperatureC - targetDelta) / (fullDelta - targetDelta);
          computedPercent = minPwm + (maxPwm - minPwm) * clampFloat(factor, 0.0f, 1.0f);
        }
      }

      pumpPercent = (uint8_t)roundf(clampFloat(computedPercent, 0.0f, 100.0f));
    }

    const bool pumpApplied = Pumps::applyHeatingCircuitPumpRequest(ctx, i, pumpOn, pumpPercent, flowC, returnC, returnValid);
    rt.pumpActive = pumpOn && pumpApplied;
    rt.pumpPercent = (pumpOn && pumpApplied) ? pumpPercent : 0;
  }
}

} // namespace HeatingCircuits
