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

#include "feature_build_flags.h"
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

bool readDs18(AppContext& ctx, Ds18Role role, float& value, bool& valid) {
  value = NAN;
  valid = false;
  if (role == Ds18Role::NONE) return false;
  return SensorAssignments::readByRole(ctx.ds18b20, ctx.assignments, role, value, valid) && valid && !isnan(value);
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

enum class MixerFsmPhase : uint8_t {
  IDLE = 0,
  PULSE_OPEN,
  PULSE_CLOSE,
  WAIT
};

struct MixerFsmState {
  MixerFsmPhase phase = MixerFsmPhase::IDLE;
  int8_t normalDirection = 0;
  int8_t safetyDirection = 0;
  uint32_t phaseStartedMs = 0;
};

MixerFsmState mixerFsm[MAX_HEATING_CIRCUITS];

uint32_t effectiveMixerPulseMs(const HeatingCircuitConfig& cfg) {
  // Schutz gegen versehentlich 0 ms bzw. extremes Kontaktflattern.
  return cfg.mixerPulseMs < 100UL ? 100UL : cfg.mixerPulseMs;
}

uint32_t effectiveMixerPauseMs(const HeatingCircuitConfig& cfg) {
  return cfg.mixerPauseMs < 100UL ? 100UL : cfg.mixerPauseMs;
}

int16_t mixerPositionStepPercent(const HeatingCircuitConfig& cfg) {
  const uint32_t pulseMs = effectiveMixerPulseMs(cfg);
  if (cfg.mixerFullTravelMs < 1000UL) return 1;

  const float step = 100.0f * ((float)pulseMs / (float)cfg.mixerFullTravelMs);
  int16_t rounded = (int16_t)lroundf(step);
  if (rounded < 1) rounded = 1;
  if (rounded > 25) rounded = 25;
  return rounded;
}

void stopMixerHardware(AppContext& ctx, const HeatingCircuitConfig& cfg, HeatingCircuitRuntime& rt) {
  setOutput(ctx, cfg.mixerOpenOutput, false);
  setOutput(ctx, cfg.mixerCloseOutput, false);
  rt.opening = false;
  rt.closing = false;
}

void resetMixerFsmInternal(
  AppContext& ctx,
  uint8_t circuitIndex,
  const HeatingCircuitConfig& cfg,
  HeatingCircuitRuntime& rt
) {
  if (circuitIndex >= MAX_HEATING_CIRCUITS) return;

  stopMixerHardware(ctx, cfg, rt);

  MixerFsmState& fsm = mixerFsm[circuitIndex];
  fsm.phase = MixerFsmPhase::IDLE;
  fsm.normalDirection = 0;
  fsm.safetyDirection = 0;
  fsm.phaseStartedMs = millis();

  rt.lastMixerActionMs = fsm.phaseStartedMs;
}

void requestMixerDirection(uint8_t circuitIndex, int direction, bool safetyRequest) {
  if (circuitIndex >= MAX_HEATING_CIRCUITS) return;

  int8_t requested = 0;
  if (direction > 0) requested = 1;
  else if (direction < 0) requested = -1;

  MixerFsmState& fsm = mixerFsm[circuitIndex];
  if (safetyRequest) {
    fsm.safetyDirection = requested;
  } else {
    fsm.normalDirection = requested;
  }
}

void startMixerPulse(
  AppContext& ctx,
  uint8_t circuitIndex,
  const HeatingCircuitConfig& cfg,
  HeatingCircuitRuntime& rt,
  int8_t direction,
  uint32_t now
) {
  if (circuitIndex >= MAX_HEATING_CIRCUITS || direction == 0) return;

  stopMixerHardware(ctx, cfg, rt);

  MixerFsmState& fsm = mixerFsm[circuitIndex];

  if (direction > 0) {
    setOutput(ctx, cfg.mixerOpenOutput, true);
    rt.opening = true;
    fsm.phase = MixerFsmPhase::PULSE_OPEN;
  } else {
    setOutput(ctx, cfg.mixerCloseOutput, true);
    rt.closing = true;
    fsm.phase = MixerFsmPhase::PULSE_CLOSE;
  }

  fsm.phaseStartedMs = now;
  rt.lastMixerActionMs = now;
}

void finishMixerPulse(
  AppContext& ctx,
  uint8_t circuitIndex,
  const HeatingCircuitConfig& cfg,
  HeatingCircuitRuntime& rt,
  int8_t completedDirection,
  uint32_t now
) {
  stopMixerHardware(ctx, cfg, rt);

  const int16_t step = mixerPositionStepPercent(cfg);
  if (completedDirection > 0) {
    rt.estimatedMixerPositionPercent += step;
  } else if (completedDirection < 0) {
    rt.estimatedMixerPositionPercent -= step;
  }

  if (rt.estimatedMixerPositionPercent < 0) rt.estimatedMixerPositionPercent = 0;
  if (rt.estimatedMixerPositionPercent > 100) rt.estimatedMixerPositionPercent = 100;

  MixerFsmState& fsm = mixerFsm[circuitIndex];
  fsm.phase = MixerFsmPhase::WAIT;
  fsm.phaseStartedMs = now;
  rt.lastMixerActionMs = now;
}

void processMixerFsm(
  AppContext& ctx,
  uint8_t circuitIndex,
  const HeatingCircuitConfig& cfg,
  HeatingCircuitRuntime& rt,
  bool safetyOverride
) {
  if (circuitIndex >= MAX_HEATING_CIRCUITS) return;

  MixerFsmState& fsm = mixerFsm[circuitIndex];

  // Sobald Safety aktiv ist, wird der alte Normalauftrag verworfen. Nach Ende
  // des Safety-Eingriffs wartet der Mischer dadurch auf den naechsten regulaeren
  // Sensorsnapshot und laeuft nicht mit einer alten Richtung weiter.
  if (safetyOverride) {
    fsm.normalDirection = 0;
  } else {
    fsm.safetyDirection = 0;
  }

  const int8_t requested =
    safetyOverride ? fsm.safetyDirection : fsm.normalDirection;

  const uint32_t now = millis();
  const uint32_t pulseMs = effectiveMixerPulseMs(cfg);
  const uint32_t pauseMs = effectiveMixerPauseMs(cfg);

  if (requested == 0) {
    if (fsm.phase != MixerFsmPhase::IDLE || rt.opening || rt.closing) {
      stopMixerHardware(ctx, cfg, rt);
      fsm.phase = MixerFsmPhase::IDLE;
      fsm.phaseStartedMs = now;
      rt.lastMixerActionMs = now;
    }
    return;
  }

  switch (fsm.phase) {
    case MixerFsmPhase::IDLE:
      startMixerPulse(ctx, circuitIndex, cfg, rt, requested, now);
      return;

    case MixerFsmPhase::PULSE_OPEN:
    case MixerFsmPhase::PULSE_CLOSE: {
      const int8_t activeDirection =
        fsm.phase == MixerFsmPhase::PULSE_OPEN ? 1 : -1;

      // Safety darf eine laufende Normalbewegung sofort in die sichere Richtung
      // umkehren. Im Normalbetrieb wird bei Richtungswechsel zuerst sauber
      // gestoppt und die konfiguriere Pause eingehalten.
      if (requested != activeDirection) {
        stopMixerHardware(ctx, cfg, rt);

        if (safetyOverride) {
          fsm.phase = MixerFsmPhase::IDLE;
          fsm.phaseStartedMs = now;
          startMixerPulse(ctx, circuitIndex, cfg, rt, requested, now);
        } else {
          fsm.phase = MixerFsmPhase::WAIT;
          fsm.phaseStartedMs = now;
          rt.lastMixerActionMs = now;
        }
        return;
      }

      if ((uint32_t)(now - fsm.phaseStartedMs) >= pulseMs) {
        finishMixerPulse(
          ctx,
          circuitIndex,
          cfg,
          rt,
          activeDirection,
          now
        );
      }
      return;
    }

    case MixerFsmPhase::WAIT:
      stopMixerHardware(ctx, cfg, rt);

      if ((uint32_t)(now - fsm.phaseStartedMs) >= pauseMs) {
        fsm.phase = MixerFsmPhase::IDLE;
        fsm.phaseStartedMs = now;
        startMixerPulse(ctx, circuitIndex, cfg, rt, requested, now);
      }
      return;
  }
}

bool sameOutput(const OutputRef& a, const OutputRef& b) {
  return a.kind == b.kind && a.index == b.index;
}

bool usesOutput(const HeatingCircuitConfig& cfg, const OutputRef& ref) {
  if (!validOutput(ref)) return false;
  return sameOutput(cfg.mixerOpenOutput, ref) || sameOutput(cfg.mixerCloseOutput, ref);
}

void releaseSingleOutput(AppContext& ctx, const OutputRef& ref) {
  if (!validOutput(ref)) return;
  setOutput(ctx, ref, false);
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
    mixerFsm[i] = MixerFsmState{};
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
}

void releaseRemovedOutputs(AppContext& ctx, const HeatingCircuitConfig& oldCfg, const HeatingCircuitConfig& newCfg) {
  if (validOutput(oldCfg.mixerOpenOutput) && !usesOutput(newCfg, oldCfg.mixerOpenOutput)) {
    releaseSingleOutput(ctx, oldCfg.mixerOpenOutput);
  }
  if (validOutput(oldCfg.mixerCloseOutput) && !usesOutput(newCfg, oldCfg.mixerCloseOutput)) {
    releaseSingleOutput(ctx, oldCfg.mixerCloseOutput);
  }
}

void allOff(AppContext& ctx) {
  for (uint8_t i = 0; i < MAX_HEATING_CIRCUITS; i++) {
    HeatingCircuitConfig& cfg = ctx.config.heatingCircuits[i];
    HeatingCircuitRuntime& rt = ctx.heatingCircuitRuntime[i];
    resetMixerFsmInternal(ctx, i, cfg, rt);
    Pumps::applyHeatingCircuitPumpRequest(ctx, i, false, 0, NAN, NAN, false);
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
      resetMixerFsmInternal(ctx, i, cfg, rt);
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
      resetMixerFsmInternal(ctx, i, cfg, rt);
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
      requestMixerDirection(i, -1, true);
      Pumps::applyHeatingCircuitPumpRequest(ctx, i, false, 0, flowC, returnC, true);
      rt.pumpActive = false;
      rt.pumpPercent = 0;
      continue;
    }

    const float marginC = 1.5f;
    if (flowC < cfg.maximumFlowTemperatureC - marginC) {
      requestMixerDirection(i, +1, true);
    } else {
      resetMixerFsmInternal(ctx, i, cfg, rt);
    }

    const int8_t pumpIndex = Pumps::configuredHeatingCircuitPumpIndex(ctx, i);
    const PumpConfig& pumpCfg = ctx.config.pumps[(uint8_t)pumpIndex];
    const uint8_t percent = (uint8_t)roundf(clampFloat(pumpCfg.maxPwmPercent, pumpCfg.minPwmPercent, 100.0f));
    const bool applied = Pumps::applyHeatingCircuitPumpRequest(ctx, i, true, percent, flowC, returnC, true);
    rt.pumpActive = applied;
    rt.pumpPercent = applied ? percent : 0;
    if (applied) {
      running++;
      DBG_PRINT("SAFETY HK-WAERMEABLEITUNG HK"); DBG_PRINT(i + 1);
      DBG_PRINT(" | Speicher="); DBG_PRINT(storageC);
      DBG_PRINT(" C | VL="); DBG_PRINT(flowC);
      DBG_PRINT(" C | RL="); DBG_PRINT(returnC);
      DBG_PRINT(" C | MaxVL="); DBG_PRINTLN(cfg.maximumFlowTemperatureC);
      DBG_FLUSH();
    }
  }

  return running;
}

void prepareSafetyCycle(AppContext& ctx) {
  (void)ctx;
  for (uint8_t i = 0; i < MAX_HEATING_CIRCUITS; i++) {
    // Jede Safety-Auswertung beginnt mit leerem Auftrag. Nur ein im aktuellen
    // SafetyManager::applyOutputs()-Durchlauf explizit gesetzter Auftrag darf
    // danach den Mischer bewegen.
    mixerFsm[i].safetyDirection = 0;
  }
}

void processFast(AppContext& ctx, bool safetyOverride) {
  for (uint8_t i = 0; i < MAX_HEATING_CIRCUITS; i++) {
    HeatingCircuitConfig& cfg = ctx.config.heatingCircuits[i];
    HeatingCircuitRuntime& rt = ctx.heatingCircuitRuntime[i];

    if (!cfg.enabled) {
      resetMixerFsmInternal(ctx, i, cfg, rt);
      continue;
    }

    processMixerFsm(ctx, i, cfg, rt, safetyOverride);
  }
}

void resetMixerFsm(AppContext& ctx, uint8_t circuitIndex) {
  if (circuitIndex >= MAX_HEATING_CIRCUITS) return;
  HeatingCircuitConfig& cfg = ctx.config.heatingCircuits[circuitIndex];
  HeatingCircuitRuntime& rt = ctx.heatingCircuitRuntime[circuitIndex];
  resetMixerFsmInternal(ctx, circuitIndex, cfg, rt);
}

void process(AppContext& ctx) {
  for (uint8_t i = 0; i < MAX_HEATING_CIRCUITS; i++) {
    HeatingCircuitConfig& cfg = ctx.config.heatingCircuits[i];
    HeatingCircuitRuntime& rt = ctx.heatingCircuitRuntime[i];

    if (!cfg.enabled) {
      resetMixerFsmInternal(ctx, i, cfg, rt);
      Pumps::applyHeatingCircuitPumpRequest(ctx, i, false, 0, NAN, NAN, false);
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
      resetMixerFsmInternal(ctx, i, cfg, rt);
      Pumps::applyHeatingCircuitPumpRequest(ctx, i, false, 0, NAN, NAN, false);
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
    const float baseDeadband =
      (cfg.mixerType == HeatingCircuitMixerType::THERMAL)
        ? 1.5f
        : 0.7f;

    const float deadband =
      MlOptimizer::adjustHeatingMixerDeadband(
        ctx,
        i,
        baseDeadband
      );

    if (error > deadband) {
      requestMixerDirection(i, +1, false);
    } else if (error < -deadband) {
      requestMixerDirection(i, -1, false);
    } else {
      resetMixerFsmInternal(ctx, i, cfg, rt);
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
