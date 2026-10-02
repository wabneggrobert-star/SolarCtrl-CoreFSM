#include "feature_pump_routing.h"

#include "feature_sensor_assignments.h"
#include "feature_valves.h"
#include "feature_forecast.h"
#include "feature_ml_optimizer.h"

#include <Arduino.h>
#include <math.h>

#include "feature_build_flags.h"
namespace {

using RouteResult = PumpRouting::RouteResult;

bool validPumpIndex(uint8_t pumpIndex) { return pumpIndex < MAX_PUMPS; }
bool validTargetIndex(uint8_t targetIndex) { return targetIndex < PUMP_ROUTE_TARGET_COUNT; }

bool readSinkByRole(AppContext& ctx, Ds18Role role, float& tempC, bool& valid) {
  tempC = NAN;
  valid = false;
  if (role == Ds18Role::NONE) return false;
  SensorAssignments::readByRole(ctx.ds18b20, ctx.assignments, role, tempC, valid);
  return valid && !isnan(tempC);
}

float targetDiffFor(const PumpConfig& pump, const PumpRouteTargetConfig& target) {
  if (target.targetDiffOverride > 0.01f) return target.targetDiffOverride;
  return pump.targetDiff;
}

float hysteresisFor(const PumpConfig& pump, const PumpRouteTargetConfig& target) {
  if (target.hysteresisOverride > 0.01f) return target.hysteresisOverride;
  return pump.hysteresis;
}

bool targetBelowMaxTemp(const PumpRouteTargetConfig& target, float sinkC) {
  if (target.maxTempC <= 0.01f) return true;
  if (isnan(sinkC)) return false;
  return sinkC < target.maxTempC;
}

bool targetBelowMinTemp(const PumpRouteTargetConfig& target, float sinkC) {
  if (target.minTempC <= 0.01f) return false;
  if (isnan(sinkC)) return false;
  return sinkC < target.minTempC;
}

bool isBufferRole(Ds18Role role) {
  return role == Ds18Role::SINK_BUFFER_TOP || role == Ds18Role::BUFFER_HIGH || role == Ds18Role::BUFFER_MID || role == Ds18Role::BUFFER_BOTTOM;
}

bool isBoilerRole(Ds18Role role) {
  return role == Ds18Role::SINK_BOILER_TOP || role == Ds18Role::BOILER_BOTTOM;
}

bool valveUsable(const AppContext& ctx, const PumpConfig& pump) {
  return pump.valveIndex != PIN_UNUSED && Valves::isConfigured(ctx, pump.valveIndex);
}

ValvePosition valvePositionForTarget(uint8_t targetIndex) {
  return (targetIndex == 1) ? ValvePosition::B : ValvePosition::A;
}

void fillValveResult(RouteResult& r, const AppContext& ctx, const PumpConfig& pump) {
  if (!valveUsable(ctx, pump)) return;
  r.hasValve = true;
  r.valveIndex = pump.valveIndex;
  r.valveOutput = Valves::outputRef(ctx, pump.valveIndex);
  r.valveRelayIndex = Valves::outputRelayIndex(ctx, pump.valveIndex);
}

void markTargetsInactive(PumpConfig& pump) {
  for (uint8_t i = 0; i < PUMP_ROUTE_TARGET_COUNT; i++) pump.targets[i].active = false;
  pump.activeTargetIndex = PIN_UNUSED;
}

void cancelValveMove(PumpConfig& pump) {
  pump.valvePendingTargetIndex = PIN_UNUSED;
}

void fillTargetResult(RouteResult& r, AppContext& ctx, PumpConfig& pump, uint8_t targetIndex, PumpRouteTargetConfig& target, float sinkC, float diffC, float targetDiff, float hyst) {
  r.active = true;
  r.targetIndex = targetIndex;
  r.sinkRole = target.sinkRole;
  r.sinkC = sinkC;
  r.sinkValid = true;
  r.diffC = diffC;
  r.targetDiff = targetDiff;
  r.hysteresis = hyst;
  fillValveResult(r, ctx, pump);
}

RouteResult legacySingleSink(AppContext& ctx, PumpConfig& pump, float sourceC, bool sourceValid) {
  RouteResult r;
  if (!sourceValid || isnan(sourceC)) return r;

  Ds18Role sinkRole = pump.sinkRole;
  if (sinkRole == Ds18Role::NONE) sinkRole = SensorAssignments::activeSinkRole(ctx.config);

  float sinkC = NAN;
  bool sinkValid = false;
  if (!readSinkByRole(ctx, sinkRole, sinkC, sinkValid)) return r;

  r.active = true;
  r.targetIndex = PIN_UNUSED;
  r.sinkRole = sinkRole;
  r.sinkC = sinkC;
  r.sinkValid = true;
  r.diffC = sourceC - sinkC;
  r.targetDiff = pump.targetDiff;
  r.hysteresis = pump.hysteresis;
  return r;
}

bool handlePendingValveMove(AppContext& ctx, PumpConfig& pump, RouteResult& r, float sourceC) {
  if (!validTargetIndex(pump.valvePendingTargetIndex)) return false;

  const uint8_t pendingIndex = pump.valvePendingTargetIndex;
  PumpRouteTargetConfig& pending = pump.targets[pendingIndex];

  float sinkC = NAN;
  bool sinkValid = false;
  if (!pending.enabled || pending.sinkRole == Ds18Role::NONE || !readSinkByRole(ctx, pending.sinkRole, sinkC, sinkValid)) {
    cancelValveMove(pump);
    return false;
  }

  const float diffC = sourceC - sinkC;
  const float targetDiff = targetDiffFor(pump, pending);
  const float hyst = hysteresisFor(pump, pending);
  pending.lastSinkC = sinkC;
  pending.lastDiffC = diffC;

  // Auch waehrend der Ventilfahrt bleibt maxTempC eine harte Grenze. Wenn das
  // Ziel in der Umschaltzeit sein Maximum erreicht (z. B. durch eine andere
  // Waermequelle), darf die Pumpe danach nicht mehr auf dieses Ziel starten.
  if (!targetBelowMaxTemp(pending, sinkC)) {
    pending.active = false;
    cancelValveMove(pump);
    return false;
  }

  fillTargetResult(r, ctx, pump, pendingIndex, pending, sinkC, diffC, targetDiff, hyst);

  if (Valves::isMoving(pump.valveIndex)) {
    r.valveMoving = true;
    r.valveMoveRemainingMs = Valves::moveRemainingMs(ctx, pump.valveIndex);
    return true;
  }

  pump.activeTargetIndex = pendingIndex;
  pending.active = true;
  cancelValveMove(pump);
  return true;
}

void requestValveTarget(AppContext& ctx, PumpConfig& pump, uint8_t targetIndex, RouteResult& r) {
  if (!valveUsable(ctx, pump)) return;

  const ValvePosition pos = valvePositionForTarget(targetIndex);
  if (Valves::currentPosition(pump.valveIndex) != pos || Valves::isMoving(pump.valveIndex)) {
    Valves::requestPosition(ctx, pump.valveIndex, pos);
    pump.valvePendingTargetIndex = targetIndex;
    r.valveMoving = Valves::isMoving(pump.valveIndex);
    r.valveMoveRemainingMs = Valves::moveRemainingMs(ctx, pump.valveIndex);

    DBG_PRINT("PUMP ROUTE Valve V2 Pumpe Ziel ");
    DBG_PRINT(targetIndex == 0 ? "A" : "B");
    DBG_PRINT(" | Ventil=");
    DBG_PRINTLN(pump.valveIndex);
    DBG_FLUSH();
  }
}

} // namespace

namespace PumpRouting {

void begin(AppContext& ctx) {
  for (uint8_t p = 0; p < MAX_PUMPS; p++) {
    ctx.config.pumps[p].activeTargetIndex = PIN_UNUSED;
    ctx.config.pumps[p].valvePendingTargetIndex = PIN_UNUSED;
    for (uint8_t t = 0; t < PUMP_ROUTE_TARGET_COUNT; t++) {
      ctx.config.pumps[p].targets[t].active = false;
      ctx.config.pumps[p].targets[t].lastSinkC = NAN;
      ctx.config.pumps[p].targets[t].lastDiffC = NAN;
    }
  }
}

void closeAllTargets(AppContext& ctx, uint8_t pumpIndex) {
  if (!validPumpIndex(pumpIndex)) return;
  PumpConfig& pump = ctx.config.pumps[pumpIndex];
  markTargetsInactive(pump);
  cancelValveMove(pump);
}

RouteResult resolve(AppContext& ctx, uint8_t pumpIndex, float sourceC, bool sourceValid) {
  RouteResult r;
  if (!validPumpIndex(pumpIndex)) return r;
  PumpConfig& pump = ctx.config.pumps[pumpIndex];

  if (!pump.enabled || !sourceValid || isnan(sourceC)) {
    closeAllTargets(ctx, pumpIndex);
    return r;
  }

  if (!valveUsable(ctx, pump)) {
    cancelValveMove(pump);
    return legacySingleSink(ctx, pump, sourceC, sourceValid);
  }

  // Eine bereits angeforderte Ventilfahrt gehoert noch zum selben Pumpenzyklus.
  // Deshalb darf die Zielwahl waehrend der Fahrt nicht neu bewertet werden.
  if (handlePendingValveMove(ctx, pump, r, sourceC)) return r;

  // Valve-V2 Prioritaet:
  // Ziel A hat eine harte Mindesttemperatur-Prioritaet. Solange A unter minTempC
  // liegt, darf Ziel B nicht geladen werden. Die Entscheidung basiert bewusst
  // ausschliesslich auf dem fuer Ziel A konfigurierten Sink-Sensor.
  PumpRouteTargetConfig& targetA = pump.targets[0];
  float targetASinkC = NAN;
  bool targetASinkValid = false;
  const bool targetAConfigured = targetA.enabled && targetA.sinkRole != Ds18Role::NONE;
  if (targetAConfigured) {
    readSinkByRole(ctx, targetA.sinkRole, targetASinkC, targetASinkValid);
    targetA.lastSinkC = targetASinkValid ? targetASinkC : NAN;
    targetA.lastDiffC = targetASinkValid ? (sourceC - targetASinkC) : NAN;
  }

  const bool targetAMinConfigured = targetAConfigured && targetA.minTempC > 0.01f;
  const bool targetABelowMinimum = targetAMinConfigured && targetASinkValid && targetBelowMinTemp(targetA, targetASinkC);
  const bool targetAMinimumKnownSatisfied = !targetAMinConfigured || (targetASinkValid && !targetABelowMinimum);

  // Wenn fuer A eine Mindesttemperatur konfiguriert ist, der A-Sensor aber
  // ungueltig ist, darf B nicht als Ersatz geladen werden. Safety kann den
  // Sensorfehler separat behandeln; Routing bleibt hier konservativ.
  const bool targetAPriorityUnknown = targetAMinConfigured && !targetASinkValid;
  if (targetAPriorityUnknown) {
    markTargetsInactive(pump);
    return r;
  }

  // ML/Forecast darf nur dann zwischen A/B umpriorisieren, wenn A seine
  // Mindesttemperatur erreicht hat. Unterhalb der Mindesttemperatur bleibt A
  // eine harte Sperre fuer Ziel B.
  const bool forecastPreferBuffer = targetAMinimumKnownSatisfied && MlOptimizer::preferBufferForSolar(ctx, pump);

  // Harte A-Prioritaet gilt auch dann, wenn aus einem frueheren Zyklus noch B
  // aktiv waere. Normalerweise wird activeTargetIndex beim Pump-Enable-AUS
  // bereits geloescht; diese Absicherung verhindert dennoch ein Durchrutschen.
  if (targetABelowMinimum && validTargetIndex(pump.activeTargetIndex) && pump.activeTargetIndex != 0) {
    markTargetsInactive(pump);
  }

  // ML darf oberhalb A-Minimum ein laufendes Boiler-Ziel zugunsten des Puffers
  // wechseln. Unterhalb A-Minimum ist forecastPreferBuffer immer false.
  if (forecastPreferBuffer && validTargetIndex(pump.activeTargetIndex) && isBoilerRole(pump.targets[pump.activeTargetIndex].sinkRole)) {
    for (uint8_t i = 0; i < PUMP_ROUTE_TARGET_COUNT; i++) {
      if (pump.targets[i].enabled && isBufferRole(pump.targets[i].sinkRole)) {
        markTargetsInactive(pump);
        break;
      }
    }
  }

  // Bereits fuer diesen Pumpenzyklus gewaehltes Ziel beibehalten:
  // minTempC beendet keinen laufenden Zyklus. Das Routing entscheidet nur,
  // welches Ziel hydraulisch zulaessig ist. Die thermische Start-/Stoplogik
  // (startDiff bzw. targetDiff/hysteresis) gehoert ausschliesslich ins
  // Pumpenmodul. Hier bleibt nur maxTempC als harte Zielgrenze.
  if (validTargetIndex(pump.activeTargetIndex)) {
    PumpRouteTargetConfig& target = pump.targets[pump.activeTargetIndex];
    if (target.enabled && target.sinkRole != Ds18Role::NONE) {
      float sinkC = NAN;
      bool sinkValid = false;
      if (readSinkByRole(ctx, target.sinkRole, sinkC, sinkValid)) {
        const float diffC = sourceC - sinkC;
        const float targetDiff = targetDiffFor(pump, target);
        const float hyst = hysteresisFor(pump, target);
        target.lastSinkC = sinkC;
        target.lastDiffC = diffC;
        if (targetBelowMaxTemp(target, sinkC)) {
          target.active = true;
          fillTargetResult(r, ctx, pump, pump.activeTargetIndex, target, sinkC, diffC, targetDiff, hyst);
          return r;
        }
      }
    }
  }

  // Kein laufendes Ziel mehr: neuer Auswahlvorgang. activeTargetIndex wird
  // durch feature_pumps beim Pump-Enable-AUS geloescht, daher entspricht diese
  // Auswahl dem Beginn eines neuen Pumpenzyklus.
  markTargetsInactive(pump);

  // A unter Minimum: ausschliesslich A auswaehlen. B bleibt in diesem Fall
  // ausdruecklich gesperrt. Ob die Pumpe fuer dieses Ziel thermisch starten
  // darf, entscheidet anschliessend feature_pumps.cpp.
  if (targetABelowMinimum) {
    if (!targetBelowMaxTemp(targetA, targetASinkC)) return r;

    const float diffC = sourceC - targetASinkC;
    const float targetDiff = targetDiffFor(pump, targetA);
    const float hyst = hysteresisFor(pump, targetA);
    targetA.lastDiffC = diffC;

    fillTargetResult(r, ctx, pump, 0, targetA, targetASinkC, diffC, targetDiff, hyst);
    requestValveTarget(ctx, pump, 0, r);
    if (r.valveMoving) return r;

    pump.activeTargetIndex = 0;
    targetA.active = true;
    return r;
  }

  // A hat Minimum erreicht (oder fuer A ist kein Minimum konfiguriert):
  // Standardmaessig darf jetzt B zuerst geladen werden. Ist B voll, ungueltig
  // oder thermisch nicht ladbar, darf A als Fallback bis zu seinem Maximum
  // weitergeladen werden. ML darf oberhalb A-Minimum die Reihenfolge A/B
  // umpriorisieren, ohne die jeweiligen Maximaltemperaturen zu umgehen.
  uint8_t order[PUMP_ROUTE_TARGET_COUNT] = {1, 0};

  if (forecastPreferBuffer) {
    int8_t bufferIndex = -1;
    for (uint8_t i = 0; i < PUMP_ROUTE_TARGET_COUNT; i++) {
      if (pump.targets[i].enabled && isBufferRole(pump.targets[i].sinkRole)) {
        bufferIndex = (int8_t)i;
        break;
      }
    }
    if (bufferIndex >= 0) {
      order[0] = (uint8_t)bufferIndex;
      order[1] = (uint8_t)(1 - bufferIndex);
    }
  }

  for (uint8_t orderIndex = 0; orderIndex < PUMP_ROUTE_TARGET_COUNT; orderIndex++) {
    const uint8_t i = order[orderIndex];
    PumpRouteTargetConfig& target = pump.targets[i];
    if (!target.enabled || target.sinkRole == Ds18Role::NONE) {
      target.lastSinkC = NAN;
      target.lastDiffC = NAN;
      continue;
    }

    float sinkC = NAN;
    bool sinkValid = false;
    if (!readSinkByRole(ctx, target.sinkRole, sinkC, sinkValid)) {
      target.lastSinkC = NAN;
      target.lastDiffC = NAN;
      continue;
    }

    const float diffC = sourceC - sinkC;
    const float targetDiff = targetDiffFor(pump, target);
    const float hyst = hysteresisFor(pump, target);
    target.lastSinkC = sinkC;
    target.lastDiffC = diffC;

    // Fuer die Zielauswahl oberhalb A-Minimum ist nur maxTempC die harte
    // thermische Zielbegrenzung. Das Routing waehlt den Sink; die Pumpe
    // entscheidet danach anhand ihrer eigenen Start-/Stoplogik, ob sie laufen
    // darf. Insbesondere darf A als Fallback oberhalb seines minTempC bis
    // maxTempC weitergeladen werden.
    if (!targetBelowMaxTemp(target, sinkC)) continue;

    fillTargetResult(r, ctx, pump, i, target, sinkC, diffC, targetDiff, hyst);

    requestValveTarget(ctx, pump, i, r);
    if (r.valveMoving) return r;

    pump.activeTargetIndex = i;
    target.active = true;
    return r;
  }

  return r;
}

RouteResult resolveHeatDumpCoolestTarget(AppContext& ctx, uint8_t pumpIndex, float sourceC, bool sourceValid) {
  RouteResult r;
  if (!validPumpIndex(pumpIndex)) return r;
  PumpConfig& pump = ctx.config.pumps[pumpIndex];

  if (!pump.enabled || !sourceValid || isnan(sourceC)) {
    closeAllTargets(ctx, pumpIndex);
    return r;
  }

  if (!valveUsable(ctx, pump)) {
    cancelValveMove(pump);
    RouteResult legacy = legacySingleSink(ctx, pump, sourceC, sourceValid);
    if (!legacy.active || !legacy.sinkValid) return r;
    const float maximumTargetTemperatureC = (pump.targets[0].maxTempC > 0.01f) ? pump.targets[0].maxTempC : ctx.config.sinkMaxC;
    if (maximumTargetTemperatureC > 0.01f && legacy.sinkC >= maximumTargetTemperatureC) return r;
    return legacy;
  }

  if (handlePendingValveMove(ctx, pump, r, sourceC)) return r;

  int8_t bestIndex = -1;
  float coolestTargetTemperatureC = NAN;
  float bestDiffC = NAN;
  float bestTargetDiff = NAN;
  float bestHysteresis = NAN;

  for (uint8_t i = 0; i < PUMP_ROUTE_TARGET_COUNT; i++) {
    PumpRouteTargetConfig& target = pump.targets[i];
    if (!target.enabled || target.sinkRole == Ds18Role::NONE) {
      target.lastSinkC = NAN;
      target.lastDiffC = NAN;
      continue;
    }

    float sinkC = NAN;
    bool sinkValid = false;
    if (!readSinkByRole(ctx, target.sinkRole, sinkC, sinkValid)) {
      target.lastSinkC = NAN;
      target.lastDiffC = NAN;
      continue;
    }

    const float diffC = sourceC - sinkC;
    const float targetDiff = targetDiffFor(pump, target);
    const float hyst = hysteresisFor(pump, target);
    target.lastSinkC = sinkC;
    target.lastDiffC = diffC;

    if (!targetBelowMaxTemp(target, sinkC)) continue;
    if (diffC <= 0.0f) continue;

    if (bestIndex < 0 || sinkC < coolestTargetTemperatureC) {
      bestIndex = i;
      coolestTargetTemperatureC = sinkC;
      bestDiffC = diffC;
      bestTargetDiff = targetDiff;
      bestHysteresis = hyst;
    }
  }

  if (bestIndex < 0) {
    markTargetsInactive(pump);
    return r;
  }

  PumpRouteTargetConfig& bestTarget = pump.targets[bestIndex];
  fillTargetResult(r, ctx, pump, (uint8_t)bestIndex, bestTarget, coolestTargetTemperatureC, bestDiffC, bestTargetDiff, bestHysteresis);

  if (pump.activeTargetIndex != (uint8_t)bestIndex) {
    markTargetsInactive(pump);
    requestValveTarget(ctx, pump, (uint8_t)bestIndex, r);
    if (r.valveMoving) return r;
  }

  bestTarget.active = true;
  pump.activeTargetIndex = (uint8_t)bestIndex;
  return r;
}

} // namespace PumpRouting
