#include "feature_pump_routing.h"

#include "feature_relay_outputs.h"
#include "feature_sensor_assignments.h"

#include <Arduino.h>
#include <math.h>

namespace {

  using RouteResult = PumpRouting::RouteResult;

  bool validPumpIndex(uint8_t pumpIndex) {
    return pumpIndex < MAX_PUMPS;
  }

  bool validTargetIndex(uint8_t targetIndex) {
    return targetIndex < PUMP_ROUTE_TARGET_COUNT;
  }

  bool readSinkByRole(AppContext& ctx, Ds18Role role, float& tempC, bool& valid) {
    tempC = NAN;
    valid = false;

    if (role == Ds18Role::NONE) return false;

    SensorAssignments::readByRole(ctx.assignments, role, tempC, valid);
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

  bool targetNeedsHeat(const PumpRouteTargetConfig& target, float sinkC) {
    if (target.minTempC <= 0.01f) return true;
    if (isnan(sinkC)) return false;
    return sinkC < target.minTempC;
  }

  bool switchValveUsable(const AppContext& ctx, const PumpConfig& pump) {
    if (!pump.switchValveEnabled) return false;
    if (pump.switchValveRelayIndex == PIN_UNUSED) return false;
    return RelayOutputs::isUsableAsZoneValve(ctx, pump.switchValveRelayIndex);
  }

  bool relayStateForTarget(const PumpConfig& pump, uint8_t targetIndex) {
    if (targetIndex == 0) return pump.switchValveStateForTargetA;
    return !pump.switchValveStateForTargetA;
  }

  void fillValveResult(RouteResult& r, const AppContext& ctx, const PumpConfig& pump, uint8_t targetIndex) {
    if (!switchValveUsable(ctx, pump)) return;
    r.hasValve = true;
    r.valveRelayIndex = pump.switchValveRelayIndex;
    r.valveState = relayStateForTarget(pump, targetIndex);
  }

  void markTargetsInactive(PumpConfig& pump) {
    for (uint8_t i = 0; i < PUMP_ROUTE_TARGET_COUNT; i++) {
      pump.targets[i].active = false;
    }
    pump.activeTargetIndex = PIN_UNUSED;
  }

  void cancelValveMove(PumpConfig& pump) {
    pump.switchValveMoving = false;
    pump.switchValveMoveStartedMs = 0;
    pump.switchValvePendingTargetIndex = PIN_UNUSED;
  }

  void startValveMove(AppContext& ctx, PumpConfig& pump, uint8_t targetIndex) {
    RelayOutputs::set(ctx, pump.switchValveRelayIndex, relayStateForTarget(pump, targetIndex));

    pump.switchValveMoving = true;
    pump.switchValveMoveStartedMs = millis();
    pump.switchValvePendingTargetIndex = targetIndex;

    Serial.print("UMSCHALTVENTIL START Pumpe Ziel ");
    Serial.print(targetIndex == 0 ? "A" : "B");
    Serial.print(" | Relais=");
    Serial.print(pump.switchValveRelayIndex);
    Serial.print(" | Fahrzeit ms=");
    Serial.println(pump.switchValveTravelTimeMs);
    Serial.flush();
  }

  bool valveMoveStillRunning(PumpConfig& pump, RouteResult& r) {
    if (!pump.switchValveMoving) return false;

    const uint32_t elapsed = (uint32_t)(millis() - pump.switchValveMoveStartedMs);
    if (elapsed >= pump.switchValveTravelTimeMs) {
      Serial.println("UMSCHALTVENTIL FAHRT FERTIG");
      Serial.flush();
      cancelValveMove(pump);
      return false;
    }

    r.valveMoving = true;
    r.valveMoveRemainingMs = pump.switchValveTravelTimeMs - elapsed;
    return true;
  }

  void fillTargetResult(
    RouteResult& r,
    AppContext& ctx,
    PumpConfig& pump,
    uint8_t targetIndex,
    PumpRouteTargetConfig& target,
    float sinkC,
    float diffC,
    float targetDiff,
    float hyst
  ) {
    r.active = true;
    r.targetIndex = targetIndex;
    r.sinkRole = target.sinkRole;
    r.sinkC = sinkC;
    r.sinkValid = true;
    r.diffC = diffC;
    r.targetDiff = targetDiff;
    r.hysteresis = hyst;
    fillValveResult(r, ctx, pump, targetIndex);
  }

  RouteResult legacySingleSink(AppContext& ctx, PumpConfig& pump, float sourceC, bool sourceValid) {
    RouteResult r;

    if (!sourceValid || isnan(sourceC)) return r;

    Ds18Role sinkRole = pump.sinkRole;
    if (sinkRole == Ds18Role::NONE) {
      sinkRole = SensorAssignments::activeSinkRole(ctx.config);
    }

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
}

namespace PumpRouting {

void begin(AppContext& ctx) {
  for (uint8_t p = 0; p < MAX_PUMPS; p++) {
    ctx.config.pumps[p].activeTargetIndex = PIN_UNUSED;
    cancelValveMove(ctx.config.pumps[p]);
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

  // Ein echtes Umschaltventil hat keine neutrale Stellung.
  // Daher wird es beim Abschalten nicht zwangsweise umgeschaltet.
  // Eine laufende Ventilfahrt wird jedoch abgebrochen, weil die Pumpe aus ist.
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

  if (!pump.switchValveEnabled) {
    cancelValveMove(pump);
    return legacySingleSink(ctx, pump, sourceC, sourceValid);
  }

  if (!switchValveUsable(ctx, pump)) {
    closeAllTargets(ctx, pumpIndex);
    return r;
  }

  // Wenn ein Ventil gerade faehrt, bleibt der Zielpfad logisch gewaehlt,
  // die Pumpe darf aber noch nicht laufen.
  if (validTargetIndex(pump.switchValvePendingTargetIndex)) {
    const uint8_t pendingIndex = pump.switchValvePendingTargetIndex;
    PumpRouteTargetConfig& pending = pump.targets[pendingIndex];
    float sinkC = NAN;
    bool sinkValid = false;
    if (pending.enabled && pending.sinkRole != Ds18Role::NONE && readSinkByRole(ctx, pending.sinkRole, sinkC, sinkValid)) {
      const float diffC = sourceC - sinkC;
      const float targetDiff = targetDiffFor(pump, pending);
      const float hyst = hysteresisFor(pump, pending);
      pending.lastSinkC = sinkC;
      pending.lastDiffC = diffC;
      fillTargetResult(r, ctx, pump, pendingIndex, pending, sinkC, diffC, targetDiff, hyst);
      if (valveMoveStillRunning(pump, r)) {
        return r;
      }
      // Fahrt ist beendet, Ziel wird aktiv.
      pump.activeTargetIndex = pendingIndex;
      pump.targets[pump.activeTargetIndex].active = true;
      pump.switchValvePendingTargetIndex = PIN_UNUSED;
      return r;
    }
  }

  // Aktiven Pfad halten, solange Ziel nicht voll ist und dT oberhalb Ausschaltgrenze liegt.
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

        if (targetBelowMaxTemp(target, sinkC) && diffC > (targetDiff - hyst)) {
          target.active = true;
          fillTargetResult(r, ctx, pump, pump.activeTargetIndex, target, sinkC, diffC, targetDiff, hyst);
          return r;
        }
      }
    }
  }

  markTargetsInactive(pump);

  // Neuen Zielpfad suchen. Reihenfolge entspricht Prioritaet: Ziel A vor Ziel B.
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

    if (!targetBelowMaxTemp(target, sinkC)) {
      Serial.print("PUMP ROUTE Ziel ");
      Serial.print(i == 0 ? "A" : "B");
      Serial.print(" gesperrt wegen maximaler Zieltemperatur | Zieltemperatur=");
      Serial.print(sinkC);
      Serial.print(" Maximaltemperatur=");
      Serial.println(target.maxTempC);
      Serial.flush();
      continue;
    }

    if (!targetNeedsHeat(target, sinkC)) {
      Serial.print("PUMP ROUTE Ziel ");
      Serial.print(i == 0 ? "A" : "B");
      Serial.print(" wartet: Zieltemperatur ueber Mindesttemperatur | Zieltemperatur=");
      Serial.print(sinkC);
      Serial.print(" Mindesttemperatur=");
      Serial.println(target.minTempC);
      Serial.flush();
      continue;
    }

    if (diffC >= (targetDiff + hyst)) {
      fillTargetResult(r, ctx, pump, i, target, sinkC, diffC, targetDiff, hyst);

      if (pump.activeTargetIndex != i) {
        startValveMove(ctx, pump, i);
        r.valveMoving = true;
        r.valveMoveRemainingMs = pump.switchValveTravelTimeMs;
        // Das Ziel ist gewaehlt, aber Pumpe darf erst nach der Ventilfahrt laufen.
        return r;
      }

      pump.activeTargetIndex = i;
      target.active = true;

      Serial.print("PUMP ROUTE P");
      Serial.print(pumpIndex + 1);
      Serial.print(" -> Ziel ");
      Serial.print(i == 0 ? "A" : "B");
      Serial.print(" SinkRole=");
      Serial.print((int)target.sinkRole);
      Serial.print(" Temperaturdifferenz=");
      Serial.println(diffC);
      Serial.flush();

      return r;
    }
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

  if (!pump.switchValveEnabled) {
    cancelValveMove(pump);

    RouteResult legacy = legacySingleSink(ctx, pump, sourceC, sourceValid);
    if (!legacy.active || !legacy.sinkValid) return r;

    const float maximumTargetTemperatureC = (pump.targets[0].maxTempC > 0.01f)
      ? pump.targets[0].maxTempC
      : ctx.config.sinkMaxC;

    if (maximumTargetTemperatureC > 0.01f && legacy.sinkC >= maximumTargetTemperatureC) {
      return r;
    }

    return legacy;
  }

  if (!switchValveUsable(ctx, pump)) {
    closeAllTargets(ctx, pumpIndex);
    return r;
  }

  if (validTargetIndex(pump.switchValvePendingTargetIndex)) {
    const uint8_t pendingIndex = pump.switchValvePendingTargetIndex;
    PumpRouteTargetConfig& pending = pump.targets[pendingIndex];

    float sinkC = NAN;
    bool sinkValid = false;

    if (pending.enabled &&
        pending.sinkRole != Ds18Role::NONE &&
        readSinkByRole(ctx, pending.sinkRole, sinkC, sinkValid)) {

      const float diffC = sourceC - sinkC;
      const float targetDiff = targetDiffFor(pump, pending);
      const float hyst = hysteresisFor(pump, pending);

      pending.lastSinkC = sinkC;
      pending.lastDiffC = diffC;

      fillTargetResult(r, ctx, pump, pendingIndex, pending, sinkC, diffC, targetDiff, hyst);

      if (valveMoveStillRunning(pump, r)) {
        return r;
      }

      pump.activeTargetIndex = pendingIndex;
      pump.targets[pump.activeTargetIndex].active = true;
      pump.switchValvePendingTargetIndex = PIN_UNUSED;
      return r;
    }
  }

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

    if (!targetBelowMaxTemp(target, sinkC)) {
      continue;
    }

    if (diffC <= 0.0f) {
      continue;
    }

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

  fillTargetResult(
    r,
    ctx,
    pump,
    (uint8_t)bestIndex,
    bestTarget,
    coolestTargetTemperatureC,
    bestDiffC,
    bestTargetDiff,
    bestHysteresis
  );

  if (pump.activeTargetIndex != (uint8_t)bestIndex) {
    markTargetsInactive(pump);
    startValveMove(ctx, pump, (uint8_t)bestIndex);
    r.valveMoving = true;
    r.valveMoveRemainingMs = pump.switchValveTravelTimeMs;
    return r;
  }

  bestTarget.active = true;
  pump.activeTargetIndex = (uint8_t)bestIndex;

  return r;
}


}
