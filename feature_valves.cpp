#include "feature_valves.h"

#include "config.h"
#include "feature_relay_outputs.h"
#include "feature_pwm_pca9685.h"

#include <Arduino.h>

namespace {

struct ValveRuntimeState {
  ValvePosition current = ValvePosition::A;
  ValvePosition target = ValvePosition::A;
  bool moving = false;
  uint32_t moveStartedMs = 0;
};

ValveRuntimeState g_runtime[MAX_VALVES];

bool validValveIndex(uint8_t index) {
  return index < MAX_VALVES;
}

bool outputAssigned(const OutputRef& output) {
  return output.kind != OutputKind::NONE && output.index != PIN_UNUSED;
}

void setOutput(AppContext& ctx, const OutputRef& output, bool on) {
  if (!outputAssigned(output)) return;

  if (output.kind == OutputKind::RELAY) {
    RelayOutputs::set(ctx, output.index, on);
    return;
  }

  if (output.kind == OutputKind::PWM_OUTPUT) {
    // PO im Schaltausgang-Modus: 0 % = AUS, 100 % = EIN
    PwmDriver::setSwitch(
    output.index,
    on,
    PwmProfile::HEATING
);
    return;
  }
}

void driveValveOutput(AppContext& ctx, const ValveConfig& cfg, ValvePosition position) {
  const bool wantB = (position == ValvePosition::B);
  const bool outputOn = (wantB == cfg.activeHighForB);
  setOutput(ctx, cfg.output, outputOn);
}

} // namespace

namespace Valves {

void begin(AppContext& ctx) {
  for (uint8_t i = 0; i < MAX_VALVES; i++) {
    ValveConfig& cfg = ctx.config.valves[i];

    g_runtime[i].current = cfg.lastRequestedPosition;
    g_runtime[i].target = cfg.lastRequestedPosition;
    g_runtime[i].moving = false;
    g_runtime[i].moveStartedMs = 0;

    if (cfg.enabled) {
      driveValveOutput(ctx, cfg, cfg.lastRequestedPosition);
    }
  }
}

bool requestPosition(AppContext& ctx, uint8_t valveIndex, ValvePosition position) {
  if (!validValveIndex(valveIndex)) return false;

  ValveConfig& cfg = ctx.config.valves[valveIndex];
  if (!cfg.enabled) return false;
  if (!outputAssigned(cfg.output)) return false;

  ValveRuntimeState& rt = g_runtime[valveIndex];

  rt.target = position;
  cfg.lastRequestedPosition = position;

  driveValveOutput(ctx, cfg, position);

  if (rt.current != position) {
    rt.moving = true;
    rt.moveStartedMs = millis();
  }

  return true;
}

void process(AppContext& ctx) {
  const uint32_t now = millis();

  for (uint8_t i = 0; i < MAX_VALVES; i++) {
    ValveConfig& cfg = ctx.config.valves[i];
    ValveRuntimeState& rt = g_runtime[i];

    if (!cfg.enabled || !outputAssigned(cfg.output)) {
      rt.moving = false;
      continue;
    }

    if (!rt.moving) continue;

    if ((now - rt.moveStartedMs) >= cfg.travelTimeMs) {
      rt.current = rt.target;
      rt.moving = false;

      // Bei bistabilen Umschaltventilen bleibt der Ausgang auf Zielstellung.
      // Falls du monostabile Ventile mit nur Impulsbetrieb nutzt, muss hier der Ausgang abgeschaltet werden.
      driveValveOutput(ctx, cfg, rt.current);
    }
  }
}

void allOff(AppContext& ctx) {
  for (uint8_t i = 0; i < MAX_VALVES; i++) {
    ValveConfig& cfg = ctx.config.valves[i];
    if (!cfg.enabled) continue;

    setOutput(ctx, cfg.output, false);
    g_runtime[i].moving = false;
  }
}

void applySafetyPosition(AppContext& ctx) {
  for (uint8_t i = 0; i < MAX_VALVES; i++) {
    ValveConfig& cfg = ctx.config.valves[i];
    if (!cfg.enabled) continue;

    requestPosition(ctx, i, cfg.safetyPosition);
  }
}

bool isMoving(uint8_t valveIndex) {
  if (!validValveIndex(valveIndex)) return false;
  return g_runtime[valveIndex].moving;
}

ValvePosition currentPosition(uint8_t valveIndex) {
  if (!validValveIndex(valveIndex)) return ValvePosition::A;
  return g_runtime[valveIndex].current;
}

ValvePosition targetPosition(uint8_t valveIndex) {
  if (!validValveIndex(valveIndex)) return ValvePosition::A;
  return g_runtime[valveIndex].target;
}

} // namespace Valves
