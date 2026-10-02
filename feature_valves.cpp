#include "feature_valves.h"

#include "config.h"
#include "feature_relay_outputs.h"
#include "feature_pwm_pca9685.h"

#include <Arduino.h>

#include "feature_build_flags.h"
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

bool outputIndexValid(const OutputRef& output) {
  if (!outputAssigned(output)) return false;
  if (output.kind == OutputKind::RELAY) return output.index < RELAY_COUNT;
  if (output.kind == OutputKind::PWM_OUTPUT) return output.index < PWM_OUTPUT_COUNT;
  return false;
}

void setOutput(AppContext& ctx, const OutputRef& output, bool on) {
  if (!outputIndexValid(output)) return;

  if (output.kind == OutputKind::RELAY) {
    RelayOutputs::set(ctx, output.index, on);
    return;
  }

  if (output.kind == OutputKind::PWM_OUTPUT) {
    const PwmOutputConfig& po = ctx.config.pwmOutputs[output.index];
    PwmDriver::setSwitch(output.index, on, po.profile);
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
    ValveRuntimeState& rt = g_runtime[i];

    rt.current = cfg.lastRequestedPosition;
    rt.target = cfg.lastRequestedPosition;
    rt.moving = false;
    rt.moveStartedMs = 0;

    if (isConfigured(ctx, i)) {
      driveValveOutput(ctx, cfg, cfg.lastRequestedPosition);
    }
  }
}

bool isConfigured(const AppContext& ctx, uint8_t valveIndex) {
  if (!validValveIndex(valveIndex)) return false;
  const ValveConfig& cfg = ctx.config.valves[valveIndex];
  return cfg.enabled && outputIndexValid(cfg.output);
}

bool requestPosition(AppContext& ctx, uint8_t valveIndex, ValvePosition position) {
  if (!validValveIndex(valveIndex)) {
    DBG_PRINTLN("VALVE V2 ERROR: invalid index");
    return false;
  }

  ValveConfig& cfg = ctx.config.valves[valveIndex];
  if (!isConfigured(ctx, valveIndex)) {
    DBG_PRINT("VALVE V2 ERROR: valve not configured index=");
    DBG_PRINTLN(valveIndex);
    return false;
  }

  ValveRuntimeState& rt = g_runtime[valveIndex];
  rt.target = position;
  cfg.lastRequestedPosition = position;

  driveValveOutput(ctx, cfg, position);

  if (rt.current != position) {
    rt.moving = true;
    rt.moveStartedMs = millis();

    DBG_PRINT("VALVE V2 START V");
    DBG_PRINT(valveIndex + 1);
    DBG_PRINT(" -> ");
    DBG_PRINT(positionToKey(position));
    DBG_PRINT(" Fahrzeit ms=");
    DBG_PRINTLN(cfg.travelTimeMs);
    DBG_FLUSH();
  }

  if (cfg.travelTimeMs == 0) {
    rt.current = position;
    rt.moving = false;
  }

  return true;
}

void process(AppContext& ctx) {
  const uint32_t now = millis();

  for (uint8_t i = 0; i < MAX_VALVES; i++) {
    ValveConfig& cfg = ctx.config.valves[i];
    ValveRuntimeState& rt = g_runtime[i];

    if (!isConfigured(ctx, i)) {
      rt.moving = false;
      continue;
    }

    if (!rt.moving) continue;

    if ((uint32_t)(now - rt.moveStartedMs) >= cfg.travelTimeMs) {
      rt.current = rt.target;
      rt.moving = false;
      driveValveOutput(ctx, cfg, rt.current);

      DBG_PRINT("VALVE V2 END V");
      DBG_PRINT(i + 1);
      DBG_PRINT(" = ");
      DBG_PRINTLN(positionToKey(rt.current));
      DBG_FLUSH();
    }
  }
}

void allOff(AppContext& ctx) {
  for (uint8_t i = 0; i < MAX_VALVES; i++) {
    if (!isConfigured(ctx, i)) continue;
    setOutput(ctx, ctx.config.valves[i].output, false);
    g_runtime[i].moving = false;
  }
}

void applySafetyPosition(AppContext& ctx) {
  for (uint8_t i = 0; i < MAX_VALVES; i++) {
    if (!isConfigured(ctx, i)) continue;
    requestPosition(ctx, i, ctx.config.valves[i].safetyPosition);
  }
}

bool isMoving(uint8_t valveIndex) {
  if (!validValveIndex(valveIndex)) return false;
  return g_runtime[valveIndex].moving;
}

uint32_t moveRemainingMs(const AppContext& ctx, uint8_t valveIndex) {
  if (!validValveIndex(valveIndex)) return 0;
  if (!g_runtime[valveIndex].moving) return 0;

  const uint32_t elapsed = (uint32_t)(millis() - g_runtime[valveIndex].moveStartedMs);
  const uint32_t travel = ctx.config.valves[valveIndex].travelTimeMs;
  if (elapsed >= travel) return 0;
  return travel - elapsed;
}

ValvePosition currentPosition(uint8_t valveIndex) {
  if (!validValveIndex(valveIndex)) return ValvePosition::A;
  return g_runtime[valveIndex].current;
}

ValvePosition targetPosition(uint8_t valveIndex) {
  if (!validValveIndex(valveIndex)) return ValvePosition::A;
  return g_runtime[valveIndex].target;
}

OutputRef outputRef(const AppContext& ctx, uint8_t valveIndex) {
  if (!validValveIndex(valveIndex)) return OutputRef{};
  return ctx.config.valves[valveIndex].output;
}

uint8_t outputRelayIndex(const AppContext& ctx, uint8_t valveIndex) {
  if (!validValveIndex(valveIndex)) return PIN_UNUSED;
  const OutputRef& ref = ctx.config.valves[valveIndex].output;
  if (ref.kind != OutputKind::RELAY || ref.index >= RELAY_COUNT) return PIN_UNUSED;
  return ref.index;
}

const char* positionToKey(ValvePosition position) {
  return (position == ValvePosition::B) ? "B" : "A";
}

ValvePosition positionFromKey(const String& key) {
  return (key == "B" || key == "b" || key == "1") ? ValvePosition::B : ValvePosition::A;
}

} // namespace Valves
