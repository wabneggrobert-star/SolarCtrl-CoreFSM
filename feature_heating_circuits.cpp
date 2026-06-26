#include "feature_heating_circuits.h"

#include "feature_sensor_assignments.h"
#include "feature_relay_outputs.h"
#include "feature_pwm_pca9685.h"

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

float targetFlowTemperature(AppContext& ctx, const HeatingCircuitConfig& cfg, HeatingCircuitRuntime& rt) {
  float target = cfg.fixedFlowTemperatureC;

  if (cfg.controlMode == HeatingCircuitControlMode::WEATHER_COMPENSATED) {
    float outsideC = NAN;
    bool outsideValid = false;
    if (readDs18(ctx, cfg.outsideSensorRole, outsideC, outsideValid)) {
      rt.outsideTemperatureC = outsideC;
      target = cfg.heatingCurveBaseC + cfg.heatingCurveSlope * (20.0f - outsideC);
    }
  }

  float roomC = NAN;
  bool roomValid = false;
  if (readDs18(ctx, cfg.roomSensorRole, roomC, roomValid)) {
    rt.roomTemperatureC = roomC;
    target += (cfg.roomTargetTemperatureC - roomC) * cfg.roomInfluenceK;
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

} // namespace

namespace HeatingCircuits {

void begin(AppContext& ctx) {
  for (uint8_t i = 0; i < MAX_HEATING_CIRCUITS; i++) {
    ctx.heatingCircuitRuntime[i] = HeatingCircuitRuntime{};
  }
}

void allOff(AppContext& ctx) {
  for (uint8_t i = 0; i < MAX_HEATING_CIRCUITS; i++) {
    HeatingCircuitConfig& cfg = ctx.config.heatingCircuits[i];
    HeatingCircuitRuntime& rt = ctx.heatingCircuitRuntime[i];
    stopMixer(ctx, cfg, rt);
    setPump(ctx, cfg, false, 0);
    rt.pumpActive = false;
  }
}

void process(AppContext& ctx) {
  for (uint8_t i = 0; i < MAX_HEATING_CIRCUITS; i++) {
    HeatingCircuitConfig& cfg = ctx.config.heatingCircuits[i];
    HeatingCircuitRuntime& rt = ctx.heatingCircuitRuntime[i];

    if (!cfg.enabled) {
      stopMixer(ctx, cfg, rt);
      setPump(ctx, cfg, false, 0);
      rt.active = false;
      rt.pumpActive = false;
      continue;
    }

    float flowC = NAN;
    bool flowValid = false;
    if (!readDs18(ctx, cfg.flowSensorRole, flowC, flowValid)) {
      // Failsafe: Sensorfehler -> Mischer mittig lassen, Pumpe EIN wenn vorhanden.
      stopMixer(ctx, cfg, rt);
      setPump(ctx, cfg, true, cfg.pumpMinPercent);
      rt.active = false;
      rt.pumpActive = true;
      continue;
    }

    rt.flowTemperatureC = flowC;

    float returnC = NAN;
    bool returnValid = false;
    if (readDs18(ctx, cfg.returnSensorRole, returnC, returnValid)) {
      rt.returnTemperatureC = returnC;
    }

    float targetC = targetFlowTemperature(ctx, cfg, rt);

    // Frostschutz ueber Aussentemperatur oder Raum/Vorlauf.
    bool frostActive = false;
    if (cfg.frostProtectionEnabled) {
      float outsideC = NAN;
      bool outsideValid = false;
      if (readDs18(ctx, cfg.outsideSensorRole, outsideC, outsideValid) && outsideC <= cfg.frostStartTemperatureC) {
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

    bool pumpOn = frostActive || targetC > cfg.minimumFlowTemperatureC;
    uint8_t pumpPercent = cfg.pumpMinPercent;
    if (cfg.pumpMode == HeatingCircuitPumpMode::PWM) {
      float demand = fabs(error) * 10.0f;
      uint8_t range = (cfg.pumpMaxPercent > cfg.pumpMinPercent) ? (cfg.pumpMaxPercent - cfg.pumpMinPercent) : 0;
      pumpPercent = cfg.pumpMinPercent + (uint8_t)clampFloat(demand, 0.0f, range);
      if (pumpPercent > cfg.pumpMaxPercent) pumpPercent = cfg.pumpMaxPercent;
    }

    setPump(ctx, cfg, pumpOn, pumpPercent);
    rt.pumpActive = pumpOn;
  }
}

} // namespace HeatingCircuits
