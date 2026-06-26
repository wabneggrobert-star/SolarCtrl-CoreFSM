#include "feature_pumps.h"

#include "config.h"
#include "feature_energy_conflicts.h"
#include "feature_pump_routing.h"
#include "feature_relay_outputs.h"
#include "feature_pwm_pca9685.h"
#include "feature_sensor_assignments.h"

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

    if (p.switchValveEnabled) {
      Serial.print(" | Umschaltventil=");
      Serial.print(p.switchValveRelayIndex);
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
      const uint8_t valveRelayIndex = route.hasValve ? route.valveRelayIndex : PIN_UNUSED;

      if (!EnergyConflicts::canActivateRoute(
            ctx,
            i,
            p.sourceRole,
            route.sinkRole,
            p.relayIndex,
            valveRelayIndex
          )) {
        forcePumpOff(ctx, i);
        if (wasOn) {
          printPumpSwitch(i, false, p);
        }
        continue;
      }

      if (route.hasValve) {
        RelayOutputs::set(ctx, route.valveRelayIndex, route.valveState);
      }

      EnergyConflicts::reserveRoute(
        ctx,
        i,
        p.sourceRole,
        route.sinkRole,
        p.relayIndex,
        valveRelayIndex
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
