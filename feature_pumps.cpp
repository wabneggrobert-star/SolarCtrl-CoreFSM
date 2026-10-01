#include "feature_pumps.h"

#include "config.h"
#include "feature_energy_conflicts.h"
#include "feature_ml_optimizer.h"
#include "feature_pump_routing.h"
#include "feature_relay_outputs.h"
#include "feature_pwm_pca9685.h"
#include "feature_sensor_assignments.h"
#include "feature_valves.h"
#include "feature_time.h"

#include <Arduino.h>
#include <SD.h>
#include <math.h>

namespace {
  struct PidState {
    float integral = 0.0f;
    float lastError = 0.0f;
  };

  PidState pid[MAX_PUMPS];

  // Feldtest-Diagnose: PWM-Regelverlauf fuer spaetere PID-Analyse.
  // Bewusst nur Logging; die eigentliche Pumpenregelung wird nicht veraendert.
  constexpr const char* PWM_TRACE_PATH = "/logs/pwm_trace.csv";
  constexpr uint32_t PWM_TRACE_INTERVAL_MS = 5000UL;
  uint32_t pwmTraceLastMs[MAX_PUMPS] = {};

  float effectiveTargetDiffForTrace(const PumpConfig& p) {
    float target = p.targetDiff;
    if (p.activeTargetIndex < PUMP_ROUTE_TARGET_COUNT) {
      const PumpRouteTargetConfig& t = p.targets[p.activeTargetIndex];
      if (t.enabled && t.targetDiffOverride > 0.0f) target = t.targetDiffOverride;
    }
    return target;
  }

  float effectiveHysteresisForTrace(const PumpConfig& p) {
    float hyst = p.hysteresis;
    if (p.activeTargetIndex < PUMP_ROUTE_TARGET_COUNT) {
      const PumpRouteTargetConfig& t = p.targets[p.activeTargetIndex];
      if (t.enabled && t.hysteresisOverride > 0.0f) hyst = t.hysteresisOverride;
    }
    return hyst;
  }

  void logPwmTrace(AppContext& ctx) {
    const uint32_t nowMs = millis();

    bool needsWrite = false;
    for (uint8_t i = 0; i < MAX_PUMPS; i++) {
      const PumpConfig& p = ctx.config.pumps[i];
      if (!p.enabled || p.mode != PumpMode::PWM) continue;
      if (pwmTraceLastMs[i] == 0 || (uint32_t)(nowMs - pwmTraceLastMs[i]) >= PWM_TRACE_INTERVAL_MS) {
        needsWrite = true;
        break;
      }
    }
    if (!needsWrite) return;

    if (!SD.exists("/logs")) SD.mkdir("/logs");
    const bool fresh = !SD.exists(PWM_TRACE_PATH);
    File f = SD.open(PWM_TRACE_PATH, FILE_APPEND);
    if (!f) return;

    if (fresh) {
      f.println("timestamp,uptime_ms,pump_index,state,active_target,source_c,sink_c,diff_c,target_diff_c,start_diff_c,hysteresis_c,pwm_pct,min_pwm_pct,max_pwm_pct,pid_kp,pid_ki,pid_kd,pid_error,pid_integral");
    }

    const String timestamp = TimeService::isoTimestamp();

    for (uint8_t i = 0; i < MAX_PUMPS; i++) {
      const PumpConfig& p = ctx.config.pumps[i];
      if (!p.enabled || p.mode != PumpMode::PWM) continue;
      if (pwmTraceLastMs[i] != 0 && (uint32_t)(nowMs - pwmTraceLastMs[i]) < PWM_TRACE_INTERVAL_MS) continue;

      const float targetDiff = effectiveTargetDiffForTrace(p);
      const float hysteresis = effectiveHysteresisForTrace(p);
      const float error = isfinite(p.lastDiffC) ? (p.lastDiffC - targetDiff) : NAN;

      f.print(timestamp); f.print(',');
      f.print(nowMs); f.print(',');
      f.print(i); f.print(',');
      f.print(p.state ? 1 : 0); f.print(',');
      if (p.activeTargetIndex < PUMP_ROUTE_TARGET_COUNT) f.print(p.activeTargetIndex);
      else f.print(-1);
      f.print(',');
      if (isfinite(p.lastSourceC)) f.print(p.lastSourceC, 2);
      f.print(',');
      if (isfinite(p.lastSinkC)) f.print(p.lastSinkC, 2);
      f.print(',');
      if (isfinite(p.lastDiffC)) f.print(p.lastDiffC, 2);
      f.print(',');
      f.print(targetDiff, 2); f.print(',');
      f.print(p.startDiff, 2); f.print(',');
      f.print(hysteresis, 2); f.print(',');
      f.print(p.lastPwmPercent, 1); f.print(',');
      f.print(p.minPwmPercent, 1); f.print(',');
      f.print(p.maxPwmPercent, 1); f.print(',');
      f.print(p.pidKp, 4); f.print(',');
      f.print(p.pidKi, 4); f.print(',');
      f.print(p.pidKd, 4); f.print(',');
      if (isfinite(error)) f.print(error, 2);
      f.print(',');
      f.println(pid[i].integral, 4);

      pwmTraceLastMs[i] = nowMs;
    }

    f.close();
  }

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
  if (pumpIndex >= FEEDBACK_INPUT_COUNT) return PIN_UNUSED;
  return FEEDBACK_INPUT_PINS[pumpIndex];
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


  bool isStorageSensorRole(Ds18Role role) {
    switch (role) {
      case Ds18Role::SINK_BOILER_TOP:
      case Ds18Role::SINK_BUFFER_TOP:
      case Ds18Role::BOILER_BOTTOM:
      case Ds18Role::BUFFER_HIGH:
      case Ds18Role::BUFFER_MID:
      case Ds18Role::BUFFER_BOTTOM:
        return true;
      default:
        return false;
    }
  }

  bool ensureValveTarget(AppContext& ctx, uint8_t pumpIndex, PumpConfig& pump, uint8_t targetIndex) {
    if (pump.valveIndex == PIN_UNUSED || !Valves::isConfigured(ctx, pump.valveIndex)) {
      return true;
    }
    if (targetIndex >= PUMP_ROUTE_TARGET_COUNT) return false;

    const ValvePosition wanted = targetIndex == 1 ? ValvePosition::B : ValvePosition::A;
    if (Valves::currentPosition(pump.valveIndex) == wanted && !Valves::isMoving(pump.valveIndex)) {
      pump.activeTargetIndex = targetIndex;
      pump.valvePendingTargetIndex = PIN_UNUSED;
      return true;
    }

    if (!Valves::requestPosition(ctx, pump.valveIndex, wanted)) return false;
    pump.valvePendingTargetIndex = targetIndex;

    if (Valves::isMoving(pump.valveIndex)) {
      stopPumpZeroPercent(ctx, pumpIndex, pump);
      Serial.print("SAFETY SPEICHERKUEHLUNG P");
      Serial.print(pumpIndex + 1);
      Serial.print(": Ventil faehrt auf Ziel ");
      Serial.println(targetIndex == 1 ? "B" : "A");
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

    // Pump Enable AUS beendet den aktuellen Valve-V2 Ladezyklus. Beim
    // naechsten regulaeren Start muss die Zielprioritaet A/B neu bewertet
    // werden. Eine laufende Ventilfahrt verwendet stopPumpZeroPercent() und
    // landet bewusst nicht hier, damit ihr Pending-Ziel erhalten bleibt.
    PumpRouting::closeAllTargets(ctx, i);

    resetPumpRuntime(p);
    pid[i] = {};

    if (p.relayIndex != PIN_UNUSED) {
      RelayOutputs::set(ctx, p.relayIndex, false);
    }

    // setPwmPercent() arbeitet mit LOGISCHER Pumpenleistung:
    // 0 % bedeutet fuer beide UPM3-Profile sicher AUS. Die unterschiedliche
    // elektrische Kennlinie von SOLAR/HEATING sowie die PCB-Invertierung
    // werden ausschliesslich zentral in PwmDriver::setDuty() umgesetzt.
    //
    // Wichtig: HEATING darf hier NICHT auf 100 % gesetzt werden. Auf der
    // invertierenden SolarCtrl-PCB wuerde das am UPM3-Profil A 0 % Eingang
    // ergeben und damit maximale Drehzahl statt AUS.
    setPwmPercent(p, 0.0f);
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
        // ML darf nur innerhalb der vom Nutzer gesetzten Pumpengrenzen feinoptimieren.
        // Bei nicht aktivem/ungueltigem Modell wird der Wert unveraendert zurueckgegeben.
        pwmPercent = MlOptimizer::adjustSolarPumpPwm(ctx, i, pwmPercent);
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

    // Ein regulaeres Pump-Enable-AUS beendet ebenfalls den Ladezyklus. Das ist
    // insbesondere bei RELAY/PWM-Abschaltung durch die Differenzregelung
    // wichtig: Der naechste Pumpenstart soll A/B wieder frisch bewerten.
    if (!relayOn) {
      PumpRouting::closeAllTargets(ctx, i);
    }

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

  // Diagnose-Logging erst nach dem kompletten Pumpenlauf, damit der CSV-Datensatz
  // exakt den fuer diesen Zyklus berechneten Runtime-/PWM-Zustand enthaelt.
  logPwmTrace(ctx);
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


uint8_t safetyForceStorageCooling(AppContext& ctx, float criticalTemperatureC, float minimumDeltaC, float pwmPercent) {
  uint8_t runningCount = 0;
  // Normal control did not run in this safety cycle; discard stale reservations
  // before selecting emergency cooling routes.
  EnergyConflicts::clear(ctx);
  pwmPercent = clampFloat(pwmPercent, 0.0f, 100.0f);
  minimumDeltaC = minimumDeltaC < 1.0f ? 1.0f : minimumDeltaC;

  // 1) Direct configured transfer paths: hot storage sensor -> cooler configured sink.
  // Example: buffer top -> boiler top. Normal min/target delta is deliberately
  // bypassed here, but hardware assignment, measured temperatures and sink max
  // remain respected.
  for (uint8_t i = 0; i < MAX_PUMPS; i++) {
    PumpConfig& pump = ctx.config.pumps[i];
    if (!pump.enabled || pump.mode == PumpMode::OFF) continue;
    if (pump.sourceType != PumpSourceType::SENSOR_ROLE) continue;
    if (!isStorageSensorRole(pump.sourceSensorRole)) continue;
    if (!pumpEnableRelayReady(ctx, pump)) continue;

    float sourceC = NAN;
    bool sourceValid = false;
    if (!pumpSourceTemp(ctx, pump, sourceC, sourceValid) || !sourceValid || isnan(sourceC)) {
      stopPumpZeroPercent(ctx, i, pump);
      continue;
    }

    // Only a genuinely critical/hot source may drive emergency heat dump.
    if (sourceC < criticalTemperatureC) {
      stopPumpZeroPercent(ctx, i, pump);
      continue;
    }

    PumpRouting::RouteResult route = PumpRouting::resolveHeatDumpCoolestTarget(ctx, i, sourceC, sourceValid);
    if (!route.active || !route.sinkValid || isnan(route.sinkC) || route.diffC < minimumDeltaC) {
      stopPumpZeroPercent(ctx, i, pump);
      continue;
    }

    const OutputRef valveOutput = (pump.valveIndex != PIN_UNUSED && Valves::isConfigured(ctx, pump.valveIndex))
      ? Valves::outputRef(ctx, pump.valveIndex)
      : OutputRef{};
    if (!EnergyConflicts::canActivateRoute(ctx, i, pump.sourceRole, route.sinkRole, pump.relayIndex, valveOutput)) {
      stopPumpZeroPercent(ctx, i, pump);
      continue;
    }

    if (route.valveMoving) {
      stopPumpZeroPercent(ctx, i, pump);
      continue;
    }

    EnergyConflicts::reserveRoute(ctx, i, pump.sourceRole, route.sinkRole, pump.relayIndex, valveOutput);
    RelayOutputs::set(ctx, pump.relayIndex, true);
    pump.state = true;
    pump.lastSourceC = sourceC;
    pump.lastSinkC = route.sinkC;
    pump.lastDiffC = route.diffC;
    pump.lastPwmPercent = pwmPercent;
    if (pump.mode == PumpMode::PWM && pump.pwmChannel != PIN_UNUSED) setPwmPercent(pump, pwmPercent);

    Serial.print("SAFETY SPEICHERKUEHLUNG P");
    Serial.print(i + 1);
    Serial.print(" | Speicher="); Serial.print(sourceC);
    Serial.print(" C | Ziel="); Serial.print(route.sinkC);
    Serial.print(" C | Delta="); Serial.print(route.diffC);
    Serial.print(" K | PWM="); Serial.println(pwmPercent);
    Serial.flush();
    runningCount++;
  }

  // 2) A configured solar loop can be used as a cooler heat sink. The pump keeps
  // its normal hydraulic direction; heat transfer simply reverses when the
  // connected storage is hotter than the collector. Only the hot configured
  // storage target is selected; this avoids cooling an unrelated cool target.
  for (uint8_t i = 0; i < MAX_PUMPS; i++) {
    PumpConfig& pump = ctx.config.pumps[i];
    if (!pump.enabled || pump.mode == PumpMode::OFF || !isSolarCollectorPump(pump)) continue;
    if (!pumpEnableRelayReady(ctx, pump)) continue;

    float collectorC = NAN;
    bool collectorValid = false;
    if (!pumpSourceTemp(ctx, pump, collectorC, collectorValid) || !collectorValid || isnan(collectorC)) {
      stopPumpZeroPercent(ctx, i, pump);
      continue;
    }

    int8_t hotTargetIndex = -1;
    Ds18Role hotSinkRole = Ds18Role::NONE;
    float hotStorageC = NAN;

    const bool hasValve = pump.valveIndex != PIN_UNUSED && Valves::isConfigured(ctx, pump.valveIndex);
    if (hasValve) {
      for (uint8_t t = 0; t < PUMP_ROUTE_TARGET_COUNT; t++) {
        const PumpRouteTargetConfig& target = pump.targets[t];
        if (!target.enabled || !isStorageSensorRole(target.sinkRole)) continue;
        float sinkC = NAN; bool sinkValid = false;
        if (!sensorSourceTempByRole(ctx, target.sinkRole, sinkC, sinkValid) || !sinkValid) continue;
        if (sinkC < criticalTemperatureC) continue;
        if ((sinkC - collectorC) < minimumDeltaC) continue;
        if (hotTargetIndex < 0 || sinkC > hotStorageC) {
          hotTargetIndex = (int8_t)t;
          hotSinkRole = target.sinkRole;
          hotStorageC = sinkC;
        }
      }
    } else if (isStorageSensorRole(pump.sinkRole)) {
      float sinkC = NAN; bool sinkValid = false;
      if (sensorSourceTempByRole(ctx, pump.sinkRole, sinkC, sinkValid) && sinkValid &&
          sinkC >= criticalTemperatureC && (sinkC - collectorC) >= minimumDeltaC) {
        hotTargetIndex = 0;
        hotSinkRole = pump.sinkRole;
        hotStorageC = sinkC;
      }
    }

    if (hotTargetIndex < 0 || hotSinkRole == Ds18Role::NONE) {
      stopPumpZeroPercent(ctx, i, pump);
      continue;
    }

    const OutputRef valveOutput = hasValve ? Valves::outputRef(ctx, pump.valveIndex) : OutputRef{};
    if (!EnergyConflicts::canActivateRoute(ctx, i, pump.sourceRole, hotSinkRole, pump.relayIndex, valveOutput)) {
      stopPumpZeroPercent(ctx, i, pump);
      continue;
    }

    if (hasValve && !ensureValveTarget(ctx, i, pump, (uint8_t)hotTargetIndex)) continue;

    EnergyConflicts::reserveRoute(ctx, i, pump.sourceRole, hotSinkRole, pump.relayIndex, valveOutput);
    RelayOutputs::set(ctx, pump.relayIndex, true);
    pump.state = true;
    pump.lastSourceC = collectorC;
    pump.lastSinkC = hotStorageC;
    pump.lastDiffC = collectorC - hotStorageC; // negative = reverse heat flow, intentional
    pump.lastPwmPercent = pwmPercent;
    if (pump.mode == PumpMode::PWM && pump.pwmChannel != PIN_UNUSED) setPwmPercent(pump, pwmPercent);

    Serial.print("SAFETY SOLAR-RUECKKUEHLUNG P");
    Serial.print(i + 1);
    Serial.print(" | Speicher="); Serial.print(hotStorageC);
    Serial.print(" C | Kollektor="); Serial.print(collectorC);
    Serial.print(" C | Delta="); Serial.print(hotStorageC - collectorC);
    Serial.print(" K | PWM="); Serial.println(pwmPercent);
    Serial.flush();
    runningCount++;
  }

  // 3) Auch eine konfigurierte, deutlich kuehlere alternative Waermequelle
  // (insbesondere der wassergefuehrte Ofen) darf im kritischen Speicherfall
  // als Not-Waermesenke genutzt werden. Die Hydraulik bleibt unveraendert;
  // durch den grossen negativen Temperaturgradienten fliesst Waerme vom
  // Speicher zur kuehleren Quelle. Nur explizit konfigurierte Pumpenpfade
  // werden verwendet und nur wenn die Quellentemperatur gueltig ist.
  for (uint8_t i = 0; i < MAX_PUMPS; i++) {
    PumpConfig& pump = ctx.config.pumps[i];
    if (!pump.enabled || pump.mode == PumpMode::OFF) continue;
    if (pump.sourceType != PumpSourceType::HEAT_SOURCE_ROLE) continue;
    if (pump.sourceRole != HeatSourceRole::ALT_SOURCE_OVEN &&
        pump.sourceRole != HeatSourceRole::ALT_SOURCE_OTHER) continue;
    if (!pumpEnableRelayReady(ctx, pump)) continue;

    float sourceC = NAN;
    bool sourceValid = false;
    if (!pumpSourceTemp(ctx, pump, sourceC, sourceValid) || !sourceValid || isnan(sourceC)) {
      stopPumpZeroPercent(ctx, i, pump);
      continue;
    }

    int8_t hotTargetIndex = -1;
    Ds18Role hotSinkRole = Ds18Role::NONE;
    float hotStorageC = NAN;
    const bool hasValve = pump.valveIndex != PIN_UNUSED && Valves::isConfigured(ctx, pump.valveIndex);

    if (hasValve) {
      for (uint8_t t = 0; t < PUMP_ROUTE_TARGET_COUNT; t++) {
        const PumpRouteTargetConfig& target = pump.targets[t];
        if (!target.enabled || !isStorageSensorRole(target.sinkRole)) continue;
        float sinkC = NAN;
        bool sinkValid = false;
        if (!sensorSourceTempByRole(ctx, target.sinkRole, sinkC, sinkValid) || !sinkValid || isnan(sinkC)) continue;
        if (sinkC < criticalTemperatureC) continue;
        if ((sinkC - sourceC) < minimumDeltaC) continue;
        if (hotTargetIndex < 0 || sinkC > hotStorageC) {
          hotTargetIndex = (int8_t)t;
          hotSinkRole = target.sinkRole;
          hotStorageC = sinkC;
        }
      }
    } else if (isStorageSensorRole(pump.sinkRole)) {
      float sinkC = NAN;
      bool sinkValid = false;
      if (sensorSourceTempByRole(ctx, pump.sinkRole, sinkC, sinkValid) && sinkValid && !isnan(sinkC) &&
          sinkC >= criticalTemperatureC && (sinkC - sourceC) >= minimumDeltaC) {
        hotTargetIndex = 0;
        hotSinkRole = pump.sinkRole;
        hotStorageC = sinkC;
      }
    }

    if (hotTargetIndex < 0 || hotSinkRole == Ds18Role::NONE) {
      stopPumpZeroPercent(ctx, i, pump);
      continue;
    }

    const OutputRef valveOutput = hasValve ? Valves::outputRef(ctx, pump.valveIndex) : OutputRef{};
    if (!EnergyConflicts::canActivateRoute(ctx, i, pump.sourceRole, hotSinkRole, pump.relayIndex, valveOutput)) {
      stopPumpZeroPercent(ctx, i, pump);
      continue;
    }

    if (hasValve && !ensureValveTarget(ctx, i, pump, (uint8_t)hotTargetIndex)) continue;

    EnergyConflicts::reserveRoute(ctx, i, pump.sourceRole, hotSinkRole, pump.relayIndex, valveOutput);
    RelayOutputs::set(ctx, pump.relayIndex, true);
    pump.state = true;
    pump.lastSourceC = sourceC;
    pump.lastSinkC = hotStorageC;
    pump.lastDiffC = sourceC - hotStorageC; // negativ = beabsichtigte Rueckkuehlung
    pump.lastPwmPercent = pwmPercent;
    if (pump.mode == PumpMode::PWM && pump.pwmChannel != PIN_UNUSED) setPwmPercent(pump, pwmPercent);

    Serial.print(pump.sourceRole == HeatSourceRole::ALT_SOURCE_OVEN
      ? "SAFETY OFEN-RUECKKUEHLUNG P"
      : "SAFETY QUELLEN-RUECKKUEHLUNG P");
    Serial.print(i + 1);
    Serial.print(" | Speicher="); Serial.print(hotStorageC);
    Serial.print(" C | Quelle="); Serial.print(sourceC);
    Serial.print(" C | Delta="); Serial.print(hotStorageC - sourceC);
    Serial.print(" K | PWM="); Serial.println(pwmPercent);
    Serial.flush();
    runningCount++;
  }

  return runningCount;
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
