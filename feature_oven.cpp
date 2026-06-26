#include "feature_oven.h"
#include "feature_sensor_assignments.h"
#include "config.h"
#include "feature_relay_outputs.h"
#include "feature_safety_manager.h"

#include <Arduino.h>
#include <math.h>

namespace {

enum class OvenState : uint8_t {
  STANDBY,
  REGULATING,
  SAFETY
};

OvenState g_state = OvenState::STANDBY;

bool g_userRequestedActive = false;
bool g_autoStarted = false;
bool g_pumpActive = false;
uint8_t g_servoAngle = 0;
float g_ovenTemperatureC = NAN;
float g_lastStandbyTemperatureC = NAN;
float g_peakTemperatureC = NAN;

float g_integral = 0.0f;
float g_lastError = 0.0f;
uint32_t g_lastPidMs = 0;

static constexpr uint32_t OVEN_PID_INTERVAL_MS = 1000;
static constexpr uint32_t SERVO_PWM_FREQUENCY_HZ = 50;
static constexpr uint8_t SERVO_PWM_RESOLUTION_BITS = 16;
static constexpr uint32_t SERVO_PWM_MAX_DUTY = (1UL << SERVO_PWM_RESOLUTION_BITS) - 1UL;

uint32_t servoPulseUsToDuty(uint16_t pulseUs) {
  return (uint32_t)((uint64_t)pulseUs * SERVO_PWM_MAX_DUTY / 20000ULL);
}

uint16_t angleToPulseUs(uint8_t angle) {
  return (uint16_t)(500 + ((uint32_t)angle * 2000UL / 180UL));
}

float clampFloat(float value, float minimum, float maximum) {
  if (value < minimum) return minimum;
  if (value > maximum) return maximum;
  return value;
}

uint8_t clampAngle(float value) {
  if (value < 0.0f) return 0;
  if (value > 180.0f) return 180;
  return (uint8_t)(value + 0.5f);
}

void writeServoAngle(uint8_t angle) {
  g_servoAngle = angle;
  const uint16_t pulseUs = angleToPulseUs(angle);
  const uint32_t duty = servoPulseUsToDuty(pulseUs);
  ledcWrite(OVEN_SERVO_PIN, duty);
}

void closeAirFlap() {
  writeServoAngle(0);
}

void setOvenPump(AppContext& ctx, bool on) {
  const uint8_t relayIndex = ctx.config.oven.pumpRelay;

  if (relayIndex == PIN_UNUSED) {
    g_pumpActive = false;
    return;
  }

  RelayOutputs::set(ctx, relayIndex, on);
  g_pumpActive = on;
}

bool readOvenTemperature(const AppContext& ctx, float& temperatureC) {
  temperatureC = NAN;

  if (!ctx.sensors.heatSources.altSourceOvenValid) return false;
  if (isnan(ctx.sensors.heatSources.altSourceOvenC)) return false;

  temperatureC = ctx.sensors.heatSources.altSourceOvenC;
  return true;
}

bool readTargetTemperature(AppContext& ctx, float& temperatureC) {
  temperatureC = NAN;

  const Ds18Role role = ctx.config.oven.targetSinkRole;
  if (role == Ds18Role::NONE) return false;

  bool valid = false;
  SensorAssignments::readByRole(ctx.assignments, role, temperatureC, valid);
  return valid && !isnan(temperatureC);
}

bool safetyRequiresOvenShutdown() {
  const SafetyManager::SafetyStatus& safety = SafetyManager::status();

  if (safety.ovenOvertemperatureActive) return true;
  if (safety.collectorStagnationActive) return true; // Solar hat Vorrang: Ofenluft zu.
  if (safety.storageOvertemperatureActive) return true;

  return false;
}

void resetPid() {
  g_integral = 0.0f;
  g_lastError = 0.0f;
  g_lastPidMs = 0;
}

void enterStandby(AppContext& ctx) {
  closeAirFlap();
  setOvenPump(ctx, false);
  resetPid();
  g_state = OvenState::STANDBY;
  g_userRequestedActive = false;
  g_autoStarted = false;
  g_peakTemperatureC = NAN;
}

void activateOven(const OvenConfig& cfg, bool automaticStart) {
  if (!g_userRequestedActive && !g_autoStarted) {
    writeServoAngle(cfg.servoBaseAngle);
    resetPid();
  }

  if (automaticStart) g_autoStarted = true;
  g_state = OvenState::REGULATING;
}

void updateAutoStart(const OvenConfig& cfg, float ovenTemperatureC) {
  if (!cfg.autoStartEnabled) return;
  if (g_userRequestedActive || g_autoStarted) return;

  bool startByTemperature = ovenTemperatureC >= cfg.autoStartTemperatureC;

  bool startByRise = false;
  if (!isnan(g_lastStandbyTemperatureC)) {
    startByRise = (ovenTemperatureC - g_lastStandbyTemperatureC) >= cfg.autoStartRiseC;
  }

  if (startByTemperature || startByRise) {
    Serial.println("OVEN: Automatischer Start erkannt");
    Serial.flush();
    activateOven(cfg, true);
  }
}

void updatePeak(float ovenTemperatureC) {
  if (isnan(g_peakTemperatureC) || ovenTemperatureC > g_peakTemperatureC) {
    g_peakTemperatureC = ovenTemperatureC;
  }
}

void updatePump(AppContext& ctx, float ovenTemperatureC) {
  const OvenConfig& cfg = ctx.config.oven;

  float targetTemperatureC = NAN;
  const bool targetValid = readTargetTemperature(ctx, targetTemperatureC);

  bool shouldRun = g_pumpActive;

  if (!g_pumpActive) {
    const bool ovenHotEnough = ovenTemperatureC >= cfg.pumpOnTemperatureC;
    const bool targetAllowsHeat =
        !targetValid || ovenTemperatureC >= (targetTemperatureC + cfg.pumpOnTemperatureDifferenceC);

    if (ovenHotEnough && targetAllowsHeat) {
      shouldRun = true;
    }
  } else {
    const bool ovenTooCold =
        ovenTemperatureC <= (cfg.pumpOnTemperatureC - cfg.pumpHysteresisC);

    const bool wouldDischargeStorage =
        targetValid && ovenTemperatureC <= (targetTemperatureC + cfg.pumpOffTemperatureDifferenceC);

    const bool droppedFromPeak =
        !isnan(g_peakTemperatureC) &&
        ovenTemperatureC <= (g_peakTemperatureC - cfg.pumpStopDropFromPeakC);

    if (ovenTooCold || wouldDischargeStorage || droppedFromPeak) {
      shouldRun = false;
    }
  }

  setOvenPump(ctx, shouldRun);
}

void updateServoPid(AppContext& ctx, float ovenTemperatureC) {
  OvenConfig& cfg = ctx.config.oven;

  const uint32_t now = millis();

  if (g_lastPidMs == 0) {
    g_lastPidMs = now;
    return;
  }

  const uint32_t elapsedMs = now - g_lastPidMs;
  if (elapsedMs < OVEN_PID_INTERVAL_MS) return;

  const float dt = elapsedMs / 1000.0f;
  g_lastPidMs = now;

  // Positiver Fehler: Ofen ist zu kalt -> Luft weiter oeffnen.
  const float error = cfg.targetOvenTemperatureC - ovenTemperatureC;

  g_integral += error * dt;
  g_integral = clampFloat(g_integral, -100.0f, 100.0f);

  const float derivative = (error - g_lastError) / dt;
  g_lastError = error;

  float output =
      cfg.servoBaseAngle
    + cfg.pidKp * error
    + cfg.pidKi * g_integral
    + cfg.pidKd * derivative;

  output = clampFloat(output, cfg.servoMinimumAngle, cfg.servoMaximumAngle);
  writeServoAngle(clampAngle(output));
}

} // namespace

namespace OvenControl {

void begin(AppContext& ctx) {
  (void)ctx;

  ledcAttach(OVEN_SERVO_PIN, SERVO_PWM_FREQUENCY_HZ, SERVO_PWM_RESOLUTION_BITS);
  closeAirFlap();
  resetPid();

  Serial.println("OvenControl::begin OK");
  Serial.flush();
}

void allOff(AppContext& ctx) {
  enterStandby(ctx);
}

void requestStart() {
  g_userRequestedActive = true;
  g_autoStarted = false;
  g_peakTemperatureC = NAN;
}

void requestStop(AppContext& ctx) {
  enterStandby(ctx);
}

bool active() {
  return g_state == OvenState::REGULATING;
}

bool userRequestedActive() {
  return g_userRequestedActive;
}

bool autoStarted() {
  return g_autoStarted;
}

bool pumpActive() {
  return g_pumpActive;
}

uint8_t servoAngle() {
  return g_servoAngle;
}

float ovenTemperatureC() {
  return g_ovenTemperatureC;
}

float peakTemperatureC() {
  return g_peakTemperatureC;
}

const char* stateText() {
  switch (g_state) {
    case OvenState::STANDBY: return "Standby";
    case OvenState::REGULATING: return "Aktiv";
    case OvenState::SAFETY: return "Safety";
    default: return "Unbekannt";
  }
}

void process(AppContext& ctx) {
  OvenConfig& cfg = ctx.config.oven;

  if (!cfg.enabled) {
    enterStandby(ctx);
    return;
  }

  float ovenTemperature = NAN;
  if (!readOvenTemperature(ctx, ovenTemperature)) {
    enterStandby(ctx);
    g_ovenTemperatureC = NAN;
    return;
  }

  g_ovenTemperatureC = ovenTemperature;

  if (safetyRequiresOvenShutdown() || ovenTemperature >= cfg.criticalOvenTemperatureC) {
    closeAirFlap();

    // Bei kritischem Ofen: Pumpe einschalten, um Waerme abzufuehren.
    if (ovenTemperature >= cfg.criticalOvenTemperatureC) {
      setOvenPump(ctx, true);
    } else {
      setOvenPump(ctx, false);
    }

    resetPid();
    g_state = OvenState::SAFETY;
    g_userRequestedActive = false;
    g_autoStarted = false;

    Serial.println("OVEN SAFETY: Luftklappe geschlossen");
    Serial.flush();
    return;
  }

  updateAutoStart(cfg, ovenTemperature);

  if (!g_userRequestedActive && !g_autoStarted) {
    // Standby: Ofen ist nicht freigegeben. Luftklappe geschlossen, Pumpe aus.
    closeAirFlap();
    setOvenPump(ctx, false);
    resetPid();
    g_state = OvenState::STANDBY;

    if (isnan(g_lastStandbyTemperatureC)) {
      g_lastStandbyTemperatureC = ovenTemperature;
    } else if (ovenTemperature < g_lastStandbyTemperatureC) {
      // Referenz langsam nach unten nachfuehren.
      g_lastStandbyTemperatureC = ovenTemperature;
    }
    return;
  }

  g_lastStandbyTemperatureC = ovenTemperature;
  activateOven(cfg, false);
  updatePeak(ovenTemperature);

  updatePump(ctx, ovenTemperature);
  updateServoPid(ctx, ovenTemperature);

  if (!g_pumpActive && ovenTemperature <= cfg.autoReturnToStandbyTemperatureC && g_autoStarted) {
    enterStandby(ctx);
  }
}

} // namespace OvenControl
