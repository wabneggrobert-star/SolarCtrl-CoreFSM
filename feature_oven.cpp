#include "feature_oven.h"
#include "feature_sensor_assignments.h"
#include "config.h"
#include "feature_relay_outputs.h"
#include "feature_pwm_pca9685.h"
#include "feature_pumps.h"
#include "feature_safety_manager.h"
#include "feature_servo_driver.h"

#include <Arduino.h>
#include <math.h>

namespace {

enum class OvenState : uint8_t {
  STANDBY,
  REGULATING,
  BURNOUT_VENTING,
  EMBER_VENTING,
  SAFETY
};

OvenState g_state = OvenState::STANDBY;

bool g_userRequestedActive = false;
bool g_autoStarted = false;
bool g_pumpActive = false;
bool g_targetTemperatureReached = false;
uint8_t g_servoAngle = 0;
uint8_t g_servoOpeningPercent = 0;
float g_ovenTemperatureC = NAN;
float g_lastStandbyTemperatureC = NAN;
float g_peakTemperatureC = NAN;

uint32_t g_lastServoControlMs = 0;
uint32_t g_lastPeakReachedMs = 0;
uint32_t g_burnoutStartedMs = 0;
bool g_lastPumpActive = false;

static constexpr float OVEN_PEAK_REACHED_EPSILON_C = 0.2f;

static constexpr uint32_t OVEN_SERVO_CONTROL_INTERVAL_MS = 1000;

void updateServoControl(AppContext& ctx, float ovenTemperatureC);

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

uint8_t clampPercent(int value) {
  if (value < 0) return 0;
  if (value > 100) return 100;
  return (uint8_t)value;
}

uint8_t mapOpeningToServoAngle(const OvenConfig& cfg, uint8_t openingPercent) {
  const float pct = clampPercent(openingPercent) / 100.0f;
  const float angle = (float)cfg.servoClosedAngle + ((float)cfg.servoOpenAngle - (float)cfg.servoClosedAngle) * pct;
  return clampAngle(angle);
}

void writeServoOpening(const OvenConfig& cfg, uint8_t openingPercent) {
  g_servoOpeningPercent = clampPercent(openingPercent);
  g_servoAngle = ServoDriver::setAngle(mapOpeningToServoAngle(cfg, g_servoOpeningPercent));
}

void closeAirFlap(const OvenConfig& cfg) {
  // Standby/Stop/Safety verwenden die kalibrierte 0-%-Oeffnung.
  // Bei Robert aktuell typisch: 0 % = 45°, 100 % = 135°.
  writeServoOpening(cfg, cfg.servoStandbyOpeningPercent);
}

void openAirFlapForStart(const OvenConfig& cfg) {
  writeServoOpening(cfg, cfg.servoStartOpeningPercent);
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

void resetServoControl() {
  g_lastServoControlMs = 0;
  g_targetTemperatureReached = false;
}

void clearPeak() {
  g_peakTemperatureC = NAN;
  g_lastPeakReachedMs = 0;
}

void initPeak(float ovenTemperatureC) {
  if (!isnan(ovenTemperatureC)) {
    g_peakTemperatureC = ovenTemperatureC;
    g_lastPeakReachedMs = millis();
  } else {
    clearPeak();
  }
}

void updatePeak(float ovenTemperatureC) {
  if (isnan(ovenTemperatureC)) return;

  const uint32_t now = millis();
  if (isnan(g_peakTemperatureC) || ovenTemperatureC > (g_peakTemperatureC + OVEN_PEAK_REACHED_EPSILON_C)) {
    g_peakTemperatureC = ovenTemperatureC;
    g_lastPeakReachedMs = now;
    return;
  }

  // Solange der Ofen wieder bis knapp an den bisherigen Peak herankommt,
  // gilt der Peak als noch erreicht. Erst wenn das laenger ausbleibt, wird
  // spaeter der Abbrand-/Nachlauf gestartet.
  if (ovenTemperatureC >= (g_peakTemperatureC - OVEN_PEAK_REACHED_EPSILON_C)) {
    g_lastPeakReachedMs = now;
  }
}

void enterStandby(AppContext& ctx) {
  closeAirFlap(ctx.config.oven);
  Pumps::applyOvenPumpRequest(ctx, false, NAN, NAN, false);
  g_pumpActive = false;
  resetServoControl();
  g_state = OvenState::STANDBY;
  g_userRequestedActive = false;
  g_autoStarted = false;
  g_burnoutStartedMs = 0;
  g_lastPumpActive = false;
  clearPeak();
}

void startBurnoutVenting(AppContext& ctx) {
  (void)ctx;
  if (g_state == OvenState::SAFETY || g_state == OvenState::STANDBY) return;
  if (g_state == OvenState::BURNOUT_VENTING || g_state == OvenState::EMBER_VENTING) return;

  g_burnoutStartedMs = millis();
  g_userRequestedActive = false;
  g_autoStarted = false;
  g_state = OvenState::BURNOUT_VENTING;

  Serial.println("OVEN: Abbrand-/Nachlauf gestartet");
  Serial.flush();
}

uint32_t burnoutMainMs(const OvenConfig& cfg) {
  if (cfg.burnoutVentMinutes == 0) return 0;
  return (uint32_t)cfg.burnoutVentMinutes * 60000UL;
}

uint32_t burnoutEmberMs(const OvenConfig& cfg) {
  return burnoutMainMs(cfg) / 2UL;
}

bool peakTimeoutReached(const OvenConfig& cfg) {
  const uint32_t timeoutMs = burnoutMainMs(cfg);
  if (timeoutMs == 0) return false;
  if (isnan(g_peakTemperatureC) || g_lastPeakReachedMs == 0) return false;
  return (uint32_t)(millis() - g_lastPeakReachedMs) >= timeoutMs;
}

void updateBurnoutVenting(AppContext& ctx, float ovenTemperatureC) {
  OvenConfig& cfg = ctx.config.oven;

  const uint32_t mainMs = burnoutMainMs(cfg);
  const uint32_t emberMs = burnoutEmberMs(cfg);
  if (mainMs == 0 || g_burnoutStartedMs == 0) {
    enterStandby(ctx);
    return;
  }

  Pumps::applyOvenPumpRequest(ctx, false, ovenTemperatureC, NAN, false);
  g_pumpActive = false;

  const uint32_t elapsedMs = millis() - g_burnoutStartedMs;
  if (elapsedMs < mainMs) {
    // Haupt-Nachlauf: normale Servo-Regelung bleibt aktiv.
    updateServoControl(ctx, ovenTemperatureC);
    return;
  }

  if (elapsedMs < (mainMs + emberMs)) {
    // Restglutphase: halbe eingestellte Zeit mit minimaler Regeloeffnung.
    g_state = OvenState::EMBER_VENTING;
    writeServoOpening(cfg, cfg.servoMinimumOpeningPercent);
    return;
  }

  enterStandby(ctx);
}

void activateOven(const OvenConfig& cfg, bool automaticStart) {
  if (g_state != OvenState::REGULATING) {
    openAirFlapForStart(cfg);
    resetServoControl();
  }

  if (automaticStart) g_autoStarted = true;
  g_state = OvenState::REGULATING;
}

void updateAutoStart(const OvenConfig& cfg, float ovenTemperatureC) {
  if (!cfg.autoStartEnabled) return;
  if (g_userRequestedActive || g_autoStarted) return;

  bool startByTemperature = ovenTemperatureC >= cfg.autoStartTemperatureC;

  bool startByRise = false;
  if (!isnan(g_lastStandbyTemperatureC) && cfg.autoStartRiseC > 0.1f) {
    startByRise = (ovenTemperatureC - g_lastStandbyTemperatureC) >= cfg.autoStartRiseC;
  }

  if (startByTemperature || startByRise) {
    Serial.println("OVEN: Automatischer Start erkannt");
    Serial.flush();
    activateOven(cfg, true);
  }
}

void setOvenPump(AppContext& ctx, bool on, float ovenTemperatureC = NAN, float targetTemperatureC = NAN, bool targetValid = false) {
  const bool handled = Pumps::applyOvenPumpRequest(ctx, on, ovenTemperatureC, targetTemperatureC, targetValid);
  g_pumpActive = on && handled;
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

    const bool peakDropStopEnabled = cfg.pumpStopDropFromPeakC > 0.1f;
    const bool droppedFromPeak =
        peakDropStopEnabled &&
        !isnan(g_peakTemperatureC) &&
        ovenTemperatureC <= (g_peakTemperatureC - cfg.pumpStopDropFromPeakC);

    if (ovenTooCold || wouldDischargeStorage || droppedFromPeak) {
      shouldRun = false;
    }
  }

  setOvenPump(ctx, shouldRun, ovenTemperatureC, targetTemperatureC, targetValid);
}

void updateServoControl(AppContext& ctx, float ovenTemperatureC) {
  OvenConfig& cfg = ctx.config.oven;

  const uint8_t minReg = clampPercent(cfg.servoMinimumOpeningPercent);
  const uint8_t maxReg = clampPercent(cfg.servoMaximumOpeningPercent);
  const uint8_t lowReg = (minReg < maxReg) ? minReg : maxReg;
  const uint8_t highReg = (minReg > maxReg) ? minReg : maxReg;
  const uint8_t step = cfg.servoStepPercent == 0 ? 1 : clampPercent(cfg.servoStepPercent);
  const float deadband = cfg.servoDeadbandC < 0.0f ? 0.0f : cfg.servoDeadbandC;

  if (!g_targetTemperatureReached) {
    openAirFlapForStart(cfg);
    if (ovenTemperatureC >= cfg.targetOvenTemperatureC) {
      g_targetTemperatureReached = true;
      g_lastServoControlMs = 0;
      uint8_t current = g_servoOpeningPercent;
      if (current < lowReg) current = lowReg;
      if (current > highReg) current = highReg;
      writeServoOpening(cfg, current);
    }
    return;
  }

  const uint32_t now = millis();
  if (g_lastServoControlMs == 0) {
    g_lastServoControlMs = now;
    return;
  }

  const uint32_t elapsedMs = now - g_lastServoControlMs;
  if (elapsedMs < OVEN_SERVO_CONTROL_INTERVAL_MS) return;
  g_lastServoControlMs = now;

  uint8_t nextOpening = g_servoOpeningPercent;
  if (nextOpening < lowReg) nextOpening = lowReg;
  if (nextOpening > highReg) nextOpening = highReg;

  if (ovenTemperatureC > (cfg.targetOvenTemperatureC + deadband)) {
    // Ofen ist ueber Ziel+Totband: Luftklappe schrittweise schliessen.
    nextOpening = (nextOpening > lowReg + step) ? (nextOpening - step) : lowReg;
  } else if (ovenTemperatureC < (cfg.targetOvenTemperatureC - deadband)) {
    // Ofen ist unter Ziel-Totband: Luftklappe schrittweise wieder oeffnen.
    nextOpening = (nextOpening + step < highReg) ? (nextOpening + step) : highReg;
  } else {
    // Innerhalb ±Totband: aktuelle Stellung halten.
    return;
  }

  writeServoOpening(cfg, nextOpening);
}

} // namespace

namespace OvenControl {

void begin(AppContext& ctx) {
  ServoDriver::begin();
  closeAirFlap(ctx.config.oven);
  resetServoControl();

  Serial.println("OvenControl::begin OK");
  Serial.flush();
}

void allOff(AppContext& ctx) {
  enterStandby(ctx);
}

void applyStandbyServoPosition(const AppContext& ctx) {
  // Nur die physikalische Luftklappenstellung absichern. Der logische
  // Ofenzustand bleibt erhalten, damit die Automatik nach einem kurzen
  // Service-/Testmodus im naechsten Reglerlauf sauber weiterbewerten kann.
  closeAirFlap(ctx.config.oven);
}

void requestStart() {
  g_userRequestedActive = true;
  g_autoStarted = false;
  g_burnoutStartedMs = 0;
  g_lastPumpActive = false;
  resetServoControl();
  clearPeak();
}

void requestStop(AppContext& ctx) {
  if (g_state == OvenState::REGULATING || g_state == OvenState::BURNOUT_VENTING || g_state == OvenState::EMBER_VENTING) {
    startBurnoutVenting(ctx);
    return;
  }
  enterStandby(ctx);
}

bool active() {
  return g_state == OvenState::REGULATING || g_state == OvenState::BURNOUT_VENTING || g_state == OvenState::EMBER_VENTING;
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
  return ServoDriver::angle();
}

uint8_t servoOpeningPercent() {
  return g_servoOpeningPercent;
}

bool targetTemperatureReached() {
  return g_targetTemperatureReached;
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
    case OvenState::REGULATING: return g_targetTemperatureReached ? "Regelbetrieb" : "Anheizen";
    case OvenState::BURNOUT_VENTING: return "Abbrand-Nachlauf";
    case OvenState::EMBER_VENTING: return "Restglut";
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
    closeAirFlap(cfg);

    // Bei kritischem Ofen: Pumpe einschalten, um Waerme abzufuehren.
    if (ovenTemperature >= cfg.criticalOvenTemperatureC) {
      setOvenPump(ctx, true);
    } else {
      setOvenPump(ctx, false);
    }

    resetServoControl();
    g_state = OvenState::SAFETY;
    g_userRequestedActive = false;
    g_autoStarted = false;
    clearPeak();

    Serial.println("OVEN SAFETY: Luftklappe geschlossen");
    Serial.flush();
    return;
  }

  if (g_state == OvenState::BURNOUT_VENTING || g_state == OvenState::EMBER_VENTING) {
    updateBurnoutVenting(ctx, ovenTemperature);
    return;
  }

  updateAutoStart(cfg, ovenTemperature);

  if (!g_userRequestedActive && !g_autoStarted) {
    // Standby: Ofen ist nicht freigegeben. Luftklappe geschlossen, Pumpe aus.
    closeAirFlap(cfg);
    setOvenPump(ctx, false);
    resetServoControl();
    g_state = OvenState::STANDBY;
    g_lastPumpActive = false;
    clearPeak();

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
  if (isnan(g_peakTemperatureC)) {
    initPeak(ovenTemperature);
  }
  updatePeak(ovenTemperature);

  const bool pumpWasActive = g_lastPumpActive;
  updatePump(ctx, ovenTemperature);
  const bool pumpSwitchedOff = pumpWasActive && !g_pumpActive;
  g_lastPumpActive = g_pumpActive;

  if (pumpSwitchedOff || peakTimeoutReached(cfg)) {
    startBurnoutVenting(ctx);
    updateBurnoutVenting(ctx, ovenTemperature);
    return;
  }

  updateServoControl(ctx, ovenTemperature);

  if (!g_pumpActive && ovenTemperature <= cfg.autoReturnToStandbyTemperatureC && g_autoStarted) {
    startBurnoutVenting(ctx);
  }
}

} // namespace OvenControl