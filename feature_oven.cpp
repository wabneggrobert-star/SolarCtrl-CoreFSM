#include "feature_oven.h"
#include "feature_sensor_assignments.h"
#include "config.h"
#include "feature_relay_outputs.h"
#include "feature_pwm_pca9685.h"
#include "feature_pumps.h"
#include "feature_ml_optimizer.h"
#include "feature_safety_manager.h"
#include "feature_servo_driver.h"

#include <Arduino.h>
#include <math.h>

#include "feature_build_flags.h"
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
uint32_t g_standbyRiseReferenceMs = 0;
float g_peakTemperatureC = NAN;
float g_burnoutMinimumTemperatureC = NAN;

uint32_t g_lastServoControlMs = 0;
uint32_t g_lastPeakReachedMs = 0;
uint32_t g_burnoutStartedMs = 0;
bool g_lastPumpActive = false;

static constexpr float OVEN_PEAK_REACHED_EPSILON_C = 0.2f;

static constexpr uint32_t OVEN_SERVO_CONTROL_INTERVAL_MS = 1000;

void updateServoControl(AppContext& ctx, float ovenTemperatureC);
void setOvenPump(AppContext& ctx, bool on, float ovenTemperatureC, float targetTemperatureC, bool targetValid);

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
  SensorAssignments::readByRole(ctx.ds18b20, ctx.assignments, role, temperatureC, valid);
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

void resetStandbyRiseReference(float ovenTemperatureC = NAN);

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
  g_burnoutMinimumTemperatureC = NAN;
  g_lastPumpActive = false;
  clearPeak();

  // Beim Eintritt in Standby beginnt ein neues, zeitlich begrenztes
  // Autostart-Beobachtungsfenster.
  resetStandbyRiseReference(g_ovenTemperatureC);
}

void startBurnoutVenting(AppContext& ctx) {
  (void)ctx;
  if (g_state == OvenState::SAFETY || g_state == OvenState::STANDBY) return;
  if (g_state == OvenState::BURNOUT_VENTING || g_state == OvenState::EMBER_VENTING) return;

  g_burnoutStartedMs = millis();
  g_burnoutMinimumTemperatureC = g_ovenTemperatureC;
  g_userRequestedActive = false;
  g_autoStarted = false;
  g_state = OvenState::BURNOUT_VENTING;

  DBG_PRINTLN("OVEN: Abbrand-/Nachlauf gestartet");
  DBG_FLUSH();
}

uint32_t burnoutMainMs(const OvenConfig& cfg) {
  if (cfg.burnoutVentMinutes == 0) return 0;
  return (uint32_t)cfg.burnoutVentMinutes * 60000UL;
}

uint32_t burnoutEmberMs(const OvenConfig& cfg) {
  return burnoutMainMs(cfg) / 2UL;
}

uint32_t autoStartRiseWindowMs(const OvenConfig& cfg) {
  // Bewusst kein zusaetzlicher UI-Parameter:
  // Das gleiche Zeitfenster wie Peak-/Abbrandzeit wird fuer die
  // Autostart-Anstiegserkennung verwendet.
  return burnoutMainMs(cfg);
}

void resetStandbyRiseReference(float ovenTemperatureC) {
  if (isnan(ovenTemperatureC)) {
    g_lastStandbyTemperatureC = NAN;
    g_standbyRiseReferenceMs = 0;
    return;
  }

  g_lastStandbyTemperatureC = ovenTemperatureC;
  g_standbyRiseReferenceMs = millis();
}

bool peakTimeoutReached(const OvenConfig& cfg) {
  const uint32_t timeoutMs = burnoutMainMs(cfg);
  if (timeoutMs == 0) return false;
  if (isnan(g_peakTemperatureC) || g_lastPeakReachedMs == 0) return false;
  return (uint32_t)(millis() - g_lastPeakReachedMs) >= timeoutMs;
}

// Liefert true, solange der Nachlauf den aktuellen process()-Durchlauf
// vollstaendig behandelt. Bei erneut erkanntem Abbrand wird false geliefert,
// damit process() noch im selben Zyklus mit der normalen Ofenregelung fortsetzt.
bool updateBurnoutVenting(AppContext& ctx, float ovenTemperatureC) {
  OvenConfig& cfg = ctx.config.oven;

  const uint32_t mainMs = burnoutMainMs(cfg);
  const uint32_t emberMs = burnoutEmberMs(cfg);
  if (mainMs == 0 || g_burnoutStartedMs == 0) {
    enterStandby(ctx);
    return true;
  }

  // Tiefsten Temperaturpunkt des Nachlaufs merken. Ein erneuter, deutlicher
  // Temperaturanstieg bedeutet, dass der Abbrand noch nicht beendet ist
  // (z.B. Holz nachgerutscht / Flamme wieder aufgelebt).
  if (isnan(g_burnoutMinimumTemperatureC) || ovenTemperatureC < g_burnoutMinimumTemperatureC) {
    g_burnoutMinimumTemperatureC = ovenTemperatureC;
  }

  const bool restartByRise =
      cfg.autoStartRiseC > 0.1f &&
      !isnan(g_burnoutMinimumTemperatureC) &&
      ovenTemperatureC >= (g_burnoutMinimumTemperatureC + cfg.autoStartRiseC);

  if (restartByRise) {
    DBG_PRINT("OVEN: Erneuter Abbrand erkannt, Rueckkehr in Regelbetrieb (min=");
    DBG_PRINT(g_burnoutMinimumTemperatureC, 1);
    DBG_PRINT(" C, aktuell=");
    DBG_PRINT(ovenTemperatureC, 1);
    DBG_PRINTLN(" C)");
    DBG_FLUSH();

    g_state = OvenState::REGULATING;
    g_userRequestedActive = false;
    g_autoStarted = true;
    g_burnoutStartedMs = 0;
    g_burnoutMinimumTemperatureC = NAN;

    // Aktuelle Klappenstellung beibehalten. Insbesondere bei bereits hoher
    // Ofentemperatur darf der Ruecksprung die Klappe nicht erneut auf 100 %
    // Startoeffnung setzen.
    if (ovenTemperatureC >= cfg.targetOvenTemperatureC) {
      g_targetTemperatureReached = true;
    }
    g_lastServoControlMs = 0;

    // Der wieder aufgeflammte Abbrand bekommt einen neuen Peak. Dadurch wird
    // der alte Peak (der den Nachlauf ausgeloest hat) nicht weiterverwendet.
    initPeak(ovenTemperatureC);
    g_lastPumpActive = g_pumpActive;
    return false;
  }

  const uint32_t elapsedMs = millis() - g_burnoutStartedMs;
  if (elapsedMs < mainMs) {
    // Haupt-Nachlauf: normale Servo-Regelung bleibt aktiv.
    // Die Pumpe bleibt jedoch AUS. Sie darf dem Ofen in dieser Phase keine
    // zusaetzliche Waerme entziehen, weil genau dieser Waermeentzug die
    // Luftklappe sonst unnoetig weiter Richtung Minimaloeffnung treiben kann.
    // Ein echter erneuter Abbrand wird oben ueber autoStartRiseC erkannt; dann
    // springt die Zustandsmaschine zurueck nach REGULATING und updatePump()
    // darf die Pumpe noch im selben process()-Durchlauf wieder einschalten.
    updateServoControl(ctx, ovenTemperatureC);
    setOvenPump(ctx, false, ovenTemperatureC, NAN, false);
    g_lastPumpActive = false;
    return true;
  }

  if (elapsedMs < (mainMs + emberMs)) {
    // Restglutphase: halbe eingestellte Zeit mit minimaler Regeloeffnung.
    // Pumpe aus; ein erneuter Temperaturanstieg kann weiterhin zurueck in den
    // Regelbetrieb fuehren.
    g_state = OvenState::EMBER_VENTING;
    writeServoOpening(cfg, cfg.servoMinimumOpeningPercent);
    setOvenPump(ctx, false, ovenTemperatureC, NAN, false);
    g_lastPumpActive = false;
    return true;
  }

  enterStandby(ctx);
  return true;
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
  if (g_userRequestedActive || g_autoStarted) return;

  const uint32_t now = millis();
  const uint32_t riseWindowMs =
    autoStartRiseWindowMs(cfg);

  // Referenz immer pflegen, auch wenn Auto-Start im UI gerade deaktiviert ist.
  // Wird Auto-Start spaeter wieder aktiviert, darf keine stundenalte
  // Raumtemperatur als Referenz verwendet werden.
  if (isnan(g_lastStandbyTemperatureC) ||
      g_standbyRiseReferenceMs == 0) {
    resetStandbyRiseReference(ovenTemperatureC);
  }

  // Sinkt die Ofentemperatur, wird der tiefere Standby-Wert sofort zur neuen
  // Referenz. Damit kann ein echter neuer Temperaturanstieg sauber erkannt
  // werden.
  if (!isnan(g_lastStandbyTemperatureC) &&
      ovenTemperatureC < g_lastStandbyTemperatureC) {
    resetStandbyRiseReference(ovenTemperatureC);
  }

  // Ist das Zeitfenster abgelaufen, wird die aktuelle Temperatur zur neuen
  // Referenz. Langsame Raumtemperatur-Aenderungen koennen sich dadurch nicht
  // ueber Stunden zu autoStartRiseC aufsummieren.
  if (riseWindowMs > 0 &&
      g_standbyRiseReferenceMs != 0 &&
      (uint32_t)(now - g_standbyRiseReferenceMs) > riseWindowMs) {
    resetStandbyRiseReference(ovenTemperatureC);
  }

  if (!cfg.autoStartEnabled) return;

  const bool startByTemperature =
    ovenTemperatureC >= cfg.autoStartTemperatureC;

  bool startByRise = false;

  if (riseWindowMs > 0 &&
      cfg.autoStartRiseC > 0.1f &&
      !isnan(g_lastStandbyTemperatureC) &&
      g_standbyRiseReferenceMs != 0) {
    const uint32_t elapsedMs =
      (uint32_t)(now - g_standbyRiseReferenceMs);

    startByRise =
      elapsedMs <= riseWindowMs &&
      (
        ovenTemperatureC -
        g_lastStandbyTemperatureC
      ) >= cfg.autoStartRiseC;
  }

  if (startByTemperature || startByRise) {
    DBG_PRINT("OVEN: Automatischer Start erkannt (");
    DBG_PRINT(
      startByTemperature
        ? "Temperatur"
        : "schneller Anstieg"
    );

    if (startByRise) {
      DBG_PRINT(", Delta=");
      DBG_PRINT(
        ovenTemperatureC -
        g_lastStandbyTemperatureC,
        1
      );
      DBG_PRINT(" K in ");
      DBG_PRINT(
        (uint32_t)(now - g_standbyRiseReferenceMs) /
        60000UL
      );
      DBG_PRINT(" min");
    }

    DBG_PRINTLN(")");
    DBG_FLUSH();

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
  const float baseDeadband =
    cfg.servoDeadbandC < 0.0f
      ? 0.0f
      : cfg.servoDeadbandC;

  const float deadband =
    MlOptimizer::adjustOvenServoDeadband(
      ctx,
      baseDeadband
    );

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

  // ML V2 darf erst nach seiner eigenen Domain-Freigabe und nur innerhalb der
  // vom Benutzer gesetzten Min-/Max-Oeffnung korrigieren. Der Optimizer
  // verhindert zusaetzlich eine Korrektur in thermisch falscher Richtung.
  nextOpening = MlOptimizer::adjustOvenServoOpening(
    ctx,
    nextOpening,
    ovenTemperatureC
  );

  writeServoOpening(cfg, nextOpening);
}

} // namespace

namespace OvenControl {

void begin(AppContext& ctx) {
  ServoDriver::begin();
  closeAirFlap(ctx.config.oven);
  resetServoControl();

  DBG_PRINTLN("OvenControl::begin OK");
  DBG_FLUSH();
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
  g_burnoutMinimumTemperatureC = NAN;
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
    g_burnoutStartedMs = 0;
    g_burnoutMinimumTemperatureC = NAN;
    clearPeak();
    resetStandbyRiseReference(ovenTemperature);

    DBG_PRINTLN("OVEN SAFETY: Luftklappe geschlossen");
    DBG_FLUSH();
    return;
  }

  if (g_state == OvenState::BURNOUT_VENTING || g_state == OvenState::EMBER_VENTING) {
    if (updateBurnoutVenting(ctx, ovenTemperature)) {
      return;
    }
    // Erneuter Abbrand erkannt: noch in diesem process()-Durchlauf mit der
    // regulaeren Pumpen-/Peak-/Servologik fortsetzen.
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

    return;
  }

  // Im aktiven Regelbetrieb ist die aktuelle Ofentemperatur die Referenz fuer
  // einen spaeteren erneuten Standby-Zyklus.
  resetStandbyRiseReference(ovenTemperature);
  activateOven(cfg, false);
  if (isnan(g_peakTemperatureC)) {
    initPeak(ovenTemperature);
  }
  updatePeak(ovenTemperature);

  const bool pumpWasActive = g_lastPumpActive;

  // Wenn der Abfall vom Peak bzw. das Peak-Timeout den Nachlauf ausloest,
  // direkt in den Nachlauf wechseln. Im Nachlauf wird die Pumpe sofort AUS
  // gehalten; die Luftklappe darf den restlichen Abbrand ohne zusaetzlichen
  // Waermeentzug weiter ausregeln. Bei erneutem deutlichem Temperaturanstieg
  // kann der Zustand wieder nach REGULATING zurueckspringen.
  const bool peakDropStopEnabled = cfg.pumpStopDropFromPeakC > 0.1f;
  const bool peakDropReached =
      pumpWasActive &&
      peakDropStopEnabled &&
      !isnan(g_peakTemperatureC) &&
      ovenTemperature <= (g_peakTemperatureC - cfg.pumpStopDropFromPeakC);

  if (peakDropReached || peakTimeoutReached(cfg)) {
    startBurnoutVenting(ctx);
    updateBurnoutVenting(ctx, ovenTemperature);
    return;
  }

  updatePump(ctx, ovenTemperature);
  const bool pumpSwitchedOff = pumpWasActive && !g_pumpActive;
  g_lastPumpActive = g_pumpActive;

  // Andere regulaere Pumpen-Abschaltgruende (z.B. Rueckkuehlgrenze) starten
  // weiterhin den Nachlauf. Die Pumpe bleibt dort AUS; nur ein erkannter
  // erneuter Abbrand fuehrt zurueck in die regulaere Pumpenlogik.
  if (pumpSwitchedOff) {
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