#include "feature_pwm_pca9685.h"
#include "config.h"

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

namespace {
  Adafruit_PWMServoDriver g_pwm = Adafruit_PWMServoDriver(PCA9685_ADDR);
  bool g_started = false;
  uint8_t g_dutyPercent[16] = {0};
  uint8_t g_effectivePercent[16] = {0};
  bool g_effectiveValid[16] = {false};

  uint16_t dutyPercentToCounts(uint8_t percent) {
    if (percent >= 100) return 4095;
    return static_cast<uint16_t>((uint32_t)percent * 4095UL / 100UL);
  }

  bool i2cDevicePresent(uint8_t addr) {
    Wire.beginTransmission(addr);
    return Wire.endTransmission() == 0;
  }
}

namespace PwmDriver {

bool begin() {
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

  if (!i2cDevicePresent(PCA9685_ADDR)) {
    g_started = false;
    Serial.print("PCA9685 PWM begin 0x");
    Serial.print(PCA9685_ADDR, HEX);
    Serial.println(": NICHT GEFUNDEN");
    return false;
  }

  g_pwm.begin();
  g_pwm.setPWMFreq(PCA9685_PWM_FREQ);
  delay(10);

  for (uint8_t ch = 0; ch < 16; ch++) {
    g_dutyPercent[ch] = 0;
    g_effectivePercent[ch] = 0;
    g_effectiveValid[ch] = false;
  }

  g_started = true;

  // Noch keinen Pegel anhand eines geratenen Profils setzen. Der Aufrufer
  // hat zu diesem Zeitpunkt bereits die Config geladen und ruft direkt
  // danach allOff(config) mit den echten SOLAR-/HEATING-Profilen auf.

  Serial.print("PCA9685 PWM begin 0x");
  Serial.print(PCA9685_ADDR, HEX);
  Serial.println(": OK");
  return true;
}

bool available() {
  return g_started;
}

void setDuty(uint8_t channel, uint8_t percent) {
  setDuty(channel, percent, PwmProfile::SOLAR);
}

void setDuty(uint8_t channel, uint8_t percent, PwmProfile profile) {
  if (!g_started) return;
  if (channel >= 16) return;
  if (percent > 100) percent = 100;

  g_dutyPercent[channel] = percent;

  // HEATING = direkte Logik, SOLAR = elektrisch invertiert.
  // SOLAR: logisch EIN/100% ergibt physikalisch 0%/LOW.
  uint8_t effectivePercent = (profile == PwmProfile::SOLAR) ? (100 - percent) : percent;

  // Wichtiger Schutz fuer Schaltausgaenge: Wenn derselbe physikalische
  // Sollwert bereits am PCA9685 anliegt, nicht erneut schreiben. Damit kann
  // die Runtime ihren Sollzustand zyklisch bestaetigen, ohne alle paar
  // Sekunden einen Schaltimpuls/Takt am Ausgang zu erzeugen.
  if (g_effectiveValid[channel] && g_effectivePercent[channel] == effectivePercent) {
    return;
  }

  g_effectivePercent[channel] = effectivePercent;
  g_effectiveValid[channel] = true;

  if (effectivePercent == 0) {
    g_pwm.setPWM(channel, 0, 0);
    return;
  }

  if (effectivePercent >= 100) {
    g_pwm.setPWM(channel, 4096, 0); // full-on Bit
    return;
  }

  uint16_t counts = dutyPercentToCounts(effectivePercent);
  g_pwm.setPWM(channel, 0, counts);
}

void setSwitch(uint8_t channel, bool on, PwmProfile profile) {
  setDuty(channel, on ? 100 : 0, profile);
}

uint8_t getDuty(uint8_t channel) {
  if (channel >= 16) return 0;
  return g_dutyPercent[channel];
}

void allOff() {
  if (!g_started) return;

  // Boot-Fallback, solange die SD-Konfiguration noch nicht geladen ist.
  // Im laufenden System immer allOff(config) verwenden.
  for (uint8_t ch = 0; ch < 16; ch++) {
    setDuty(ch, 0, PwmProfile::SOLAR);
  }
}

void allOff(const ConfigData& config) {
  if (!g_started) return;

  // Jeder verwendbare PO-Kanal wird logisch AUS geschaltet. Die elektrische
  // Pegellage ergibt sich aus dem jeweiligen Profil:
  //   SOLAR   -> logisch AUS = physikalisch 100 % / HIGH
  //   HEATING -> logisch AUS = physikalisch   0 % / LOW
  // Damit ist ein globales All-Off nicht mehr auf ein einziges Profil festgelegt.
  for (uint8_t ch = 0; ch < PWM_OUTPUT_COUNT; ch++) {
    setDuty(ch, 0, config.pwmOutputs[ch].profile);
  }

  // Nicht konfigurierte PCA-Kanaele ausserhalb des aktuell freigegebenen
  // PO-Bereichs behalten den bisherigen Boot-Fallback.
  for (uint8_t ch = PWM_OUTPUT_COUNT; ch < 16; ch++) {
    setDuty(ch, 0, PwmProfile::SOLAR);
  }
}

}
