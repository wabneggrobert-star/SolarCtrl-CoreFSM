#include "feature_pwm_pca9685.h"
#include "config.h"

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

namespace {
  Adafruit_PWMServoDriver g_pwm = Adafruit_PWMServoDriver(PCA9685_ADDR);
  bool g_started = false;
  uint8_t g_dutyPercent[16] = {0};

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

  g_started = true;
  allOff();

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
  // Falls deine Platine genau umgekehrt reagiert, nur diese Zeile tauschen.
  uint8_t effectivePercent = (profile == PwmProfile::SOLAR) ? (100 - percent) : percent;

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

  for (uint8_t ch = 0; ch < 16; ch++) {
    setDuty(ch, 0, PwmProfile::SOLAR);
  }
}

}
