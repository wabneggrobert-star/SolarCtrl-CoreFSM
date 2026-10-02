#include "feature_servo_driver.h"
#include "config.h"

#include "feature_build_flags.h"
namespace {

static constexpr uint32_t SERVO_PWM_FREQUENCY_HZ = 50;
static constexpr uint8_t SERVO_PWM_RESOLUTION_BITS = 16;
static constexpr uint32_t SERVO_PWM_MAX_DUTY = (1UL << SERVO_PWM_RESOLUTION_BITS) - 1UL;

bool g_initialized = false;
uint8_t g_angle = 0;

uint8_t clampAngle(uint8_t angle) {
  if (angle > 180) return 180;
  return angle;
}

uint16_t angleToPulseUs(uint8_t angle) {
  return (uint16_t)(500 + ((uint32_t)angle * 2000UL / 180UL));
}

uint32_t servoPulseUsToDuty(uint16_t pulseUs) {
  return (uint32_t)((uint64_t)pulseUs * SERVO_PWM_MAX_DUTY / 20000ULL);
}

void writeAngle(uint8_t angle) {
  const uint16_t pulseUs = angleToPulseUs(angle);
  const uint32_t duty = servoPulseUsToDuty(pulseUs);
  ledcWrite(OVEN_SERVO_PIN, duty);
  g_angle = angle;
}

} // namespace

namespace ServoDriver {

bool begin() {
  if (!g_initialized) {
    ledcAttach(OVEN_SERVO_PIN, SERVO_PWM_FREQUENCY_HZ, SERVO_PWM_RESOLUTION_BITS);
    g_initialized = true;
  }

  // Keine rohe 0-Grad-Stellung erzwingen. Die fachlich sichere Stellung
  // gehoert zur Ofenkalibrierung (0 % Oeffnung -> servoClosedAngle) und wird
  // direkt danach von OvenControl gesetzt. So gibt es auch beim Start keinen
  // kurzen Sprung ausserhalb des kalibrierten Arbeitsbereichs.

  DBG_PRINT("ServoDriver::begin OK GPIO");
  DBG_PRINTLN(OVEN_SERVO_PIN);
  DBG_FLUSH();

  return true;
}

uint8_t setAngle(uint8_t angle) {
  if (!g_initialized) {
    begin();
  }

  const uint8_t limited = clampAngle(angle);
  writeAngle(limited);
  return limited;
}

void close() {
  writeAngle(0);
}

uint8_t angle() {
  return g_angle;
}

bool initialized() {
  return g_initialized;
}

} // namespace ServoDriver
