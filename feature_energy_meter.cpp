#include "feature_energy_meter.h"
#include "config.h"
#include "feature_sensor_assignments.h"
#include "feature_storage.h"
#include <Arduino.h>
#include <SD.h>
#include <math.h>

namespace {
volatile uint32_t g_pendingPulses = 0;
volatile uint32_t g_lastPulseUs = 0;
portMUX_TYPE g_pulseMux = portMUX_INITIALIZER_UNLOCKED;
uint8_t g_attachedPin = PIN_UNUSED;
constexpr uint32_t MIN_PULSE_GAP_US = 100UL;
constexpr uint32_t TEMPERATURE_SAMPLE_INTERVAL_MS = 10000UL;

void IRAM_ATTR pulseIsr() {
  const uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - g_lastPulseUs) < MIN_PULSE_GAP_US) return;
  g_lastPulseUs = nowUs;
  portENTER_CRITICAL_ISR(&g_pulseMux);
  ++g_pendingPulses;
  portEXIT_CRITICAL_ISR(&g_pulseMux);
}

uint32_t takePendingPulses() {
  portENTER_CRITICAL(&g_pulseMux);
  const uint32_t pulses = g_pendingPulses;
  g_pendingPulses = 0;
  portEXIT_CRITICAL(&g_pulseMux);
  return pulses;
}

void detachFeedback() {
  if (g_attachedPin != PIN_UNUSED) {
    detachInterrupt(digitalPinToInterrupt(g_attachedPin));
    g_attachedPin = PIN_UNUSED;
  }
  takePendingPulses();
  g_lastPulseUs = 0;
}

bool ensureLogFile() {
  if (!SD.exists("/logs")) SD.mkdir("/logs");
  const bool needsHeader = !SD.exists(FILE_ENERGY_METER_LOG);
  File f = SD.open(FILE_ENERGY_METER_LOG, FILE_APPEND);
  if (!f) return false;
  if (needsHeader || f.size() == 0) {
    f.println("boot_count,uptime_s,pulses_total,volume_l_total,flow_l_min,flow_temp_c,return_temp_c,delta_t_k,power_kw,energy_kwh_total,temp_valid");
  }
  f.close();
  return true;
}

String csvFloat(float value, uint8_t decimals) {
  if (isnan(value)) return "";
  return String((double)value, (unsigned int)decimals);
}

void appendLog(const AppContext& ctx) {
  if (!ctx.sdAvailable || !ctx.config.energyMeter.enabled) return;
  if (!ensureLogFile()) return;
  const EnergyMeterRuntime& rt = ctx.energyMeter;
  File f = SD.open(FILE_ENERGY_METER_LOG, FILE_APPEND);
  if (!f) return;
  f.print(ctx.diag.bootCount); f.print(',');
  f.print(millis() / 1000UL); f.print(',');
  f.print(rt.totalPulses); f.print(',');
  f.print(rt.totalVolumeLiters, 3); f.print(',');
  f.print(rt.flowLitersPerMinute, 3); f.print(',');
  f.print(csvFloat(rt.flowTemperatureC, 2)); f.print(',');
  f.print(csvFloat(rt.returnTemperatureC, 2)); f.print(',');
  f.print(csvFloat(rt.deltaTemperatureK, 2)); f.print(',');
  f.print(rt.thermalPowerKw, 3); f.print(',');
  f.print(rt.totalEnergyKWh, 5); f.print(',');
  f.println(rt.temperatureValid ? 1 : 0);
  f.close();
}

bool sampleTemperatures(AppContext& ctx) {
  EnergyMeterRuntime& rt = ctx.energyMeter;
  const EnergyMeterConfig& cfg = ctx.config.energyMeter;
  rt.flowTemperatureC = NAN;
  rt.returnTemperatureC = NAN;
  rt.deltaTemperatureK = NAN;
  rt.temperatureValid = false;
  if (cfg.flowSensorRole == Ds18Role::NONE || cfg.returnSensorRole == Ds18Role::NONE) return false;
  float flowC = NAN, returnC = NAN;
  bool flowValid = false, returnValid = false;
  const bool flowRead = SensorAssignments::readByRole(ctx.assignments, cfg.flowSensorRole, flowC, flowValid);
  const bool returnRead = SensorAssignments::readByRole(ctx.assignments, cfg.returnSensorRole, returnC, returnValid);
  rt.flowTemperatureC = flowC;
  rt.returnTemperatureC = returnC;
  rt.temperatureValid = flowRead && returnRead && flowValid && returnValid && !isnan(flowC) && !isnan(returnC);
  if (rt.temperatureValid) rt.deltaTemperatureK = flowC - returnC;
  return rt.temperatureValid;
}
}

namespace EnergyMeter {
uint8_t gpioForFeedbackInput(uint8_t inputIndex) {
  if (inputIndex >= FEEDBACK_INPUT_COUNT) return PIN_UNUSED;
  return FEEDBACK_INPUT_PINS[inputIndex];
}

void reconfigure(AppContext& ctx) {
  detachFeedback();
  EnergyMeterRuntime& rt = ctx.energyMeter;
  rt.feedbackAttached = false;
  rt.flowLitersPerMinute = 0.0f;
  rt.thermalPowerKw = 0.0f;
  rt.lastProcessMs = millis();
  rt.lastTemperatureSampleMs = 0;
  rt.lastLogMs = millis();
  const EnergyMeterConfig& cfg = ctx.config.energyMeter;
  if (!cfg.enabled || cfg.feedbackInputIndex == PIN_UNUSED || cfg.pulsesPerLiter <= 0.0f) return;
  const uint8_t gpio = gpioForFeedbackInput(cfg.feedbackInputIndex);
  if (gpio == PIN_UNUSED) return;
  pinMode(gpio, INPUT);
  g_attachedPin = gpio;
  attachInterrupt(digitalPinToInterrupt(gpio), pulseIsr, RISING);
  rt.feedbackAttached = true;
  if (ctx.sdAvailable) ensureLogFile();
}

void begin(AppContext& ctx) {
  ctx.energyMeter.flowLitersPerMinute = 0.0f;
  ctx.energyMeter.flowTemperatureC = NAN;
  ctx.energyMeter.returnTemperatureC = NAN;
  ctx.energyMeter.deltaTemperatureK = NAN;
  ctx.energyMeter.thermalPowerKw = 0.0f;
  ctx.energyMeter.temperatureValid = false;
  reconfigure(ctx);
}

void process(AppContext& ctx) {
  EnergyMeterRuntime& rt = ctx.energyMeter;
  const EnergyMeterConfig& cfg = ctx.config.energyMeter;
  const uint32_t now = millis();
  if (!cfg.enabled || !rt.feedbackAttached || cfg.pulsesPerLiter <= 0.0f) {
    takePendingPulses();
    rt.flowLitersPerMinute = 0.0f;
    rt.thermalPowerKw = 0.0f;
    rt.lastProcessMs = now;
    return;
  }
  if (rt.lastProcessMs == 0) rt.lastProcessMs = now;
  const uint32_t elapsedMs = (uint32_t)(now - rt.lastProcessMs);
  if (elapsedMs == 0) return;
  rt.lastProcessMs = now;
  const uint32_t pulses = takePendingPulses();
  rt.totalPulses += pulses;
  const double liters = (double)pulses / (double)cfg.pulsesPerLiter;
  rt.totalVolumeLiters += liters;
  rt.flowLitersPerMinute = (float)(liters * 60000.0 / (double)elapsedMs);
  if (rt.lastTemperatureSampleMs == 0 || (uint32_t)(now - rt.lastTemperatureSampleMs) >= TEMPERATURE_SAMPLE_INTERVAL_MS) {
    rt.lastTemperatureSampleMs = now;
    sampleTemperatures(ctx);
  }
  if (rt.temperatureValid && !isnan(rt.deltaTemperatureK) && rt.deltaTemperatureK > 0.0f && cfg.energyFactorWhPerLiterK > 0.0f) {
    const double energyWh = liters * (double)rt.deltaTemperatureK * (double)cfg.energyFactorWhPerLiterK;
    rt.totalEnergyKWh += energyWh / 1000.0;
    rt.thermalPowerKw = rt.flowLitersPerMinute * rt.deltaTemperatureK * cfg.energyFactorWhPerLiterK * 0.06f;
  } else {
    rt.thermalPowerKw = 0.0f;
  }
  const uint32_t interval = cfg.logIntervalMs >= 10000UL ? cfg.logIntervalMs : DEFAULT_ENERGY_LOG_INTERVAL_MS;
  if (rt.lastLogMs == 0 || (uint32_t)(now - rt.lastLogMs) >= interval) {
    rt.lastLogMs = now;
    appendLog(ctx);
  }
}

void saveRuntime(const AppContext& ctx) {
  if (ctx.sdAvailable) Storage::saveEnergyMeterRuntime(ctx.energyMeter);
}
}
