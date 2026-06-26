#include "feature_heat_sources_max31865.h"

#include "config.h"
#include "feature_heat_source_assignments.h"
#include "feature_io_expander_pcf8574.h"

#include <Arduino.h>
#include <SPI.h>
#include <math.h>

namespace {
  SPIClass maxSpi(HSPI);

  static constexpr uint8_t REG_CONFIG     = 0x00;
  static constexpr uint8_t REG_RTD_MSB    = 0x01;
  static constexpr uint8_t REG_FAULT_STAT = 0x07;

  static constexpr uint8_t CFG_VBIAS       = 0x80;
  static constexpr uint8_t CFG_1SHOT       = 0x20;
  static constexpr uint8_t CFG_3WIRE       = 0x10;
  static constexpr uint8_t CFG_FAULT_CLEAR = 0x02;
  static constexpr uint8_t CFG_FILTER_50HZ = 0x01;

  static constexpr uint32_t MAX_BIAS_WAIT_US = 10000;
  static constexpr uint32_t MAX_CONV_WAIT_US = 70000;
  static constexpr uint32_t MAX_CS_SETTLE_US = 1000;
  static constexpr uint32_t MAX_SPI_HZ = 10000;

  bool g_started = false;

  bool plausible(float t) {
    return !isnan(t) &&
           t >= HEAT_SOURCE_TEMP_MIN_C &&
           t <= HEAT_SOURCE_TEMP_MAX_C;
  }

  MaxChannelConfig& cfgFor(AppContext& ctx, MaxChannel ch) {
    switch (ch) {
      case MaxChannel::CH1: return ctx.config.max1;
      case MaxChannel::CH2: return ctx.config.max2;
      case MaxChannel::CH3: return ctx.config.max3;
      default: return ctx.config.max1;
    }
  }

  const MaxChannelConfig& cfgFor(const AppContext& ctx, MaxChannel ch) {
    switch (ch) {
      case MaxChannel::CH1: return ctx.config.max1;
      case MaxChannel::CH2: return ctx.config.max2;
      case MaxChannel::CH3: return ctx.config.max3;
      default: return ctx.config.max1;
    }
  }

  MaxChannelReading& readingFor(AppContext& ctx, MaxChannel ch) {
    switch (ch) {
      case MaxChannel::CH1: return ctx.maxReadings[0];
      case MaxChannel::CH2: return ctx.maxReadings[1];
      case MaxChannel::CH3: return ctx.maxReadings[2];
      default: return ctx.maxReadings[0];
    }
  }

  MaxChannelRuntime& runtimeFor(AppContext& ctx, MaxChannel ch) {
    switch (ch) {
      case MaxChannel::CH1: return ctx.maxRuntime[0];
      case MaxChannel::CH2: return ctx.maxRuntime[1];
      case MaxChannel::CH3: return ctx.maxRuntime[2];
      default: return ctx.maxRuntime[0];
    }
  }

  bool channelEnabled(const AppContext& ctx, MaxChannel ch) {
    return cfgFor(ctx, ch).enabled;
  }

  void resetReading(MaxChannelReading& out) {
    out.present = false;
    out.valid = false;
    out.tempC = NAN;
    out.rawTempC = NAN;
    out.resistanceOhm = NAN;
    out.rawRtd = 0;
    out.fault = 0;
  }

  void settleUs(uint32_t us) {
    const uint32_t t0 = micros();
    while ((uint32_t)(micros() - t0) < us) {
    }
  }

  void selectChannel(MaxChannel ch) {
    Pcf8574Io::deselectMaxCs();
    settleUs(MAX_CS_SETTLE_US);
    Pcf8574Io::selectMax(ch);
    settleUs(MAX_CS_SETTLE_US);
  }

  void deselectChannel() {
    Pcf8574Io::deselectMaxCs();
    settleUs(MAX_CS_SETTLE_US);
  }

  uint8_t spiRead8(MaxChannel ch, uint8_t reg) {
    selectChannel(ch);
    maxSpi.beginTransaction(SPISettings(MAX_SPI_HZ, MSBFIRST, SPI_MODE1));
    maxSpi.transfer(reg & 0x7F);
    const uint8_t v = maxSpi.transfer(0x00);
    maxSpi.endTransaction();
    deselectChannel();
    return v;
  }

  uint16_t spiRead16(MaxChannel ch, uint8_t regMsb) {
    selectChannel(ch);
    maxSpi.beginTransaction(SPISettings(MAX_SPI_HZ, MSBFIRST, SPI_MODE1));
    maxSpi.transfer(regMsb & 0x7F);
    const uint8_t msb = maxSpi.transfer(0x00);
    const uint8_t lsb = maxSpi.transfer(0x00);
    maxSpi.endTransaction();
    deselectChannel();
    return (uint16_t(msb) << 8) | uint16_t(lsb);
  }

  void spiWrite8(MaxChannel ch, uint8_t reg, uint8_t value) {
    selectChannel(ch);
    maxSpi.beginTransaction(SPISettings(MAX_SPI_HZ, MSBFIRST, SPI_MODE1));
    maxSpi.transfer(reg | 0x80);
    maxSpi.transfer(value);
    maxSpi.endTransaction();
    deselectChannel();
  }

  void clearFault(MaxChannel ch) {
    uint8_t cfg = CFG_VBIAS | CFG_FAULT_CLEAR;
    if (MAX31865_USE_3WIRE) cfg |= CFG_3WIRE;
    spiWrite8(ch, REG_CONFIG, cfg);
  }

  void setBiasOn(MaxChannel ch) {
    uint8_t cfg = CFG_VBIAS | CFG_FILTER_50HZ;
    if (MAX31865_USE_3WIRE) cfg |= CFG_3WIRE;
    spiWrite8(ch, REG_CONFIG, cfg);
  }

  void startOneShot(MaxChannel ch) {
    uint8_t cfg = CFG_VBIAS | CFG_1SHOT | CFG_FILTER_50HZ;
    if (MAX31865_USE_3WIRE) cfg |= CFG_3WIRE;
    spiWrite8(ch, REG_CONFIG, cfg);
  }

  float resistanceToTempPt1000(float resistanceOhm) {
    if (isnan(resistanceOhm)) return NAN;
    if (resistanceOhm < 100.0f || resistanceOhm > 5000.0f) return NAN;

    constexpr float R0 = 1000.0f;
    constexpr float A = 3.9083e-3f;
    constexpr float B = -5.775e-7f;

    if (resistanceOhm < R0) {
      return (resistanceOhm / R0 - 1.0f) / 0.00385f;
    }

    const float c = 1.0f - (resistanceOhm / R0);
    const float disc = A * A - 4.0f * B * c;
    if (disc < 0.0f) return NAN;

    return (-A + sqrtf(disc)) / (2.0f * B);
  }

  void finishRead(AppContext& ctx, MaxChannel ch) {
    MaxChannelReading& out = readingFor(ctx, ch);
    MaxChannelConfig& cfg = cfgFor(ctx, ch);
    MaxChannelRuntime& rt = runtimeFor(ctx, ch);

    const uint16_t raw = spiRead16(ch, REG_RTD_MSB);
    const uint8_t fault = spiRead8(ch, REG_FAULT_STAT);

    const uint16_t rawShifted = raw >> 1;
    const float ratio = rawShifted / 32768.0f;
    const float resistance = ratio * MAX31865_RREF;

    float rawTemp = NAN;
    float temp = NAN;

    out.present = true;
    out.rawRtd = raw;
    out.resistanceOhm = resistance;
    out.fault = fault;

    if (rawShifted > 0 && fault == 0) {
      rawTemp = resistanceToTempPt1000(resistance);
      if (!isnan(rawTemp)) {
        temp = rawTemp * cfg.calFactor + cfg.offsetC;
      }
    }

    out.rawTempC = rawTemp;
    out.tempC = temp;
    out.valid = (fault == 0) && plausible(temp);

    if (fault != 0) {
      clearFault(ch);
    }

    rt.step = MaxReadStep::IDLE;
    rt.cycleDone = true;
  }

  bool anyCycleInProgress(const AppContext& ctx) {
    return ctx.maxRuntime[0].step != MaxReadStep::IDLE ||
           ctx.maxRuntime[1].step != MaxReadStep::IDLE ||
           ctx.maxRuntime[2].step != MaxReadStep::IDLE;
  }

  bool allDone(const AppContext& ctx) {
    const bool ch1Done = (!ctx.config.max1.enabled) || ctx.maxRuntime[0].cycleDone;
    const bool ch2Done = (!ctx.config.max2.enabled) || ctx.maxRuntime[1].cycleDone;
    const bool ch3Done = (!ctx.config.max3.enabled) || ctx.maxRuntime[2].cycleDone;
    return ch1Done && ch2Done && ch3Done;
  }

  void processChannel(AppContext& ctx, MaxChannel ch) {
    if (!channelEnabled(ctx, ch)) return;

    MaxChannelRuntime& rt = runtimeFor(ctx, ch);

    switch (rt.step) {
      case MaxReadStep::IDLE:
        break;

      case MaxReadStep::BIAS_ON:
        setBiasOn(ch);
        rt.tMarkUs = micros();
        rt.step = MaxReadStep::WAIT_BIAS;
        break;

      case MaxReadStep::WAIT_BIAS:
        if ((uint32_t)(micros() - rt.tMarkUs) >= MAX_BIAS_WAIT_US) {
          rt.step = MaxReadStep::START_1SHOT;
        }
        break;

      case MaxReadStep::START_1SHOT:
        startOneShot(ch);
        rt.tMarkUs = micros();
        rt.step = MaxReadStep::WAIT_CONVERSION;
        break;

      case MaxReadStep::WAIT_CONVERSION:
        if ((uint32_t)(micros() - rt.tMarkUs) >= MAX_CONV_WAIT_US) {
          rt.step = MaxReadStep::READ_RESULT;
        }
        break;

      case MaxReadStep::READ_RESULT:
        finishRead(ctx, ch);
        break;
    }
  }
}

namespace HeatSourcesMax {

bool begin(AppContext& ctx) {
  Serial.println("HeatSourcesMax::begin() START");
  Serial.flush();

  Serial.println("MAX-PCF begin...");
  Serial.flush();

  if (!Pcf8574Io::begin()) {
    Serial.println("MAX-PCF FEHLER");
    Serial.flush();
    g_started = false;
    return false;
  }

  Serial.println("MAX-PCF OK");
  Serial.flush();

  Serial.println("MAX-CS deselect...");
  Serial.flush();
  Pcf8574Io::deselectMaxCs();

  Serial.println("MAX SPI begin...");
  Serial.flush();
  maxSpi.begin(PIN_MAX_SCK, PIN_MAX_MISO, PIN_MAX_MOSI);

  Serial.println("MAX runtime reset...");
  Serial.flush();

  for (uint8_t i = 0; i < 3; i++) {
    ctx.maxRuntime[i].step = MaxReadStep::IDLE;
    ctx.maxRuntime[i].tMarkUs = 0;
    ctx.maxRuntime[i].cycleDone = true;
  }

  g_started = true;

  Serial.println("HeatSourcesMax::begin() OK");
  Serial.flush();

  return true;
}

void startCycle(AppContext& ctx) {
  if (!g_started) return;
  if (anyCycleInProgress(ctx)) return;

  for (int i = 0; i < 3; i++) {
    ctx.maxRuntime[i].cycleDone = false;
  }

  if (ctx.config.max1.enabled) {
    resetReading(ctx.maxReadings[0]);
    ctx.maxRuntime[0].step = MaxReadStep::BIAS_ON;
  }

  if (ctx.config.max2.enabled) {
    resetReading(ctx.maxReadings[1]);
    ctx.maxRuntime[1].step = MaxReadStep::BIAS_ON;
  }

  if (ctx.config.max3.enabled) {
    resetReading(ctx.maxReadings[2]);
    ctx.maxRuntime[2].step = MaxReadStep::BIAS_ON;
  }
}

void process(AppContext& ctx) {
  if (!g_started) return;

  processChannel(ctx, MaxChannel::CH1);
  processChannel(ctx, MaxChannel::CH2);
  processChannel(ctx, MaxChannel::CH3);

  if (allDone(ctx)) {
    HeatSourceAssignments::resolveHeatSources(ctx);
  }
}

bool cycleComplete(const AppContext& ctx) {
  return allDone(ctx);
}

}
