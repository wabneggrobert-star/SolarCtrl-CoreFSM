#include "feature_heat_sources_max31865.h"

#include "config.h"
#include "feature_heat_source_assignments.h"
#include "feature_io_expander_pcf8574.h"

#include <Arduino.h>
#include <SPI.h>
#include <math.h>
#include <string.h>

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

  bool plausibleTemperature(float t) {
    return !isnan(t) &&
           t >= MAX31865_VALID_TEMP_MIN_C &&
           t <= MAX31865_VALID_TEMP_MAX_C;
  }

  bool plausibleResistance(float r) {
    return !isnan(r) &&
           r >= MAX31865_VALID_R_MIN_OHM &&
           r <= MAX31865_VALID_R_MAX_OHM;
  }

  void setStatus(MaxChannelReading& out, const char* status) {
    strncpy(out.status, status, sizeof(out.status) - 1);
    out.status[sizeof(out.status) - 1] = '\0';
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

  const MaxChannelReading& readingFor(const AppContext& ctx, MaxChannel ch) {
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
    out.rtdCode = 0;
    out.fault = 0;
    out.rawFaultBit = false;
    setStatus(out, "not_read");
  }

  void markDisabled(MaxChannelReading& out) {
    resetReading(out);
    setStatus(out, "disabled");
  }

  void settleUs(uint32_t us) {
    const uint32_t t0 = micros();
    while ((uint32_t)(micros() - t0) < us) {
      // short busy wait; keeps MAX31865 timing deterministic
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

  uint8_t baseConfig(bool includeOneShot = false, bool includeFaultClear = false) {
    uint8_t cfg = CFG_VBIAS | CFG_FILTER_50HZ;
    if (includeOneShot) cfg |= CFG_1SHOT;
    if (includeFaultClear) cfg |= CFG_FAULT_CLEAR;
    if (MAX31865_USE_3WIRE) cfg |= CFG_3WIRE;
    return cfg;
  }

  void clearFault(MaxChannel ch) {
    spiWrite8(ch, REG_CONFIG, baseConfig(false, true));
  }

  void setBiasOn(MaxChannel ch) {
    spiWrite8(ch, REG_CONFIG, baseConfig(false, false));
  }

  void startOneShot(MaxChannel ch) {
    spiWrite8(ch, REG_CONFIG, baseConfig(true, false));
  }

  float resistanceToTempPt1000(float resistanceOhm) {
    if (isnan(resistanceOhm)) return NAN;
    if (resistanceOhm < 100.0f || resistanceOhm > 5000.0f) return NAN;

    const float R0 = MAX31865_RNOMINAL;
    constexpr float A = 3.9083e-3f;
    constexpr float B = -5.775e-7f;

    // Simplified negative-temperature approximation. This is sufficient for the
    // controller validity check and avoids the full CVD cubic below 0 °C.
    if (resistanceOhm < R0) {
      return (resistanceOhm / R0 - 1.0f) / 0.00385f;
    }

    const float c = 1.0f - (resistanceOhm / R0);
    const float disc = A * A - 4.0f * B * c;
    if (disc < 0.0f) return NAN;

    return (-A + sqrtf(disc)) / (2.0f * B);
  }

  void evaluateRawReading(MaxChannelReading& out,
                          const MaxChannelConfig& cfg,
                          uint16_t raw,
                          uint8_t fault) {
    resetReading(out);

    const bool rawFaultBit = (raw & 0x0001u) != 0;
    const uint16_t rtdCode = raw >> 1;
    const float resistance = (float(rtdCode) / 32768.0f) * MAX31865_RREF;

    out.present = true;
    out.rawRtd = raw;
    out.rtdCode = rtdCode;
    out.rawFaultBit = rawFaultBit;
    out.resistanceOhm = resistance;
    out.fault = fault;
    setStatus(out, "ok");

    if (rtdCode == 0) {
      setStatus(out, "raw_zero");
      return;
    }

    if (rawFaultBit) {
      setStatus(out, "raw_fault_bit");
      return;
    }

    if (fault != 0) {
      setStatus(out, "fault_register");
      return;
    }

    if (!plausibleResistance(resistance)) {
      setStatus(out, "resistance_out_of_range");
      return;
    }

    const float rawTemp = resistanceToTempPt1000(resistance);
    if (isnan(rawTemp)) {
      setStatus(out, "temp_calc_failed");
      return;
    }

    const float temp = rawTemp * cfg.calFactor + cfg.offsetC;
    out.rawTempC = rawTemp;
    out.tempC = temp;

    if (!plausibleTemperature(temp)) {
      setStatus(out, "temp_out_of_range");
      return;
    }

    out.valid = true;
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

  String hex2(uint8_t v) {
    char buf[5];
    snprintf(buf, sizeof(buf), "0x%02X", v);
    return String(buf);
  }

  String hex4(uint16_t v) {
    char buf[7];
    snprintf(buf, sizeof(buf), "0x%04X", v);
    return String(buf);
  }

  String maxChannelKey(MaxChannel ch) {
    switch (ch) {
      case MaxChannel::CH1: return "ch1";
      case MaxChannel::CH2: return "ch2";
      case MaxChannel::CH3: return "ch3";
      default: return "unknown";
    }
  }

  String maxChannelLabel(MaxChannel ch) {
    switch (ch) {
      case MaxChannel::CH1: return "ADC1 / CS_MAX1 / PCF P0";
      case MaxChannel::CH2: return "ADC2 / CS_MAX2 / PCF P1";
      case MaxChannel::CH3: return "ADC3 / CS_MAX3 / PCF P2";
      default: return "Unbekannt";
    }
  }

  uint8_t maxCsBit(MaxChannel ch) {
    switch (ch) {
      case MaxChannel::CH1: return PCF_BIT_MAX1_CS;
      case MaxChannel::CH2: return PCF_BIT_MAX2_CS;
      case MaxChannel::CH3: return PCF_BIT_MAX3_CS;
      default: return 255;
    }
  }

  String maxFaultBitsJson(uint8_t fault) {
    String json = "{";
    json += "\"highThreshold\":" + String((fault & 0x80) ? "true" : "false") + ",";
    json += "\"lowThreshold\":" + String((fault & 0x40) ? "true" : "false") + ",";
    json += "\"refinLow\":" + String((fault & 0x20) ? "true" : "false") + ",";
    json += "\"refinHigh\":" + String((fault & 0x10) ? "true" : "false") + ",";
    json += "\"rtdinLow\":" + String((fault & 0x08) ? "true" : "false") + ",";
    json += "\"overUnderVoltage\":" + String((fault & 0x04) ? "true" : "false");
    json += "}";
    return json;
  }

  String configBitsJson(uint8_t cfg) {
    String json = "{";
    json += "\"vbias\":" + String((cfg & CFG_VBIAS) ? "true" : "false") + ",";
    json += "\"conversionModeAuto\":" + String((cfg & 0x40) ? "true" : "false") + ",";
    json += "\"oneShot\":" + String((cfg & CFG_1SHOT) ? "true" : "false") + ",";
    json += "\"threeWire\":" + String((cfg & CFG_3WIRE) ? "true" : "false") + ",";
    json += "\"faultClear\":" + String((cfg & CFG_FAULT_CLEAR) ? "true" : "false") + ",";
    json += "\"filter50Hz\":" + String((cfg & CFG_FILTER_50HZ) ? "true" : "false");
    json += "}";
    return json;
  }

  void appendReadingJson(String& json, const MaxChannelReading& r) {
    json += "\"present\":" + String(r.present ? "true" : "false") + ",";
    json += "\"valid\":" + String(r.valid ? "true" : "false") + ",";
    json += "\"rawRtd\":" + String(r.rawRtd) + ",";
    json += "\"rtdCode\":" + String(r.rtdCode) + ",";
    json += "\"rawFaultBit\":" + String(r.rawFaultBit ? "true" : "false") + ",";
    json += "\"fault\":" + String(r.fault) + ",";
    json += "\"resistanceOhm\":" + String(isnan(r.resistanceOhm) ? "null" : String(r.resistanceOhm, 2)) + ",";
    json += "\"rawTempC\":" + String(isnan(r.rawTempC) ? "null" : String(r.rawTempC, 2)) + ",";
    json += "\"tempC\":" + String(isnan(r.tempC) ? "null" : String(r.tempC, 2)) + ",";
    json += "\"status\":\"" + String(r.status) + "\"";
  }

  void appendMaxDebugChannelJson(String& json, AppContext& ctx, MaxChannel ch, bool runOneShot) {
    const MaxChannelConfig& c = cfgFor(ctx, ch);
    const MaxChannelReading& last = readingFor(ctx, ch);
    const uint8_t csBit = maxCsBit(ch);

    const uint8_t pcfBefore = Pcf8574Io::currentState();
    const uint8_t pcfReadBefore = Pcf8574Io::readBack();

    Pcf8574Io::deselectMaxCs();
    settleUs(MAX_CS_SETTLE_US);
    const bool selectOk = Pcf8574Io::selectMax(ch);
    settleUs(MAX_CS_SETTLE_US);
    const uint8_t pcfSelected = Pcf8574Io::currentState();
    const uint8_t pcfReadSelected = Pcf8574Io::readBack();
    Pcf8574Io::deselectMaxCs();
    settleUs(MAX_CS_SETTLE_US);

    const uint8_t configBefore = spiRead8(ch, REG_CONFIG);
    const uint8_t faultBefore = spiRead8(ch, REG_FAULT_STAT);
    const uint16_t rawBefore = spiRead16(ch, REG_RTD_MSB);

    uint8_t configAfter = configBefore;
    uint8_t faultAfter = faultBefore;
    uint16_t rawAfter = rawBefore;
    MaxChannelReading oneShotReading;
    resetReading(oneShotReading);

    if (runOneShot) {
      HeatSourcesMax::readChannelNow(ctx, ch);
      oneShotReading = readingFor(ctx, ch);
      configAfter = spiRead8(ch, REG_CONFIG);
      rawAfter = oneShotReading.rawRtd;
      faultAfter = oneShotReading.fault;
    } else {
      evaluateRawReading(oneShotReading, c, rawAfter, faultAfter);
      if (!c.enabled) {
        oneShotReading.valid = false;
        setStatus(oneShotReading, "disabled");
      }
    }

    json += "{";
    json += "\"channel\":\"" + maxChannelKey(ch) + "\",";
    json += "\"label\":\"" + maxChannelLabel(ch) + "\",";
    json += "\"enabled\":" + String(c.enabled ? "true" : "false") + ",";
    json += "\"csBit\":" + String(csBit) + ",";
    json += "\"csSelectOk\":" + String(selectOk ? "true" : "false") + ",";
    json += "\"pcfStateBefore\":\"" + hex2(pcfBefore) + "\",";
    json += "\"pcfReadBefore\":\"" + hex2(pcfReadBefore) + "\",";
    json += "\"pcfStateSelected\":\"" + hex2(pcfSelected) + "\",";
    json += "\"pcfReadSelected\":\"" + hex2(pcfReadSelected) + "\",";
    json += "\"expectedSelectedMask\":\"" + hex2(uint8_t(0xFFu & ~(1u << csBit))) + "\",";
    json += "\"configBefore\":\"" + hex2(configBefore) + "\",";
    json += "\"configBeforeBits\":" + configBitsJson(configBefore) + ",";
    json += "\"faultBefore\":\"" + hex2(faultBefore) + "\",";
    json += "\"faultBeforeBits\":" + maxFaultBitsJson(faultBefore) + ",";
    json += "\"rawBefore\":\"" + hex4(rawBefore) + "\",";
    json += "\"configAfter\":\"" + hex2(configAfter) + "\",";
    json += "\"configAfterBits\":" + configBitsJson(configAfter) + ",";
    json += "\"faultAfter\":\"" + hex2(faultAfter) + "\",";
    json += "\"faultAfterBits\":" + maxFaultBitsJson(faultAfter) + ",";
    json += "\"rawAfter\":\"" + hex4(rawAfter) + "\",";
    appendReadingJson(json, oneShotReading);
    json += ",\"lastReading\":{";
    appendReadingJson(json, last);
    json += "}";
    json += "}";
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
    markDisabled(ctx.maxReadings[i]);
  }

  g_started = true;

  Serial.println("HeatSourcesMax::begin() OK");
  Serial.flush();

  return true;
}

bool readChannelNow(AppContext& ctx, MaxChannel ch) {
  MaxChannelReading& out = readingFor(ctx, ch);
  MaxChannelRuntime& rt = runtimeFor(ctx, ch);

  rt.step = MaxReadStep::IDLE;
  rt.tMarkUs = 0;
  rt.cycleDone = true;

  if (!g_started) {
    resetReading(out);
    setStatus(out, "not_started");
    return false;
  }

  if (!channelEnabled(ctx, ch)) {
    markDisabled(out);
    return false;
  }

  resetReading(out);

  // This exact sequence is intentionally blocking and deterministic. It mirrors
  // the successful /api/max-debug one-shot path and avoids the old runtime path
  // returning early RTD values such as rawRtd=2/212.
  Pcf8574Io::deselectMaxCs();
  settleUs(MAX_CS_SETTLE_US);

  clearFault(ch);
  settleUs(2000);

  setBiasOn(ch);
  settleUs(MAX_BIAS_WAIT_US);

  startOneShot(ch);
  settleUs(MAX_CONV_WAIT_US);

  const uint16_t raw = spiRead16(ch, REG_RTD_MSB);
  const uint8_t fault = spiRead8(ch, REG_FAULT_STAT);

  evaluateRawReading(out, cfgFor(ctx, ch), raw, fault);

  if (out.fault != 0 || out.rawFaultBit) {
    clearFault(ch);
  }

  Pcf8574Io::deselectMaxCs();
  return out.valid;
}

void readAllNow(AppContext& ctx) {
  readChannelNow(ctx, MaxChannel::CH1);
  readChannelNow(ctx, MaxChannel::CH2);
  readChannelNow(ctx, MaxChannel::CH3);
  HeatSourceAssignments::resolveHeatSources(ctx);
}

void startCycle(AppContext& ctx) {
  if (!g_started) return;
  if (anyCycleInProgress(ctx)) return;

  // Centralized runtime read: startCycle now performs a complete measurement
  // cycle synchronously, so status, regulation and debug use one authoritative
  // MAX31865 reading path.
  readAllNow(ctx);
}

void process(AppContext& ctx) {
  if (!g_started) return;

  // The runtime read is synchronous in startCycle(). Keep this function for the
  // existing FSM contract and make sure heat sources are resolved after reads.
  if (allDone(ctx)) {
    HeatSourceAssignments::resolveHeatSources(ctx);
  }
}

bool cycleComplete(const AppContext& ctx) {
  return allDone(ctx);
}

String debugJson(AppContext& ctx, bool runOneShot) {
  String json = "{";
  json += "\"started\":" + String(g_started ? "true" : "false") + ",";
  json += "\"runOneShot\":" + String(runOneShot ? "true" : "false") + ",";
  json += "\"spiHz\":" + String(MAX_SPI_HZ) + ",";
  json += "\"spiMode\":\"MODE1\",";
  json += "\"pinSck\":" + String(PIN_MAX_SCK) + ",";
  json += "\"pinMiso\":" + String(PIN_MAX_MISO) + ",";
  json += "\"pinMosi\":" + String(PIN_MAX_MOSI) + ",";
  json += "\"pcfMaxAddr\":\"0x";
  if (PCF8574_MAX_ADDR < 16) json += "0";
  json += String(PCF8574_MAX_ADDR, HEX);
  json += "\",";
  json += "\"pcfCurrentState\":\"" + hex2(Pcf8574Io::currentState()) + "\",";
  json += "\"pcfReadBack\":\"" + hex2(Pcf8574Io::readBack()) + "\",";
  json += "\"rNominalOhm\":" + String(MAX31865_RNOMINAL, 2) + ",";
  json += "\"rRefOhm\":" + String(MAX31865_RREF, 2) + ",";
  json += "\"wireMode\":\"" + String(MAX31865_USE_3WIRE ? "3-wire" : "2/4-wire") + "\",";
  json += "\"validResistanceMinOhm\":" + String(MAX31865_VALID_R_MIN_OHM, 1) + ",";
  json += "\"validResistanceMaxOhm\":" + String(MAX31865_VALID_R_MAX_OHM, 1) + ",";
  json += "\"validTempMinC\":" + String(MAX31865_VALID_TEMP_MIN_C, 1) + ",";
  json += "\"validTempMaxC\":" + String(MAX31865_VALID_TEMP_MAX_C, 1) + ",";
  json += "\"channels\":[";
  appendMaxDebugChannelJson(json, ctx, MaxChannel::CH1, runOneShot);
  json += ",";
  appendMaxDebugChannelJson(json, ctx, MaxChannel::CH2, runOneShot);
  json += ",";
  appendMaxDebugChannelJson(json, ctx, MaxChannel::CH3, runOneShot);
  json += "]}";

  Pcf8574Io::deselectMaxCs();
  return json;
}

}
