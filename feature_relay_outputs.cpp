#include "feature_relay_outputs.h"
#include "config.h"

#include <Arduino.h>
#include <Wire.h>

namespace {
  uint8_t g_state = 0xFF;
  bool g_started = false;

  bool validRelay(uint8_t relayIndex) {
    return relayIndex < RELAY_COUNT;
  }

  bool applyState() {
    Wire.beginTransmission(PCF8574_RELAY_ADDR);
    Wire.write(g_state);
    return Wire.endTransmission() == 0;
  }

  void setRawBit(uint8_t bit, bool activeLow, bool on) {
    if (bit >= 8) return;

    bool highLevel = activeLow ? !on : on;

    if (highLevel) {
      g_state |= (1u << bit);
    } else {
      g_state &= ~(1u << bit);
    }
  }
}

namespace RelayOutputs {

bool begin(AppContext& ctx) {
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

  // Sicherheitszustand: alle PCF-Ausgaenge HIGH.
  // Bei den ueblichen aktiven-Low Relaismodulen bedeutet das AUS.
  // Wichtig: hier noch nicht ctx.config.relays auswerten, weil begin() auch
  // vor dem Laden der SD-Konfiguration laufen kann.
  g_state = 0xFF;
  g_started = applyState();

  Serial.print("Relais-PCF 0x");
  Serial.print(PCF8574_RELAY_ADDR, HEX);
  Serial.print(": ");
  Serial.println(g_started ? "OK" : "FEHLER");

  for (uint8_t i = 0; i < RELAY_COUNT; i++) {
    ctx.relayRuntime[i].state = false;
  }

  return g_started;
}

bool set(AppContext& ctx, uint8_t relayIndex, bool on) {
  if (!g_started) return false;
  if (!validRelay(relayIndex)) return false;

  RelayOutputConfig& cfg = ctx.config.relays[relayIndex];

  if (!cfg.enabled || cfg.function == RelayFunction::NONE) {
    on = false;
  }

  setRawBit(relayIndex, cfg.activeLow, on);

  if (!applyState()) {
    Serial.print("RELAIS ");
    Serial.print(relayIndex);
    Serial.println(" -> PCF FEHLER");
    Serial.flush();
    return false;
  }

  ctx.relayRuntime[relayIndex].state = on;

  Serial.print("RELAIS ");
  Serial.print(relayIndex);
  Serial.print(" -> ");
  Serial.print(on ? "EIN" : "AUS");

  Serial.print(" | Funktion: ");

  switch (cfg.function) {
    case RelayFunction::PUMP_ENABLE:
      Serial.print("PUMPE");
      break;

    case RelayFunction::ZONE_VALVE:
      Serial.print("ZONENVENTIL");
      break;

    default:
      Serial.print("NONE");
      break;
  }

  Serial.print(" | ActiveLow=");
  Serial.println(cfg.activeLow ? "true" : "false");
  Serial.flush();

  return true;
}

bool testSetRaw(AppContext& ctx, uint8_t relayIndex, bool on) {
  if (!g_started) return false;
  if (!validRelay(relayIndex)) return false;

  const bool activeLow = ctx.config.relays[relayIndex].activeLow;
  setRawBit(relayIndex, activeLow, on);

  if (!applyState()) {
    Serial.print("TEST RELAIS ");
    Serial.print(relayIndex + 1);
    Serial.println(" -> PCF FEHLER");
    Serial.flush();
    return false;
  }

  ctx.relayRuntime[relayIndex].state = on;

  Serial.print("TEST RELAIS ");
  Serial.print(relayIndex + 1);
  Serial.print(" -> ");
  Serial.println(on ? "EIN" : "AUS");
  Serial.flush();

  return true;
}

bool get(const AppContext& ctx, uint8_t relayIndex) {
  if (!validRelay(relayIndex)) return false;
  return ctx.relayRuntime[relayIndex].state;
}

void allOff(AppContext& ctx) {
  if (!g_started) return;

  for (uint8_t i = 0; i < RELAY_COUNT; i++) {
    setRawBit(i, ctx.config.relays[i].activeLow, false);
    ctx.relayRuntime[i].state = false;
  }

  applyState();
}

bool configure(
  AppContext& ctx,
  uint8_t relayIndex,
  bool enabled,
  RelayFunction function,
  bool activeLow
) {
  if (!validRelay(relayIndex)) return false;

  ctx.config.relays[relayIndex].enabled = enabled;
  ctx.config.relays[relayIndex].function = function;
  ctx.config.relays[relayIndex].activeLow = activeLow;

  if (!enabled || function == RelayFunction::NONE) {
    set(ctx, relayIndex, false);
  }

  return true;
}

bool isUsableAsPumpEnable(const AppContext& ctx, uint8_t relayIndex) {
  if (!validRelay(relayIndex)) return false;

  const RelayOutputConfig& cfg = ctx.config.relays[relayIndex];
  return cfg.enabled && cfg.function == RelayFunction::PUMP_ENABLE;
}

bool isUsableAsZoneValve(const AppContext& ctx, uint8_t relayIndex) {
  if (!validRelay(relayIndex)) return false;

  const RelayOutputConfig& cfg = ctx.config.relays[relayIndex];
  return cfg.enabled && cfg.function == RelayFunction::ZONE_VALVE;
}

const char* functionToKey(RelayFunction f) {
  switch (f) {
    case RelayFunction::PUMP_ENABLE:
      return "pump_enable";
    case RelayFunction::ZONE_VALVE:
      return "zone_valve";
    case RelayFunction::HEATER_ROD:
      return "heater_rod";
    case RelayFunction::MIXER:
      return "mixer";
    case RelayFunction::NONE:
    default:
      return "none";
  }
}

RelayFunction functionFromKey(const String& key) {
  if (key == "pump_enable") return RelayFunction::PUMP_ENABLE;
  if (key == "zone_valve") return RelayFunction::ZONE_VALVE;
  if (key == "heater_rod") return RelayFunction::HEATER_ROD;
  if (key == "mixer") return RelayFunction::MIXER;
  return RelayFunction::NONE;
}

}

bool RelayOutputs::isUsableAsHeaterRod(const AppContext& ctx, uint8_t relayIndex) {
  if (relayIndex >= RELAY_COUNT) return false;
  const RelayOutputConfig& cfg = ctx.config.relays[relayIndex];
  return cfg.enabled && cfg.function == RelayFunction::HEATER_ROD;
}


bool RelayOutputs::isUsableAsMixer(const AppContext& ctx, uint8_t relayIndex) {
  if (relayIndex >= RELAY_COUNT) return false;
  const RelayOutputConfig& cfg = ctx.config.relays[relayIndex];
  return cfg.enabled && cfg.function == RelayFunction::MIXER;
}
