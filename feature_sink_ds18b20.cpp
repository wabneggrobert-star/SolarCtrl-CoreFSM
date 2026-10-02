#include "feature_sink_ds18b20.h"
#include "config.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

namespace {
  OneWire oneWire(PIN_ONEWIRE);
  DallasTemperature ds18b20(&oneWire);
  bool g_started = false;

  enum class DsFsmStep : uint8_t {
    IDLE = 0,
    WAIT_CONVERSION,
    READ_DEVICE
  };

  DsFsmStep g_step = DsFsmStep::IDLE;
  uint32_t g_conversionStartedMs = 0;
  uint8_t g_readIndex = 0;
  bool g_cycleDone = false;

  bool plausible(float t) {
    return !isnan(t) && t >= SINK_TEMP_MIN_C && t <= SINK_TEMP_MAX_C;
  }

  bool sameAddress(const uint8_t a[8], const uint8_t b[8]) {
    for (uint8_t i = 0; i < 8; i++) {
      if (a[i] != b[i]) return false;
    }
    return true;
  }

  const Ds18b20DeviceInfo* findOldDevice(
    const Ds18b20Inventory& inventory,
    const uint8_t address[8]
  ) {
    for (uint8_t i = 0; i < inventory.count; i++) {
      if (inventory.devices[i].present && sameAddress(inventory.devices[i].address, address)) {
        return &inventory.devices[i];
      }
    }
    return nullptr;
  }

  void clearInventory(Ds18b20Inventory& inventory) {
    inventory.count = 0;

    for (uint8_t i = 0; i < MAX_DS18B20_SENSORS; i++) {
      inventory.devices[i] = Ds18b20DeviceInfo{};
      inventory.devices[i].error = Ds18Error::NOT_READ;
    }
  }

  void finishCycle() {
    g_step = DsFsmStep::IDLE;
    g_readIndex = 0;
    g_cycleDone = true;
  }

  void updateDeviceReading(Ds18b20DeviceInfo& dev) {
    const uint32_t now = millis();
    const float t = ds18b20.getTempC(dev.address);

    dev.lastUpdateMs = now;
    dev.sequence++;

    if (t == DEVICE_DISCONNECTED_C) {
      dev.lastTempC = NAN;
      dev.lastValid = false;
      dev.error = Ds18Error::DISCONNECTED;
      return;
    }

    if (!plausible(t)) {
      dev.lastTempC = NAN;
      dev.lastValid = false;
      dev.error = Ds18Error::OUT_OF_RANGE;
      return;
    }

    dev.lastTempC = t;
    dev.lastValid = true;
    dev.error = Ds18Error::NONE;
  }
}

namespace SinkSensor {

bool begin() {
  ds18b20.begin();

  // Zentraler Punkt des Refactorings: DallasTemperature darf nicht auf die
  // DS18B20-Conversion warten. Die Wartezeit wird von der FSM ueber millis()
  // abgebildet, waehrend der ESP alle anderen Aufgaben weiterbearbeitet.
  ds18b20.setWaitForConversion(false);

  g_step = DsFsmStep::IDLE;
  g_conversionStartedMs = 0;
  g_readIndex = 0;
  g_cycleDone = false;
  g_started = true;
  return true;
}

String addressToString(const uint8_t address[8]) {
  char buf[24];
  snprintf(
    buf,
    sizeof(buf),
    "%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X",
    address[0], address[1], address[2], address[3],
    address[4], address[5], address[6], address[7]
  );
  return String(buf);
}

bool parseAddressString(const String& text, uint8_t address[8]) {
  unsigned int vals[8];
  int parsed = sscanf(
    text.c_str(),
    "%2X-%2X-%2X-%2X-%2X-%2X-%2X-%2X",
    &vals[0], &vals[1], &vals[2], &vals[3],
    &vals[4], &vals[5], &vals[6], &vals[7]
  );

  if (parsed != 8) return false;

  for (uint8_t i = 0; i < 8; i++) {
    address[i] = static_cast<uint8_t>(vals[i]);
  }

  return true;
}

uint8_t scanBus(Ds18b20Inventory& inventory) {
  if (!g_started) return 0;

  // Einen laufenden Messzyklus nicht durch einen UI-Scan zerstoeren.
  if (cycleActive()) return inventory.count;

  const Ds18b20Inventory previous = inventory;
  clearInventory(inventory);

  oneWire.reset_search();
  uint8_t addr[8];

  while (oneWire.search(addr)) {
    if (inventory.count >= MAX_DS18B20_SENSORS) break;
    if (OneWire::crc8(addr, 7) != addr[7]) continue;
    if (addr[0] != 0x28) continue;

    Ds18b20DeviceInfo& dev = inventory.devices[inventory.count];
    const Ds18b20DeviceInfo* old = findOldDevice(previous, addr);
    if (old) {
      dev = *old; // Snapshot/Sequence bei einem Re-Scan erhalten.
    } else {
      dev = Ds18b20DeviceInfo{};
      dev.error = Ds18Error::NOT_READ;
    }

    dev.present = true;
    for (uint8_t i = 0; i < 8; i++) dev.address[i] = addr[i];

    String txt = addressToString(addr);
    txt.toCharArray(dev.addressText, sizeof(dev.addressText));
    inventory.count++;
  }

  return inventory.count;
}

bool startCycle(Ds18b20Inventory& inventory) {
  if (!g_started || cycleActive()) return false;

  g_cycleDone = false;
  g_readIndex = 0;

  if (inventory.count == 0) {
    finishCycle();
    return true;
  }

  ds18b20.requestTemperatures();
  g_conversionStartedMs = millis();
  g_step = DsFsmStep::WAIT_CONVERSION;
  return true;
}

void process(Ds18b20Inventory& inventory) {
  if (!g_started || g_step == DsFsmStep::IDLE) return;

  if (g_step == DsFsmStep::WAIT_CONVERSION) {
    if ((uint32_t)(millis() - g_conversionStartedMs) < SENSOR_CONVERSION_WAIT_MS) {
      return;
    }

    g_readIndex = 0;
    g_step = DsFsmStep::READ_DEVICE;
  }

  if (g_step != DsFsmStep::READ_DEVICE) return;

  // Pro loop()-Durchlauf nur einen Sensor lesen. Damit bleibt auch das Auslesen
  // vieler DS18B20 kurz und andere FSMs erhalten Rechenzeit.
  while (g_readIndex < inventory.count && !inventory.devices[g_readIndex].present) {
    g_readIndex++;
  }

  if (g_readIndex >= inventory.count) {
    finishCycle();
    return;
  }

  updateDeviceReading(inventory.devices[g_readIndex]);
  g_readIndex++;

  if (g_readIndex >= inventory.count) {
    finishCycle();
  }
}

bool cycleComplete() {
  return g_cycleDone && g_step == DsFsmStep::IDLE;
}

bool cycleActive() {
  return g_step != DsFsmStep::IDLE;
}

bool readCachedByAddress(
  const Ds18b20Inventory& inventory,
  const uint8_t address[8],
  float& tempC,
  bool& valid
) {
  tempC = NAN;
  valid = false;

  for (uint8_t i = 0; i < inventory.count; i++) {
    const Ds18b20DeviceInfo& dev = inventory.devices[i];
    if (!dev.present || !sameAddress(dev.address, address)) continue;

    tempC = dev.lastTempC;
    valid = dev.lastValid && isfinite(dev.lastTempC);
    return valid;
  }

  return false;
}

bool inventoryContainsAddress(const Ds18b20Inventory& inventory, const uint8_t address[8]) {
  for (uint8_t i = 0; i < inventory.count; i++) {
    if (inventory.devices[i].present && sameAddress(inventory.devices[i].address, address)) {
      return true;
    }
  }
  return false;
}

}
