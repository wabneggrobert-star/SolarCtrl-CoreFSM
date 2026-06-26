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

  bool plausible(float t) {
    return !isnan(t) && t >= SINK_TEMP_MIN_C && t <= SINK_TEMP_MAX_C;
  }

  bool readByAddress(const uint8_t address[8], float& tempC, bool& valid) {
    tempC = NAN;
    valid = false;

    if (!g_started) return false;

    ds18b20.requestTemperatures();
    delay(SENSOR_CONVERSION_WAIT_MS);

    tempC = ds18b20.getTempC(address);
    if (tempC == DEVICE_DISCONNECTED_C) {
      tempC = NAN;
      valid = false;
      return false;
    }

    valid = plausible(tempC);
    if (!valid) tempC = NAN;

    return valid;
  }

  void clearInventory(Ds18b20Inventory& inventory) {
    inventory.count = 0;

    for (uint8_t i = 0; i < MAX_DS18B20_SENSORS; i++) {
      inventory.devices[i].present = false;
      inventory.devices[i].addressText[0] = '\0';
      inventory.devices[i].lastTempC = NAN;
      inventory.devices[i].lastValid = false;
      inventory.devices[i].role = Ds18Role::NONE;

      for (uint8_t j = 0; j < 8; j++) {
        inventory.devices[i].address[j] = 0;
      }
    }
  }
}

namespace SinkSensor {

bool begin() {
  ds18b20.begin();
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
  clearInventory(inventory);
  if (!g_started) return 0;

  oneWire.reset_search();
  uint8_t addr[8];

  while (oneWire.search(addr)) {
    if (inventory.count >= MAX_DS18B20_SENSORS) break;
    if (OneWire::crc8(addr, 7) != addr[7]) continue;
    if (addr[0] != 0x28) continue;

    Ds18b20DeviceInfo& dev = inventory.devices[inventory.count];
    dev.present = true;

    for (uint8_t i = 0; i < 8; i++) {
      dev.address[i] = addr[i];
    }

    String txt = addressToString(addr);
    txt.toCharArray(dev.addressText, sizeof(dev.addressText));

    inventory.count++;
  }

  refreshInventoryTemps(inventory);
  return inventory.count;
}

void refreshInventoryTemps(Ds18b20Inventory& inventory) {
  if (!g_started) return;

  ds18b20.requestTemperatures();
  delay(SENSOR_CONVERSION_WAIT_MS);

  for (uint8_t i = 0; i < inventory.count; i++) {
    Ds18b20DeviceInfo& dev = inventory.devices[i];
    if (!dev.present) continue;

    float t = ds18b20.getTempC(dev.address);
    if (t == DEVICE_DISCONNECTED_C) {
      dev.lastTempC = NAN;
      dev.lastValid = false;
      continue;
    }

    bool valid = plausible(t);
    dev.lastValid = valid;
    dev.lastTempC = valid ? t : NAN;
  }
}

bool inventoryContainsAddress(const Ds18b20Inventory& inventory, const uint8_t address[8]) {
  for (uint8_t i = 0; i < inventory.count; i++) {
    bool equal = true;

    for (uint8_t j = 0; j < 8; j++) {
      if (inventory.devices[i].address[j] != address[j]) {
        equal = false;
        break;
      }
    }

    if (equal) return true;
  }

  return false;
}

bool autoAssignSink(Ds18b20Inventory& inventory, SensorAssignment& assignment) {
  assignment.sinkAssigned = false;
  assignment.sinkAddressText[0] = '\0';

  for (uint8_t i = 0; i < 8; i++) {
    assignment.sinkAddress[i] = 0;
  }

  if (inventory.count == 1 && inventory.devices[0].present) {
    assignment.sinkAssigned = true;

    for (uint8_t i = 0; i < 8; i++) {
      assignment.sinkAddress[i] = inventory.devices[0].address[i];
    }

    strncpy(
      assignment.sinkAddressText,
      inventory.devices[0].addressText,
      sizeof(assignment.sinkAddressText) - 1
    );
    assignment.sinkAddressText[sizeof(assignment.sinkAddressText) - 1] = '\0';

    return true;
  }

  return false;
}

bool assignSinkByAddressText(const Ds18b20Inventory& inventory, const String& addressText, SensorAssignment& assignment) {
  uint8_t parsed[8] = {0};

  if (!parseAddressString(addressText, parsed)) return false;
  if (!inventoryContainsAddress(inventory, parsed)) return false;

  assignment.sinkAssigned = true;

  for (uint8_t i = 0; i < 8; i++) {
    assignment.sinkAddress[i] = parsed[i];
  }

  addressText.toCharArray(assignment.sinkAddressText, sizeof(assignment.sinkAddressText));
  return true;
}

bool hasAssignedSink(const SensorAssignment& assignment) {
  return assignment.sinkAssigned;
}

bool readAssignedSink(const SensorAssignment& assignment, float& tempC, bool& valid) {
  tempC = NAN;
  valid = false;

  if (!assignment.sinkAssigned) return false;
  return readByAddress(assignment.sinkAddress, tempC, valid);
}

}