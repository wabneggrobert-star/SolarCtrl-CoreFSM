#pragma once
#include <Arduino.h>
#include "app_types.h"

namespace SinkSensor {
  bool begin();

  uint8_t scanBus(Ds18b20Inventory& inventory);
  void refreshInventoryTemps(Ds18b20Inventory& inventory);

  bool autoAssignSink(Ds18b20Inventory& inventory, SensorAssignment& assignment);
  bool hasAssignedSink(const SensorAssignment& assignment);
  bool readAssignedSink(const SensorAssignment& assignment, float& tempC, bool& valid);

  bool inventoryContainsAddress(const Ds18b20Inventory& inventory, const uint8_t address[8]);
  bool assignSinkByAddressText(const Ds18b20Inventory& inventory, const String& addressText, SensorAssignment& assignment);

  String addressToString(const uint8_t address[8]);
  bool parseAddressString(const String& text, uint8_t address[8]);
}