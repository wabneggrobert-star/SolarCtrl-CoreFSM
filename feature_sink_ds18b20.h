#pragma once
#include <Arduino.h>
#include "app_types.h"

namespace SinkSensor {
  bool begin();

  // Bus-Inventar erfassen. Die Messwerte bereits bekannter Adressen bleiben
  // beim Re-Scan erhalten. Es wird keine blockierende Temperaturmessung gestartet.
  uint8_t scanBus(Ds18b20Inventory& inventory);

  // Nicht blockierender DS18B20-Messzyklus.
  // startCycle() startet die gemeinsame Conversion aller Sensoren und kehrt sofort zurueck.
  // process() wird aus der App-FSM wiederholt aufgerufen und liest nach Ablauf der
  // Conversion-Zeit pro Aufruf hoechstens einen Sensor aus.
  bool startCycle(Ds18b20Inventory& inventory);
  void process(Ds18b20Inventory& inventory);
  bool cycleComplete();
  bool cycleActive();

  // Reiner Snapshot-Zugriff; fuehrt keine OneWire-Conversion und keine Wartezeit aus.
  bool readCachedByAddress(
    const Ds18b20Inventory& inventory,
    const uint8_t address[8],
    float& tempC,
    bool& valid
  );

  bool inventoryContainsAddress(const Ds18b20Inventory& inventory, const uint8_t address[8]);

  String addressToString(const uint8_t address[8]);
  bool parseAddressString(const String& text, uint8_t address[8]);
}
