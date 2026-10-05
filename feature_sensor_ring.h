#pragma once

#include <Arduino.h>
#include "app_types.h"

// Stage 3A: fester RAM-Ring fuer konsistente, vollstaendig abgeschlossene
// Sensorsnapshots. Der Ring ist absichtlich unabhaengig von SD, History und UI.
// Die bestehende Regelung liest weiterhin direkt aus AppContext.
namespace SensorRing {

static constexpr uint16_t CAPACITY = 64;

struct Record {
  uint32_t sequence = 0;
  uint32_t capturedMs = 0;
  uint32_t epoch = 0;  // 0 solange die Systemzeit noch nicht gueltig ist.

  uint32_t ds18PresentMask = 0;
  uint32_t ds18ValidMask = 0;
  float ds18C[MAX_DS18B20_SENSORS];

  uint8_t maxPresentMask = 0;
  uint8_t maxValidMask = 0;
  uint8_t ds18Count = 0;
  uint8_t reserved = 0;
  float max31865C[MAX_MAX31865_CHANNELS];
};

struct Stats {
  uint16_t capacity = CAPACITY;
  uint16_t count = 0;
  uint16_t recordSize = sizeof(Record);
  uint16_t reserved = 0;
  uint32_t totalWrites = 0;
  uint32_t overwrites = 0;
  uint32_t lastSequence = 0;
  uint32_t lastCapturedMs = 0;
  uint32_t lastAgeMs = 0;
  uint32_t maxCaptureUs = 0;
};

void begin();

// Genau einmal nach einem vollstaendig abgeschlossenen DS18B20/MAX31865-Zyklus
// aufrufen. Keine Heap-Allokation, kein String, kein SD-Zugriff.
void capture(const AppContext& ctx);

Stats stats();

// Kopiert den neuesten Record. Liefert false, solange noch kein Record vorliegt.
bool copyLatest(Record& out);

// age=0 = neuester Record, age=1 = vorletzter usw.
// Diese Schnittstelle ist fuer spaetere Stage-3B-Consumer vorbereitet.
bool copyNewest(uint16_t age, Record& out);

// Diagnose-Endpunkt fuer Stage-3A-Abnahme. Erzeugt Strings nur auf der
// Service/UI-Seite; capture() selbst bleibt heap- und String-frei.
String json();

}  // namespace SensorRing
