#include "feature_sensor_ring.h"
#include "feature_time.h"

namespace {
static_assert(MAX_DS18B20_SENSORS <= 32, "SensorRing ds18 mask supports max. 32 sensors");
static_assert(MAX_MAX31865_CHANNELS <= 8, "SensorRing MAX mask supports max. 8 channels");

portMUX_TYPE g_ringMux = portMUX_INITIALIZER_UNLOCKED;
SensorRing::Record g_records[SensorRing::CAPACITY];
uint16_t g_writeIndex = 0;
uint16_t g_count = 0;
uint32_t g_totalWrites = 0;
uint32_t g_overwrites = 0;
uint32_t g_lastSequence = 0;
uint32_t g_lastCapturedMs = 0;
uint32_t g_maxCaptureUs = 0;
}

namespace SensorRing {

void begin() {
  portENTER_CRITICAL(&g_ringMux);
  g_writeIndex = 0;
  g_count = 0;
  g_totalWrites = 0;
  g_overwrites = 0;
  g_lastSequence = 0;
  g_lastCapturedMs = 0;
  g_maxCaptureUs = 0;
  portEXIT_CRITICAL(&g_ringMux);
}

void capture(const AppContext& ctx) {
  const uint32_t startedUs = micros();

  Record rec;
  rec.capturedMs = millis();
  rec.epoch = (uint32_t)TimeService::nowEpoch();
  rec.ds18Count = ctx.ds18b20.count;

  for (uint8_t i = 0; i < MAX_DS18B20_SENSORS; ++i) {
    rec.ds18C[i] = NAN;

    const Ds18b20DeviceInfo& device = ctx.ds18b20.devices[i];
    if (device.present) rec.ds18PresentMask |= (1UL << i);
    if (device.present && device.lastValid) {
      rec.ds18ValidMask |= (1UL << i);
      rec.ds18C[i] = device.lastTempC;
    }
  }

  for (uint8_t i = 0; i < MAX_MAX31865_CHANNELS; ++i) {
    rec.max31865C[i] = NAN;

    const MaxChannelReading& reading = ctx.maxReadings[i];
    if (reading.present) rec.maxPresentMask |= (uint8_t)(1U << i);
    if (reading.present && reading.valid) {
      rec.maxValidMask |= (uint8_t)(1U << i);
      rec.max31865C[i] = reading.tempC;
    }
  }

  // Nur der kurze feste Record-Copy liegt im Critical-Abschnitt. Das Erzeugen
  // des Snapshots und der Zeitstempel geschieht davor.
  portENTER_CRITICAL(&g_ringMux);
  rec.sequence = g_lastSequence + 1U;
  g_records[g_writeIndex] = rec;
  g_writeIndex = (uint16_t)((g_writeIndex + 1U) % CAPACITY);
  if (g_count < CAPACITY) {
    ++g_count;
  } else {
    ++g_overwrites;
  }
  ++g_totalWrites;
  g_lastSequence = rec.sequence;
  g_lastCapturedMs = rec.capturedMs;
  portEXIT_CRITICAL(&g_ringMux);

  const uint32_t elapsedUs = (uint32_t)(micros() - startedUs);
  portENTER_CRITICAL(&g_ringMux);
  if (elapsedUs > g_maxCaptureUs) g_maxCaptureUs = elapsedUs;
  portEXIT_CRITICAL(&g_ringMux);
}

Stats stats() {
  Stats out;
  portENTER_CRITICAL(&g_ringMux);
  out.count = g_count;
  out.totalWrites = g_totalWrites;
  out.overwrites = g_overwrites;
  out.lastSequence = g_lastSequence;
  out.lastCapturedMs = g_lastCapturedMs;
  out.maxCaptureUs = g_maxCaptureUs;
  portEXIT_CRITICAL(&g_ringMux);

  if (out.count > 0) out.lastAgeMs = (uint32_t)(millis() - out.lastCapturedMs);
  return out;
}

bool copyNewest(uint16_t age, Record& out) {
  bool ok = false;
  portENTER_CRITICAL(&g_ringMux);
  if (age < g_count) {
    const uint16_t newest = (uint16_t)((g_writeIndex + CAPACITY - 1U) % CAPACITY);
    const uint16_t index = (uint16_t)((newest + CAPACITY - age) % CAPACITY);
    out = g_records[index];
    ok = true;
  }
  portEXIT_CRITICAL(&g_ringMux);
  return ok;
}

bool copyLatest(Record& out) {
  return copyNewest(0, out);
}

String json() {
  const Stats s = stats();
  Record latest;
  const bool hasLatest = copyLatest(latest);

  String out;
  out.reserve(900);
  out += "{";
  out += "\"capacity\":" + String(s.capacity) + ",";
  out += "\"count\":" + String(s.count) + ",";
  out += "\"recordSize\":" + String(s.recordSize) + ",";
  out += "\"ramBytes\":" + String((uint32_t)s.capacity * (uint32_t)s.recordSize) + ",";
  out += "\"totalWrites\":" + String(s.totalWrites) + ",";
  out += "\"overwrites\":" + String(s.overwrites) + ",";
  out += "\"lastSequence\":" + String(s.lastSequence) + ",";
  out += "\"lastCapturedMs\":" + String(s.lastCapturedMs) + ",";
  out += "\"lastAgeMs\":" + String(s.lastAgeMs) + ",";
  out += "\"maxCaptureUs\":" + String(s.maxCaptureUs) + ",";
  out += "\"latest\":";

  if (!hasLatest) {
    out += "null}";
    return out;
  }

  out += "{";
  out += "\"sequence\":" + String(latest.sequence) + ",";
  out += "\"capturedMs\":" + String(latest.capturedMs) + ",";
  out += "\"epoch\":" + String(latest.epoch) + ",";
  out += "\"ds18Count\":" + String(latest.ds18Count) + ",";
  out += "\"ds18PresentMask\":" + String(latest.ds18PresentMask) + ",";
  out += "\"ds18ValidMask\":" + String(latest.ds18ValidMask) + ",";
  out += "\"ds18C\":[";
  for (uint8_t i = 0; i < MAX_DS18B20_SENSORS; ++i) {
    if (i) out += ",";
    if (latest.ds18ValidMask & (1UL << i)) out += String(latest.ds18C[i], 3);
    else out += "null";
  }
  out += "],";
  out += "\"maxPresentMask\":" + String((unsigned)latest.maxPresentMask) + ",";
  out += "\"maxValidMask\":" + String((unsigned)latest.maxValidMask) + ",";
  out += "\"max31865C\":[";
  for (uint8_t i = 0; i < MAX_MAX31865_CHANNELS; ++i) {
    if (i) out += ",";
    if (latest.maxValidMask & (uint8_t)(1U << i)) out += String(latest.max31865C[i], 3);
    else out += "null";
  }
  out += "]}}";
  return out;
}

}  // namespace SensorRing
