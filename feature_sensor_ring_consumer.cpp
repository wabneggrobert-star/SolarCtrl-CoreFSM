#include "feature_sensor_ring_consumer.h"
#include <math.h>

namespace {
portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;
SensorRing::Cursor g_cursor;
SensorRingConsumer::Stats g_stats;

uint32_t recordChecksum(const SensorRing::Record& rec) {
  // Lightweight diagnostic checksum. It is not a protocol CRC; it only proves
  // that the consumer actually touched the copied payload.
  uint32_t h = 2166136261UL;
  const uint8_t* p = reinterpret_cast<const uint8_t*>(&rec);
  for (size_t i = 0; i < sizeof(rec); ++i) {
    h ^= p[i];
    h *= 16777619UL;
  }
  return h;
}
}

namespace SensorRingConsumer {

void begin() {
  SensorRing::initCursor(g_cursor, SensorRing::CursorStart::OLDEST_AVAILABLE);
  portENTER_CRITICAL(&g_mux);
  g_stats = Stats{};
  g_stats.initialized = true;
  portEXIT_CRITICAL(&g_mux);
}

void process(uint8_t maxRecords) {
  if (maxRecords == 0) return;
  const uint32_t startedUs = micros();

  uint8_t processed = 0;
  while (processed < maxRecords) {
    SensorRing::Record rec;
    const SensorRing::CursorStats beforeRead = SensorRing::cursorStats(g_cursor);
    const uint32_t readStartedUs = micros();
    if (!SensorRing::readNext(g_cursor, rec)) break;
    const uint32_t readUs = (uint32_t)(micros() - readStartedUs);
    const SensorRing::CursorStats afterRead = SensorRing::cursorStats(g_cursor);

    uint32_t seqErrors = 0;
    uint32_t previousSequence = 0;
    portENTER_CRITICAL(&g_mux);
    previousSequence = g_stats.lastSequence;
    portEXIT_CRITICAL(&g_mux);

    // A sequence jump is only an error when the ring did not explicitly
    // account for lost records because the consumer was too slow.
    const bool droppedDuringRead = afterRead.droppedRecords > beforeRead.droppedRecords;
    if (!droppedDuringRead && previousSequence != 0 && rec.sequence != previousSequence + 1U) {
      ++seqErrors;
    }

    const uint32_t checksum = recordChecksum(rec);
    portENTER_CRITICAL(&g_mux);
    ++g_stats.totalReads;
    g_stats.lastSequence = rec.sequence;
    g_stats.lastChecksum = checksum;
    if (seqErrors) g_stats.sequenceErrors += seqErrors;
    if (readUs > g_stats.maxSingleReadUs) g_stats.maxSingleReadUs = readUs;
    portEXIT_CRITICAL(&g_mux);
    ++processed;
  }

  const SensorRing::CursorStats cursorStats = SensorRing::cursorStats(g_cursor);
  const uint32_t elapsedUs = (uint32_t)(micros() - startedUs);
  portENTER_CRITICAL(&g_mux);
  g_stats.droppedRecords = cursorStats.droppedRecords;
  g_stats.pending = cursorStats.pending;
  if (elapsedUs > g_stats.maxProcessUs) g_stats.maxProcessUs = elapsedUs;
  portEXIT_CRITICAL(&g_mux);
}

Stats stats() {
  Stats out;
  portENTER_CRITICAL(&g_mux);
  out = g_stats;
  portEXIT_CRITICAL(&g_mux);
  return out;
}

String json() {
  const Stats s = stats();
  String out;
  out.reserve(260);
  out += "{";
  out += "\"initialized\":" + String(s.initialized ? "true" : "false") + ",";
  out += "\"totalReads\":" + String(s.totalReads) + ",";
  out += "\"droppedRecords\":" + String(s.droppedRecords) + ",";
  out += "\"sequenceErrors\":" + String(s.sequenceErrors) + ",";
  out += "\"lastSequence\":" + String(s.lastSequence) + ",";
  out += "\"pending\":" + String(s.pending) + ",";
  out += "\"maxProcessUs\":" + String(s.maxProcessUs) + ",";
  out += "\"maxSingleReadUs\":" + String(s.maxSingleReadUs) + ",";
  out += "\"lastChecksum\":" + String(s.lastChecksum);
  out += "}";
  return out;
}

}  // namespace SensorRingConsumer
