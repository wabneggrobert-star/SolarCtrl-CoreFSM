#include "feature_runtime_diag.h"
#include "feature_sensor_ring.h"

namespace {
portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;
RuntimeDiag::Snapshot g_diag;
uint32_t g_lastLoopStartUs = 0;
uint32_t g_lastPrintMs = 0;

inline void updateThresholds(uint32_t us, uint32_t& over5, uint32_t& over20, uint32_t& over100) {
  if (us >= 5000U) ++over5;
  if (us >= 20000U) ++over20;
  if (us >= 100000U) ++over100;
}
}

namespace RuntimeDiag {

void begin() {
  portENTER_CRITICAL(&g_mux);
  g_diag = Snapshot{};
  g_lastLoopStartUs = micros();
  g_lastPrintMs = millis();
  portEXIT_CRITICAL(&g_mux);
}

void noteLoopStart() {
  const uint32_t nowUs = micros();
  portENTER_CRITICAL(&g_mux);
  if (g_lastLoopStartUs != 0) {
    const uint32_t gap = (uint32_t)(nowUs - g_lastLoopStartUs);
    if (gap > g_diag.maxLoopGapUs) g_diag.maxLoopGapUs = gap;
  }
  g_lastLoopStartUs = nowUs;
  ++g_diag.loopCount;
  portEXIT_CRITICAL(&g_mux);
}

void recordControl(uint32_t elapsedUs) {
  const uint8_t core = (uint8_t)xPortGetCoreID();
  portENTER_CRITICAL(&g_mux);
  g_diag.controlCore = core;
  if (elapsedUs > g_diag.maxControlUs) g_diag.maxControlUs = elapsedUs;
  updateThresholds(elapsedUs, g_diag.controlOver5ms, g_diag.controlOver20ms, g_diag.controlOver100ms);
  portEXIT_CRITICAL(&g_mux);
}

void recordWeb(uint32_t elapsedUs) {
  const uint8_t core = (uint8_t)xPortGetCoreID();
  portENTER_CRITICAL(&g_mux);
  g_diag.webCore = core;
  if (elapsedUs > g_diag.maxWebUs) g_diag.maxWebUs = elapsedUs;
  updateThresholds(elapsedUs, g_diag.webOver5ms, g_diag.webOver20ms, g_diag.webOver100ms);
  portEXIT_CRITICAL(&g_mux);
}

void recordServices(uint32_t elapsedUs) {
  portENTER_CRITICAL(&g_mux);
  if (elapsedUs > g_diag.maxServiceUs) g_diag.maxServiceUs = elapsedUs;
  updateThresholds(elapsedUs, g_diag.serviceOver5ms, g_diag.serviceOver20ms, g_diag.serviceOver100ms);
  portEXIT_CRITICAL(&g_mux);
}


void recordSection(ControlSection section, uint32_t elapsedUs) {
  const uint8_t idx = (uint8_t)section;
  if (idx >= CONTROL_SECTION_COUNT) return;
  portENTER_CRITICAL(&g_mux);
  if (elapsedUs > g_diag.sectionMaxUs[idx]) g_diag.sectionMaxUs[idx] = elapsedUs;
  portEXIT_CRITICAL(&g_mux);
}

void recordState(uint8_t state, uint32_t elapsedUs) {
  if (state >= SYSTEM_STATE_COUNT) return;
  portENTER_CRITICAL(&g_mux);
  if (elapsedUs > g_diag.stateMaxUs[state]) g_diag.stateMaxUs[state] = elapsedUs;
  if (elapsedUs > g_diag.slowestStateUs) {
    g_diag.slowestStateUs = elapsedUs;
    g_diag.slowestState = state;
  }
  portEXIT_CRITICAL(&g_mux);
}

Snapshot snapshot() {
  Snapshot copy;
  portENTER_CRITICAL(&g_mux);
  copy = g_diag;
  portEXIT_CRITICAL(&g_mux);
  return copy;
}

String json() {
  const Snapshot d = snapshot();
  String out;
  out.reserve(560);
  out += "{";
  out += "\"loopCount\":" + String(d.loopCount) + ",";
  out += "\"maxLoopGapUs\":" + String(d.maxLoopGapUs) + ",";
  out += "\"maxControlUs\":" + String(d.maxControlUs) + ",";
  out += "\"maxWebUs\":" + String(d.maxWebUs) + ",";
  out += "\"maxServiceUs\":" + String(d.maxServiceUs) + ",";
  out += "\"controlOver5ms\":" + String(d.controlOver5ms) + ",";
  out += "\"controlOver20ms\":" + String(d.controlOver20ms) + ",";
  out += "\"controlOver100ms\":" + String(d.controlOver100ms) + ",";
  out += "\"webOver5ms\":" + String(d.webOver5ms) + ",";
  out += "\"webOver20ms\":" + String(d.webOver20ms) + ",";
  out += "\"webOver100ms\":" + String(d.webOver100ms) + ",";
  out += "\"serviceOver5ms\":" + String(d.serviceOver5ms) + ",";
  out += "\"serviceOver20ms\":" + String(d.serviceOver20ms) + ",";
  out += "\"serviceOver100ms\":" + String(d.serviceOver100ms) + ",";
  out += "\"controlCore\":" + String((unsigned)d.controlCore) + ",";
  out += "\"webCore\":" + String((unsigned)d.webCore) + ",";
  out += "\"slowestState\":" + String((unsigned)d.slowestState) + ",";
  out += "\"slowestStateUs\":" + String(d.slowestStateUs) + ",";
  out += "\"sections\":{";
  static const char* names[CONTROL_SECTION_COUNT] = {"state","fast","network","time","forecast","mqtt","ml","history","alarms"};
  for (uint8_t i = 0; i < CONTROL_SECTION_COUNT; ++i) {
    if (i) out += ",";
    out += "\"" + String(names[i]) + "\":" + String(d.sectionMaxUs[i]);
  }
  out += "},\"states\":[";
  for (uint8_t i = 0; i < SYSTEM_STATE_COUNT; ++i) {
    if (i) out += ",";
    out += String(d.stateMaxUs[i]);
  }
  out += "],\"sensorRing\":{";
  const SensorRing::Stats ring = SensorRing::stats();
  out += "\"capacity\":" + String(ring.capacity) + ",";
  out += "\"count\":" + String(ring.count) + ",";
  out += "\"recordSize\":" + String(ring.recordSize) + ",";
  out += "\"totalWrites\":" + String(ring.totalWrites) + ",";
  out += "\"overwrites\":" + String(ring.overwrites) + ",";
  out += "\"lastSequence\":" + String(ring.lastSequence) + ",";
  out += "\"lastCapturedMs\":" + String(ring.lastCapturedMs) + ",";
  out += "\"lastAgeMs\":" + String(ring.lastAgeMs) + ",";
  out += "\"maxCaptureUs\":" + String(ring.maxCaptureUs);
  out += "}}";
  return out;
}

void printPeriodic(uint32_t intervalMs) {
  const uint32_t now = millis();
  if ((uint32_t)(now - g_lastPrintMs) < intervalMs) return;
  g_lastPrintMs = now;

  const Snapshot d = snapshot();
  Serial.printf(
    "RUNTIME: loops=%lu loopGapMax=%luus controlMax=%luus serviceMax=%luus webMax=%luus "
    "ctrl>5/20/100ms=%lu/%lu/%lu svc>5/20/100ms=%lu/%lu/%lu web>5/20/100ms=%lu/%lu/%lu cores=%u/%u free=%u min=%u\n",
    (unsigned long)d.loopCount,
    (unsigned long)d.maxLoopGapUs,
    (unsigned long)d.maxControlUs,
    (unsigned long)d.maxServiceUs,
    (unsigned long)d.maxWebUs,
    (unsigned long)d.controlOver5ms,
    (unsigned long)d.controlOver20ms,
    (unsigned long)d.controlOver100ms,
    (unsigned long)d.serviceOver5ms,
    (unsigned long)d.serviceOver20ms,
    (unsigned long)d.serviceOver100ms,
    (unsigned long)d.webOver5ms,
    (unsigned long)d.webOver20ms,
    (unsigned long)d.webOver100ms,
    (unsigned)d.controlCore,
    (unsigned)d.webCore,
    (unsigned)ESP.getFreeHeap(),
    (unsigned)ESP.getMinFreeHeap()
  );

  Serial.printf(
    "CTRLSEC: state=%lu fast=%lu network=%lu time=%lu forecast=%lu mqtt=%lu ml=%lu history=%lu alarms=%lu slowState=%u/%luus\n",
    (unsigned long)d.sectionMaxUs[(uint8_t)ControlSection::STATE],
    (unsigned long)d.sectionMaxUs[(uint8_t)ControlSection::FAST_CONTROL],
    (unsigned long)d.sectionMaxUs[(uint8_t)ControlSection::NETWORK],
    (unsigned long)d.sectionMaxUs[(uint8_t)ControlSection::TIME_SERVICE],
    (unsigned long)d.sectionMaxUs[(uint8_t)ControlSection::FORECAST],
    (unsigned long)d.sectionMaxUs[(uint8_t)ControlSection::MQTT],
    (unsigned long)d.sectionMaxUs[(uint8_t)ControlSection::ML],
    (unsigned long)d.sectionMaxUs[(uint8_t)ControlSection::HISTORY],
    (unsigned long)d.sectionMaxUs[(uint8_t)ControlSection::ALARMS],
    (unsigned)d.slowestState,
    (unsigned long)d.slowestStateUs
  );

  const SensorRing::Stats ring = SensorRing::stats();
  Serial.printf(
    "SENSORRING: count=%u/%u writes=%lu overwrites=%lu seq=%lu age=%lums record=%uB captureMax=%luus\n",
    (unsigned)ring.count,
    (unsigned)ring.capacity,
    (unsigned long)ring.totalWrites,
    (unsigned long)ring.overwrites,
    (unsigned long)ring.lastSequence,
    (unsigned long)ring.lastAgeMs,
    (unsigned)ring.recordSize,
    (unsigned long)ring.maxCaptureUs
  );
}

}  // namespace RuntimeDiag
