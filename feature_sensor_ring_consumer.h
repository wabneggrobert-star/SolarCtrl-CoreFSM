#pragma once

#include <Arduino.h>
#include "feature_sensor_ring.h"

// Stage 3B: bounded, read-only consumer of the Stage-3A sensor ring.
// This module deliberately performs no SD, History, ML or control work. It is
// an active proof that a downstream reader can consume snapshots by sequence
// without blocking the producer or changing the control path.
namespace SensorRingConsumer {

struct Stats {
  bool initialized = false;
  uint32_t totalReads = 0;
  uint32_t droppedRecords = 0;
  uint32_t sequenceErrors = 0;
  uint32_t lastSequence = 0;
  uint32_t pending = 0;
  uint32_t maxProcessUs = 0;
  uint32_t maxSingleReadUs = 0;
  uint32_t lastChecksum = 0;
};

void begin();

// Reads at most maxRecords per call. The bound is intentional: service work
// stays deterministic even if a future consumer temporarily falls behind.
void process(uint8_t maxRecords = 2);

Stats stats();
String json();

}  // namespace SensorRingConsumer
