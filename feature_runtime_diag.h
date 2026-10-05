#pragma once

#include <Arduino.h>

namespace RuntimeDiag {

enum class ControlSection : uint8_t {
  STATE = 0,
  FAST_CONTROL,
  NETWORK,
  TIME_SERVICE,
  FORECAST,
  MQTT,
  ML,
  HISTORY,
  ALARMS,
  COUNT
};

static constexpr uint8_t CONTROL_SECTION_COUNT = (uint8_t)ControlSection::COUNT;
static constexpr uint8_t SYSTEM_STATE_COUNT = 16;

struct Snapshot {
  uint32_t loopCount = 0;
  uint32_t maxLoopGapUs = 0;
  uint32_t maxControlUs = 0;
  uint32_t maxWebUs = 0;
  uint32_t maxServiceUs = 0;
  uint32_t controlOver5ms = 0;
  uint32_t controlOver20ms = 0;
  uint32_t controlOver100ms = 0;
  uint32_t webOver5ms = 0;
  uint32_t webOver20ms = 0;
  uint32_t webOver100ms = 0;
  uint32_t serviceOver5ms = 0;
  uint32_t serviceOver20ms = 0;
  uint32_t serviceOver100ms = 0;
  uint8_t controlCore = 0xFF;
  uint8_t webCore = 0xFF;
  uint32_t sectionMaxUs[CONTROL_SECTION_COUNT] = {};
  uint32_t stateMaxUs[SYSTEM_STATE_COUNT] = {};
  uint8_t slowestState = 0xFF;
  uint32_t slowestStateUs = 0;
};

void begin();
void noteLoopStart();
void recordControl(uint32_t elapsedUs);
void recordWeb(uint32_t elapsedUs);
void recordServices(uint32_t elapsedUs);
void recordSection(ControlSection section, uint32_t elapsedUs);
void recordState(uint8_t state, uint32_t elapsedUs);
Snapshot snapshot();
void printPeriodic(uint32_t intervalMs = 10000UL);
String json();

}  // namespace RuntimeDiag
