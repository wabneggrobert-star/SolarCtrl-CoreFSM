#pragma once

#include <Arduino.h>
#include "app_types.h"

namespace UIBridge {

static constexpr uint16_t PROTOCOL_MAGIC = 0x4353; // 'S''C' little-endian on wire
static constexpr uint8_t PROTOCOL_VERSION = 2;
static constexpr uint8_t PACKET_PLANT_LIVE = 1;
static constexpr uint16_t UI_VERSION = 2;
static constexpr uint8_t LIVE_TEMP_COUNT = 12;
static constexpr uint8_t LIVE_PUMP_COUNT = MAX_PUMPS;
static constexpr uint8_t LIVE_HC_COUNT = MAX_HEATING_CIRCUITS;
static constexpr uint8_t STATE_TEXT_LEN = 20;
static constexpr uint8_t ALARM_TEXT_LEN = 96;

// Fixed-size snapshot. It is intentionally independent from AppContext so the
// Web/UI core never needs to dereference live control state once the core split
// is enabled.
struct LiveSnapshot {
  uint32_t sequence = 0;
  uint32_t capturedMs = 0;
  int16_t tempC100[LIVE_TEMP_COUNT] = {};
  uint16_t tempValidMask = 0;
  uint8_t collectorPresentMask = 0;
  uint16_t pumpEnabledMask = 0;
  uint16_t pumpActiveMask = 0;
  uint8_t pumpPwm[LIVE_PUMP_COUNT] = {};
  int8_t pumpTarget[LIVE_PUMP_COUNT] = {};
  uint16_t hcEnabledMask = 0;
  uint16_t hcActiveMask = 0;
  uint16_t hcPumpMask = 0;
  uint16_t hcNightSetbackMask = 0;
  uint8_t hcPumpPercent[LIVE_HC_COUNT] = {};
  int16_t hcFlowC100[LIVE_HC_COUNT] = {};
  int16_t hcReturnC100[LIVE_HC_COUNT] = {};
  int16_t hcTargetC100[LIVE_HC_COUNT] = {};
  int16_t hcRoomC100[LIVE_HC_COUNT] = {};
  int16_t hcEffectiveRoomTargetC100[LIVE_HC_COUNT] = {};
  uint8_t ovenFlags = 0;     // bit0 active, bit1 temperature valid
  int16_t ovenTempC100 = 0;
  uint8_t ovenServoPercent = 0;
  char ovenState[STATE_TEXT_LEN] = {};
  uint8_t auxFlags = 0;      // bit0 released, bit1 pump active
  uint8_t auxStages = 0;
  char auxState[STATE_TEXT_LEN] = {};
  uint8_t energyFlags = 0;   // bit0 enabled
  uint32_t energyTotalWh = 0;
  int32_t energyPowerW = 0;
  uint16_t energyFlowClMin = 0; // centiliter/min
  uint16_t alarmActiveCount = 0;
  uint8_t systemFlags = 0;   // bit0 critical, bit1 buzzer muted
  char alarmMessage[ALARM_TEXT_LEN] = {};
};

// Wire header is written field-by-field; no compiler struct packing is relied on.
struct PacketHeader {
  uint16_t magic;
  uint8_t version;
  uint8_t type;
  uint16_t payloadLength;
  uint32_t sequence;
};

enum class CommandType : uint8_t {
  NONE = 0,
  USER_COMMAND = 1
};

enum class ControlAction : uint16_t {
  NONE = 0,
  BUZZER_MUTE = 1,
  OVEN_START = 2,
  OVEN_STOP = 3
};

enum class CommandState : uint8_t {
  UNKNOWN = 0,
  PENDING = 1,
  APPLIED = 2,
  REJECTED = 3
};

struct ControlCommand {
  uint32_t id = 0;
  CommandType type = CommandType::NONE;
  uint16_t action = 0;
  int32_t valueA = 0;
  int32_t valueB = 0;
};

struct CommandResult {
  uint32_t id = 0;
  CommandState state = CommandState::UNKNOWN;
  uint16_t httpStatus = 0;
  char message[64] = {};
};

void begin();
void publish(const AppContext& ctx);
bool copyLive(LiveSnapshot& out);

// Non-blocking UI -> Control handshake. submitCommand assigns an ID when the
// caller passes id=0, stores PENDING, and never waits for the control owner.
bool submitCommand(ControlCommand& command);
bool dequeueCommand(ControlCommand& out);
uint8_t commandDepth();
void completeCommand(uint32_t id, bool applied, uint16_t httpStatus, const char* message);
bool getCommandResult(uint32_t id, CommandResult& out);

// Encodes one current live packet into caller-owned memory. No heap allocation.
size_t encodePlantLiveV2(uint8_t* dst, size_t capacity, LiveSnapshot* copied = nullptr);

} // namespace UIBridge
