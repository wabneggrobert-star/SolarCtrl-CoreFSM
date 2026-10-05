#include "feature_ui_bridge.h"
#include "feature_sensor_assignments.h"
#include "feature_sensor_roles.h"
#include "feature_heat_source_assignments.h"
#include "feature_oven.h"
#include "feature_aux_heater.h"
#include "feature_alarms.h"
#include <math.h>
#include <string.h>

namespace {
UIBridge::LiveSnapshot g_snapshots[2];
volatile uint8_t g_activeSnapshot = 0;
uint32_t g_sequence = 0;
portMUX_TYPE g_snapshotMux = portMUX_INITIALIZER_UNLOCKED;

static constexpr uint8_t CMD_CAPACITY = 16;
UIBridge::ControlCommand g_commands[CMD_CAPACITY];
UIBridge::CommandResult g_results[CMD_CAPACITY];
volatile uint8_t g_cmdHead = 0;
volatile uint8_t g_cmdTail = 0;
volatile uint8_t g_cmdCount = 0;
uint32_t g_nextCommandId = 1;
portMUX_TYPE g_commandMux = portMUX_INITIALIZER_UNLOCKED;

int16_t c100(float value) {
  if (isnan(value)) return 0;
  const float clipped = constrain(value, -327.67f, 327.67f);
  return (int16_t)lroundf(clipped * 100.0f);
}

void setTemp(UIBridge::LiveSnapshot& s, uint8_t index, float value, bool valid) {
  if (index >= UIBridge::LIVE_TEMP_COUNT) return;
  s.tempC100[index] = c100(value);
  if (valid && !isnan(value)) s.tempValidMask |= (uint16_t)(1U << index);
}

void readRole(const AppContext& ctx, Ds18Role role, UIBridge::LiveSnapshot& s, uint8_t index) {
  float v = NAN;
  bool valid = false;
  SensorAssignments::readByRole(ctx.ds18b20, ctx.assignments, role, v, valid);
  setTemp(s, index, v, valid);
}

inline void put8(uint8_t*& p, uint8_t v) { *p++ = v; }
inline void put16(uint8_t*& p, uint16_t v) { *p++ = (uint8_t)(v & 0xff); *p++ = (uint8_t)(v >> 8); }
inline void putS16(uint8_t*& p, int16_t v) { put16(p, (uint16_t)v); }
inline void put32(uint8_t*& p, uint32_t v) {
  *p++ = (uint8_t)(v & 0xff); *p++ = (uint8_t)((v >> 8) & 0xff);
  *p++ = (uint8_t)((v >> 16) & 0xff); *p++ = (uint8_t)((v >> 24) & 0xff);
}
inline void putS32(uint8_t*& p, int32_t v) { put32(p, (uint32_t)v); }
inline void putBytes(uint8_t*& p, const char* src, size_t len) {
  memcpy(p, src, len);
  p += len;
}

void copyText(char* dst, size_t dstSize, const char* src) {
  if (!dst || dstSize == 0) return;
  strncpy(dst, src ? src : "", dstSize - 1);
  dst[dstSize - 1] = '\0';
}
}

namespace UIBridge {

void begin() {
  portENTER_CRITICAL(&g_snapshotMux);
  memset(g_snapshots, 0, sizeof(g_snapshots));
  g_activeSnapshot = 0;
  g_sequence = 0;
  portEXIT_CRITICAL(&g_snapshotMux);

  portENTER_CRITICAL(&g_commandMux);
  memset(g_commands, 0, sizeof(g_commands));
  memset(g_results, 0, sizeof(g_results));
  g_cmdHead = g_cmdTail = g_cmdCount = 0;
  g_nextCommandId = 1;
  portEXIT_CRITICAL(&g_commandMux);
}

void publish(const AppContext& ctx) {
  const uint8_t next = (uint8_t)(g_activeSnapshot ^ 1U);
  LiveSnapshot& s = g_snapshots[next];
  memset(&s, 0, sizeof(s));
  s.sequence = ++g_sequence;
  s.capturedMs = millis();

  // Temperature indices are protocol-stable:
  // 0 boilerTop, 1 boilerBottom, 2 bufferTop, 3 bufferHigh, 4 bufferMid,
  // 5 bufferBottom, 6 pool, 7 outside, 8..10 collectors 1..3, 11 other source.
  readRole(ctx, Ds18Role::SINK_BOILER_TOP, s, 0);
  readRole(ctx, Ds18Role::BOILER_BOTTOM, s, 1);
  readRole(ctx, Ds18Role::SINK_BUFFER_TOP, s, 2);
  readRole(ctx, Ds18Role::BUFFER_HIGH, s, 3);
  readRole(ctx, Ds18Role::BUFFER_MID, s, 4);
  readRole(ctx, Ds18Role::BUFFER_BOTTOM, s, 5);
  readRole(ctx, Ds18Role::SWIMMINGPOOL, s, 6);
  readRole(ctx, Ds18Role::OUTSIDE_TEMPERATURE, s, 7);
  setTemp(s, 8, ctx.sensors.heatSources.solarCollector1C, ctx.sensors.heatSources.solarCollector1Valid);
  setTemp(s, 9, ctx.sensors.heatSources.solarCollector2C, ctx.sensors.heatSources.solarCollector2Valid);
  setTemp(s, 10, ctx.sensors.heatSources.solarCollector3C, ctx.sensors.heatSources.solarCollector3Valid);
  setTemp(s, 11, ctx.sensors.heatSources.altSourceOtherC, ctx.sensors.heatSources.altSourceOtherValid);

  const HeatSourceRole collectorRoles[] = {
    HeatSourceRole::SOLAR_COLLECTOR_1,
    HeatSourceRole::SOLAR_COLLECTOR_2,
    HeatSourceRole::SOLAR_COLLECTOR_3
  };
  for (uint8_t i = 0; i < 3; ++i) {
    if (HeatSourceAssignments::hasRole(ctx.heatSourceAssignments, collectorRoles[i])) {
      s.collectorPresentMask |= (uint8_t)(1U << i);
    }
  }

  for (uint8_t i = 0; i < MAX_PUMPS; ++i) {
    const PumpConfig& p = ctx.config.pumps[i];
    if (p.enabled && p.mode != PumpMode::OFF) s.pumpEnabledMask |= (uint16_t)(1U << i);
    if (p.state) s.pumpActiveMask |= (uint16_t)(1U << i);
    s.pumpPwm[i] = (uint8_t)constrain((int)lroundf(p.lastPwmPercent), 0, 100);
    s.pumpTarget[i] = (int8_t)p.activeTargetIndex;
  }

  for (uint8_t i = 0; i < MAX_HEATING_CIRCUITS; ++i) {
    const HeatingCircuitConfig& cfg = ctx.config.heatingCircuits[i];
    const HeatingCircuitRuntime& rt = ctx.heatingCircuitRuntime[i];
    if (cfg.enabled) s.hcEnabledMask |= (uint16_t)(1U << i);
    if (rt.active) s.hcActiveMask |= (uint16_t)(1U << i);
    if (rt.pumpActive) s.hcPumpMask |= (uint16_t)(1U << i);
    if (rt.nightSetbackActive) s.hcNightSetbackMask |= (uint16_t)(1U << i);
    s.hcPumpPercent[i] = (uint8_t)constrain((int)rt.pumpPercent, 0, 100);
    s.hcFlowC100[i] = c100(rt.flowTemperatureC);
    s.hcReturnC100[i] = c100(rt.returnTemperatureC);
    s.hcTargetC100[i] = c100(rt.targetFlowTemperatureC);
    s.hcRoomC100[i] = c100(rt.roomTemperatureC);
    s.hcEffectiveRoomTargetC100[i] = c100(rt.effectiveRoomTargetTemperatureC);
  }

  const float ovenTemp = OvenControl::ovenTemperatureC();
  if (OvenControl::active()) s.ovenFlags |= 0x01;
  if (!isnan(ovenTemp)) s.ovenFlags |= 0x02;
  s.ovenTempC100 = c100(ovenTemp);
  s.ovenServoPercent = (uint8_t)constrain((int)OvenControl::servoOpeningPercent(), 0, 100);
  copyText(s.ovenState, sizeof(s.ovenState), OvenControl::stateText());

  if (ctx.config.auxHeater.userReleaseEnabled) s.auxFlags |= 0x01;
  if (AuxHeater::pumpActive()) s.auxFlags |= 0x02;
  s.auxStages = AuxHeater::activeStageCount();
  copyText(s.auxState, sizeof(s.auxState), AuxHeater::stateText());

  if (ctx.config.energyMeter.enabled) s.energyFlags |= 0x01;
  const double totalWh = ctx.energyMeter.totalEnergyKWh * 1000.0;
  s.energyTotalWh = totalWh <= 0.0 ? 0U : (uint32_t)min(totalWh, 4294967295.0);
  s.energyPowerW = (int32_t)lroundf(ctx.energyMeter.thermalPowerKw * 1000.0f);
  const float flowCl = ctx.energyMeter.flowLitersPerMinute * 100.0f;
  s.energyFlowClMin = (uint16_t)constrain((int)lroundf(flowCl), 0, 65535);

  s.alarmActiveCount = Alarms::activeCount();
  if (Alarms::hasCriticalActive()) s.systemFlags |= 0x01;
  if (Alarms::buzzerMuted()) s.systemFlags |= 0x02;
  copyText(s.alarmMessage, sizeof(s.alarmMessage), Alarms::currentMessage());

  // Publish only the index atomically. The inactive buffer is never read while
  // being filled; copyLive performs a bounded copy under the same short guard.
  portENTER_CRITICAL(&g_snapshotMux);
  g_activeSnapshot = next;
  portEXIT_CRITICAL(&g_snapshotMux);
}

bool copyLive(LiveSnapshot& out) {
  portENTER_CRITICAL(&g_snapshotMux);
  out = g_snapshots[g_activeSnapshot];
  portEXIT_CRITICAL(&g_snapshotMux);
  return out.sequence != 0;
}

bool submitCommand(ControlCommand& command) {
  bool ok = false;
  portENTER_CRITICAL(&g_commandMux);
  if (g_cmdCount < CMD_CAPACITY) {
    if (command.id == 0) {
      command.id = g_nextCommandId++;
      if (g_nextCommandId == 0) g_nextCommandId = 1;
    }
    g_commands[g_cmdHead] = command;
    g_cmdHead = (uint8_t)((g_cmdHead + 1U) % CMD_CAPACITY);
    ++g_cmdCount;

    CommandResult& r = g_results[command.id % CMD_CAPACITY];
    memset(&r, 0, sizeof(r));
    r.id = command.id;
    r.state = CommandState::PENDING;
    r.httpStatus = 202;
    copyText(r.message, sizeof(r.message), "queued");
    ok = true;
  }
  portEXIT_CRITICAL(&g_commandMux);
  return ok;
}

bool dequeueCommand(ControlCommand& out) {
  bool ok = false;
  portENTER_CRITICAL(&g_commandMux);
  if (g_cmdCount) {
    out = g_commands[g_cmdTail];
    g_cmdTail = (uint8_t)((g_cmdTail + 1U) % CMD_CAPACITY);
    --g_cmdCount;
    ok = true;
  }
  portEXIT_CRITICAL(&g_commandMux);
  return ok;
}

uint8_t commandDepth() {
  portENTER_CRITICAL(&g_commandMux);
  const uint8_t n = g_cmdCount;
  portEXIT_CRITICAL(&g_commandMux);
  return n;
}

void completeCommand(uint32_t id, bool applied, uint16_t httpStatus, const char* message) {
  if (!id) return;
  portENTER_CRITICAL(&g_commandMux);
  CommandResult& r = g_results[id % CMD_CAPACITY];
  if (r.id == id) {
    r.state = applied ? CommandState::APPLIED : CommandState::REJECTED;
    r.httpStatus = httpStatus;
    copyText(r.message, sizeof(r.message), message);
  }
  portEXIT_CRITICAL(&g_commandMux);
}

bool getCommandResult(uint32_t id, CommandResult& out) {
  if (!id) return false;
  bool found = false;
  portENTER_CRITICAL(&g_commandMux);
  const CommandResult& r = g_results[id % CMD_CAPACITY];
  if (r.id == id && r.state != CommandState::UNKNOWN) {
    out = r;
    found = true;
  }
  portEXIT_CRITICAL(&g_commandMux);
  return found;
}

size_t encodePlantLiveV2(uint8_t* dst, size_t capacity, LiveSnapshot* copied) {
  LiveSnapshot s;
  if (!dst || !copyLive(s)) return 0;

  const size_t payloadLen =
    4 + // capturedMs
    2 + (LIVE_TEMP_COUNT * 2) + 1 +
    2 + 2 + (LIVE_PUMP_COUNT * 2) +
    2 + 2 + 2 + 2 + (LIVE_HC_COUNT * (1 + 2 + 2 + 2 + 2 + 2)) +
    1 + 2 + 1 + STATE_TEXT_LEN +
    1 + 1 + STATE_TEXT_LEN +
    1 + 4 + 4 + 2 +
    2 + 1 + ALARM_TEXT_LEN;
  const size_t totalLen = 10 + payloadLen;
  if (capacity < totalLen || payloadLen > 65535) return 0;

  uint8_t* p = dst;
  put16(p, PROTOCOL_MAGIC);
  put8(p, PROTOCOL_VERSION);
  put8(p, PACKET_PLANT_LIVE);
  put16(p, (uint16_t)payloadLen);
  put32(p, s.sequence);
  put32(p, s.capturedMs);
  put16(p, s.tempValidMask);
  for (uint8_t i = 0; i < LIVE_TEMP_COUNT; ++i) putS16(p, s.tempC100[i]);
  put8(p, s.collectorPresentMask);
  put16(p, s.pumpEnabledMask);
  put16(p, s.pumpActiveMask);
  for (uint8_t i = 0; i < LIVE_PUMP_COUNT; ++i) { put8(p, s.pumpPwm[i]); put8(p, (uint8_t)s.pumpTarget[i]); }
  put16(p, s.hcEnabledMask);
  put16(p, s.hcActiveMask);
  put16(p, s.hcPumpMask);
  put16(p, s.hcNightSetbackMask);
  for (uint8_t i = 0; i < LIVE_HC_COUNT; ++i) {
    put8(p, s.hcPumpPercent[i]);
    putS16(p, s.hcFlowC100[i]);
    putS16(p, s.hcReturnC100[i]);
    putS16(p, s.hcTargetC100[i]);
    putS16(p, s.hcRoomC100[i]);
    putS16(p, s.hcEffectiveRoomTargetC100[i]);
  }
  put8(p, s.ovenFlags); putS16(p, s.ovenTempC100); put8(p, s.ovenServoPercent); putBytes(p, s.ovenState, STATE_TEXT_LEN);
  put8(p, s.auxFlags); put8(p, s.auxStages); putBytes(p, s.auxState, STATE_TEXT_LEN);
  put8(p, s.energyFlags); put32(p, s.energyTotalWh); putS32(p, s.energyPowerW); put16(p, s.energyFlowClMin);
  put16(p, s.alarmActiveCount); put8(p, s.systemFlags); putBytes(p, s.alarmMessage, ALARM_TEXT_LEN);

  if (copied) *copied = s;
  return (size_t)(p - dst);
}

} // namespace UIBridge
