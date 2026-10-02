#pragma once
#include "app_types.h"

namespace HeatingCircuits {
  void begin(AppContext& ctx);
  void process(AppContext& ctx);

  // 100-ms-Mischer-FSM. Fuehrt nur die bereits vom langsamen Regler
  // angeforderte Mischerrichtung als PULSE -> WAIT -> PULSE aus.
  void processFast(AppContext& ctx, bool safetyOverride);

  // Vor jedem Safety-Apply werden alte Safety-Mischerauftraege verworfen.
  void prepareSafetyCycle(AppContext& ctx);

  // Nach Konfigurationsaenderungen/Testbetrieb kann ein einzelner
  // Mischerzustand sicher verworfen werden.
  void resetMixerFsm(AppContext& ctx, uint8_t circuitIndex);
  void allOff(AppContext& ctx);
  // Emergency heat dump from a critically hot configured storage through safe
  // mixed heating circuits. Only runs with valid flow/return sensors and never
  // exceeds maximumFlowTemperatureC. Returns number of running circuits.
  uint8_t safetyHeatDump(AppContext& ctx, float criticalStorageTemperatureC, float minimumDeltaC);

  Ds18Role defaultFlowSensorRole(uint8_t circuitIndex);
  Ds18Role defaultReturnSensorRole(uint8_t circuitIndex);
  Ds18Role defaultRoomSensorRole(uint8_t circuitIndex);
  Ds18Role effectiveFlowSensorRole(uint8_t circuitIndex, const HeatingCircuitConfig& cfg);
  Ds18Role effectiveReturnSensorRole(uint8_t circuitIndex, const HeatingCircuitConfig& cfg);
  Ds18Role effectiveRoomSensorRole(uint8_t circuitIndex, const HeatingCircuitConfig& cfg);
  Ds18Role effectiveOutsideSensorRole(const HeatingCircuitConfig& cfg);

  // Schaltet alle Ausgaenge ab, die in cfg referenziert sind.
  void releaseOutputs(AppContext& ctx, const HeatingCircuitConfig& cfg);

  // Schaltet nur alte Ausgaenge aus, die in newCfg nicht mehr vorkommen.
  void releaseRemovedOutputs(AppContext& ctx, const HeatingCircuitConfig& oldCfg, const HeatingCircuitConfig& newCfg);
}
