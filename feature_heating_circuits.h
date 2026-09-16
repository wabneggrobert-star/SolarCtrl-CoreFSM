#pragma once
#include "app_types.h"

namespace HeatingCircuits {
  void begin(AppContext& ctx);
  void process(AppContext& ctx);
  void allOff(AppContext& ctx);

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
