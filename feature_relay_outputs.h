#pragma once
#include "app_types.h"

namespace RelayOutputs {

  bool begin(AppContext& ctx);

  bool set(AppContext& ctx, uint8_t relayIndex, bool on);
  bool testSetRaw(AppContext& ctx, uint8_t relayIndex, bool on);
  bool get(const AppContext& ctx, uint8_t relayIndex);

  void allOff(AppContext& ctx);

  bool configure(
    AppContext& ctx,
    uint8_t relayIndex,
    bool enabled,
    RelayFunction function,
    bool activeLow
  );

  bool isUsableAsPumpEnable(const AppContext& ctx, uint8_t relayIndex);
  bool isUsableAsZoneValve(const AppContext& ctx, uint8_t relayIndex);
  bool isUsableAsHeaterRod(const AppContext& ctx, uint8_t relayIndex);
  bool isUsableAsMixer(const AppContext& ctx, uint8_t relayIndex);

  const char* functionToKey(RelayFunction f);
  RelayFunction functionFromKey(const String& key);
}