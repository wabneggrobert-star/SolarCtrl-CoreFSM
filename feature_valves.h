#pragma once

#include "app_types.h"

namespace Valves {

void begin(AppContext& ctx);
void process(AppContext& ctx);

bool isConfigured(const AppContext& ctx, uint8_t valveIndex);
bool requestPosition(AppContext& ctx, uint8_t valveIndex, ValvePosition position);

void allOff(AppContext& ctx);
void applySafetyPosition(AppContext& ctx);

bool isMoving(uint8_t valveIndex);
uint32_t moveRemainingMs(const AppContext& ctx, uint8_t valveIndex);
ValvePosition currentPosition(uint8_t valveIndex);
ValvePosition targetPosition(uint8_t valveIndex);

OutputRef outputRef(const AppContext& ctx, uint8_t valveIndex);
uint8_t outputRelayIndex(const AppContext& ctx, uint8_t valveIndex);
const char* positionToKey(ValvePosition position);
ValvePosition positionFromKey(const String& key);

} // namespace Valves
