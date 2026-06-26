#pragma once

#include "app_types.h"

namespace Valves {

void begin(AppContext& ctx);
void process(AppContext& ctx);

bool requestPosition(AppContext& ctx, uint8_t valveIndex, ValvePosition position);

void allOff(AppContext& ctx);
void applySafetyPosition(AppContext& ctx);

bool isMoving(uint8_t valveIndex);
ValvePosition currentPosition(uint8_t valveIndex);
ValvePosition targetPosition(uint8_t valveIndex);

} // namespace Valves