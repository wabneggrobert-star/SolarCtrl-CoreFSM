#pragma once
#include "app_types.h"

namespace AuxHeater {
void begin(AppContext& ctx);
void process(AppContext& ctx);
void allOff(AppContext& ctx);
bool heatingActive();
bool cooldownActive();
}
