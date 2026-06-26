#pragma once

#include "app_types.h"
#include <Arduino.h>

namespace OutputValidation {

bool validateAll(const AppContext& ctx, String& error);
bool validateConfig(const ConfigData& cfg, String& error);

} // namespace OutputValidation
