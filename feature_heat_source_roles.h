#pragma once
#include "app_types.h"

namespace HeatSourceRoles {

  const char* toKey(HeatSourceRole role);
  const char* toLabel(HeatSourceRole role);

  HeatSourceRole fromKey(const String& key);

  HeatSourceKind kindOf(HeatSourceRole role);
  ControlMode controlModeOf(HeatSourceRole role);

  bool isSolarRole(HeatSourceRole role);
  bool isAltSourceRole(HeatSourceRole role);
  bool isAssignable(HeatSourceRole role);

}