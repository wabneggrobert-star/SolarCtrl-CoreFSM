#pragma once
#include "app_types.h"

namespace SensorRoles {
  const char* toKey(Ds18Role role);
  const char* toLabel(Ds18Role role);
  Ds18Role fromKey(const String& key);
  bool isSinkRole(Ds18Role role);
  bool isAssignable(Ds18Role role);
}
