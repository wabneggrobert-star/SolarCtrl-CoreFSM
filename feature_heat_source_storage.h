#pragma once
#include "app_types.h"

namespace HeatSourceStorage {

  bool loadAssignments(HeatSourceAssignmentTable& table);
  bool saveAssignments(const HeatSourceAssignmentTable& table);

}