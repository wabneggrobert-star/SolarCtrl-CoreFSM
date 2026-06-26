#pragma once
#include "app_types.h"

namespace HeatSourceAssignments {

  void clearTable(HeatSourceAssignmentTable& table);

  bool hasRole(const HeatSourceAssignmentTable& table, HeatSourceRole role);
  bool hasChannel(const HeatSourceAssignmentTable& table, MaxChannel channel);

  bool getByRole(
    const HeatSourceAssignmentTable& table,
    HeatSourceRole role,
    HeatSourceAssignment& out
  );

  bool getByChannel(
    const HeatSourceAssignmentTable& table,
    MaxChannel channel,
    HeatSourceAssignment& out
  );

  bool setAssignment(
    HeatSourceAssignmentTable& table,
    MaxChannel channel,
    HeatSourceRole role
  );

  bool removeByRole(
    HeatSourceAssignmentTable& table,
    HeatSourceRole role
  );

  bool removeByChannel(
    HeatSourceAssignmentTable& table,
    MaxChannel channel
  );

  bool roleAssignedAndValid(
    const AppContext& ctx,
    HeatSourceRole role
  );

  bool resolveHeatSources(AppContext& ctx);

  bool findFirstValidHeatSource(
    const AppContext& ctx,
    ActiveHeatSource& out
  );

}