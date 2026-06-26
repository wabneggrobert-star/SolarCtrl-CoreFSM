#pragma once
#include "app_types.h"

namespace Storage {
  void begin(bool sdAvailable);

  bool loadConfig(ConfigData& cfg);
  bool saveConfig(const ConfigData& cfg);

  bool loadDiagnostics(DiagnosticData& d);
  bool saveDiagnostics(const DiagnosticData& d);

  bool loadMaintenance(MaintenanceData& m);
  bool saveMaintenance(const MaintenanceData& m);

  bool loadSensorAssignments(SensorAssignmentTable& table);
  bool saveSensorAssignments(const SensorAssignmentTable& table);
}