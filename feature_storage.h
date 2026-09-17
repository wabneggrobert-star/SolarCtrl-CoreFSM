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

  bool loadEnergyMeterRuntime(EnergyMeterRuntime& runtime);
  bool saveEnergyMeterRuntime(const EnergyMeterRuntime& runtime);

  bool loadSensorAssignments(SensorAssignmentTable& table);
  bool saveSensorAssignments(const SensorAssignmentTable& table);

  void resetConfig(ConfigData& cfg);
  void resetDiagnostics(DiagnosticData& d);
  void resetMaintenance(MaintenanceData& m);
  void resetEnergyMeterRuntime(EnergyMeterRuntime& runtime);
  void resetSensorAssignments(SensorAssignmentTable& table);
}
