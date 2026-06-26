#pragma once
#include "app_types.h"

namespace SensorAssignments {
  void clearTable(SensorAssignmentTable& table);
  bool hasRole(const SensorAssignmentTable& table, Ds18Role role);
  bool getAssignment(const SensorAssignmentTable& table, Ds18Role role, Ds18RoleAssignment& out);
  bool setRole(SensorAssignmentTable& table, Ds18Role role, const uint8_t address[8], const char* addressText);
  bool removeRole(SensorAssignmentTable& table, Ds18Role role);
  bool inventoryContainsAddress(const Ds18b20Inventory& inventory, const uint8_t address[8]);
  bool assignRoleByAddressText(const Ds18b20Inventory& inventory, const String& addressText, Ds18Role role, SensorAssignmentTable& table);
  bool readByRole(const SensorAssignmentTable& table, Ds18Role role, float& tempC, bool& valid);
  Ds18Role activeSinkRole(const ConfigData& config);
  bool autoAssignSingleSensorAsActiveSink(const Ds18b20Inventory& inventory, const ConfigData& config, SensorAssignmentTable& table);
  bool resolveAssignments(const Ds18b20Inventory& inventory, const ConfigData& config, SensorAssignmentTable& table);
}
