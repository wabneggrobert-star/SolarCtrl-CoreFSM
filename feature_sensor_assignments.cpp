#include "feature_sensor_assignments.h"
#include "feature_sink_ds18b20.h"
#include <string.h>

namespace {
  bool sameAddress(const uint8_t a[8], const uint8_t b[8]) {
    for (uint8_t i = 0; i < 8; i++) if (a[i] != b[i]) return false;
    return true;
  }

  void copyAddress(uint8_t dst[8], const uint8_t src[8]) {
    for (uint8_t i = 0; i < 8; i++) dst[i] = src[i];
  }

  void copyText(char* dst, size_t dstSize, const char* src) {
    if (!dst || dstSize == 0) return;
    strncpy(dst, src ? src : "", dstSize - 1);
    dst[dstSize - 1] = '\0';
  }

  int findRoleIndex(const SensorAssignmentTable& table, Ds18Role role) {
    for (uint8_t i = 0; i < table.count; i++) {
      if (table.items[i].role == role) return i;
    }
    return -1;
  }
}

namespace SensorAssignments {

void clearTable(SensorAssignmentTable& table) {
  table.count = 0;
  for (uint8_t i = 0; i < MAX_DS18B20_SENSORS; i++) {
    table.items[i].role = Ds18Role::NONE;
    table.items[i].assigned = false;
    table.items[i].addressText[0] = '\0';
    for (uint8_t j = 0; j < 8; j++) table.items[i].address[j] = 0;
  }
}

bool hasRole(const SensorAssignmentTable& table, Ds18Role role) {
  return findRoleIndex(table, role) >= 0;
}

bool getAssignment(const SensorAssignmentTable& table, Ds18Role role, Ds18RoleAssignment& out) {
  int idx = findRoleIndex(table, role);
  if (idx < 0) return false;
  out = table.items[idx];
  return true;
}

bool setRole(SensorAssignmentTable& table, Ds18Role role, const uint8_t address[8], const char* addressText) {
  if (role == Ds18Role::NONE) return false;
  int idx = findRoleIndex(table, role);
  if (idx < 0) {
    if (table.count >= MAX_DS18B20_SENSORS) return false;
    idx = table.count++;
  }
  table.items[idx].role = role;
  table.items[idx].assigned = true;
  copyAddress(table.items[idx].address, address);
  copyText(table.items[idx].addressText, sizeof(table.items[idx].addressText), addressText);
  return true;
}

bool removeRole(SensorAssignmentTable& table, Ds18Role role) {
  int idx = findRoleIndex(table, role);
  if (idx < 0) return false;
  for (uint8_t i = idx; i + 1 < table.count; i++) table.items[i] = table.items[i + 1];
  if (table.count > 0) table.count--;
  return true;
}

bool inventoryContainsAddress(const Ds18b20Inventory& inventory, const uint8_t address[8]) {
  for (uint8_t i = 0; i < inventory.count; i++) {
    if (inventory.devices[i].present && sameAddress(inventory.devices[i].address, address)) return true;
  }
  return false;
}

bool assignRoleByAddressText(const Ds18b20Inventory& inventory, const String& addressText, Ds18Role role, SensorAssignmentTable& table) {
  for (uint8_t i = 0; i < inventory.count; i++) {
    if (inventory.devices[i].present && addressText.equalsIgnoreCase(inventory.devices[i].addressText)) {
      return setRole(table, role, inventory.devices[i].address, inventory.devices[i].addressText);
    }
  }
  return false;
}

bool readByRole(const SensorAssignmentTable& table, Ds18Role role, float& tempC, bool& valid) {
  tempC = NAN;
  valid = false;
  Ds18RoleAssignment a;
  if (!getAssignment(table, role, a)) return false;

  SensorAssignment legacy;
  legacy.sinkAssigned = a.assigned;
  for (uint8_t i = 0; i < 8; i++) legacy.sinkAddress[i] = a.address[i];
  strncpy(legacy.sinkAddressText, a.addressText, sizeof(legacy.sinkAddressText) - 1);
  legacy.sinkAddressText[sizeof(legacy.sinkAddressText) - 1] = '\0';

  return SinkSensor::readAssignedSink(legacy, tempC, valid);
}

Ds18Role activeSinkRole(const ConfigData& config) {
  return (config.activeSinkTarget == SinkTarget::BUFFER_TOP) ? Ds18Role::SINK_BUFFER_TOP : Ds18Role::SINK_BOILER_TOP;
}

bool autoAssignSingleSensorAsActiveSink(const Ds18b20Inventory& inventory, const ConfigData& config, SensorAssignmentTable& table) {
  if (inventory.count != 1 || !inventory.devices[0].present) return false;
  clearTable(table);
  Ds18Role role = activeSinkRole(config);
  return setRole(table, role, inventory.devices[0].address, inventory.devices[0].addressText);
}

bool resolveAssignments(const Ds18b20Inventory& inventory, const ConfigData& config, SensorAssignmentTable& table) {
  if (inventory.count == 0) return false;

  Ds18Role sinkRole = activeSinkRole(config);
  Ds18RoleAssignment a;
  if (getAssignment(table, sinkRole, a) && inventoryContainsAddress(inventory, a.address)) {
    return true;
  }

  if (inventory.count == 1) {
    return autoAssignSingleSensorAsActiveSink(inventory, config, table);
  }

  return false;
}

}
