#include "feature_heat_source_roles.h"

namespace HeatSourceRoles {

struct HeatSourceRoleMap {
  HeatSourceRole role;
  const char* key;
  const char* label;
  HeatSourceKind kind;
  ControlMode controlMode;
};

static const HeatSourceRoleMap kRoleMap[] = {
  { HeatSourceRole::NONE, "none", "Keine", HeatSourceKind::NONE, ControlMode::NONE },
  { HeatSourceRole::SOLAR_COLLECTOR_1, "solar_collector_1", "Solar Kollektor 1", HeatSourceKind::SOLAR, ControlMode::SOLAR_DIFF },
  { HeatSourceRole::SOLAR_COLLECTOR_2, "solar_collector_2", "Solar Kollektor 2", HeatSourceKind::SOLAR, ControlMode::SOLAR_DIFF },
  { HeatSourceRole::SOLAR_COLLECTOR_3, "solar_collector_3", "Solar Kollektor 3", HeatSourceKind::SOLAR, ControlMode::SOLAR_DIFF },
  { HeatSourceRole::ALT_SOURCE_OVEN, "alt_source_oven", "Ofen", HeatSourceKind::OVEN, ControlMode::OVEN_TRANSFER },
  { HeatSourceRole::ALT_SOURCE_OTHER, "alt_source_other", "Zusatzheizung / E-Kessel", HeatSourceKind::OTHER, ControlMode::ALT_SOURCE_TRANSFER },

  { HeatSourceRole::SENSOR_BOILER_TOP, "sensor_boiler_top", "Boiler Top", HeatSourceKind::NONE, ControlMode::NONE },
  { HeatSourceRole::SENSOR_BOILER_BOTTOM, "sensor_boiler_bottom", "Boiler Bottom", HeatSourceKind::NONE, ControlMode::NONE },
  { HeatSourceRole::SENSOR_BUFFER_TOP, "sensor_buffer_top", "Puffer Top", HeatSourceKind::NONE, ControlMode::NONE },
  { HeatSourceRole::SENSOR_BUFFER_MIDDLE, "sensor_buffer_middle", "Puffer Mitte", HeatSourceKind::NONE, ControlMode::NONE },
  { HeatSourceRole::SENSOR_BUFFER_BOTTOM, "sensor_buffer_bottom", "Puffer Bottom", HeatSourceKind::NONE, ControlMode::NONE },
  { HeatSourceRole::SENSOR_OUTSIDE_TEMPERATURE, "sensor_outside_temperature", "Aussentemperatur", HeatSourceKind::NONE, ControlMode::NONE },
  { HeatSourceRole::SENSOR_ROOM_1, "sensor_room_1", "Raum 1", HeatSourceKind::NONE, ControlMode::NONE },
  { HeatSourceRole::SENSOR_ROOM_2, "sensor_room_2", "Raum 2", HeatSourceKind::NONE, ControlMode::NONE },
  { HeatSourceRole::SENSOR_ROOM_3, "sensor_room_3", "Raum 3", HeatSourceKind::NONE, ControlMode::NONE },
  { HeatSourceRole::SENSOR_ROOM_4, "sensor_room_4", "Raum 4", HeatSourceKind::NONE, ControlMode::NONE },
  { HeatSourceRole::SENSOR_HK1_FLOW, "sensor_hk1_flow", "HK1 Vorlauf", HeatSourceKind::NONE, ControlMode::NONE },
  { HeatSourceRole::SENSOR_HK2_FLOW, "sensor_hk2_flow", "HK2 Vorlauf", HeatSourceKind::NONE, ControlMode::NONE },
  { HeatSourceRole::SENSOR_HK3_FLOW, "sensor_hk3_flow", "HK3 Vorlauf", HeatSourceKind::NONE, ControlMode::NONE },
  { HeatSourceRole::SENSOR_HK4_FLOW, "sensor_hk4_flow", "HK4 Vorlauf", HeatSourceKind::NONE, ControlMode::NONE },
  { HeatSourceRole::SENSOR_HK1_RETURN, "sensor_hk1_return", "HK1 Ruecklauf", HeatSourceKind::NONE, ControlMode::NONE },
  { HeatSourceRole::SENSOR_HK2_RETURN, "sensor_hk2_return", "HK2 Ruecklauf", HeatSourceKind::NONE, ControlMode::NONE },
  { HeatSourceRole::SENSOR_HK3_RETURN, "sensor_hk3_return", "HK3 Ruecklauf", HeatSourceKind::NONE, ControlMode::NONE },
  { HeatSourceRole::SENSOR_HK4_RETURN, "sensor_hk4_return", "HK4 Ruecklauf", HeatSourceKind::NONE, ControlMode::NONE }
};

static constexpr size_t kRoleMapCount = sizeof(kRoleMap) / sizeof(kRoleMap[0]);

const char* toKey(HeatSourceRole role) {
  for (size_t i = 0; i < kRoleMapCount; i++) {
    if (kRoleMap[i].role == role) {
      return kRoleMap[i].key;
    }
  }
  return "none";
}

const char* toLabel(HeatSourceRole role) {
  for (size_t i = 0; i < kRoleMapCount; i++) {
    if (kRoleMap[i].role == role) {
      return kRoleMap[i].label;
    }
  }
  return "Unbekannt";
}

HeatSourceRole fromKey(const String& key) {
  for (size_t i = 0; i < kRoleMapCount; i++) {
    if (key.equalsIgnoreCase(kRoleMap[i].key)) {
      return kRoleMap[i].role;
    }
  }
  return HeatSourceRole::NONE;
}

HeatSourceKind kindOf(HeatSourceRole role) {
  for (size_t i = 0; i < kRoleMapCount; i++) {
    if (kRoleMap[i].role == role) {
      return kRoleMap[i].kind;
    }
  }
  return HeatSourceKind::NONE;
}

ControlMode controlModeOf(HeatSourceRole role) {
  for (size_t i = 0; i < kRoleMapCount; i++) {
    if (kRoleMap[i].role == role) {
      return kRoleMap[i].controlMode;
    }
  }
  return ControlMode::NONE;
}

bool isSolarRole(HeatSourceRole role) {
  return
    role == HeatSourceRole::SOLAR_COLLECTOR_1 ||
    role == HeatSourceRole::SOLAR_COLLECTOR_2 ||
    role == HeatSourceRole::SOLAR_COLLECTOR_3;
}

bool isAltSourceRole(HeatSourceRole role) {
  return
    role == HeatSourceRole::ALT_SOURCE_OVEN ||
    role == HeatSourceRole::ALT_SOURCE_OTHER;
}

bool isAssignable(HeatSourceRole role) {
  return role != HeatSourceRole::NONE;
}

}