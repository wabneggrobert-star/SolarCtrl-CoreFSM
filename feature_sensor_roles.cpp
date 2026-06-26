#include "feature_sensor_roles.h"

namespace SensorRoles {

struct RoleMap {
  Ds18Role role;
  const char* key;
  const char* label;
  bool assignable;
};

static const RoleMap roleMap[] = {
  {Ds18Role::NONE, "none", "Keine", true},

  // Bestehende Rollen bleiben fuer Rueckwaertskompatibilitaet erhalten.
  {Ds18Role::SINK_BOILER_TOP, "sink_boiler_top", "Boiler Top", true},
  {Ds18Role::SINK_BUFFER_TOP, "sink_buffer_top", "Puffer Top", true},
  {Ds18Role::BOILER_BOTTOM, "boiler_bottom", "Boiler Bottom", true},
  {Ds18Role::BUFFER_HIGH, "buffer_high", "Puffer High", true},
  {Ds18Role::BUFFER_MID, "buffer_mid", "Puffer Mitte", true},
  {Ds18Role::BUFFER_BOTTOM, "buffer_bottom", "Puffer Bottom", true},

  // Alte Kollektor-/Quellenrollen fuer DS18 werden nicht mehr neu angeboten.
  {Ds18Role::FLOW_COLLECTOR_1, "flow_collector_1", "ALT: Vorlauf Kollektor 1", false},
  {Ds18Role::FLOW_COLLECTOR_2, "flow_collector_2", "ALT: Vorlauf Kollektor 2", false},
  {Ds18Role::FLOW_COLLECTOR_3, "flow_collector_3", "ALT: Vorlauf Kollektor 3", false},
  {Ds18Role::FLOW_ALT_SOURCE, "flow_alt_source", "ALT: Vorlauf Alternative Quelle", false},
  {Ds18Role::RETURN_COLLECTOR_1, "return_collector_1", "ALT: Ruecklauf Kollektor 1", false},
  {Ds18Role::RETURN_COLLECTOR_2, "return_collector_2", "ALT: Ruecklauf Kollektor 2", false},
  {Ds18Role::RETURN_COLLECTOR_3, "return_collector_3", "ALT: Ruecklauf Kollektor 3", false},
  {Ds18Role::RETURN_ALT_SOURCE, "return_alt_source", "ALT: Ruecklauf Alternative Quelle", false},

  {Ds18Role::CIRCULATION, "circulation", "Zirkulation", true},
  {Ds18Role::SWIMMINGPOOL, "swimmingpool", "Swimmingpool", true},
  {Ds18Role::RESERVE_1, "reserve_1", "Reserve 1", true},
  {Ds18Role::RESERVE_2, "reserve_2", "Reserve 2", true},
  {Ds18Role::RESERVE_3, "reserve_3", "Reserve 3", true},
  {Ds18Role::RESERVE_4, "reserve_4", "Reserve 4", true},

  {Ds18Role::OUTSIDE_TEMPERATURE, "outside_temperature", "Aussentemperatur", true},
  {Ds18Role::ROOM_1, "room_1", "Raum 1", true},
  {Ds18Role::ROOM_2, "room_2", "Raum 2", true},
  {Ds18Role::ROOM_3, "room_3", "Raum 3", true},
  {Ds18Role::ROOM_4, "room_4", "Raum 4", true},
  {Ds18Role::HK1_FLOW, "hk1_flow", "HK1 Vorlauf", true},
  {Ds18Role::HK2_FLOW, "hk2_flow", "HK2 Vorlauf", true},
  {Ds18Role::HK3_FLOW, "hk3_flow", "HK3 Vorlauf", true},
  {Ds18Role::HK4_FLOW, "hk4_flow", "HK4 Vorlauf", true},
  {Ds18Role::HK1_RETURN, "hk1_return", "HK1 Ruecklauf", true},
  {Ds18Role::HK2_RETURN, "hk2_return", "HK2 Ruecklauf", true},
  {Ds18Role::HK3_RETURN, "hk3_return", "HK3 Ruecklauf", true},
  {Ds18Role::HK4_RETURN, "hk4_return", "HK4 Ruecklauf", true}
};

static constexpr size_t ROLE_COUNT = sizeof(roleMap) / sizeof(roleMap[0]);

const char* toKey(Ds18Role role) {
  for (size_t i = 0; i < ROLE_COUNT; i++) if (roleMap[i].role == role) return roleMap[i].key;
  return "none";
}

const char* toLabel(Ds18Role role) {
  for (size_t i = 0; i < ROLE_COUNT; i++) if (roleMap[i].role == role) return roleMap[i].label;
  return "Unbekannt";
}

Ds18Role fromKey(const String& key) {
  for (size_t i = 0; i < ROLE_COUNT; i++) {
    if (key.equalsIgnoreCase(roleMap[i].key)) return roleMap[i].role;
  }
  return Ds18Role::NONE;
}

bool isSinkRole(Ds18Role role) {
  return role == Ds18Role::SINK_BOILER_TOP ||
         role == Ds18Role::SINK_BUFFER_TOP ||
         role == Ds18Role::BOILER_BOTTOM ||
         role == Ds18Role::BUFFER_HIGH ||
         role == Ds18Role::BUFFER_MID ||
         role == Ds18Role::BUFFER_BOTTOM;
}

bool isAssignable(Ds18Role role) {
  for (size_t i = 0; i < ROLE_COUNT; i++) {
    if (roleMap[i].role == role) return roleMap[i].assignable;
  }
  return false;
}

}
