#include "feature_heat_source_assignments.h"
#include "feature_heat_source_roles.h"

namespace {
  int findRoleIndex(const HeatSourceAssignmentTable& table, HeatSourceRole role) {
    for (uint8_t i = 0; i < table.count; i++) {
      if (table.items[i].assigned && table.items[i].role == role) {
        return i;
      }
    }
    return -1;
  }

  int findChannelIndex(const HeatSourceAssignmentTable& table, MaxChannel channel) {
    for (uint8_t i = 0; i < table.count; i++) {
      if (table.items[i].assigned && table.items[i].channel == channel) {
        return i;
      }
    }
    return -1;
  }

  int channelToIndex(MaxChannel ch) {
    switch (ch) {
      case MaxChannel::CH1: return 0;
      case MaxChannel::CH2: return 1;
      case MaxChannel::CH3: return 2;
      default: return -1;
    }
  }

  void assignReadingToRole(
    HeatSourceSnapshot& out,
    HeatSourceRole role,
    const MaxChannelReading& reading
  ) {
    switch (role) {
      case HeatSourceRole::SOLAR_COLLECTOR_1:
        out.solarCollector1C = reading.tempC;
        out.solarCollector1Valid = reading.valid;
        break;

      case HeatSourceRole::SOLAR_COLLECTOR_2:
        out.solarCollector2C = reading.tempC;
        out.solarCollector2Valid = reading.valid;
        break;

      case HeatSourceRole::SOLAR_COLLECTOR_3:
        out.solarCollector3C = reading.tempC;
        out.solarCollector3Valid = reading.valid;
        break;

      case HeatSourceRole::ALT_SOURCE_OVEN:
        out.altSourceOvenC = reading.tempC;
        out.altSourceOvenValid = reading.valid;
        break;

      case HeatSourceRole::ALT_SOURCE_OTHER:
        out.altSourceOtherC = reading.tempC;
        out.altSourceOtherValid = reading.valid;
        break;

      case HeatSourceRole::NONE:
      default:
        break;
    }
  }

  bool roleValue(
    const HeatSourceSnapshot& src,
    HeatSourceRole role,
    float& tempC,
    bool& valid
  ) {
    tempC = NAN;
    valid = false;

    switch (role) {
      case HeatSourceRole::SOLAR_COLLECTOR_1:
        tempC = src.solarCollector1C;
        valid = src.solarCollector1Valid;
        return true;

      case HeatSourceRole::SOLAR_COLLECTOR_2:
        tempC = src.solarCollector2C;
        valid = src.solarCollector2Valid;
        return true;

      case HeatSourceRole::SOLAR_COLLECTOR_3:
        tempC = src.solarCollector3C;
        valid = src.solarCollector3Valid;
        return true;

      case HeatSourceRole::ALT_SOURCE_OVEN:
        tempC = src.altSourceOvenC;
        valid = src.altSourceOvenValid;
        return true;

      case HeatSourceRole::ALT_SOURCE_OTHER:
        tempC = src.altSourceOtherC;
        valid = src.altSourceOtherValid;
        return true;

      case HeatSourceRole::NONE:
      default:
        return false;
    }
  }
}

namespace HeatSourceAssignments {

void clearTable(HeatSourceAssignmentTable& table) {
  table.count = 0;
  for (uint8_t i = 0; i < MAX_HEAT_SOURCE_ASSIGNMENTS; i++) {
    table.items[i].assigned = false;
    table.items[i].channel = MaxChannel::CH1;
    table.items[i].role = HeatSourceRole::NONE;
  }
}

bool hasRole(const HeatSourceAssignmentTable& table, HeatSourceRole role) {
  return findRoleIndex(table, role) >= 0;
}

bool hasChannel(const HeatSourceAssignmentTable& table, MaxChannel channel) {
  return findChannelIndex(table, channel) >= 0;
}

bool getByRole(
  const HeatSourceAssignmentTable& table,
  HeatSourceRole role,
  HeatSourceAssignment& out
) {
  int idx = findRoleIndex(table, role);
  if (idx < 0) {
    return false;
  }
  out = table.items[idx];
  return true;
}

bool getByChannel(
  const HeatSourceAssignmentTable& table,
  MaxChannel channel,
  HeatSourceAssignment& out
) {
  int idx = findChannelIndex(table, channel);
  if (idx < 0) {
    return false;
  }
  out = table.items[idx];
  return true;
}

bool setAssignment(
  HeatSourceAssignmentTable& table,
  MaxChannel channel,
  HeatSourceRole role
) {
  if (!HeatSourceRoles::isAssignable(role)) {
    return false;
  }

  // Doppelte Rollen verhindern
  int existingRoleIdx = findRoleIndex(table, role);
  if (existingRoleIdx >= 0) {
    // Gleiche Rolle bereits vorhanden -> auf neuen Kanal umbiegen
    table.items[existingRoleIdx].assigned = true;
    table.items[existingRoleIdx].channel = channel;
    table.items[existingRoleIdx].role = role;
    return true;
  }

  // Falls derselbe Kanal bereits eine andere Rolle hat, überschreiben
  int existingChannelIdx = findChannelIndex(table, channel);
  if (existingChannelIdx >= 0) {
    table.items[existingChannelIdx].assigned = true;
    table.items[existingChannelIdx].channel = channel;
    table.items[existingChannelIdx].role = role;
    return true;
  }

  if (table.count >= MAX_HEAT_SOURCE_ASSIGNMENTS) {
    return false;
  }

  table.items[table.count].assigned = true;
  table.items[table.count].channel = channel;
  table.items[table.count].role = role;
  table.count++;

  return true;
}

bool removeByRole(
  HeatSourceAssignmentTable& table,
  HeatSourceRole role
) {
  int idx = findRoleIndex(table, role);
  if (idx < 0) {
    return false;
  }

  for (uint8_t i = idx; i + 1 < table.count; i++) {
    table.items[i] = table.items[i + 1];
  }

  if (table.count > 0) {
    table.count--;
    table.items[table.count].assigned = false;
    table.items[table.count].channel = MaxChannel::CH1;
    table.items[table.count].role = HeatSourceRole::NONE;
  }

  return true;
}

bool removeByChannel(
  HeatSourceAssignmentTable& table,
  MaxChannel channel
) {
  int idx = findChannelIndex(table, channel);
  if (idx < 0) {
    return false;
  }

  for (uint8_t i = idx; i + 1 < table.count; i++) {
    table.items[i] = table.items[i + 1];
  }

  if (table.count > 0) {
    table.count--;
    table.items[table.count].assigned = false;
    table.items[table.count].channel = MaxChannel::CH1;
    table.items[table.count].role = HeatSourceRole::NONE;
  }

  return true;
}

bool roleAssignedAndValid(
  const AppContext& ctx,
  HeatSourceRole role
) {
  float t = NAN;
  bool valid = false;
  if (!roleValue(ctx.sensors.heatSources, role, t, valid)) {
    return false;
  }
  return valid;
}

bool resolveHeatSources(AppContext& ctx) {
  // Snapshot zurücksetzen
  ctx.sensors.heatSources = HeatSourceSnapshot{};
  ctx.sensors.activeHeatSource = ActiveHeatSource{};

  for (uint8_t i = 0; i < ctx.heatSourceAssignments.count; i++) {
    const HeatSourceAssignment& a = ctx.heatSourceAssignments.items[i];
    if (!a.assigned || a.role == HeatSourceRole::NONE) {
      continue;
    }

    int idx = channelToIndex(a.channel);
    if (idx < 0 || idx >= 3) {
      continue;
    }

    assignReadingToRole(ctx.sensors.heatSources, a.role, ctx.maxReadings[idx]);
  }

  ActiveHeatSource active;
  if (findFirstValidHeatSource(ctx, active)) {
    ctx.sensors.activeHeatSource = active;

    // Kompatibilität zum bestehenden collector-Pfad
    ctx.sensors.collectorC = active.tempC;
    ctx.sensors.collectorValid = active.valid;
    return true;
  }

  // Keine gültige Wärmequelle gefunden
  ctx.sensors.collectorC = NAN;
  ctx.sensors.collectorValid = false;
  return false;
}

bool findFirstValidHeatSource(
  const AppContext& ctx,
  ActiveHeatSource& out
) {
  // Priorität: Solar 1 -> Solar 2 -> Solar 3 -> Ofen -> sonstige Quelle
  static const HeatSourceRole priority[] = {
    HeatSourceRole::SOLAR_COLLECTOR_1,
    HeatSourceRole::SOLAR_COLLECTOR_2,
    HeatSourceRole::SOLAR_COLLECTOR_3,
    HeatSourceRole::ALT_SOURCE_OVEN,
    HeatSourceRole::ALT_SOURCE_OTHER
  };

  for (uint8_t i = 0; i < sizeof(priority) / sizeof(priority[0]); i++) {
    float tempC = NAN;
    bool valid = false;

    if (!roleValue(ctx.sensors.heatSources, priority[i], tempC, valid)) {
      continue;
    }

    if (!valid) {
      continue;
    }

    out.role = priority[i];
    out.kind = HeatSourceRoles::kindOf(priority[i]);
    out.controlMode = HeatSourceRoles::controlModeOf(priority[i]);
    out.tempC = tempC;
    out.valid = true;
    return true;
  }

  out = ActiveHeatSource{};
  return false;
}

}