#include "feature_heat_source_storage.h"
#include "config.h"
#include "feature_heat_source_roles.h"
#include "feature_heat_source_assignments.h"

#include <Arduino.h>
#include <SD.h>

namespace {
  String valueOf(const String& text, const String& key) {
    String needle = key + "=";
    int start = text.indexOf(needle);
    if (start < 0) {
      return "";
    }

    start += needle.length();
    int end = text.indexOf('\n', start);
    if (end < 0) {
      end = text.length();
    }

    String value = text.substring(start, end);
    value.trim();
    return value;
  }

  String readFileText(const char* path) {
    File f = SD.open(path, FILE_READ);
    if (!f) {
      return "";
    }

    String text;
    while (f.available()) {
      text += char(f.read());
    }
    f.close();
    return text;
  }

  bool ensureParentDir(const char* path) {
    if (!path || path[0] != '/') {
      return false;
    }

    String s(path);
    int slash = s.lastIndexOf('/');
    if (slash <= 0) {
      return true;
    }

    String dir = s.substring(0, slash);
    if (dir.length() == 0) {
      return true;
    }

    if (SD.exists(dir.c_str())) {
      return true;
    }

    return SD.mkdir(dir.c_str());
  }

  bool writeFileText(const char* path, const String& text) {
    ensureParentDir(path);
    SD.remove(path);

    File f = SD.open(path, FILE_WRITE);
    if (!f) {
      return false;
    }

    size_t written = f.print(text);
    f.close();
    return written == text.length();
  }

  const char* channelToKey(MaxChannel ch) {
    switch (ch) {
      case MaxChannel::CH1: return "ch1";
      case MaxChannel::CH2: return "ch2";
      case MaxChannel::CH3: return "ch3";
      default: return "ch1";
    }
  }

  MaxChannel channelFromKey(const String& key) {
    if (key.equalsIgnoreCase("ch2")) return MaxChannel::CH2;
    if (key.equalsIgnoreCase("ch3")) return MaxChannel::CH3;
    return MaxChannel::CH1;
  }
}

namespace HeatSourceStorage {

bool loadAssignments(HeatSourceAssignmentTable& table) {
  HeatSourceAssignments::clearTable(table);

  String text = readFileText(FILE_HEAT_SOURCE_ASSIGNMENTS);
  if (text.isEmpty()) {
    return false;
  }

  String v = valueOf(text, "count");
  int count = v.length() ? v.toInt() : 0;

  if (count < 0) count = 0;
  if (count > MAX_HEAT_SOURCE_ASSIGNMENTS) count = MAX_HEAT_SOURCE_ASSIGNMENTS;

  for (int i = 0; i < count; i++) {
    String chKey   = valueOf(text, "channel_" + String(i));
    String roleKey = valueOf(text, "role_" + String(i));

    if (!chKey.length() || !roleKey.length()) {
      continue;
    }

    MaxChannel channel = channelFromKey(chKey);
    HeatSourceRole role = HeatSourceRoles::fromKey(roleKey);

    if (role == HeatSourceRole::NONE) {
      continue;
    }

    HeatSourceAssignments::setAssignment(table, channel, role);
  }

  return table.count > 0;
}

bool saveAssignments(const HeatSourceAssignmentTable& table) {
  String text;

  text += "count=" + String(table.count) + "\n";

  for (uint8_t i = 0; i < table.count; i++) {
    if (!table.items[i].assigned || table.items[i].role == HeatSourceRole::NONE) {
      continue;
    }

    text += "channel_" + String(i) + "=" + String(channelToKey(table.items[i].channel)) + "\n";
    text += "role_" + String(i) + "=" + String(HeatSourceRoles::toKey(table.items[i].role)) + "\n";
  }

  return writeFileText(FILE_HEAT_SOURCE_ASSIGNMENTS, text);
}

}