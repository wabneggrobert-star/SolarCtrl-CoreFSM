#include "feature_alarms.h"

#include "feature_pwm_pca9685.h"
#include "feature_relay_outputs.h"

#include <Arduino.h>
#include <SD.h>
#include <string.h>

namespace Alarms {
namespace {
  static constexpr uint8_t MAX_ACTIVE_ALARMS = 24;
  static constexpr uint8_t MAX_HISTORY_ALARMS = 24;
  static constexpr const char* ALARM_HISTORY_PATH = "/logs/alarm_history.csv";
  static constexpr uint32_t WARNING_BUZZER_PERIOD_MS = 60000UL;
  static constexpr uint32_t WARNING_BUZZER_ON_MS = 1000UL;

  struct AlarmRecord {
    bool used = false;
    bool active = false;
    bool acknowledged = false;
    bool acknowledgeable = true;
    Severity severity = Severity::INFO;
    char id[48] = "";
    char message[128] = "";
    uint32_t firstSeenMs = 0;
    uint32_t lastSeenMs = 0;
    uint32_t clearedAtMs = 0;
    uint32_t count = 0;
  };

  struct BuzzerRuntime {
    OutputKind lastKind = OutputKind::NONE;
    uint8_t lastIndex = PIN_UNUSED;
    bool lastState = false;
    bool muted = false;
    uint32_t lastProcessMs = 0;
  };

  AlarmRecord g_active[MAX_ACTIVE_ALARMS];
  AlarmRecord g_history[MAX_HISTORY_ALARMS];
  uint8_t g_nextHistorySlot = 0;
  bool g_persistentHistoryEnabled = false;
  BuzzerRuntime g_buzzer;

  void copyText(char* dst, size_t dstSize, const char* src) {
    if (!dst || dstSize == 0) return;
    strncpy(dst, src ? src : "", dstSize - 1);
    dst[dstSize - 1] = '\0';
  }

  int findActive(const char* id) {
    if (!id || !id[0]) return -1;
    for (uint8_t i = 0; i < MAX_ACTIVE_ALARMS; i++) {
      if (g_active[i].used && strcmp(g_active[i].id, id) == 0) return i;
    }
    return -1;
  }

  int firstFreeActiveSlot() {
    for (uint8_t i = 0; i < MAX_ACTIVE_ALARMS; i++) {
      if (!g_active[i].used) return i;
    }
    return -1;
  }

  void pushHistory(const AlarmRecord& src, uint32_t clearedAtMs) {
    AlarmRecord& dst = g_history[g_nextHistorySlot];
    dst = src;
    dst.used = true;
    dst.active = false;
    dst.clearedAtMs = clearedAtMs;
    g_nextHistorySlot = (uint8_t)((g_nextHistorySlot + 1) % MAX_HISTORY_ALARMS);
  }

  bool ensureParentDir(const char* path) {
    if (!path || path[0] != '/') return false;
    String s(path);
    const int slash = s.lastIndexOf('/');
    if (slash <= 0) return true;
    const String dir = s.substring(0, slash);
    if (dir.length() == 0) return true;
    if (SD.exists(dir.c_str())) return true;
    return SD.mkdir(dir.c_str());
  }

  void appendCsvEscaped(File& f, const char* value) {
    f.print('"');
    const char* p = value ? value : "";
    while (*p) {
      if (*p == '"') f.print("\"\"");
      else if (*p == '\r' || *p == '\n') f.print(' ');
      else f.print(*p);
      ++p;
    }
    f.print('"');
  }

  void appendHistoryCsv(const char* event, const AlarmRecord& a) {
    if (!g_persistentHistoryEnabled) return;
    ensureParentDir(ALARM_HISTORY_PATH);

    const bool writeHeader = !SD.exists(ALARM_HISTORY_PATH);
    File f = SD.open(ALARM_HISTORY_PATH, FILE_APPEND);
    if (!f) return;

    if (writeHeader) {
      f.println("millis,event,id,severity,message,acknowledged,firstSeenMs,lastSeenMs,clearedAtMs,count");
    }

    f.print(millis());
    f.print(',');
    appendCsvEscaped(f, event ? event : "EVENT");
    f.print(',');
    appendCsvEscaped(f, a.id);
    f.print(',');
    appendCsvEscaped(f, severityToKey(a.severity));
    f.print(',');
    appendCsvEscaped(f, a.message);
    f.print(',');
    f.print(a.acknowledged ? 1 : 0);
    f.print(',');
    f.print(a.firstSeenMs);
    f.print(',');
    f.print(a.lastSeenMs);
    f.print(',');
    f.print(a.clearedAtMs);
    f.print(',');
    f.println(a.count);
    f.close();
  }

  void appendEscaped(String& json, const char* value) {
    const char* p = value ? value : "";
    while (*p) {
      const char c = *p++;
      if (c == '\\' || c == '"') {
        json += '\\';
        json += c;
      } else if (c == '\n') {
        json += "\\n";
      } else if (c == '\r') {
        json += "\\r";
      } else {
        json += c;
      }
    }
  }

  void appendAlarmJson(String& json, const AlarmRecord& a, bool includeCleared) {
    json += "{";
    json += "\"id\":\"";
    appendEscaped(json, a.id);
    json += "\",";
    json += "\"severity\":\"";
    json += severityToKey(a.severity);
    json += "\",";
    json += "\"severityLabel\":\"";
    json += severityToLabel(a.severity);
    json += "\",";
    json += "\"message\":\"";
    appendEscaped(json, a.message);
    json += "\",";
    json += "\"active\":";
    json += (a.active ? "true" : "false");
    json += ",\"acknowledged\":";
    json += (a.acknowledged ? "true" : "false");
    json += ",\"acknowledgeable\":";
    json += (a.acknowledgeable ? "true" : "false");
    json += ",\"firstSeenMs\":";
    json += String(a.firstSeenMs);
    json += ",\"lastSeenMs\":";
    json += String(a.lastSeenMs);
    json += ",\"count\":";
    json += String(a.count);
    if (includeCleared) {
      json += ",\"clearedAtMs\":";
      json += String(a.clearedAtMs);
    }
    json += "}";
  }

  bool outputMatchesBuzzerFunction(const AppContext& ctx, OutputKind kind, uint8_t index) {
    if (kind == OutputKind::RELAY) {
      if (index >= RELAY_COUNT) return false;
      const RelayOutputConfig& out = ctx.config.relays[index];
      return out.enabled && out.function == RelayFunction::ALARM_BUZZER;
    }

    if (kind == OutputKind::PWM_OUTPUT) {
      if (index >= PWM_OUTPUT_COUNT) return false;
      const PwmOutputConfig& out = ctx.config.pwmOutputs[index];
      return out.enabled && out.mode == PwmOutputMode::SWITCH && out.function == RelayFunction::ALARM_BUZZER;
    }

    return false;
  }

  bool findBuzzerOutput(const AppContext& ctx, OutputKind& kind, uint8_t& index) {
    for (uint8_t i = 0; i < RELAY_COUNT; i++) {
      if (outputMatchesBuzzerFunction(ctx, OutputKind::RELAY, i)) {
        kind = OutputKind::RELAY;
        index = i;
        return true;
      }
    }

    for (uint8_t i = 0; i < PWM_OUTPUT_COUNT; i++) {
      if (outputMatchesBuzzerFunction(ctx, OutputKind::PWM_OUTPUT, i)) {
        kind = OutputKind::PWM_OUTPUT;
        index = i;
        return true;
      }
    }

    kind = OutputKind::NONE;
    index = PIN_UNUSED;
    return false;
  }


  bool readBuzzerOutputState(const AppContext& ctx, OutputKind kind, uint8_t index) {
    if (kind == OutputKind::RELAY) {
      return RelayOutputs::get(ctx, index);
    }

    if (kind == OutputKind::PWM_OUTPUT) {
      if (index >= PWM_OUTPUT_COUNT) return false;
      return PwmDriver::getDuty(index) > 0;
    }

    return false;
  }

  bool setBuzzerOutput(AppContext& ctx, OutputKind kind, uint8_t index, bool on) {
    if (kind == OutputKind::RELAY) {
      return RelayOutputs::set(ctx, index, on);
    }

    if (kind == OutputKind::PWM_OUTPUT) {
      if (index >= PWM_OUTPUT_COUNT) return false;
      const PwmOutputConfig& out = ctx.config.pwmOutputs[index];
      if (!out.enabled || out.mode != PwmOutputMode::SWITCH || out.function != RelayFunction::ALARM_BUZZER) return false;
      PwmDriver::setSwitch(index, on, out.profile);
      return true;
    }

    return false;
  }

  void turnPreviousBuzzerOffIfNeeded(AppContext& ctx, OutputKind kind, uint8_t index) {
    if (g_buzzer.lastKind == OutputKind::NONE || g_buzzer.lastIndex == PIN_UNUSED) return;
    if (g_buzzer.lastKind == kind && g_buzzer.lastIndex == index) return;
    setBuzzerOutput(ctx, g_buzzer.lastKind, g_buzzer.lastIndex, false);
    g_buzzer.lastState = false;
  }

  const char* buzzerKindKey(OutputKind kind) {
    if (kind == OutputKind::RELAY) return "relay";
    if (kind == OutputKind::PWM_OUTPUT) return "pwm";
    return "none";
  }
}

void begin() {
  resetAll();
}

void setPersistentHistoryEnabled(bool enabled) {
  g_persistentHistoryEnabled = enabled;
  if (enabled) {
    ensureParentDir(ALARM_HISTORY_PATH);
    if (!SD.exists(ALARM_HISTORY_PATH)) {
      File f = SD.open(ALARM_HISTORY_PATH, FILE_WRITE);
      if (f) {
        f.println("millis,event,id,severity,message,acknowledged,firstSeenMs,lastSeenMs,clearedAtMs,count");
        f.close();
      }
    }
  }
}

void resetAll() {
  memset(g_active, 0, sizeof(g_active));
  memset(g_history, 0, sizeof(g_history));
  g_nextHistorySlot = 0;
  g_buzzer = BuzzerRuntime{};
}

void raise(const char* id, Severity severity, const char* message, bool acknowledgeable) {
  if (!id || !id[0]) return;

  const uint32_t now = millis();
  int idx = findActive(id);
  const bool isNew = idx < 0;
  if (idx < 0) {
    idx = firstFreeActiveSlot();
    if (idx < 0) return;
    g_active[idx] = AlarmRecord{};
    g_active[idx].used = true;
    g_active[idx].active = true;
    g_active[idx].firstSeenMs = now;
    copyText(g_active[idx].id, sizeof(g_active[idx].id), id);
  }

  AlarmRecord& a = g_active[idx];
  a.active = true;
  a.severity = severity;
  a.acknowledgeable = acknowledgeable && severity != Severity::CRITICAL;
  if (severity == Severity::CRITICAL) a.acknowledged = false;
  a.lastSeenMs = now;
  a.count++;
  copyText(a.message, sizeof(a.message), message);

  if (isNew) {
    if (severity == Severity::WARNING || severity == Severity::CRITICAL) g_buzzer.muted = false;
    appendHistoryCsv("RAISE", a);
  }
}

void clear(const char* id) {
  const int idx = findActive(id);
  if (idx < 0) return;
  g_active[idx].clearedAtMs = millis();
  pushHistory(g_active[idx], g_active[idx].clearedAtMs);
  appendHistoryCsv("CLEAR", g_active[idx]);
  g_active[idx] = AlarmRecord{};
  if (activeCount() == 0) g_buzzer.muted = false;
}

void recordInfo(const char* id, const char* message) {
  AlarmRecord a;
  a.used = true;
  a.active = false;
  a.acknowledged = true;
  a.acknowledgeable = false;
  a.severity = Severity::INFO;
  copyText(a.id, sizeof(a.id), id ? id : "info");
  copyText(a.message, sizeof(a.message), message);
  a.firstSeenMs = millis();
  a.lastSeenMs = a.firstSeenMs;
  a.clearedAtMs = a.firstSeenMs;
  a.count = 1;
  pushHistory(a, a.clearedAtMs);
  appendHistoryCsv("INFO", a);
}

bool acknowledge(const char* id) {
  const int idx = findActive(id);
  if (idx < 0) return false;
  AlarmRecord& a = g_active[idx];
  if (!a.acknowledgeable || a.severity == Severity::CRITICAL) return false;
  a.acknowledged = true;
  appendHistoryCsv("ACK", a);
  return true;
}

uint8_t acknowledgeAllNonCritical() {
  uint8_t count = 0;
  for (uint8_t i = 0; i < MAX_ACTIVE_ALARMS; i++) {
    AlarmRecord& a = g_active[i];
    if (!a.used || !a.active) continue;
    if (!a.acknowledgeable || a.severity == Severity::CRITICAL) continue;
    if (!a.acknowledged) {
      a.acknowledged = true;
      appendHistoryCsv("ACK", a);
      count++;
    }
  }
  return count;
}

void setBuzzerMuted(bool muted) {
  g_buzzer.muted = muted && activeCount() > 0;
}

bool buzzerMuted() { return g_buzzer.muted; }

Severity currentSeverity() {
  Severity best = Severity::INFO;
  bool found = false;
  for (uint8_t i = 0; i < MAX_ACTIVE_ALARMS; i++) {
    const AlarmRecord& a = g_active[i];
    if (!a.used || !a.active) continue;
    if (!found || (uint8_t)a.severity > (uint8_t)best) { best = a.severity; found = true; }
  }
  return best;
}

const char* currentMessage() {
  int bestIdx = -1;
  Severity best = Severity::INFO;
  for (uint8_t i = 0; i < MAX_ACTIVE_ALARMS; i++) {
    const AlarmRecord& a = g_active[i];
    if (!a.used || !a.active) continue;
    if (bestIdx < 0 || (uint8_t)a.severity > (uint8_t)best) { bestIdx = i; best = a.severity; }
  }
  return bestIdx >= 0 ? g_active[bestIdx].message : "";
}

void clearHistory() {
  memset(g_history, 0, sizeof(g_history));
  g_nextHistorySlot = 0;
  if (g_persistentHistoryEnabled) {
    SD.remove(ALARM_HISTORY_PATH);
    setPersistentHistoryEnabled(true);
  }
}

void process(AppContext& ctx) {
  OutputKind kind = OutputKind::NONE;
  uint8_t index = PIN_UNUSED;
  const bool configured = findBuzzerOutput(ctx, kind, index);

  if (!configured) {
    turnPreviousBuzzerOffIfNeeded(ctx, OutputKind::NONE, PIN_UNUSED);
    g_buzzer.lastKind = OutputKind::NONE;
    g_buzzer.lastIndex = PIN_UNUSED;
    return;
  }

  turnPreviousBuzzerOffIfNeeded(ctx, kind, index);

  bool requestedOn = false;
  if (!g_buzzer.muted) {
    if (hasCriticalActive()) {
      requestedOn = true;
    } else if (hasUnacknowledgedWarning()) {
      requestedOn = (millis() % WARNING_BUZZER_PERIOD_MS) < WARNING_BUZZER_ON_MS;
    }
  }

  const bool actualOn = readBuzzerOutputState(ctx, kind, index);
  if (g_buzzer.lastKind != kind ||
      g_buzzer.lastIndex != index ||
      g_buzzer.lastState != requestedOn ||
      actualOn != requestedOn) {
    setBuzzerOutput(ctx, kind, index, requestedOn);
    g_buzzer.lastKind = kind;
    g_buzzer.lastIndex = index;
    g_buzzer.lastState = requestedOn;
  }

  g_buzzer.lastProcessMs = millis();
}

uint8_t activeCount() {
  uint8_t count = 0;
  for (uint8_t i = 0; i < MAX_ACTIVE_ALARMS; i++) {
    if (g_active[i].used && g_active[i].active) count++;
  }
  return count;
}

uint8_t unacknowledgedCount() {
  uint8_t count = 0;
  for (uint8_t i = 0; i < MAX_ACTIVE_ALARMS; i++) {
    const AlarmRecord& a = g_active[i];
    if (a.used && a.active && !a.acknowledged) count++;
  }
  return count;
}

bool hasCriticalActive() {
  for (uint8_t i = 0; i < MAX_ACTIVE_ALARMS; i++) {
    const AlarmRecord& a = g_active[i];
    if (a.used && a.active && a.severity == Severity::CRITICAL) return true;
  }
  return false;
}

bool hasUnacknowledgedWarning() {
  for (uint8_t i = 0; i < MAX_ACTIVE_ALARMS; i++) {
    const AlarmRecord& a = g_active[i];
    if (a.used && a.active && !a.acknowledged && a.severity == Severity::WARNING) return true;
  }
  return false;
}

const char* severityToKey(Severity severity) {
  switch (severity) {
    case Severity::INFO: return "info";
    case Severity::WARNING: return "warning";
    case Severity::CRITICAL: return "critical";
  }
  return "info";
}

const char* severityToLabel(Severity severity) {
  switch (severity) {
    case Severity::INFO: return "Info";
    case Severity::WARNING: return "Warnung";
    case Severity::CRITICAL: return "Kritisch";
  }
  return "Info";
}

void appendJson(String& json) {
  OutputKind buzzerKind = OutputKind::NONE;
  uint8_t buzzerIndex = PIN_UNUSED;
  const bool buzzerConfigured = (g_buzzer.lastKind != OutputKind::NONE && g_buzzer.lastIndex != PIN_UNUSED);

  json += "{";
  json += "\"summary\":{";
  json += "\"activeCount\":";
  json += String(activeCount());
  json += ",";
  json += "\"unacknowledgedCount\":";
  json += String(unacknowledgedCount());
  json += ",";
  json += "\"criticalActive\":";
  json += (hasCriticalActive() ? "true" : "false");
  json += ",\"unacknowledgedWarningActive\":";
  json += (hasUnacknowledgedWarning() ? "true" : "false");
  json += "},";

  (void)buzzerKind;
  (void)buzzerIndex;
  json += "\"buzzer\":{";
  json += "\"configured\":";
  json += (buzzerConfigured ? "true" : "false");
  json += ",\"kind\":\"";
  json += buzzerKindKey(g_buzzer.lastKind);
  json += "\",\"index\":";
  if (g_buzzer.lastIndex == PIN_UNUSED) json += "null"; else json += String(g_buzzer.lastIndex);
  json += ",\"state\":";
  json += (g_buzzer.lastState ? "true" : "false");
  json += ",\"muted\":";
  json += (g_buzzer.muted ? "true" : "false");
  json += ",\"warningPulseMs\":";
  json += String(WARNING_BUZZER_ON_MS);
  json += ",\"warningPeriodMs\":";
  json += String(WARNING_BUZZER_PERIOD_MS);
  json += ",\"historyPath\":\"/logs/alarm_history.csv\"";
  json += "},";

  json += "\"active\":[";
  bool first = true;
  for (uint8_t i = 0; i < MAX_ACTIVE_ALARMS; i++) {
    if (!g_active[i].used || !g_active[i].active) continue;
    if (!first) json += ",";
    first = false;
    appendAlarmJson(json, g_active[i], false);
  }
  json += "],";

  json += "\"history\":[";
  first = true;
  for (uint8_t n = 0; n < MAX_HISTORY_ALARMS; n++) {
    const uint8_t idx = (uint8_t)((g_nextHistorySlot + MAX_HISTORY_ALARMS - 1 - n) % MAX_HISTORY_ALARMS);
    if (!g_history[idx].used) continue;
    if (!first) json += ",";
    first = false;
    appendAlarmJson(json, g_history[idx], true);
  }
  json += "]";
  json += "}";
}

}  // namespace Alarms
