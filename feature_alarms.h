#pragma once

#include "app_types.h"

#include <Arduino.h>
#include <stdint.h>

namespace Alarms {

enum class Severity : uint8_t {
  INFO = 0,
  WARNING = 1,
  CRITICAL = 2
};

void begin();
void setPersistentHistoryEnabled(bool enabled);

// Aktiver Alarm. Bei erneutem raise() derselben ID wird der vorhandene
// Eintrag aktualisiert und nicht dupliziert.
void raise(const char* id, Severity severity, const char* message, bool acknowledgeable = true);

// Beendet einen aktiven Alarm und verschiebt ihn in Historie/SD-Log.
void clear(const char* id);

// Schreibt einen reinen Historien-/Infoeintrag, ohne aktiven Alarm zu erzeugen.
void recordInfo(const char* id, const char* message);

bool acknowledge(const char* id);
uint8_t acknowledgeAllNonCritical();
void clearHistory();
void resetAll();

// Zyklische Alarm-Ausgabe, z. B. Summer/Alarm-Ausgang.
void process(AppContext& ctx);

uint8_t activeCount();
uint8_t unacknowledgedCount();
bool hasCriticalActive();
bool hasUnacknowledgedWarning();

const char* severityToKey(Severity severity);
const char* severityToLabel(Severity severity);

// Fuegt ein JSON-Objekt an den uebergebenen String an:
// {"summary":...,"buzzer":...,"active":[...],"history":[...]}
void appendJson(String& json);

}  // namespace Alarms
