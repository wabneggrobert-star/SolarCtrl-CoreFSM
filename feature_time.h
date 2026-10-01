#pragma once
#include "app_types.h"
#include <Arduino.h>

namespace TimeService {
void begin(AppContext& ctx);
void process(AppContext& ctx);
bool valid();
time_t nowEpoch();
String isoTimestamp();
String dateText();
String timeText();
uint32_t lastSyncEpoch();
const char* statusText();
void forceSync();
bool setManualLocalTime(AppContext& ctx, int year, int month, int day, int hour, int minute, int second = 0);
bool localMinutesOfDay(uint16_t& minutesOut);
}
