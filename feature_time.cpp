#include "feature_time.h"
#include <WiFi.h>
#include <time.h>
#include <esp_sntp.h>
#include <sys/time.h>

namespace {
bool g_configured = false;
bool g_valid = false;
uint32_t g_lastSyncEpoch = 0;
uint32_t g_lastAttemptMs = 0;
const uint32_t RETRY_MS = 60000UL;

bool clockLooksValid() {
  time_t now = time(nullptr);
  return now > 1704067200; // 2024-01-01
}

void onTimeSync(struct timeval*) {
  g_valid = true;
  g_lastSyncEpoch = (uint32_t)time(nullptr);
}

void configure(const ConfigData& cfg) {
  // Österreich/Deutschland: CET/CEST mit automatischer Sommerzeit.
  const char* tz = cfg.ntpTimezone[0] ? cfg.ntpTimezone : "CET-1CEST,M3.5.0,M10.5.0/3";
  const char* server = cfg.ntpServer[0] ? cfg.ntpServer : "pool.ntp.org";
  sntp_set_time_sync_notification_cb(onTimeSync);
  configTzTime(tz, server);
  g_configured = true;
  g_lastAttemptMs = millis();
}
}

namespace TimeService {
void begin(AppContext& ctx) {
  g_configured = false;
  g_valid = clockLooksValid();
  g_lastSyncEpoch = g_valid ? (uint32_t)time(nullptr) : 0;
  g_lastAttemptMs = 0;
  if (ctx.config.ntpEnabled && WiFi.status() == WL_CONNECTED) configure(ctx.config);
}

void process(AppContext& ctx) {
  if (!ctx.config.ntpEnabled) {
    g_configured = false;
    g_valid = clockLooksValid();
    return;
  }
  if (WiFi.status() != WL_CONNECTED) {
    g_valid = clockLooksValid();
    return;
  }

  const uint32_t nowMs = millis();
  if (!g_configured) configure(ctx.config);

  if (clockLooksValid()) g_valid = true;

  const uint32_t intervalMs = max((uint32_t)3600000UL, (uint32_t)ctx.config.ntpSyncIntervalMs);
  const uint32_t nowEpoch = clockLooksValid() ? (uint32_t)time(nullptr) : 0;
  const bool due = g_lastSyncEpoch == 0 || (nowEpoch > g_lastSyncEpoch && (nowEpoch - g_lastSyncEpoch) >= (intervalMs / 1000UL));
  if (due && (g_lastAttemptMs == 0 || (uint32_t)(nowMs - g_lastAttemptMs) >= RETRY_MS)) {
    // configTzTime ist asynchron; onTimeSync() bestaetigt den erfolgreichen Abgleich.
    configure(ctx.config);
  }
}

bool valid() { return g_valid && clockLooksValid(); }
time_t nowEpoch() { return valid() ? time(nullptr) : 0; }

String isoTimestamp() {
  if (!valid()) return "";
  struct tm t;
  time_t now = time(nullptr);
  localtime_r(&now, &t);
  char buf[32];
  strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S%z", &t);
  return String(buf);
}

String dateText() {
  if (!valid()) return "--.--.----";
  struct tm t; time_t now = time(nullptr); localtime_r(&now, &t);
  char buf[16]; strftime(buf, sizeof(buf), "%d.%m.%Y", &t); return String(buf);
}

String timeText() {
  if (!valid()) return "--:--";
  struct tm t; time_t now = time(nullptr); localtime_r(&now, &t);
  char buf[8]; strftime(buf, sizeof(buf), "%H:%M", &t); return String(buf);
}

uint32_t lastSyncEpoch() { return g_lastSyncEpoch; }
const char* statusText() {
  if (valid()) return "synchronisiert";
  if (g_configured) return "warte auf NTP";
  return "nicht synchronisiert";
}
void forceSync() { g_configured = false; g_lastAttemptMs = 0; g_lastSyncEpoch = 0; }

bool setManualLocalTime(AppContext& ctx, int year, int month, int day, int hour, int minute, int second) {
  if (year < 2024 || year > 2099 || month < 1 || month > 12 || day < 1 || day > 31 ||
      hour < 0 || hour > 23 || minute < 0 || minute > 59 || second < 0 || second > 59) {
    return false;
  }

  const char* tz = ctx.config.ntpTimezone[0] ? ctx.config.ntpTimezone : "CET-1CEST,M3.5.0,M10.5.0/3";
  setenv("TZ", tz, 1);
  tzset();

  struct tm localTm = {};
  localTm.tm_year = year - 1900;
  localTm.tm_mon = month - 1;
  localTm.tm_mday = day;
  localTm.tm_hour = hour;
  localTm.tm_min = minute;
  localTm.tm_sec = second;
  localTm.tm_isdst = -1;

  time_t epoch = mktime(&localTm);
  if (epoch <= 1704067200) return false;
  // mktime normalisiert ungueltige Daten (z.B. 31.02.). Solche Eingaben ablehnen.
  if (localTm.tm_year != year - 1900 || localTm.tm_mon != month - 1 || localTm.tm_mday != day ||
      localTm.tm_hour != hour || localTm.tm_min != minute) return false;

  struct timeval tv = {};
  tv.tv_sec = epoch;
  tv.tv_usec = 0;
  if (settimeofday(&tv, nullptr) != 0) return false;

  g_valid = true;
  // Manuelles Setzen ist keine NTP-Synchronisation. lastSyncEpoch bleibt daher unveraendert.
  return true;
}

bool localMinutesOfDay(uint16_t& minutesOut) {
  if (!valid()) return false;
  time_t now = time(nullptr);
  struct tm t;
  localtime_r(&now, &t);
  minutesOut = (uint16_t)(t.tm_hour * 60 + t.tm_min);
  return true;
}
}
