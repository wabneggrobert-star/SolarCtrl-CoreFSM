#pragma once
#include "app_types.h"
#include <Arduino.h>
#include <WebServer.h>

namespace History {
  void begin(AppContext& ctx);
  void process(AppContext& ctx);
  bool enabled();
  void setEnabled(bool enabled);
  bool displayEnabled(const char* sensorId);
  void setDisplayEnabled(const char* sensorId, bool enabled);
  bool savePreferences();
  String configJson(const AppContext& ctx);
  void handleHistoryRequest(AppContext& ctx, WebServer& server);
}
