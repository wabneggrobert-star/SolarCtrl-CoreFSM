#pragma once

#include "app_types.h"
#include <Arduino.h>

namespace MqttBridge {

void begin(AppContext& ctx);
void process(AppContext& ctx);
void forceReconnect();
void publishSnapshot(AppContext& ctx);

bool connected();
bool connecting();
const char* lastError();
uint32_t lastConnectAttemptMs();
uint32_t lastConnectedMs();

}  // namespace MqttBridge
