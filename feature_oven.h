#pragma once

#include "app_types.h"

namespace OvenControl {

void begin(AppContext& ctx);
void process(AppContext& ctx);
void allOff(AppContext& ctx);

void requestStart();
void requestStop(AppContext& ctx);

bool active();
bool userRequestedActive();
bool autoStarted();
bool pumpActive();
uint8_t servoAngle();
float ovenTemperatureC();
float peakTemperatureC();
const char* stateText();

}
