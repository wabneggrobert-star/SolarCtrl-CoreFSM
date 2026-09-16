#pragma once

#include "app_types.h"

namespace OvenControl {

void begin(AppContext& ctx);
void process(AppContext& ctx);
void allOff(AppContext& ctx);

// Faellt nur den Servo auf die konfigurierte Standby-/Safety-Oeffnung zurueck,
// ohne den logischen Ofenzustand oder den Abbrandzyklus zu veraendern.
void applyStandbyServoPosition(const AppContext& ctx);

void requestStart();
void requestStop(AppContext& ctx);

bool active();
bool userRequestedActive();
bool autoStarted();
bool pumpActive();
uint8_t servoAngle();
uint8_t servoOpeningPercent();
bool targetTemperatureReached();
float ovenTemperatureC();
float peakTemperatureC();
const char* stateText();

}
