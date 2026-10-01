#pragma once
#include "app_types.h"

namespace AuxHeater {
void begin(AppContext& ctx);
void process(AppContext& ctx);
void allOff(AppContext& ctx);

// Nach einem externen Hardware-Override (z. B. Inbetriebnahme-Testmodus)
// nur den Ausgangs-Cache verwerfen. Der laufende Heizzyklus bleibt erhalten;
// beim naechsten process() werden Pumpe und Heizstufen garantiert neu geschrieben.
void resumeAfterTestMode();
bool heatingActive();
bool cooldownActive();
bool pumpActive();
uint8_t activeStageCount();
const char* stateText();
}
