#pragma once
#include "app_types.h"

namespace Pumps {

  void begin(AppContext& ctx);
  void process(AppContext& ctx);
  void allOff(AppContext& ctx);
  void safetyAllOff(AppContext& ctx);
  void safetyAllOffExceptOven(AppContext& ctx);
  bool safetyForceRunForSource(AppContext& ctx, HeatSourceRole sourceRole, float pwmPercent);
  bool safetyForceHeatDumpForSource(AppContext& ctx, HeatSourceRole sourceRole, float pwmPercent);
  bool safetyForceNightCooling(AppContext& ctx, float pwmPercent);
  // Critical storage overtemperature: use only configured hydraulic paths whose
  // measured destination/collector is sufficiently cooler than the hot storage.
  // Returns the number of pumps that are actually running for heat dump.
  uint8_t safetyForceStorageCooling(AppContext& ctx, float criticalTemperatureC, float minimumDeltaC, float pwmPercent);

  // Spezialanforderung der Ofenlogik: Die Pumpe wird weiterhin im Pumpenmenue
  // konfiguriert. Das Ofenmodul entscheidet nur, ob Waerme abgefuehrt werden soll.
  // Diese Funktion setzt den passend konfigurierten Pumpenausgang und ggf. den
  // PWM-Kanal gemaess Pumpenmenue.
  bool applyOvenPumpRequest(AppContext& ctx, bool run, float ovenTemperatureC, float targetTemperatureC, bool targetValid);
  int8_t configuredOvenPumpIndex(const AppContext& ctx);

  // Heizkreispumpen werden zentral im Pumpenmenue konfiguriert.
  // Eine Pumpe gehoert zu HKx, wenn Quelle = HKx Vorlauf und Ziel = HKx Ruecklauf ist.
  int8_t configuredHeatingCircuitPumpIndex(const AppContext& ctx, uint8_t circuitIndex);
  bool applyHeatingCircuitPumpRequest(AppContext& ctx, uint8_t circuitIndex, bool run, uint8_t percent, float flowTemperatureC, float returnTemperatureC, bool returnValid);


}