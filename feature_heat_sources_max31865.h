#pragma once
#include "app_types.h"

namespace HeatSourcesMax {

  bool begin(AppContext& ctx);

  void startCycle(AppContext& ctx);
  void process(AppContext& ctx);
  bool cycleComplete(const AppContext& ctx);

  // Blocking one-shot read. This is the single authoritative MAX31865
  // read routine used by the normal runtime path and the debug endpoint.
  bool readChannelNow(AppContext& ctx, MaxChannel ch);
  void readAllNow(AppContext& ctx);

  // Blocking debug read for browser endpoint /api/max-debug.
  // Does not change application configuration; it only toggles MAX31865 CS lines
  // and optionally triggers one-shot conversions for diagnosis.
  String debugJson(AppContext& ctx, bool runOneShot);

  // Detailed MAX31865 fault trace. Executes the same fault-clear / bias /
  // one-shot sequence as the normal reader and records CONFIG, RTD, FAULT and
  // threshold registers after each stage. It never writes threshold registers.
  String faultTraceJson(AppContext& ctx);

  // Targeted sequence diagnostic: compares ADC3 bias-settling times, forces
  // ADC4 through an explicit idle/reset/one-shot sequence, and alternates
  // ADC3/ADC4 register reads to expose possible CS cross-talk. CONFIG is
  // restored afterwards; threshold registers are never written.
  String sequenceDiagnosticJson(AppContext& ctx);

}