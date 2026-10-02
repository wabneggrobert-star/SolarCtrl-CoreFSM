#pragma once
#include "app_types.h"
#include "feature_build_flags.h"

namespace HeatSourcesMax {

  bool begin(AppContext& ctx);

  void startCycle(AppContext& ctx);
  void process(AppContext& ctx);
  bool cycleComplete(const AppContext& ctx);

  // Blocking helpers are retained only for explicit diagnostics/service use.
  // The normal controller path uses startCycle()/process() and never waits for
  // MAX31865 bias or conversion timing inside one loop pass.
  bool readChannelNow(AppContext& ctx, MaxChannel ch);
  void readAllNow(AppContext& ctx);

#if SOLARCTRL_MAX_DIAGNOSTICS
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
#endif

}