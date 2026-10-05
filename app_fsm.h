#pragma once

#include "app_types.h"

class AppFSM {
public:
  AppFSM();

  void begin();
  void loop();
  void update() { loop(); }

  // Phase 1C: explicit runtime boundary. Both functions still execute on the
  // Arduino loop task in Stage 1; later the control side can be pinned to
  // Core 0 without moving service code with it.
  void updateControl();
  void updateServices();

  AppContext& context() { return ctx_; }

private:
  AppContext ctx_;
  bool selfTestSensorCycleStarted_ = false;
  uint32_t lastFastControlAtMs_ = 0;

  void runFastControlScheduler();

  void changeState(SystemState next);
  void ensureSoftAp();
  void startStaConnect();
  void serviceNetwork();

  void stateInitHw();
  void stateInitSd();
  void stateInitStorage();
  void stateLoadConfig();
  void stateInitNetwork();
  void stateInitUi();
  void stateInitSensors();
  void stateSelfTest();
  void stateIdle();
  void stateReadSensors();
  void stateValidateSensors();
  void stateComputeControl();
  void stateApplyOutputs();
  void stateUpdateRuntime();
  void stateFault();
};