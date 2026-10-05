

#include "config.h"
#include "app_fsm.h"
#include "feature_ui.h"
#include "feature_runtime_diag.h"
#include "math.h"
AppFSM app;

//Debug_only
unsigned long zeit = 0;

void setup() {
  Serial.begin(SERIAL_BAUDRATE);
  delay(1000);
  Serial.println();
  Serial.println("SolarCtrl startet...");
  
  RuntimeDiag::begin();
  app.begin();
}

void loop() {
  RuntimeDiag::noteLoopStart();

  uint32_t t0 = micros();
  app.updateControl();
  RuntimeDiag::recordControl((uint32_t)(micros() - t0));

  t0 = micros();
  app.updateServices();
  RuntimeDiag::recordServices((uint32_t)(micros() - t0));

  t0 = micros();
  UI::update();
  RuntimeDiag::recordWeb((uint32_t)(micros() - t0));

  RuntimeDiag::printPeriodic();

  //Debug_only
  float temp =temperatureRead();
  if((millis() - zeit) >= 10000){
  zeit = millis();
  Serial.print("Interne Temperatur: ");
  Serial.print(temp);
  Serial.println(" °C");
  
  };

}
