

#include "config.h"
#include "app_fsm.h"
#include "feature_ui.h"
#include "math.h"
AppFSM app;

//Debug_only
unsigned long zeit = 0;

void setup() {
  Serial.begin(SERIAL_BAUDRATE);
  delay(1000);
  Serial.println();
  Serial.println("SolarCtrl startet...");
  
  app.begin();
}

void loop() {
  app.update();
  UI::update();
  //Debug_only
  float temp =temperatureRead();
  if((millis() - zeit) >= 10000){
  zeit = millis();
  Serial.print("Interne Temperatur: ");
  Serial.print(temp);
  Serial.println(" °C");
  
  };

}
