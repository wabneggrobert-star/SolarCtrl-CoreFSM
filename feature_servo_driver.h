#pragma once

#include <Arduino.h>

namespace ServoDriver {

// Initialisiert den Ofenklappen-Servo auf OVEN_SERVO_PIN.
// Mehrfacher Aufruf ist erlaubt.
bool begin();

// Setzt den Servo auf einen Winkel von 0 bis 180 Grad.
// Rueckgabe ist der tatsaechlich gesetzte, begrenzte Winkel.
uint8_t setAngle(uint8_t angle);

// Schaltet den Servo auf die geschlossene Stellung.
void close();

// Liefert den zuletzt gesetzten Winkel.
uint8_t angle();

// Test-/Diagnoseinformation.
bool initialized();

} // namespace ServoDriver
