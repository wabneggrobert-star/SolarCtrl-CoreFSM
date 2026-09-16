#pragma once
#include <stdint.h>
#include "app_types.h"

namespace PwmDriver {
  bool begin();
  bool available();

  // Rueckwaertskompatibel: Standardprofil SOLAR.
  void setDuty(uint8_t channel, uint8_t percent);
  void setDuty(uint8_t channel, uint8_t percent, PwmProfile profile);

  // Schaltausgang ueber PCA: logisch AUS/EIN, elektrisch gemaess Profil umgesetzt.
  void setSwitch(uint8_t channel, bool on, PwmProfile profile);

  uint8_t getDuty(uint8_t channel);

  // Frueher Start-Fallback ohne geladene Konfiguration: alle Kanaele nach
  // SOLAR-Profil logisch AUS. Fuer den normalen Betrieb die profilbewusste
  // Ueberladung mit ConfigData verwenden.
  void allOff();

  // Schaltet jeden konfigurierbaren PO-Kanal logisch AUS und beruecksichtigt
  // dabei sein gespeichertes PWM-Profil (SOLAR/HEATING).
  void allOff(const ConfigData& config);
}
