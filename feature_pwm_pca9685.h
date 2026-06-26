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
  void allOff();
}
