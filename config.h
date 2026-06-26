#pragma once
#include <Arduino.h>

// ======================================================
// Serielle Schnittstelle
// ======================================================
static constexpr uint32_t SERIAL_BAUDRATE = 115200;

// ======================================================
// I2C Bus
// ======================================================
static constexpr uint8_t PIN_I2C_SDA = 21;
static constexpr uint8_t PIN_I2C_SCL = 22;

// ======================================================
// PCF8574
// ======================================================
// 0x20: MAX31865 Chip-Selects
// 0x21: Relais-Ausgänge fuer Pumpen und Zonenventile
static constexpr uint8_t PCF8574_MAX_ADDR   = 0x20;
static constexpr uint8_t PCF8574_RELAY_ADDR = 0x21;

// Rueckwaertskompatibel fuer bestehenden MAX-PCF-Code
static constexpr uint8_t PCF8574_ADDR = PCF8574_MAX_ADDR;

// MAX-CS am PCF 0x20
static constexpr uint8_t PCF_BIT_MAX1_CS = 0;   // P0
static constexpr uint8_t PCF_BIT_MAX2_CS = 1;   // P1
static constexpr uint8_t PCF_BIT_MAX3_CS = 2;   // P2

// ======================================================
// PCA9685 PWM-Treiber
// ======================================================
static constexpr uint8_t  PCA9685_ADDR = 0x40;
static constexpr uint16_t PCA9685_PWM_FREQ = 1000;

// Pumpen 1..5 -> PCA9685 Kanaele 0..6
static constexpr uint8_t PUMP_PWM_CHANNELS[7] = { 0, 1, 2, 3, 4, 5, 6 };

constexpr uint8_t FEEDBACK_INPUT_COUNT = 8;

static constexpr uint8_t FEEDBACK_INPUT_PINS[FEEDBACK_INPUT_COUNT] = {
  33, // FB0
  35, // FB1
  36, // FB2
  39, // FB3
  27, // FB4
  13, // FB5
  16, // FB6
  17  // FB7
};

// ======================================================
// SD-Karte: eigener SPI-Bus, CS direkt am ESP32
// ======================================================
static constexpr uint8_t PIN_SD_SCK  = 18;
static constexpr uint8_t PIN_SD_MISO = 19;
static constexpr uint8_t PIN_SD_MOSI = 23;
static constexpr uint8_t PIN_SD_CS   = 5;

// ======================================================
// MAX31865: gemeinsamer separater SPI-Bus, CS ueber PCF8574
// ======================================================
static constexpr uint8_t PIN_MAX_SCK  = 14;
static constexpr uint8_t PIN_MAX_MISO = 34;
static constexpr uint8_t PIN_MAX_MOSI = 32;

// ======================================================
// DS18B20
// ======================================================
static constexpr uint8_t PIN_ONEWIRE = 4;

// ======================================================
// Servo für Ofenluftklappe
// ======================================================
static constexpr uint8_t OVEN_SERVO_PIN = 25; // freien ESP32-Pin eintragen

// ======================================================
// MAX31865 / PT1000
// ======================================================
static constexpr bool  MAX31865_USE_3WIRE      = false;
static constexpr float MAX31865_RNOMINAL       = 1000.0f;
static constexpr float MAX31865_RREF           = 4700.0f;
static constexpr float HEAT_SOURCE_TEMP_MIN_C  = -40.0f;
static constexpr float HEAT_SOURCE_TEMP_MAX_C  = 400.0f;

// ======================================================
// DS18B20 Plausibilitaet
// ======================================================
static constexpr float SINK_TEMP_MIN_C = -20.0f;
static constexpr float SINK_TEMP_MAX_C = 125.0f;

// ======================================================
// Netzwerk / AP
// ======================================================
static constexpr const char* DEFAULT_AP_SSID     = "SolarCtrl";
static constexpr const char* DEFAULT_AP_PASSWORD = "12345678";

// ======================================================
// Service
// ======================================================
static constexpr const char* DEFAULT_SERVICE_PIN = "1234";

// ======================================================
// Regelungs-Defaults
// ======================================================
static constexpr float   DEFAULT_DIFF_ON         = 3.0f;
static constexpr float   DEFAULT_DIFF_OFF        = 2.0f;
static constexpr float   DEFAULT_PWM_START_DIFF  = 3.0f;
static constexpr uint8_t DEFAULT_PWM_START_PCT   = 30;
static constexpr float   DEFAULT_PWM_FULL_DIFF   = 10.0f;
static constexpr uint8_t DEFAULT_PWM_FULL_PCT    = 100;

// Frostschutz
static constexpr bool    DEFAULT_FROST_ENABLED          = true;
static constexpr float   DEFAULT_FROST_COLLECTOR_ON_C   = 4.0f;
static constexpr float   DEFAULT_FROST_COLLECTOR_OFF_C  = 6.0f;
static constexpr float   DEFAULT_FROST_SINK_MIN_C       = 10.0f;
static constexpr uint8_t DEFAULT_FROST_PUMP_PERCENT     = 40;

// Stagnation
static constexpr bool    DEFAULT_STAGNATION_ENABLED          = true;
static constexpr float   DEFAULT_STAGNATION_COLLECTOR_ON_C   = 110.0f;
static constexpr float   DEFAULT_STAGNATION_COLLECTOR_OFF_C  = 100.0f;
static constexpr float   DEFAULT_SINK_MAX_C                  = 70.0f;
static constexpr uint8_t DEFAULT_STAGNATION_PUMP_PERCENT     = 100;

// ======================================================
// Zeiten
// ======================================================
static constexpr uint32_t DEFAULT_SAMPLE_INTERVAL_MS       = 2000;
static constexpr uint32_t DEFAULT_RUNTIME_SAVE_INTERVAL_MS = 60000;
static constexpr uint32_t FAULT_RETRY_INTERVAL_MS          = 5000;
static constexpr uint32_t SENSOR_CONVERSION_WAIT_MS        = 800;
static constexpr uint32_t UI_SESSION_TIMEOUT_MS            = 300000;

// ======================================================
// Dateien
// ======================================================
static constexpr const char* FILE_CONFIG                  = "/config/system.cfg";
static constexpr const char* FILE_DIAGNOSTICS             = "/runtime/diagnostics.cfg";
static constexpr const char* FILE_MAINTENANCE             = "/runtime/maintenance.cfg";
static constexpr const char* FILE_SENSOR_ASSIGNMENTS      = "/config/sensors.cfg";
static constexpr const char* FILE_HEAT_SOURCE_ASSIGNMENTS = "/config/heat_sources.cfg";
