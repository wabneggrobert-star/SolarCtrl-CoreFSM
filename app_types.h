#pragma once
#include <Arduino.h>

static constexpr uint8_t MAX_DS18B20_SENSORS = 20;
static constexpr uint8_t MAX_HEAT_SOURCE_ASSIGNMENTS = 4;
static constexpr uint8_t MAX_MAX31865_CHANNELS = 4;
static constexpr uint8_t PIN_UNUSED = 255;
static constexpr uint8_t RELAY_COUNT = 8;
static constexpr uint8_t PWM_OUTPUT_COUNT = 8;
static constexpr uint8_t MAX_PUMPS = 5;
static constexpr uint8_t MAX_HEATING_CIRCUITS = 4;
constexpr uint8_t HEATING_CIRCUIT_COUNT = 4;
static constexpr uint8_t PUMP_ROUTE_TARGET_COUNT = 2;



// ===================== FSM =====================
enum class SystemState : uint8_t {
  BOOT = 0,
  INIT_HW,
  INIT_SD,
  INIT_STORAGE,
  LOAD_CONFIG,
  INIT_NETWORK,
  INIT_UI,
  INIT_SENSORS,
  SELF_TEST,
  IDLE,
  READ_SENSORS,
  VALIDATE_SENSORS,
  COMPUTE_CONTROL,
  APPLY_OUTPUTS,
  UPDATE_RUNTIME,
  FAULT
};

// ===================== Schutz / Regelung =====================
enum class ProtectionMode : uint8_t {
  NONE = 0,
  FROST,
  STAGNATION
};


enum class SolarFluidType : uint8_t {
  GLYCOL = 0,
  WATER = 1
};

enum class GlycolType : uint8_t {
  PROPYLENE = 0,
  ETHYLENE = 1
};

enum class MlMode : uint8_t {
  OFF = 0,
  LEARN_ONLY = 1,
  AUTOMATIC = 2
};

enum class HeatingEmitterType : uint8_t {
  UNKNOWN = 0,
  UNDERFLOOR = 1,
  RADIATOR = 2,
  OTHER = 3
};

enum class SolarHydraulicType : uint8_t {
  CLOSED_PRESSURIZED = 0,
  DRAINBACK = 1
};

enum class SolarCollectorType : uint8_t {
  FLAT_PLATE = 0,
  EVACUATED_TUBE = 1
};

enum class ControlMode : uint8_t {
  NONE = 0,
  SOLAR_DIFF,
  OVEN_TRANSFER,
  ALT_SOURCE_TRANSFER
};

// ===================== DS18B20 Rollen =====================
enum class Ds18Role : uint8_t {
  NONE = 0,

  SINK_BOILER_TOP,
  SINK_BUFFER_TOP,

  BOILER_BOTTOM,
  BUFFER_HIGH,
  BUFFER_MID,
  BUFFER_BOTTOM,

  FLOW_COLLECTOR_1,
  FLOW_COLLECTOR_2,
  FLOW_COLLECTOR_3,
  FLOW_ALT_SOURCE,

  RETURN_COLLECTOR_1,
  RETURN_COLLECTOR_2,
  RETURN_COLLECTOR_3,
  RETURN_ALT_SOURCE,

  CIRCULATION,
  SWIMMINGPOOL,

  RESERVE_1,
  RESERVE_2,
  RESERVE_3,
  RESERVE_4,

  // Neue allgemeine Sensorrollen fuer Heizkreise / Wetter / Raum.
  OUTSIDE_TEMPERATURE,

  ROOM_1,
  ROOM_2,
  ROOM_3,
  ROOM_4,

  HK1_FLOW,
  HK2_FLOW,
  HK3_FLOW,
  HK4_FLOW,

  HK1_RETURN,
  HK2_RETURN,
  HK3_RETURN,
  HK4_RETURN
};

enum class SinkTarget : uint8_t {
  BOILER_TOP = 0,
  BUFFER_TOP = 1
};

// ===================== MAX31865 / Wärmequellen =====================
enum class MaxChannel : uint8_t {
  CH1 = 0,
  CH2,
  CH3,
  CH4
};

enum class HeatSourceRole : uint8_t {
  NONE = 0,

  SOLAR_COLLECTOR_1,
  SOLAR_COLLECTOR_2,
  SOLAR_COLLECTOR_3,

  ALT_SOURCE_OVEN,
  ALT_SOURCE_OTHER,

  // Sensorrollen fuer ADC/MAX31865. Solar/Ofen bleiben echte Waermequellen,
  // die folgenden Rollen duerfen ADC1..ADC4 ebenfalls verwenden.
  SENSOR_BOILER_TOP,
  SENSOR_BOILER_BOTTOM,
  SENSOR_BUFFER_TOP,
  SENSOR_BUFFER_MIDDLE,
  SENSOR_BUFFER_BOTTOM,
  SENSOR_OUTSIDE_TEMPERATURE,
  SENSOR_ROOM_1,
  SENSOR_ROOM_2,
  SENSOR_ROOM_3,
  SENSOR_ROOM_4,
  SENSOR_HK1_FLOW,
  SENSOR_HK2_FLOW,
  SENSOR_HK3_FLOW,
  SENSOR_HK4_FLOW,
  SENSOR_HK1_RETURN,
  SENSOR_HK2_RETURN,
  SENSOR_HK3_RETURN,
  SENSOR_HK4_RETURN
};

enum class HeatSourceKind : uint8_t {
  NONE = 0,
  SOLAR,
  OVEN,
  OTHER
};

enum class MaxReadStep : uint8_t {
  IDLE = 0,
  CLEAR_FAULT,
  BIAS_ON,
  WAIT_BIAS,
  START_1SHOT,
  WAIT_CONVERSION,
  READ_RTD,
  READ_FAULT,
  FINALIZE
};

struct CommissioningState {
  bool active = false;
};

struct MaxChannelRuntime {
  MaxReadStep step = MaxReadStep::IDLE;
  uint32_t tMarkUs = 0;
  bool cycleDone = false;
  uint16_t pendingRawRtd = 0;
  uint8_t pendingFault = 0;
};

// ===================== DS18B20 Inventar / FSM-Snapshot =====================
enum class Ds18Error : uint8_t {
  NONE = 0,
  NOT_READ,
  DISCONNECTED,
  OUT_OF_RANGE
};

struct Ds18b20DeviceInfo {
  bool present = false;
  uint8_t address[8] = {0};
  char addressText[24] = "";
  float lastTempC = NAN;
  bool lastValid = false;
  uint32_t lastUpdateMs = 0;
  uint32_t sequence = 0;
  Ds18Error error = Ds18Error::NOT_READ;
  Ds18Role role = Ds18Role::NONE;
};

struct Ds18b20Inventory {
  uint8_t count = 0;
  Ds18b20DeviceInfo devices[MAX_DS18B20_SENSORS];
};

struct Ds18RoleAssignment {
  Ds18Role role = Ds18Role::NONE;
  bool assigned = false;
  uint8_t address[8] = {0};
  char addressText[24] = "";
};

struct SensorAssignmentTable {
  uint8_t count = 0;
  Ds18RoleAssignment items[MAX_DS18B20_SENSORS];
};

// ===================== MAX Konfiguration / Messwerte =====================
struct MaxChannelConfig {
  bool enabled = false;
  float offsetC = 0.0f;
  float calFactor = 1.0f;
};

struct MaxChannelReading {
  bool present = false;
  bool valid = false;
  float tempC = NAN;
  float rawTempC = NAN;
  float resistanceOhm = NAN;
  uint16_t rawRtd = 0;        // Rohregister inkl. Fault-Bit im LSB
  uint16_t rtdCode = 0;       // 15-bit RTD-Code: rawRtd >> 1
  uint8_t fault = 0;
  bool rawFaultBit = false;
  uint32_t lastUpdateMs = 0;  // Zeitpunkt des letzten abgeschlossenen Messwerts
  uint32_t sequence = 0;      // steigt bei jedem abgeschlossenen Messwert
  char status[40] = "not_read";
};

struct HeatSourceAssignment {
  bool assigned = false;
  MaxChannel channel = MaxChannel::CH1;
  HeatSourceRole role = HeatSourceRole::NONE;
};

struct HeatSourceAssignmentTable {
  uint8_t count = 0;
  HeatSourceAssignment items[MAX_HEAT_SOURCE_ASSIGNMENTS];
};

struct HeatSourceSnapshot {
  float solarCollector1C = NAN;
  bool solarCollector1Valid = false;

  float solarCollector2C = NAN;
  bool solarCollector2Valid = false;

  float solarCollector3C = NAN;
  bool solarCollector3Valid = false;

  float altSourceOvenC = NAN;
  bool altSourceOvenValid = false;

  float altSourceOtherC = NAN;
  bool altSourceOtherValid = false;
};

struct ActiveHeatSource {
  HeatSourceRole role = HeatSourceRole::NONE;
  HeatSourceKind kind = HeatSourceKind::NONE;
  ControlMode controlMode = ControlMode::NONE;

  float tempC = NAN;
  bool valid = false;
};

// ===================== Sensorwerte =====================
struct SensorSnapshot {
  float collectorC = NAN;
  bool collectorValid = false;

  float sinkC = NAN;
  bool sinkValid = false;

  HeatSourceSnapshot heatSources;
  ActiveHeatSource activeHeatSource;
};

// ===================== Regelung =====================
struct ControlData {
  bool relayEnable = false;
  uint8_t pwmPercent = 0;
  float diffC = NAN;
  ProtectionMode protectionMode = ProtectionMode::NONE;
};

// ===================== Pumpen =====================
enum class PumpMode : uint8_t {
  OFF = 0,
  RELAY,
  PWM
};

// Elektrisches Profil des PCA9685-Ausgangs.
// SOLAR und HEATING sind zueinander invertiert, damit unterschiedliche Platinenprofile
// mit gleicher logischer Pumpenanforderung 0..100 % betrieben werden koennen.
enum class PwmProfile : uint8_t {
  SOLAR = 0,
  HEATING = 1
};

// Quelle einer Pumpe:
// HEAT_SOURCE_ROLE = MAX31865/Wärmequelle
// SENSOR_ROLE      = aktive DS18B20-Rolle, z. B. Puffer oben für Speicher-zu-Speicher-Ladung
enum class PumpSourceType : uint8_t {
  HEAT_SOURCE_ROLE = 0,
  SENSOR_ROLE = 1
};

enum class RelayFunction : uint8_t {
  NONE = 0,
  PUMP_ENABLE,
  ZONE_VALVE,
  HEATER_ROD,
  MIXER,
  ALARM_BUZZER
};

struct RelayOutputConfig {
  bool enabled = false;
  RelayFunction function = RelayFunction::NONE;
  bool activeLow = true;
};

enum class PwmOutputMode : uint8_t {
  PWM = 0,
  SWITCH = 1
};

struct PwmOutputConfig {
  bool enabled = true;
  PwmOutputMode mode = PwmOutputMode::PWM;
  RelayFunction function = RelayFunction::NONE;
  PwmProfile profile = PwmProfile::SOLAR;
};

struct RelayOutputRuntime {
  bool state = false;
};

struct PumpRouteTargetConfig {
  bool enabled = false;
  Ds18Role sinkRole = Ds18Role::NONE;

  // 0.0 bedeutet: Wert der Pumpe verwenden
  float targetDiffOverride = 0.0f;
  float hysteresisOverride = 0.0f;

  // 0.0 bedeutet: keine Mindesttemperatur-Anforderung.
  // Wenn gesetzt, wird ein neues Ziel erst aktiviert, wenn der Ziel-Sensor unter minTempC liegt.
  float minTempC = 0.0f;

  // 0.0 bedeutet: keine Maximaltemperaturbegrenzung.
  // Wenn der Ziel-Sensor >= maxTempC ist, wird dieses Ziel gesperrt.
  float maxTempC = 0.0f;

  // Runtime / Diagnose
  bool active = false;
  float lastSinkC = NAN;
  float lastDiffC = NAN;
};

struct PumpConfig {
  bool enabled = false;
  PumpMode mode = PumpMode::OFF;

  // Regelparameter
  float targetDiff = 5.0f;
  float hysteresis = 1.0f;
  float startDiff = 3.0f;

  // PID fuer PWM-Pumpen
  float pidKp = 10.0f;
  float pidKi = 0.2f;
  float pidKd = 0.0f;

  // ML V2 Block 3: harte Benutzergrenzen fuer lernbare Solar-Betriebsparameter.
  // 0.0 / ungueltiges Min/Max-Paar bedeutet: konservative automatische Bounds
  // relativ zum jeweiligen Basiswert verwenden.
  float mlStartDiffMinC = 0.0f;
  float mlStartDiffMaxC = 0.0f;
  float mlTargetDiffMinC = 0.0f;
  float mlTargetDiffMaxC = 0.0f;
  float mlHysteresisMinC = 0.0f;
  float mlHysteresisMaxC = 0.0f;

  // Zuordnung
  // sourceRole: Waermequelle, deren Temperatur fuer diese Pumpe verwendet wird.
  // sinkRole: Ziel-/Abnehmer-Messstelle. NONE bedeutet: aktuell priorisierter Sink aus Config.
  // relayIndex: Relais am Relais-PCF, das als PUMP_ENABLE konfiguriert sein muss.
  // pwmChannel: PCA9685-Kanal fuer die Drehzahlvorgabe.
  // feedbackPin: ESP32 GPIO fuer PWM-Pumpenfeedback. PIN_UNUSED bedeutet: kein Feedback.
  PumpSourceType sourceType = PumpSourceType::HEAT_SOURCE_ROLE;
  HeatSourceRole sourceRole = HeatSourceRole::NONE;
  Ds18Role sourceSensorRole = Ds18Role::NONE;
  Ds18Role sinkRole = Ds18Role::NONE;
  uint8_t relayIndex = PIN_UNUSED;
  uint8_t pwmChannel = PIN_UNUSED;
  PwmProfile pwmProfile = PwmProfile::SOLAR;
  uint8_t feedbackPin = PIN_UNUSED;

  // Valve V2: optionales Umschaltventil aus ConfigData.valves[].
  // PIN_UNUSED = kein Ventil. Die Ventil-Hardware wird nur noch in ValveConfig.output definiert.
  uint8_t valveIndex = PIN_UNUSED;

  // Valve V2 ersetzt die alte switchValve-Relaislogik.
  // Ziel A/B werden ueber targets[] konfiguriert; die Hardwarezuordnung liegt
  // ausschliesslich in ConfigData.valves[pump.valveIndex].

  PumpRouteTargetConfig targets[PUMP_ROUTE_TARGET_COUNT];
  uint8_t activeTargetIndex = PIN_UNUSED;
  uint8_t valvePendingTargetIndex = PIN_UNUSED;

  // Begrenzung fuer Profil C / Solar-PWM
  float minPwmPercent = 10.0f;
  float maxPwmPercent = 100.0f;

  // Runtime / Diagnose
  bool state = false;
  float lastSourceC = NAN;
  float lastSinkC = NAN;
  float lastDiffC = NAN;
  float lastPwmPercent = 0.0f;

  // PWM-Feedback Diagnose
  bool feedbackSignalPresent = false;
  bool feedbackError = false;
  float feedbackDutyPercent = NAN;
  uint32_t feedbackLastCheckedMs = 0;
};

struct EnergyRouteReservation {
  bool active = false;

  uint8_t pumpIndex = PIN_UNUSED;

  PumpSourceType sourceType = PumpSourceType::HEAT_SOURCE_ROLE;
  HeatSourceRole sourceRole = HeatSourceRole::NONE;
  Ds18Role sourceSensorRole = Ds18Role::NONE;
  Ds18Role sinkRole = Ds18Role::NONE;

  uint8_t pumpRelayIndex = PIN_UNUSED;

  // Valve V2: echtes Ventil-Output-Target. Kann RELAY oder PWM_OUTPUT sein.
  // Als Rohwerte gespeichert, weil OutputRef erst im Heizkreis-/Ventil-Block definiert wird.
  uint8_t valveOutputKind = 0;
  uint8_t valveOutputIndex = PIN_UNUSED;

  // Legacy-/Diagnosefeld fuer bestehende Anzeigen/Logs bei Relaisventilen.
  uint8_t valveRelayIndex = PIN_UNUSED;
};
enum class OutputKind : uint8_t {
  NONE = 0,
  RELAY = 1,
  PWM_OUTPUT = 2
};

struct OutputRef {
  OutputKind kind = OutputKind::NONE;
  uint8_t index = PIN_UNUSED;

  OutputRef() = default;
  OutputRef(OutputKind outputKind, uint8_t outputIndex)
    : kind(outputKind), index(outputIndex) {}
};

//==========Heizstab/Elektrokessel========
struct AuxHeaterConfig {
  bool enabled = false;

  // Manuelle Betriebsfreigabe aus der Anlagenuebersicht. Diese Freigabe ist
  // bewusst getrennt von "enabled": enabled beschreibt, ob die Zusatzheizung
  // konfiguriert ist; userReleaseEnabled darf den normalen Heizbetrieb sperren,
  // aber niemals Safety-Funktionen beeinflussen oder umgehen.
  bool userReleaseEnabled = true;

  float minimumTemperatureC = 45.0f;
  float targetTemperatureC = 55.0f;
  float hysteresisC = 2.0f;

  Ds18Role sinkRole = Ds18Role::NONE;

  OutputRef pumpOutput;

  OutputRef heaterOutput1;
  OutputRef heaterOutput2;
  OutputRef heaterOutput3;

  uint32_t preRunMs = 300000UL;
  uint32_t cooldownMs = 300000UL;
};

// ===================== Ofenkonfiguration =====================
struct OvenConfig {
  bool enabled = false;

  Ds18Role targetSinkRole = Ds18Role::NONE;

  // Standby / Startlogik
  bool autoStartEnabled = true;
  float autoStartTemperatureC = 45.0f;
  float autoStartRiseC = 5.0f;
  float autoReturnToStandbyTemperatureC = 35.0f;

  float targetOvenTemperatureC = 75.0f;
  float criticalOvenTemperatureC = 90.0f;

  float pumpOnTemperatureC = 60.0f;
  float pumpHysteresisC = 5.0f;

  float pumpOnTemperatureDifferenceC = 8.0f;
  float pumpOffTemperatureDifferenceC = 2.0f;
  float pumpStopDropFromPeakC = 5.0f;

  // Luftklappen-Kalibrierung: Die Ofenlogik arbeitet in 0..100 % Oeffnung.
  // Erst ganz am Ende wird daraus der echte Servo-Winkel berechnet.
  uint8_t servoClosedAngle = 45;                 // Winkel bei 0 % Oeffnung
  uint8_t servoOpenAngle = 135;                  // Winkel bei 100 % Oeffnung
  uint8_t servoStandbyOpeningPercent = 0;        // Standby/Stop/Safety
  uint8_t servoStartOpeningPercent = 100;        // Anheizen bis Zieltemperatur
  uint8_t servoMinimumOpeningPercent = 20;       // kleinste Oeffnung im Regelbetrieb/Restglut
  uint8_t servoMaximumOpeningPercent = 100;      // groesste Oeffnung im Regelbetrieb
  uint8_t servoStepPercent = 5;                  // Schrittweite je Regelintervall
  float servoDeadbandC = 3.0f;                   // Totband um Zieltemperatur ±K

  // Ein einziger Zeitwert fuer Peak-Timeout und Abbrand-Nachlauf.
  // Ablauf: X Minuten normale Servo-Regelung, danach X/2 Minuten minimale
  // Regeloeffnung, danach Standby-/Safety-Oeffnung.
  uint16_t burnoutVentMinutes = 10;

  float pidKp = 2.0f;
  float pidKi = 0.02f;
  float pidKd = 0.0f;
};


// ===================== Heizkreise / Mischer =====================
enum class ValvePosition : uint8_t {
  A = 0,
  B = 1
};


struct ValveConfig {
  bool enabled = false;
  OutputRef output;
  uint32_t travelTimeMs = 30000;
  bool activeHighForB = true;
  ValvePosition safetyPosition = ValvePosition::A;
  ValvePosition lastRequestedPosition = ValvePosition::A;
};

enum class HeatingCircuitMixerType : uint8_t {
  THREE_POINT = 0,
  THERMAL = 1
};

enum class HeatingCircuitControlMode : uint8_t {
  FIXED_FLOW = 0,
  WEATHER_COMPENSATED = 1
};

struct HeatingCircuitConfig {
  bool enabled = false;

  HeatingCircuitMixerType mixerType = HeatingCircuitMixerType::THREE_POINT;
  HeatingCircuitControlMode controlMode = HeatingCircuitControlMode::FIXED_FLOW;

  OutputRef mixerOpenOutput;
  OutputRef mixerCloseOutput;

  Ds18Role flowSensorRole = Ds18Role::NONE;
  Ds18Role returnSensorRole = Ds18Role::NONE;
  Ds18Role roomSensorRole = Ds18Role::NONE;
  Ds18Role bufferReferenceRole = Ds18Role::NONE;
  Ds18Role outsideSensorRole = Ds18Role::NONE;

  float fixedFlowTemperatureC = 35.0f;
  float maximumFlowTemperatureC = 55.0f;
  float minimumFlowTemperatureC = 20.0f;

  bool roomControlEnabled = false;
  float roomTargetTemperatureC = 21.0f;
  float roomInfluenceK = 3.0f;

  // Optionale Nachtabsenkung. Sie veraendert nur den wirksamen Raum-Sollwert;
  // Safety und die Grundregelung bleiben davon unabhaengig.
  bool nightSetbackEnabled = false;
  uint16_t nightSetbackStartMinute = 22 * 60;
  uint16_t nightSetbackEndMinute = 6 * 60;
  float nightSetbackK = 2.0f;

  // PWM-Heizkreispumpe: Regelung nach Spreizung Vorlauf/Ruecklauf.
  // Die Hardware/PWM-Ausgaenge bleiben zentral im Pumpenmenue.
  float pumpTargetDeltaC = 5.0f;
  float pumpFullDeltaC = 12.0f;

  // einfache Heizkurve: Soll-Vorlauf = base + slope * (20 - Aussentemperatur)
  float heatingCurveBaseC = 25.0f;
  float heatingCurveSlope = 1.0f;

  bool frostProtectionEnabled = true;
  float frostStartTemperatureC = 3.0f;
  float frostTargetFlowTemperatureC = 12.0f;

  uint32_t mixerFullTravelMs = 120000UL;
  uint32_t mixerPulseMs = 1500UL;
  uint32_t mixerPauseMs = 8000UL;
};

struct HeatingCircuitRuntime {
  bool active = false;
  bool pumpActive = false;
  bool opening = false;
  bool closing = false;
  float flowTemperatureC = NAN;
  float returnTemperatureC = NAN;
  float spreadTemperatureC = NAN;
  float roomTemperatureC = NAN;
  float outsideTemperatureC = NAN;
  float targetFlowTemperatureC = NAN;
  float effectiveRoomTargetTemperatureC = NAN;
  bool nightSetbackActive = false;
  uint8_t pumpPercent = 0;
  int16_t estimatedMixerPositionPercent = 50;
  uint32_t lastMixerActionMs = 0;
};


static constexpr uint8_t MAX_VALVES = 10;

// ===================== Energiezaehler =====================
struct EnergyMeterConfig {
  bool enabled = false;
  uint8_t feedbackInputIndex = PIN_UNUSED;
  float pulsesPerLiter = 450.0f;
  Ds18Role flowSensorRole = Ds18Role::NONE;
  Ds18Role returnSensorRole = Ds18Role::NONE;
  float energyFactorWhPerLiterK = 1.00f;
  uint32_t logIntervalMs = 60000UL;
};

struct EnergyMeterRuntime {
  bool feedbackAttached = false;
  bool temperatureValid = false;
  uint32_t totalPulses = 0;
  double totalVolumeLiters = 0.0;
  double totalEnergyKWh = 0.0;
  float flowLitersPerMinute = 0.0f;
  float flowTemperatureC = NAN;
  float returnTemperatureC = NAN;
  float deltaTemperatureK = NAN;
  float thermalPowerKw = 0.0f;
  uint32_t lastProcessMs = 0;
  uint32_t lastTemperatureSampleMs = 0;
  uint32_t lastLogMs = 0;
};

// ===================== Konfiguration =====================
struct ConfigData {
  SinkTarget activeSinkTarget = SinkTarget::BOILER_TOP;


  uint32_t sampleIntervalMs = 2000;
  uint32_t runtimeSaveIntervalMs = 60000;

  char apName[32] = "SolarCtrl";
  char apPassword[32] = "12345678";

  bool staEnabled = false;
  char staSsid[32] = "";
  char staPassword[64] = "";
  char hostName[32] = "solarctrl";

  // Internet-Zeit / Forecast sind reine Komfortfunktionen.
  // Die Grundregelung darf niemals von Internet, NTP oder Forecast abhaengen.
  bool ntpEnabled = true;
  char ntpServer[64] = "pool.ntp.org";
  char ntpTimezone[64] = "CET-1CEST,M3.5.0,M10.5.0/3";
  uint32_t ntpSyncIntervalMs = 86400000UL;

  bool forecastEnabled = false;
  float forecastLatitude = 46.7536f;
  float forecastLongitude = 15.3697f;
  uint32_t forecastIntervalMs = 1800000UL;
  bool forecastAuxDelayEnabled = false;
  uint32_t forecastAuxMaxWaitMs = 10800000UL;
  bool forecastSolarPriorityEnabled = false;

  // MQTT / Home Assistant. Netzwerk- und Zugangsdaten bleiben nur lokal konfigurierbar.
  bool mqttEnabled = false;
  char mqttHost[64] = "";
  uint16_t mqttPort = 1883;
  char mqttUser[48] = "";
  char mqttPassword[64] = "";
  char mqttClientId[32] = "SolarCtrl";
  char mqttBaseTopic[64] = "haus/solarctrl";
  bool mqttDiscoveryEnabled = true;
  uint32_t mqttPublishIntervalMs = 10000UL;

  char servicePin[16] = "1234";

  // Physikalische Anlagendaten. Sie beschreiben die Anlage, blockieren aber
  // weder Grundregelung noch Safety wenn Werte fehlen oder unplausibel sind.
  SolarFluidType solarFluidType = SolarFluidType::GLYCOL;
  GlycolType glycolType = GlycolType::PROPYLENE;
  float glycolMeasuredFreezeProtectionC = -25.0f;
  SolarHydraulicType solarHydraulicType = SolarHydraulicType::CLOSED_PRESSURIZED;
  SolarCollectorType solarCollectorType = SolarCollectorType::FLAT_PLATE;
  float collectorApertureM2 = 1.0f;
  float collectorTiltDeg = 25.0f;
  float collectorAzimuthDeg = 235.0f;
  float bufferVolumeLiters = 0.0f;
  float boilerVolumeLiters = 0.0f;
  float heatingCircuitAreaM2[MAX_HEATING_CIRCUITS] = {};
  HeatingEmitterType heatingCircuitEmitterType[MAX_HEATING_CIRCUITS] = {};

  // ML ist ein optionaler Optimierungs-Layer. OFF/NUR LERNEN/AUTOMATISCH.
  MlMode mlMode = MlMode::AUTOMATIC;

  bool frostEnabled = true;
  float frostCollectorOnC = 4.0f;
  float frostCollectorOffC = 6.0f;
  float frostSinkMinC = 10.0f;
  uint8_t frostPumpPercent = 40;
  Ds18Role frostProtectionStorageRole = Ds18Role::SINK_BUFFER_TOP;
  float frostSafeCollectorTemperatureC = 8.0f;
  float frostRequiredProtectionC = -20.0f;
  float frostProtectionReserveK = 5.0f;

  bool stagnationEnabled = true;
  float stagnationCollectorOnC = 110.0f;
  float stagnationCollectorOffC = 100.0f;
  float sinkMaxC = 70.0f;
  uint8_t stagnationPumpPercent = 100;

  bool storageProtectionEnabled = true;
  float storageCriticalTemperatureC = 90.0f;
  bool safetyNightCoolingEnabled = false;
  float safetyNightCoolingTargetTemperatureC = 75.0f;

  bool ovenProtectionEnabled = true;
  float ovenOvertemperatureOnC = 85.0f;
  float ovenOvertemperatureOffC = 78.0f;

  MaxChannelConfig max1 = { true, 0.0f, 1.0f };
  MaxChannelConfig max2 = { false, 0.0f, 1.0f };
  MaxChannelConfig max3 = { false, 0.0f, 1.0f };
  MaxChannelConfig max4 = { false, 0.0f, 1.0f };

  RelayOutputConfig relays[RELAY_COUNT];
  PwmOutputConfig pwmOutputs[PWM_OUTPUT_COUNT];
  PumpConfig pumps[MAX_PUMPS];

  AuxHeaterConfig auxHeater;
  OvenConfig oven;
  HeatingCircuitConfig heatingCircuits[MAX_HEATING_CIRCUITS];
  ValveConfig valves[MAX_VALVES];
  EnergyMeterConfig energyMeter;
};

// ===================== Diagnose =====================
struct DiagnosticData {
  bool faultActive = false;
  char faultText[96] = "";

  uint32_t bootCount = 0;
  uint32_t sensorErrorCount = 0;
  uint32_t sdErrorCount = 0;
  uint32_t faultCount = 0;
  uint32_t maxErrorCount = 0;

  float lastCollectorC = NAN;
  float lastSinkC = NAN;
  float lastDiffC = NAN;

  uint32_t frostEventCount = 0;
  uint32_t stagnationEventCount = 0;
  uint32_t storageReachedMaximumAtMs = 0;
  uint8_t lastProtectionMode = 0;
};

// ===================== Wartung =====================
struct MaintenanceData {
  uint32_t pumpStarts = 0;
  uint32_t relaySwitchCount = 0;
  uint64_t pumpRuntimeSeconds = 0;
  uint64_t controllerRuntimeSeconds = 0;

  char lastServiceDate[24] = "";
  bool serviceDue = false;
};


// ===================== App Context =====================
struct AppContext {
  SystemState state = SystemState::BOOT;

  SensorSnapshot sensors;
  ControlData control;
  ConfigData config;
  DiagnosticData diag;
  MaintenanceData maintenance;

  Ds18b20Inventory ds18b20;
  SensorAssignmentTable assignments;

  MaxChannelReading maxReadings[MAX_MAX31865_CHANNELS];
  MaxChannelRuntime maxRuntime[MAX_MAX31865_CHANNELS];

  HeatSourceAssignmentTable heatSourceAssignments;

  bool sdAvailable = false;
  bool pumpWasEnabled = false;

  bool networkInitialized = false;
  bool networkApActive = false;
  bool networkStaConfigured = false;
  bool networkStaConnected = false;
  bool networkStaConnectAttemptActive = false;
  bool networkStaEverConnected = false;
  uint8_t networkWifiMode = 0;
  uint8_t networkStaStatus = 0;
  int32_t networkStaRssi = 0;
  uint32_t networkLastStaReconnectAttemptMs = 0;
  uint32_t networkStaConnectedSinceMs = 0;
  uint32_t networkStaDisconnectedSinceMs = 0;

  uint32_t stateEnteredAtMs = 0;
  uint32_t lastSampleAtMs = 0;
  uint32_t lastRuntimeSaveAtMs = 0;

  RelayOutputRuntime relayRuntime[RELAY_COUNT];
  EnergyRouteReservation reservations[MAX_PUMPS];

  CommissioningState commissioning;
  HeatingCircuitRuntime heatingCircuitRuntime[MAX_HEATING_CIRCUITS];
  EnergyMeterRuntime energyMeter;
};

