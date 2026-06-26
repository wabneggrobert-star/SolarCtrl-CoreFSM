#pragma once
#include <Arduino.h>

static constexpr uint8_t MAX_DS18B20_SENSORS = 20;
static constexpr uint8_t MAX_HEAT_SOURCE_ASSIGNMENTS = 3;
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
  CH3
};

enum class HeatSourceRole : uint8_t {
  NONE = 0,

  SOLAR_COLLECTOR_1,
  SOLAR_COLLECTOR_2,
  SOLAR_COLLECTOR_3,

  ALT_SOURCE_OVEN,
  ALT_SOURCE_OTHER,

  // Sensorrollen fuer ADC/MAX31865. Solar/Ofen bleiben echte Waermequellen,
  // die folgenden Rollen duerfen ADC1..ADC3 ebenfalls verwenden.
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
  BIAS_ON,
  WAIT_BIAS,
  START_1SHOT,
  WAIT_CONVERSION,
  READ_RESULT
};

struct CommissioningState {
  bool active = false;
};

struct MaxChannelRuntime {
  MaxReadStep step = MaxReadStep::IDLE;
  uint32_t tMarkUs = 0;
  bool cycleDone = false;
};

// ===================== DS18B20 Inventar =====================
struct Ds18b20DeviceInfo {
  bool present = false;
  uint8_t address[8] = {0};
  char addressText[24] = "";
  float lastTempC = NAN;
  bool lastValid = false;
  Ds18Role role = Ds18Role::NONE;
};

struct Ds18b20Inventory {
  uint8_t count = 0;
  Ds18b20DeviceInfo devices[MAX_DS18B20_SENSORS];
};

// Kompatibilitätsstruktur alter Sink-Pfad
struct SensorAssignment {
  bool sinkAssigned = false;
  uint8_t sinkAddress[8] = {0};
  char sinkAddressText[24] = "";
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
  uint16_t rawRtd = 0;
  uint8_t fault = 0;
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
  MIXER
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

  // Optionales 2-Ziel-Umschaltventil.
  // Wenn deaktiviert, verwendet die Pumpe sinkRole wie bisher.
  // Wenn aktiviert, werden targets[0] und targets[1] als Ziel A/B verwendet.
  bool switchValveEnabled = false;
  uint8_t switchValveRelayIndex = PIN_UNUSED;

  // Zeit, die das Umschaltventil mechanisch zum Umstellen braucht.
  // Waehrend dieser Zeit bleibt die Pumpe ausgeschaltet.
  uint32_t switchValveTravelTimeMs = 15000;

  // Runtime fuer Ventilbewegung. Nicht dauerhaft relevant, wird aber im UI angezeigt.
  bool switchValveMoving = false;
  uint32_t switchValveMoveStartedMs = 0;
  uint8_t switchValvePendingTargetIndex = PIN_UNUSED;

  // Logischer RelayOutputs::set(...)-Zustand fuer Ziel A.
  // Ziel B verwendet automatisch den invertierten Zustand.
  bool switchValveStateForTargetA = false;

  PumpRouteTargetConfig targets[PUMP_ROUTE_TARGET_COUNT];
  uint8_t activeTargetIndex = PIN_UNUSED;

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
  uint8_t valveRelayIndex = PIN_UNUSED;
};
//==========Heizstab/Elektrokessel========
struct AuxHeaterConfig {
  bool enabled = false;

  float minimumTemperatureC = 45.0f;
  float targetTemperatureC = 55.0f;
  float hysteresisC = 2.0f;

  Ds18Role sinkRole = Ds18Role::NONE;

  uint8_t pumpRelay = 255;

  uint8_t heaterRelay1 = 255;
  uint8_t heaterRelay2 = 255;
  uint8_t heaterRelay3 = 255;

  uint32_t preRunMs = 300000UL;
  uint32_t cooldownMs = 300000UL;
};

// ===================== Ofenkonfiguration =====================
struct OvenConfig {
  bool enabled = false;

  uint8_t pumpRelay = PIN_UNUSED;
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

  uint8_t servoMinimumAngle = 0;
  uint8_t servoMaximumAngle = 90;
  uint8_t servoBaseAngle = 45;

  float pidKp = 2.0f;
  float pidKi = 0.02f;
  float pidKd = 0.0f;
};


// ===================== Heizkreise / Mischer =====================
enum class OutputKind : uint8_t {
  NONE = 0,
  RELAY = 1,
  PWM_OUTPUT = 2
};

enum class ValvePosition : uint8_t {
  A = 0,
  B = 1
};


struct OutputRef {
  OutputKind kind = OutputKind::NONE;
  uint8_t index = PIN_UNUSED;
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

enum class HeatingCircuitPumpMode : uint8_t {
  NONE = 0,
  SWITCHED = 1,
  PWM = 2
};

struct HeatingCircuitConfig {
  bool enabled = false;

  HeatingCircuitMixerType mixerType = HeatingCircuitMixerType::THREE_POINT;
  HeatingCircuitControlMode controlMode = HeatingCircuitControlMode::FIXED_FLOW;

  OutputRef mixerOpenOutput;
  OutputRef mixerCloseOutput;

  HeatingCircuitPumpMode pumpMode = HeatingCircuitPumpMode::NONE;
  OutputRef pumpOutput;
  uint8_t pumpMinPercent = 30;
  uint8_t pumpMaxPercent = 100;

  Ds18Role flowSensorRole = Ds18Role::NONE;
  Ds18Role returnSensorRole = Ds18Role::NONE;
  Ds18Role roomSensorRole = Ds18Role::NONE;
  Ds18Role bufferReferenceRole = Ds18Role::NONE;
  Ds18Role outsideSensorRole = Ds18Role::NONE;

  float fixedFlowTemperatureC = 35.0f;
  float maximumFlowTemperatureC = 55.0f;
  float minimumFlowTemperatureC = 20.0f;

  float roomTargetTemperatureC = 21.0f;
  float roomInfluenceK = 3.0f;

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
  float roomTemperatureC = NAN;
  float outsideTemperatureC = NAN;
  float targetFlowTemperatureC = NAN;
  int16_t estimatedMixerPositionPercent = 50;
  uint32_t lastMixerActionMs = 0;
};


static constexpr uint8_t MAX_VALVES = 10;


// ===================== Konfiguration =====================
struct ConfigData {
  SinkTarget activeSinkTarget = SinkTarget::BOILER_TOP;

  float diffOnC = 3.0f;
  float diffOffC = 2.0f;

  float pwmStartDiffC = 3.0f;
  uint8_t pwmStartPercent = 30;

  float pwmFullDiffC = 10.0f;
  uint8_t pwmFullPercent = 100;

  uint32_t sampleIntervalMs = 2000;
  uint32_t runtimeSaveIntervalMs = 60000;

  char apName[32] = "SolarCtrl";
  char apPassword[32] = "12345678";
  char servicePin[16] = "1234";

  SolarFluidType solarFluidType = SolarFluidType::GLYCOL;
  SolarHydraulicType solarHydraulicType = SolarHydraulicType::CLOSED_PRESSURIZED;
  SolarCollectorType solarCollectorType = SolarCollectorType::FLAT_PLATE;

  bool frostEnabled = true;
  float frostCollectorOnC = 4.0f;
  float frostCollectorOffC = 6.0f;
  float frostSinkMinC = 10.0f;
  uint8_t frostPumpPercent = 40;
  Ds18Role frostProtectionStorageRole = Ds18Role::SINK_BUFFER_TOP;
  float frostSafeCollectorTemperatureC = 8.0f;

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

  RelayOutputConfig relays[RELAY_COUNT];
  PwmOutputConfig pwmOutputs[PWM_OUTPUT_COUNT];
  PumpConfig pumps[MAX_PUMPS];

  AuxHeaterConfig auxHeater;
  OvenConfig oven;
  HeatingCircuitConfig heatingCircuits[MAX_HEATING_CIRCUITS];
  ValveConfig valves[MAX_VALVES];
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

  MaxChannelReading maxReadings[3];
  MaxChannelRuntime maxRuntime[3];

  HeatSourceAssignmentTable heatSourceAssignments;

  bool sdAvailable = false;
  bool pumpWasEnabled = false;

  uint32_t stateEnteredAtMs = 0;
  uint32_t lastSampleAtMs = 0;
  uint32_t lastRuntimeSaveAtMs = 0;

  RelayOutputRuntime relayRuntime[RELAY_COUNT];
  EnergyRouteReservation reservations[MAX_PUMPS];

  CommissioningState commissioning;
  HeatingCircuitRuntime heatingCircuitRuntime[MAX_HEATING_CIRCUITS];
};

