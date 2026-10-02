#include "feature_mqtt.h"

#include "feature_alarms.h"
#include "feature_aux_heater.h"
#include "feature_oven.h"
#include "feature_heat_source_assignments.h"
#include "feature_heat_source_roles.h"
#include "feature_sensor_roles.h"
#include "feature_storage.h"
#include "feature_valves.h"

#include <WiFi.h>
#include <WiFiClient.h>
#include <math.h>
#include <string.h>

namespace {
constexpr uint32_t MQTT_RECONNECT_MS = 10000UL;
constexpr uint32_t MQTT_CONNECT_TIMEOUT_MS = 50UL;
constexpr uint32_t MQTT_HANDSHAKE_TIMEOUT_MS = 3000UL;
constexpr uint16_t MQTT_KEEPALIVE_S = 30;
constexpr uint32_t MQTT_PING_AFTER_MS = 15000UL;
constexpr uint32_t MQTT_PING_TIMEOUT_MS = 10000UL;
constexpr size_t MQTT_RX_MAX = 768;

WiFiClient g_client;
AppContext* g_ctx = nullptr;
bool g_connected = false;
bool g_connecting = false;
bool g_discoveryPending = false;
bool g_snapshotPending = false;
uint16_t g_discoveryTarget = 0;
uint16_t g_discoveryVisit = 0;
bool g_discoverySent = false;
uint16_t g_snapshotTarget = 0;
uint16_t g_snapshotVisit = 0;
bool g_snapshotSent = false;
bool g_snapshotEnumerating = false;
uint32_t g_lastAttemptMs = 0;
uint32_t g_connectedSinceMs = 0;
uint32_t g_lastIoMs = 0;
uint32_t g_pingSentMs = 0;
uint32_t g_lastPublishMs = 0;
uint16_t g_packetId = 1;
uint32_t g_lastStateSignature = 0;
char g_lastError[128] = "";
char g_lastConfigError[128] = "";

uint8_t g_rxHeader = 0;
uint32_t g_rxRemaining = 0;
uint32_t g_rxMultiplier = 1;
bool g_rxReadingRemaining = false;
size_t g_rxPos = 0;
uint8_t g_rxBuffer[MQTT_RX_MAX];

void copyText(char* dst, size_t dstSize, const char* src) {
  if (!dst || dstSize == 0) return;
  strncpy(dst, src ? src : "", dstSize - 1);
  dst[dstSize - 1] = '\0';
}

void setError(const String& text) {
  copyText(g_lastError, sizeof(g_lastError), text.c_str());
}

void setConfigError(const String& text) {
  copyText(g_lastConfigError, sizeof(g_lastConfigError), text.c_str());
}

String jsonEscape(const String& in) {
  String out;
  out.reserve(in.length() + 8);
  for (size_t i = 0; i < in.length(); ++i) {
    const char c = in[i];
    if (c == '\\' || c == '"') { out += '\\'; out += c; }
    else if (c == '\n') out += "\\n";
    else if (c == '\r') out += "\\r";
    else if ((uint8_t)c >= 0x20) out += c;
  }
  return out;
}

String baseTopic(const ConfigData& cfg) {
  String b = cfg.mqttBaseTopic[0] ? String(cfg.mqttBaseTopic) : String("haus/solarctrl");
  while (b.endsWith("/")) b.remove(b.length() - 1);
  return b;
}

String topicFor(const ConfigData& cfg, const String& suffix) {
  return baseTopic(cfg) + "/" + suffix;
}

String objectId(const String& raw) {
  String out;
  out.reserve(raw.length());
  for (size_t i = 0; i < raw.length(); ++i) {
    char c = raw[i];
    if ((c >= 'A' && c <= 'Z')) c = char(c - 'A' + 'a');
    if ((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9')) out += c;
    else out += '_';
  }
  while (out.indexOf("__") >= 0) out.replace("__", "_");
  return out;
}

size_t encodeRemainingLength(uint32_t value, uint8_t out[4]) {
  size_t n = 0;
  do {
    uint8_t encoded = value % 128;
    value /= 128;
    if (value > 0) encoded |= 0x80;
    out[n++] = encoded;
  } while (value > 0 && n < 4);
  return n;
}

bool writeAll(const uint8_t* data, size_t len) {
  if (!g_client.connected()) return false;
  size_t sent = 0;
  while (sent < len) {
    const size_t n = g_client.write(data + sent, len - sent);
    if (n == 0) return false;
    sent += n;
  }
  g_lastIoMs = millis();
  return true;
}

void appendMqttString(uint8_t* buffer, size_t& pos, size_t cap, const String& value) {
  const size_t len = value.length();
  if (pos + 2 + len > cap) return;
  buffer[pos++] = (uint8_t)((len >> 8) & 0xFF);
  buffer[pos++] = (uint8_t)(len & 0xFF);
  memcpy(buffer + pos, value.c_str(), len);
  pos += len;
}

bool sendConnect(const ConfigData& cfg) {
  uint8_t body[640];
  size_t pos = 0;
  appendMqttString(body, pos, sizeof(body), "MQTT");
  body[pos++] = 4;  // MQTT 3.1.1

  uint8_t flags = 0x02; // clean session
  flags |= 0x04;        // will flag
  flags |= 0x20;        // will retain
  if (cfg.mqttUser[0]) flags |= 0x80;
  if (cfg.mqttPassword[0]) flags |= 0x40;
  body[pos++] = flags;
  body[pos++] = (uint8_t)(MQTT_KEEPALIVE_S >> 8);
  body[pos++] = (uint8_t)(MQTT_KEEPALIVE_S & 0xFF);

  appendMqttString(body, pos, sizeof(body), cfg.mqttClientId[0] ? String(cfg.mqttClientId) : String("SolarCtrl"));
  appendMqttString(body, pos, sizeof(body), topicFor(cfg, "status"));
  appendMqttString(body, pos, sizeof(body), "offline");
  if (cfg.mqttUser[0]) appendMqttString(body, pos, sizeof(body), String(cfg.mqttUser));
  if (cfg.mqttPassword[0]) appendMqttString(body, pos, sizeof(body), String(cfg.mqttPassword));

  uint8_t header[5];
  header[0] = 0x10;
  const size_t rl = encodeRemainingLength((uint32_t)pos, &header[1]);
  return writeAll(header, 1 + rl) && writeAll(body, pos);
}

bool publishRaw(const String& topic, const String& payload, bool retain = false) {
  if (!g_connected || !g_client.connected()) return false;
  const uint32_t remaining = 2 + topic.length() + payload.length();
  uint8_t header[5];
  header[0] = retain ? 0x31 : 0x30;
  const size_t rl = encodeRemainingLength(remaining, &header[1]);
  if (!writeAll(header, 1 + rl)) return false;
  uint8_t topicLen[2] = {(uint8_t)(topic.length() >> 8), (uint8_t)(topic.length() & 0xFF)};
  if (!writeAll(topicLen, 2)) return false;
  if (!writeAll((const uint8_t*)topic.c_str(), topic.length())) return false;
  if (payload.length() && !writeAll((const uint8_t*)payload.c_str(), payload.length())) return false;
  return true;
}

bool subscribeCommands(const ConfigData& cfg) {
  const String topic = topicFor(cfg, "cmd/#");
  uint8_t body[192];
  size_t pos = 0;
  const uint16_t id = ++g_packetId;
  body[pos++] = (uint8_t)(id >> 8);
  body[pos++] = (uint8_t)(id & 0xFF);
  appendMqttString(body, pos, sizeof(body), topic);
  body[pos++] = 0; // QoS 0
  uint8_t header[5];
  header[0] = 0x82;
  const size_t rl = encodeRemainingLength((uint32_t)pos, &header[1]);
  return writeAll(header, 1 + rl) && writeAll(body, pos);
}

void sendDisconnect() {
  if (g_client.connected()) {
    const uint8_t packet[2] = {0xE0, 0x00};
    writeAll(packet, sizeof(packet));
  }
  g_client.stop();
  g_connected = false;
  g_connecting = false;
}

void resetRx() {
  g_rxHeader = 0;
  g_rxRemaining = 0;
  g_rxMultiplier = 1;
  g_rxReadingRemaining = false;
  g_rxPos = 0;
}

String boolPayload(bool v) { return v ? "ON" : "OFF"; }
String numberPayload(float v, uint8_t decimals = 2) {
  if (isnan(v) || isinf(v)) return "unknown";
  return String((double)v, (unsigned int)decimals);
}

uint32_t runtimeStateSignature(const AppContext& ctx) {
  uint32_t h = 2166136261UL;
  auto mix = [&](uint32_t v) { h ^= v; h *= 16777619UL; };
  mix(OvenControl::active() ? 1 : 0);
  mix(OvenControl::pumpActive() ? 1 : 0);
  mix(OvenControl::servoOpeningPercent());
  mix(AuxHeater::heatingActive() ? 1 : 0);
  mix(AuxHeater::cooldownActive() ? 1 : 0);
  mix(Alarms::activeCount());
  mix(Alarms::hasCriticalActive() ? 1 : 0);
  mix(Alarms::buzzerMuted() ? 1 : 0);
  for (uint8_t i = 0; i < MAX_PUMPS; ++i) mix(ctx.config.pumps[i].state ? (17 + i) : i);
  for (uint8_t i = 0; i < MAX_VALVES; ++i) {
    mix((uint32_t)Valves::currentPosition(i) + i * 3U);
    mix(Valves::isMoving(i) ? (101 + i) : i);
  }
  for (uint8_t i = 0; i < MAX_HEATING_CIRCUITS; ++i) {
    const HeatingCircuitRuntime& r = ctx.heatingCircuitRuntime[i];
    mix(r.active ? (201 + i) : i);
    mix(r.pumpActive ? (221 + i) : i);
    mix(r.opening ? (241 + i) : i);
    mix(r.closing ? (261 + i) : i);
  }
  return h;
}

void publishState(const String& key, const String& value) {
  if (!g_ctx) return;
  if (g_snapshotEnumerating) {
    const uint16_t current = g_snapshotVisit++;
    if (current != g_snapshotTarget) return;
    g_snapshotSent = publishRaw(topicFor(g_ctx->config, "state/" + key), value, false);
    return;
  }
  publishRaw(topicFor(g_ctx->config, "state/" + key), value, false);
}

String deviceJson() {
  return "\"device\":{\"identifiers\":[\"solarctrl\"],\"name\":\"SolarCtrl\",\"manufacturer\":\"DIY\",\"model\":\"ESP32 SolarCtrl\"}";
}

void publishDiscoveryEntity(const char* component,
                            const String& object,
                            const String& name,
                            const String& stateSuffix,
                            const String& commandSuffix = "",
                            const String& extra = "") {
  if (!g_ctx || !g_ctx->config.mqttDiscoveryEnabled) return;
  const uint16_t current = g_discoveryVisit++;
  if (current != g_discoveryTarget) return;
  const String oid = objectId(object);
  const String topic = "homeassistant/" + String(component) + "/solarctrl_" + oid + "/config";
  String json = "{";
  json += "\"name\":\"" + jsonEscape(name) + "\",";
  json += "\"unique_id\":\"solarctrl_" + oid + "\",";
  if (stateSuffix.length()) json += "\"state_topic\":\"" + jsonEscape(topicFor(g_ctx->config, "state/" + stateSuffix)) + "\",";
  if (commandSuffix.length()) json += "\"command_topic\":\"" + jsonEscape(topicFor(g_ctx->config, "cmd/" + commandSuffix)) + "\",";
  json += "\"availability_topic\":\"" + jsonEscape(topicFor(g_ctx->config, "status")) + "\",";
  json += "\"payload_available\":\"online\",\"payload_not_available\":\"offline\",";
  if (extra.length()) json += extra + ",";
  json += deviceJson();
  json += "}";
  g_discoverySent = publishRaw(topic, json, true);
}

void discoverSensor(const String& object, const String& name, const String& stateSuffix,
                    const String& unit = "", const String& deviceClass = "", const String& stateClass = "measurement") {
  String extra;
  if (unit.length()) extra += "\"unit_of_measurement\":\"" + jsonEscape(unit) + "\"";
  if (deviceClass.length()) { if (extra.length()) extra += ","; extra += "\"device_class\":\"" + deviceClass + "\""; }
  if (stateClass.length()) { if (extra.length()) extra += ","; extra += "\"state_class\":\"" + stateClass + "\""; }
  publishDiscoveryEntity("sensor", object, name, stateSuffix, "", extra);
}

void discoverBinary(const String& object, const String& name, const String& stateSuffix, const String& deviceClass = "") {
  String extra = "\"payload_on\":\"ON\",\"payload_off\":\"OFF\"";
  if (deviceClass.length()) extra += ",\"device_class\":\"" + deviceClass + "\"";
  publishDiscoveryEntity("binary_sensor", object, name, stateSuffix, "", extra);
}

void discoverSwitch(const String& object, const String& name, const String& stateSuffix, const String& commandSuffix) {
  publishDiscoveryEntity("switch", object, name, stateSuffix, commandSuffix,
                         "\"payload_on\":\"ON\",\"payload_off\":\"OFF\",\"state_on\":\"ON\",\"state_off\":\"OFF\"");
}

void discoverNumber(const String& object, const String& name, const String& stateSuffix, const String& commandSuffix,
                    float minV, float maxV, float step, const String& unit = "") {
  String extra = "\"min\":" + String(minV, 3) + ",\"max\":" + String(maxV, 3) + ",\"step\":" + String(step, 3) + ",\"mode\":\"box\"";
  if (unit.length()) extra += ",\"unit_of_measurement\":\"" + jsonEscape(unit) + "\"";
  publishDiscoveryEntity("number", object, name, stateSuffix, commandSuffix, extra);
}

void discoverSelect(const String& object, const String& name, const String& stateSuffix, const String& commandSuffix, const String& optionsJson) {
  publishDiscoveryEntity("select", object, name, stateSuffix, commandSuffix, "\"options\":" + optionsJson);
}

void discoverButton(const String& object, const String& name, const String& commandSuffix) {
  publishDiscoveryEntity("button", object, name, "", commandSuffix, "\"payload_press\":\"PRESS\"");
}

void publishDiscovery(AppContext& ctx) {
  discoverBinary("mqtt_connected", "MQTT verbunden", "mqtt/connected", "connectivity");
  discoverSensor("wifi_rssi", "WLAN RSSI", "network/rssi", "dBm", "signal_strength");
  discoverSensor("uptime", "Uptime", "system/uptime_s", "s", "duration", "total_increasing");
  discoverSensor("mqtt_last_config_error", "MQTT letzter Konfigurationsfehler", "mqtt/last_config_error", "", "", "");

  discoverSensor("alarm_status", "Alarmstatus", "alarm/status", "", "", "");
  discoverSensor("alarm_message", "Aktueller Alarm", "alarm/message", "", "", "");
  discoverSensor("alarm_count", "Aktive Alarme", "alarm/count", "", "", "measurement");
  discoverSwitch("buzzer_muted", "Summer stumm", "alarm/buzzer_muted", "alarm/buzzer_mute");
  discoverButton("alarm_ack", "Alarm quittieren", "alarm/ack");

  discoverSensor("energy_total", "Gesamtenergie", "energy/total_kwh", "kWh", "energy", "total_increasing");
  discoverSensor("energy_power", "Aktuelle thermische Leistung", "energy/power_kw", "kW", "power", "measurement");
  discoverSensor("energy_flow", "Durchfluss", "energy/flow_l_min", "L/min", "", "measurement");
  discoverSensor("energy_volume", "Gesamtvolumen", "energy/volume_l", "L", "water", "total_increasing");
  discoverSensor("energy_flow_temp", "Energiezähler Vorlauf", "energy/flow_temp_c", "°C", "temperature");
  discoverSensor("energy_return_temp", "Energiezähler Rücklauf", "energy/return_temp_c", "°C", "temperature");
  discoverSensor("energy_delta_t", "Energiezähler Spreizung", "energy/delta_t_k", "K", "", "measurement");
  discoverSensor("energy_pulses", "Flowmeter Impulse gesamt", "energy/pulses_total", "", "", "total_increasing");

  discoverSensor("oven_state", "Ofen Status", "oven/state", "", "", "");
  discoverSensor("oven_temp", "Ofen Temperatur", "oven/temperature_c", "°C", "temperature");
  discoverSensor("oven_opening", "Ofen Luftklappe", "oven/opening_pct", "%", "", "measurement");
  discoverBinary("oven_pump", "Ofenpumpe", "oven/pump_active", "running");
  discoverButton("oven_start", "Ofen starten", "oven/start");
  discoverButton("oven_stop", "Ofen beenden", "oven/stop");

  discoverBinary("aux_heating", "Zusatzheizung aktiv", "aux/heating_active", "heat");
  discoverBinary("aux_cooldown", "Zusatzheizung Nachlauf", "aux/cooldown_active", "running");

  for (uint8_t i = 0; i < MAX_PUMPS; ++i) {
    const String p = String(i + 1);
    discoverBinary("pump_" + p + "_running", "Pumpe " + p + " läuft", "pump/" + p + "/running", "running");
    discoverSensor("pump_" + p + "_pwm", "Pumpe " + p + " PWM", "pump/" + p + "/pwm_pct", "%", "", "measurement");
    discoverSensor("pump_" + p + "_source", "Pumpe " + p + " Quelle", "pump/" + p + "/source_c", "°C", "temperature");
    discoverSensor("pump_" + p + "_sink", "Pumpe " + p + " Ziel", "pump/" + p + "/sink_c", "°C", "temperature");
    discoverSensor("pump_" + p + "_diff", "Pumpe " + p + " Spreizung", "pump/" + p + "/diff_c", "K", "", "measurement");
  }

  for (uint8_t i = 0; i < MAX_VALVES; ++i) {
    const String n = String(i + 1);
    discoverSensor("valve_" + n + "_position", "Ventil " + n + " Position", "valve/" + n + "/position", "", "", "");
    discoverBinary("valve_" + n + "_moving", "Ventil " + n + " fährt", "valve/" + n + "/moving", "moving");
  }

  for (uint8_t i = 0; i < MAX_HEATING_CIRCUITS; ++i) {
    const String n = String(i + 1);
    discoverBinary("hk_" + n + "_active", "Heizkreis " + n + " aktiv", "hk/" + n + "/active", "heat");
    discoverBinary("hk_" + n + "_pump", "Heizkreis " + n + " Pumpe", "hk/" + n + "/pump_active", "running");
    discoverSensor("hk_" + n + "_flow", "Heizkreis " + n + " Vorlauf", "hk/" + n + "/flow_c", "°C", "temperature");
    discoverSensor("hk_" + n + "_return", "Heizkreis " + n + " Rücklauf", "hk/" + n + "/return_c", "°C", "temperature");
    discoverSensor("hk_" + n + "_target", "Heizkreis " + n + " Soll-Vorlauf", "hk/" + n + "/target_flow_c", "°C", "temperature");
    discoverSensor("hk_" + n + "_room", "Heizkreis " + n + " Raum", "hk/" + n + "/room_c", "°C", "temperature");
    discoverSensor("hk_" + n + "_mixer", "Heizkreis " + n + " Mischer", "hk/" + n + "/mixer_pct", "%", "", "measurement");
  }

  for (uint8_t i = 0; i < ctx.ds18b20.count; ++i) {
    const Ds18b20DeviceInfo& d = ctx.ds18b20.devices[i];
    String label;
    if (d.role != Ds18Role::NONE) label = SensorRoles::toLabel(d.role);
    else label = "DS18B20 " + String(d.addressText);
    const String id = "ds18_" + objectId(String(d.addressText));
    discoverSensor(id, label, "sensor/ds18b20/" + String(i) + "/temperature_c", "°C", "temperature");
  }
  for (uint8_t i = 0; i < MAX_MAX31865_CHANNELS; ++i) {
    String label = "MAX31865 " + String(i + 1);
    HeatSourceAssignment a;
    if (HeatSourceAssignments::getByChannel(ctx.heatSourceAssignments, (MaxChannel)i, a) && a.role != HeatSourceRole::NONE) {
      label = HeatSourceRoles::toLabel(a.role);
    }
    discoverSensor("max31865_" + String(i + 1), label, "sensor/max/" + String(i + 1) + "/temperature_c", "°C", "temperature");
  }

  // Schreibbare normale Anlagenkonfiguration. Hardware-, Netzwerk- und Safety-Zuordnungen bleiben bewusst read-only.
  discoverSelect("cfg_active_sink", "Priorität Wärmeabnehmer", "config/active_sink", "config/active_sink", "[\"Boiler\",\"Puffer\"]");

  for (uint8_t i = 0; i < MAX_PUMPS; ++i) {
    const String n = String(i + 1);
    discoverSwitch("cfg_pump_" + n + "_enabled", "Pumpe " + n + " aktiviert", "config/pump/" + n + "/enabled", "config/pump/" + n + "/enabled");
    discoverNumber("cfg_pump_" + n + "_target", "Pumpe " + n + " Zielspreizung", "config/pump/" + n + "/target_diff_c", "config/pump/" + n + "/target_diff_c", 0.5f, 40, 0.5f, "K");
    discoverNumber("cfg_pump_" + n + "_hyst", "Pumpe " + n + " Hysterese", "config/pump/" + n + "/hysteresis_c", "config/pump/" + n + "/hysteresis_c", 0, 20, 0.5f, "K");
    discoverNumber("cfg_pump_" + n + "_start", "Pumpe " + n + " Startdifferenz", "config/pump/" + n + "/start_diff_c", "config/pump/" + n + "/start_diff_c", 0.5f, 40, 0.5f, "K");
    discoverNumber("cfg_pump_" + n + "_min_pwm", "Pumpe " + n + " Minimum PWM", "config/pump/" + n + "/min_pwm_pct", "config/pump/" + n + "/min_pwm_pct", 0, 100, 1, "%");
    discoverNumber("cfg_pump_" + n + "_max_pwm", "Pumpe " + n + " Maximum PWM", "config/pump/" + n + "/max_pwm_pct", "config/pump/" + n + "/max_pwm_pct", 0, 100, 1, "%");
  }

  discoverSwitch("cfg_aux_enabled", "Zusatzheizung freigegeben", "config/aux/enabled", "config/aux/enabled");
  discoverNumber("cfg_aux_min", "Zusatzheizung Mindesttemperatur", "config/aux/minimum_c", "config/aux/minimum_c", 5, 80, 0.5f, "°C");
  discoverNumber("cfg_aux_target", "Zusatzheizung Zieltemperatur", "config/aux/target_c", "config/aux/target_c", 10, 85, 0.5f, "°C");
  discoverNumber("cfg_aux_hyst", "Zusatzheizung Hysterese", "config/aux/hysteresis_c", "config/aux/hysteresis_c", 0, 20, 0.5f, "K");
  discoverNumber("cfg_aux_prerun", "Zusatzheizung Pumpenvorlauf", "config/aux/prerun_s", "config/aux/prerun_s", 0, 1800, 10, "s");
  discoverNumber("cfg_aux_cooldown", "Zusatzheizung Pumpennachlauf", "config/aux/cooldown_s", "config/aux/cooldown_s", 0, 1800, 10, "s");

  discoverSwitch("cfg_oven_enabled", "Ofenregelung freigegeben", "config/oven/enabled", "config/oven/enabled");
  discoverSwitch("cfg_oven_auto", "Ofen Autostart", "config/oven/auto_start", "config/oven/auto_start");
  discoverNumber("cfg_oven_target", "Ofen Zieltemperatur", "config/oven/target_c", "config/oven/target_c", 40, 88, 1, "°C");
  discoverNumber("cfg_oven_pump_on", "Ofen Pumpenstart", "config/oven/pump_on_c", "config/oven/pump_on_c", 20, 85, 1, "°C");
  discoverNumber("cfg_oven_pump_diff", "Ofen Pumpen-Einschaltdifferenz", "config/oven/pump_on_diff_c", "config/oven/pump_on_diff_c", 1, 30, 0.5f, "K");
  discoverNumber("cfg_oven_pump_off_diff", "Ofen Pumpen-Ausschaltdifferenz", "config/oven/pump_off_diff_c", "config/oven/pump_off_diff_c", 0, 20, 0.5f, "K");
  discoverNumber("cfg_oven_min_open", "Ofen minimale Luftklappe", "config/oven/min_open_pct", "config/oven/min_open_pct", 0, 100, 1, "%");
  discoverNumber("cfg_oven_max_open", "Ofen maximale Luftklappe", "config/oven/max_open_pct", "config/oven/max_open_pct", 0, 100, 1, "%");
  discoverNumber("cfg_oven_step", "Ofen Luftklappen-Schritt", "config/oven/step_pct", "config/oven/step_pct", 1, 50, 1, "%");
  discoverNumber("cfg_oven_deadband", "Ofen Servo-Hysterese", "config/oven/deadband_c", "config/oven/deadband_c", 0.5f, 20, 0.5f, "K");
  discoverNumber("cfg_oven_burnout", "Ofen Abbrand-Nachlüftzeit", "config/oven/burnout_min", "config/oven/burnout_min", 1, 120, 1, "min");

  for (uint8_t i = 0; i < MAX_HEATING_CIRCUITS; ++i) {
    const String n = String(i + 1);
    discoverSwitch("cfg_hk_" + n + "_enabled", "Heizkreis " + n + " freigegeben", "config/hk/" + n + "/enabled", "config/hk/" + n + "/enabled");
    discoverSelect("cfg_hk_" + n + "_mode", "Heizkreis " + n + " Regelart", "config/hk/" + n + "/mode", "config/hk/" + n + "/mode", "[\"Festwert\",\"Witterungsgeführt\"]");
    discoverNumber("cfg_hk_" + n + "_fixed", "Heizkreis " + n + " Festwert Vorlauf", "config/hk/" + n + "/fixed_flow_c", "config/hk/" + n + "/fixed_flow_c", 15, 55, 0.5f, "°C");
    discoverSwitch("cfg_hk_" + n + "_room_on", "Heizkreis " + n + " Raumregelung", "config/hk/" + n + "/room_control", "config/hk/" + n + "/room_control");
    discoverNumber("cfg_hk_" + n + "_room_target", "Heizkreis " + n + " Raum Soll", "config/hk/" + n + "/room_target_c", "config/hk/" + n + "/room_target_c", 10, 30, 0.5f, "°C");
    discoverNumber("cfg_hk_" + n + "_room_inf", "Heizkreis " + n + " Raumeinfluss", "config/hk/" + n + "/room_influence_k", "config/hk/" + n + "/room_influence_k", 0, 15, 0.5f, "K");
    discoverNumber("cfg_hk_" + n + "_pump_delta", "Heizkreis " + n + " Pumpen-Zielspreizung", "config/hk/" + n + "/pump_target_delta_c", "config/hk/" + n + "/pump_target_delta_c", 1, 20, 0.5f, "K");
    discoverNumber("cfg_hk_" + n + "_curve_base", "Heizkreis " + n + " Heizkurve Basis", "config/hk/" + n + "/curve_base_c", "config/hk/" + n + "/curve_base_c", 10, 45, 0.5f, "°C");
    discoverNumber("cfg_hk_" + n + "_curve_slope", "Heizkreis " + n + " Heizkurve Steigung", "config/hk/" + n + "/curve_slope", "config/hk/" + n + "/curve_slope", 0, 3, 0.05f, "");
  }

  discoverSwitch("cfg_energy_enabled", "Energiezähler freigegeben", "config/energy/enabled", "config/energy/enabled");
  discoverNumber("cfg_energy_ppl", "Flowmeter Impulse pro Liter", "config/energy/pulses_per_l", "config/energy/pulses_per_l", 0.1f, 100000, 0.1f, "Imp/L");
  discoverNumber("cfg_energy_factor", "Energiezähler Wärmeträgerfaktor", "config/energy/factor", "config/energy/factor", 0.1f, 2.0f, 0.001f, "Wh/(L·K)");
  discoverNumber("cfg_energy_log", "Energiezähler Logintervall", "config/energy/log_s", "config/energy/log_s", 10, 3600, 10, "s");

  // Safety-Werte absichtlich nur lesbar.
  discoverSensor("safety_storage_limit", "Safety Speicher Grenztemperatur", "safety/storage_critical_c", "°C", "temperature", "");
  discoverSensor("safety_oven_on", "Safety Ofen Übertemperatur EIN", "safety/oven_on_c", "°C", "temperature", "");
  discoverSensor("safety_oven_off", "Safety Ofen Übertemperatur AUS", "safety/oven_off_c", "°C", "temperature", "");
}

void publishConfigStates(AppContext& ctx) {
  publishState("config/active_sink", ctx.config.activeSinkTarget == SinkTarget::BUFFER_TOP ? "Puffer" : "Boiler");

  for (uint8_t i = 0; i < MAX_PUMPS; ++i) {
    const PumpConfig& p = ctx.config.pumps[i];
    const String n = String(i + 1);
    publishState("config/pump/" + n + "/enabled", boolPayload(p.enabled));
    publishState("config/pump/" + n + "/target_diff_c", numberPayload(p.targetDiff, 2));
    publishState("config/pump/" + n + "/hysteresis_c", numberPayload(p.hysteresis, 2));
    publishState("config/pump/" + n + "/start_diff_c", numberPayload(p.startDiff, 2));
    publishState("config/pump/" + n + "/min_pwm_pct", numberPayload(p.minPwmPercent, 1));
    publishState("config/pump/" + n + "/max_pwm_pct", numberPayload(p.maxPwmPercent, 1));
  }

  const AuxHeaterConfig& a = ctx.config.auxHeater;
  publishState("config/aux/enabled", boolPayload(a.enabled));
  publishState("config/aux/minimum_c", numberPayload(a.minimumTemperatureC, 2));
  publishState("config/aux/target_c", numberPayload(a.targetTemperatureC, 2));
  publishState("config/aux/hysteresis_c", numberPayload(a.hysteresisC, 2));
  publishState("config/aux/prerun_s", String(a.preRunMs / 1000UL));
  publishState("config/aux/cooldown_s", String(a.cooldownMs / 1000UL));

  const OvenConfig& o = ctx.config.oven;
  publishState("config/oven/enabled", boolPayload(o.enabled));
  publishState("config/oven/auto_start", boolPayload(o.autoStartEnabled));
  publishState("config/oven/target_c", numberPayload(o.targetOvenTemperatureC, 1));
  publishState("config/oven/pump_on_c", numberPayload(o.pumpOnTemperatureC, 1));
  publishState("config/oven/pump_on_diff_c", numberPayload(o.pumpOnTemperatureDifferenceC, 1));
  publishState("config/oven/pump_off_diff_c", numberPayload(o.pumpOffTemperatureDifferenceC, 1));
  publishState("config/oven/min_open_pct", String(o.servoMinimumOpeningPercent));
  publishState("config/oven/max_open_pct", String(o.servoMaximumOpeningPercent));
  publishState("config/oven/step_pct", String(o.servoStepPercent));
  publishState("config/oven/deadband_c", numberPayload(o.servoDeadbandC, 1));
  publishState("config/oven/burnout_min", String(o.burnoutVentMinutes));

  for (uint8_t i = 0; i < MAX_HEATING_CIRCUITS; ++i) {
    const HeatingCircuitConfig& h = ctx.config.heatingCircuits[i];
    const String n = String(i + 1);
    publishState("config/hk/" + n + "/enabled", boolPayload(h.enabled));
    publishState("config/hk/" + n + "/mode", h.controlMode == HeatingCircuitControlMode::WEATHER_COMPENSATED ? "Witterungsgeführt" : "Festwert");
    publishState("config/hk/" + n + "/fixed_flow_c", numberPayload(h.fixedFlowTemperatureC, 1));
    publishState("config/hk/" + n + "/room_control", boolPayload(h.roomControlEnabled));
    publishState("config/hk/" + n + "/room_target_c", numberPayload(h.roomTargetTemperatureC, 1));
    publishState("config/hk/" + n + "/room_influence_k", numberPayload(h.roomInfluenceK, 1));
    publishState("config/hk/" + n + "/pump_target_delta_c", numberPayload(h.pumpTargetDeltaC, 1));
    publishState("config/hk/" + n + "/curve_base_c", numberPayload(h.heatingCurveBaseC, 1));
    publishState("config/hk/" + n + "/curve_slope", numberPayload(h.heatingCurveSlope, 2));
  }

  publishState("config/energy/enabled", boolPayload(ctx.config.energyMeter.enabled));
  publishState("config/energy/pulses_per_l", numberPayload(ctx.config.energyMeter.pulsesPerLiter, 3));
  publishState("config/energy/factor", numberPayload(ctx.config.energyMeter.energyFactorWhPerLiterK, 4));
  publishState("config/energy/log_s", String(ctx.config.energyMeter.logIntervalMs / 1000UL));
}

void publishSnapshotInternal(AppContext& ctx) {
  publishState("mqtt/connected", "ON");
  publishState("mqtt/last_config_error", g_lastConfigError[0] ? String(g_lastConfigError) : String("OK"));
  publishState("network/rssi", ctx.networkStaConnected ? String(ctx.networkStaRssi) : String("unknown"));
  publishState("system/uptime_s", String(millis() / 1000UL));

  String alarmStatus = "Kein Alarm";
  if (Alarms::hasCriticalActive()) alarmStatus = "Kritisch";
  else if (Alarms::hasUnacknowledgedWarning()) alarmStatus = "Warnung";
  else if (Alarms::activeCount() > 0) alarmStatus = "Info/Quittiert";
  publishState("alarm/status", alarmStatus);
  publishState("alarm/message", String(Alarms::currentMessage()));
  publishState("alarm/count", String(Alarms::activeCount()));
  publishState("alarm/buzzer_muted", boolPayload(Alarms::buzzerMuted()));

  publishState("energy/total_kwh", String(ctx.energyMeter.totalEnergyKWh, 3));
  publishState("energy/power_kw", numberPayload(ctx.energyMeter.thermalPowerKw, 3));
  publishState("energy/flow_l_min", numberPayload(ctx.energyMeter.flowLitersPerMinute, 2));
  publishState("energy/volume_l", String(ctx.energyMeter.totalVolumeLiters, 1));
  publishState("energy/flow_temp_c", numberPayload(ctx.energyMeter.flowTemperatureC, 1));
  publishState("energy/return_temp_c", numberPayload(ctx.energyMeter.returnTemperatureC, 1));
  publishState("energy/delta_t_k", numberPayload(ctx.energyMeter.deltaTemperatureK, 1));
  publishState("energy/pulses_total", String(ctx.energyMeter.totalPulses));

  publishState("oven/state", String(OvenControl::stateText()));
  publishState("oven/temperature_c", numberPayload(OvenControl::ovenTemperatureC(), 1));
  publishState("oven/opening_pct", String(OvenControl::servoOpeningPercent()));
  publishState("oven/pump_active", boolPayload(OvenControl::pumpActive()));
  publishState("aux/heating_active", boolPayload(AuxHeater::heatingActive()));
  publishState("aux/cooldown_active", boolPayload(AuxHeater::cooldownActive()));

  for (uint8_t i = 0; i < MAX_PUMPS; ++i) {
    const PumpConfig& p = ctx.config.pumps[i];
    const String n = String(i + 1);
    publishState("pump/" + n + "/running", boolPayload(p.state));
    publishState("pump/" + n + "/pwm_pct", numberPayload(p.lastPwmPercent, 1));
    publishState("pump/" + n + "/source_c", numberPayload(p.lastSourceC, 1));
    publishState("pump/" + n + "/sink_c", numberPayload(p.lastSinkC, 1));
    publishState("pump/" + n + "/diff_c", numberPayload(p.lastDiffC, 1));
  }

  for (uint8_t i = 0; i < MAX_VALVES; ++i) {
    const String n = String(i + 1);
    publishState("valve/" + n + "/position", Valves::positionToKey(Valves::currentPosition(i)));
    publishState("valve/" + n + "/moving", boolPayload(Valves::isMoving(i)));
  }

  for (uint8_t i = 0; i < MAX_HEATING_CIRCUITS; ++i) {
    const HeatingCircuitRuntime& h = ctx.heatingCircuitRuntime[i];
    const String n = String(i + 1);
    publishState("hk/" + n + "/active", boolPayload(h.active));
    publishState("hk/" + n + "/pump_active", boolPayload(h.pumpActive));
    publishState("hk/" + n + "/flow_c", numberPayload(h.flowTemperatureC, 1));
    publishState("hk/" + n + "/return_c", numberPayload(h.returnTemperatureC, 1));
    publishState("hk/" + n + "/target_flow_c", numberPayload(h.targetFlowTemperatureC, 1));
    publishState("hk/" + n + "/room_c", numberPayload(h.roomTemperatureC, 1));
    publishState("hk/" + n + "/mixer_pct", String(h.estimatedMixerPositionPercent));
  }

  for (uint8_t i = 0; i < ctx.ds18b20.count; ++i) {
    const Ds18b20DeviceInfo& d = ctx.ds18b20.devices[i];
    publishState("sensor/ds18b20/" + String(i) + "/temperature_c", d.lastValid ? numberPayload(d.lastTempC, 1) : "unknown");
  }
  for (uint8_t i = 0; i < MAX_MAX31865_CHANNELS; ++i) {
    const MaxChannelReading& r = ctx.maxReadings[i];
    publishState("sensor/max/" + String(i + 1) + "/temperature_c", r.valid ? numberPayload(r.tempC, 1) : "unknown");
  }

  publishState("safety/storage_critical_c", numberPayload(ctx.config.storageCriticalTemperatureC, 1));
  publishState("safety/oven_on_c", numberPayload(ctx.config.ovenOvertemperatureOnC, 1));
  publishState("safety/oven_off_c", numberPayload(ctx.config.ovenOvertemperatureOffC, 1));
  publishConfigStates(ctx);
}

bool parseBoolPayload(const String& p, bool& value) {
  String v = p; v.trim(); v.toUpperCase();
  if (v == "ON" || v == "1" || v == "TRUE" || v == "JA") { value = true; return true; }
  if (v == "OFF" || v == "0" || v == "FALSE" || v == "NEIN") { value = false; return true; }
  return false;
}

bool inRange(float v, float lo, float hi) { return isfinite(v) && v >= lo && v <= hi; }

bool applyConfigCommand(AppContext& ctx, const String& key, const String& payload, String& error) {
  if (ctx.commissioning.active) {
    error = "Konfigurationsaenderung waehrend Testmodus gesperrt";
    return false;
  }

  ConfigData candidate = ctx.config;
  bool b = false;
  const float f = payload.toFloat();
  const long iv = payload.toInt();

  if (key == "active_sink") {
    if (payload == "Puffer") candidate.activeSinkTarget = SinkTarget::BUFFER_TOP;
    else if (payload == "Boiler") candidate.activeSinkTarget = SinkTarget::BOILER_TOP;
    else { error = "Prioritaet muss Boiler oder Puffer sein"; return false; }
  } else if (key.startsWith("pump/")) {
    const int slash = key.indexOf('/', 5);
    if (slash < 0) { error = "Pumpenparameter ungueltig"; return false; }
    const int idx = key.substring(5, slash).toInt() - 1;
    if (idx < 0 || idx >= MAX_PUMPS) { error = "Pumpenindex ungueltig"; return false; }
    const String field = key.substring(slash + 1);
    PumpConfig& p = candidate.pumps[idx];
    if (field == "enabled") { if (!parseBoolPayload(payload, b)) { error = "Bool-Wert ungueltig"; return false; } p.enabled = b; }
    else if (field == "target_diff_c") { if (!inRange(f, .5f, 40)) { error = "Zielspreizung ausserhalb 0.5..40 K"; return false; } p.targetDiff = f; }
    else if (field == "hysteresis_c") { if (!inRange(f, 0, 20)) { error = "Hysterese ausserhalb 0..20 K"; return false; } p.hysteresis = f; }
    else if (field == "start_diff_c") { if (!inRange(f, .5f, 40)) { error = "Startdifferenz ausserhalb 0.5..40 K"; return false; } p.startDiff = f; }
    else if (field == "min_pwm_pct") { if (!inRange(f, 0, 100)) { error = "Minimum PWM ausserhalb 0..100 %"; return false; } p.minPwmPercent = f; }
    else if (field == "max_pwm_pct") { if (!inRange(f, 0, 100)) { error = "Maximum PWM ausserhalb 0..100 %"; return false; } p.maxPwmPercent = f; }
    else { error = "Pumpenparameter nicht freigegeben"; return false; }
    if (p.minPwmPercent > p.maxPwmPercent) { error = "Minimum PWM darf Maximum PWM nicht uebersteigen"; return false; }
  } else if (key.startsWith("aux/")) {
    const String field = key.substring(4);
    AuxHeaterConfig& a = candidate.auxHeater;
    if (field == "enabled") { if (!parseBoolPayload(payload, b)) { error = "Bool-Wert ungueltig"; return false; } a.enabled = b; }
    else if (field == "minimum_c") { if (!inRange(f, 5, 80)) { error = "Mindesttemperatur ausserhalb 5..80 C"; return false; } a.minimumTemperatureC = f; }
    else if (field == "target_c") { if (!inRange(f, 10, 85)) { error = "Zieltemperatur ausserhalb 10..85 C"; return false; } a.targetTemperatureC = f; }
    else if (field == "hysteresis_c") { if (!inRange(f, 0, 20)) { error = "Hysterese ausserhalb 0..20 K"; return false; } a.hysteresisC = f; }
    else if (field == "prerun_s") { if (iv < 0 || iv > 1800) { error = "Pumpenvorlauf ausserhalb 0..1800 s"; return false; } a.preRunMs = (uint32_t)iv * 1000UL; }
    else if (field == "cooldown_s") { if (iv < 0 || iv > 1800) { error = "Pumpennachlauf ausserhalb 0..1800 s"; return false; } a.cooldownMs = (uint32_t)iv * 1000UL; }
    else { error = "Zusatzheizungsparameter nicht freigegeben"; return false; }
    if (a.minimumTemperatureC >= a.targetTemperatureC) { error = "Mindesttemperatur muss unter Zieltemperatur liegen"; return false; }
  } else if (key.startsWith("oven/")) {
    const String field = key.substring(5);
    OvenConfig& o = candidate.oven;
    if (field == "enabled") { if (!parseBoolPayload(payload, b)) { error = "Bool-Wert ungueltig"; return false; } o.enabled = b; }
    else if (field == "auto_start") { if (!parseBoolPayload(payload, b)) { error = "Bool-Wert ungueltig"; return false; } o.autoStartEnabled = b; }
    else if (field == "target_c") { if (!inRange(f, 40, 88)) { error = "Ofen-Zieltemperatur ausserhalb 40..88 C"; return false; } o.targetOvenTemperatureC = f; }
    else if (field == "pump_on_c") { if (!inRange(f, 20, 85)) { error = "Pumpenstart ausserhalb 20..85 C"; return false; } o.pumpOnTemperatureC = f; }
    else if (field == "pump_on_diff_c") { if (!inRange(f, 1, 30)) { error = "Pumpen-Einschaltdifferenz ausserhalb 1..30 K"; return false; } o.pumpOnTemperatureDifferenceC = f; }
    else if (field == "pump_off_diff_c") { if (!inRange(f, 0, 20)) { error = "Pumpen-Ausschaltdifferenz ausserhalb 0..20 K"; return false; } o.pumpOffTemperatureDifferenceC = f; }
    else if (field == "min_open_pct") { if (iv < 0 || iv > 100) { error = "Minimale Oeffnung ausserhalb 0..100 %"; return false; } o.servoMinimumOpeningPercent = (uint8_t)iv; }
    else if (field == "max_open_pct") { if (iv < 0 || iv > 100) { error = "Maximale Oeffnung ausserhalb 0..100 %"; return false; } o.servoMaximumOpeningPercent = (uint8_t)iv; }
    else if (field == "step_pct") { if (iv < 1 || iv > 50) { error = "Servo-Schritt ausserhalb 1..50 %"; return false; } o.servoStepPercent = (uint8_t)iv; }
    else if (field == "deadband_c") { if (!inRange(f, .5f, 20)) { error = "Servo-Hysterese ausserhalb 0.5..20 K"; return false; } o.servoDeadbandC = f; }
    else if (field == "burnout_min") { if (iv < 1 || iv > 120) { error = "Abbrand-Nachlueftzeit ausserhalb 1..120 min"; return false; } o.burnoutVentMinutes = (uint16_t)iv; }
    else { error = "Ofenparameter nicht freigegeben"; return false; }
    if (o.servoMinimumOpeningPercent > o.servoMaximumOpeningPercent) { error = "Minimale Oeffnung darf Maximum nicht uebersteigen"; return false; }
    if (o.targetOvenTemperatureC >= o.criticalOvenTemperatureC) { error = "Zieltemperatur muss unter kritischer Ofentemperatur liegen"; return false; }
  } else if (key.startsWith("hk/")) {
    const int slash = key.indexOf('/', 3);
    if (slash < 0) { error = "Heizkreisparameter ungueltig"; return false; }
    const int idx = key.substring(3, slash).toInt() - 1;
    if (idx < 0 || idx >= MAX_HEATING_CIRCUITS) { error = "Heizkreisindex ungueltig"; return false; }
    const String field = key.substring(slash + 1);
    HeatingCircuitConfig& h = candidate.heatingCircuits[idx];
    if (field == "enabled") { if (!parseBoolPayload(payload, b)) { error = "Bool-Wert ungueltig"; return false; } h.enabled = b; }
    else if (field == "mode") {
      if (payload == "Festwert") h.controlMode = HeatingCircuitControlMode::FIXED_FLOW;
      else if (payload == "Witterungsgeführt" || payload == "Witterungsgefuehrt") h.controlMode = HeatingCircuitControlMode::WEATHER_COMPENSATED;
      else { error = "Regelart ungueltig"; return false; }
    }
    else if (field == "fixed_flow_c") { if (!inRange(f, 15, 55)) { error = "Festwert Vorlauf ausserhalb 15..55 C"; return false; } h.fixedFlowTemperatureC = f; }
    else if (field == "room_control") { if (!parseBoolPayload(payload, b)) { error = "Bool-Wert ungueltig"; return false; } h.roomControlEnabled = b; }
    else if (field == "room_target_c") { if (!inRange(f, 10, 30)) { error = "Raum Soll ausserhalb 10..30 C"; return false; } h.roomTargetTemperatureC = f; }
    else if (field == "room_influence_k") { if (!inRange(f, 0, 15)) { error = "Raumeinfluss ausserhalb 0..15 K"; return false; } h.roomInfluenceK = f; }
    else if (field == "pump_target_delta_c") { if (!inRange(f, 1, 20)) { error = "Pumpen-Zielspreizung ausserhalb 1..20 K"; return false; } h.pumpTargetDeltaC = f; }
    else if (field == "curve_base_c") { if (!inRange(f, 10, 45)) { error = "Heizkurve Basis ausserhalb 10..45 C"; return false; } h.heatingCurveBaseC = f; }
    else if (field == "curve_slope") { if (!inRange(f, 0, 3)) { error = "Heizkurve Steigung ausserhalb 0..3"; return false; } h.heatingCurveSlope = f; }
    else { error = "Heizkreisparameter nicht freigegeben"; return false; }
  } else if (key.startsWith("energy/")) {
    const String field = key.substring(7);
    EnergyMeterConfig& e = candidate.energyMeter;
    if (field == "enabled") { if (!parseBoolPayload(payload, b)) { error = "Bool-Wert ungueltig"; return false; } e.enabled = b; }
    else if (field == "pulses_per_l") { if (!inRange(f, .1f, 100000)) { error = "Impulse/Liter ausserhalb Bereich"; return false; } e.pulsesPerLiter = f; }
    else if (field == "factor") { if (!inRange(f, .1f, 2.0f)) { error = "Waermetraegerfaktor ausserhalb 0.1..2.0"; return false; } e.energyFactorWhPerLiterK = f; }
    else if (field == "log_s") { if (iv < 10 || iv > 3600) { error = "Logintervall ausserhalb 10..3600 s"; return false; } e.logIntervalMs = (uint32_t)iv * 1000UL; }
    else { error = "Energiezaehlerparameter nicht freigegeben"; return false; }
  } else {
    error = "Konfigurationsparameter nicht freigegeben";
    return false;
  }

  if (!Storage::saveConfig(candidate)) {
    error = "Konfiguration konnte nicht auf SD gespeichert werden";
    return false;
  }
  ctx.config = candidate;
  return true;
}

void publishCommandResult(const String& error = "") {
  if (error.length()) {
    setConfigError(error);
    publishState("mqtt/last_config_error", error);
  } else {
    setConfigError("");
    publishState("mqtt/last_config_error", "OK");
  }
  // Bestaetigte Zustandswerte werden ueber den nicht-blockierenden Snapshot
  // zurueckgesendet. Dadurch wartet HA auf den ESP, ohne den Regler zu blockieren.
  g_snapshotTarget = 0;
  g_snapshotPending = true;
}

void handleCommand(AppContext& ctx, const String& topic, const String& payload) {
  const String prefix = topicFor(ctx.config, "cmd/");
  if (!topic.startsWith(prefix)) return;
  const String cmd = topic.substring(prefix.length());

  if (cmd == "alarm/ack") {
    const uint8_t count = Alarms::acknowledgeAllNonCritical();
    publishCommandResult(count ? "" : "Kein quittierbarer Alarm vorhanden");
    g_snapshotPending = true;
    return;
  }
  if (cmd == "alarm/buzzer_mute") {
    bool mute = false;
    if (!parseBoolPayload(payload, mute)) { publishCommandResult("Summer-Befehl ungueltig"); return; }
    Alarms::setBuzzerMuted(mute);
    publishCommandResult();
    g_snapshotPending = true;
    return;
  }
  if (cmd == "oven/start") {
    if (ctx.commissioning.active) { publishCommandResult("Ofenstart waehrend Testmodus gesperrt"); return; }
    if (!ctx.config.oven.enabled) { publishCommandResult("Ofenregelung ist deaktiviert"); return; }
    OvenControl::requestStart();
    publishCommandResult();
    g_snapshotPending = true;
    return;
  }
  if (cmd == "oven/stop") {
    if (ctx.commissioning.active) { publishCommandResult("Ofenstopp waehrend Testmodus gesperrt"); return; }
    OvenControl::requestStop(ctx);
    publishCommandResult();
    g_snapshotPending = true;
    return;
  }
  if (cmd.startsWith("config/")) {
    const String key = cmd.substring(7);
    String error;
    if (!applyConfigCommand(ctx, key, payload, error)) publishCommandResult(error);
    else {
      Alarms::recordInfo("mqtt_config_changed", (String("MQTT/HA Konfiguration geaendert: ") + key).c_str());
      publishCommandResult();
      g_snapshotPending = true;
    }
    return;
  }
}

void handlePacket(uint8_t header, const uint8_t* data, size_t len) {
  const uint8_t type = header >> 4;
  if (type == 2) { // CONNACK
    if (len >= 2 && data[1] == 0) {
      g_connected = true;
      g_connecting = false;
      g_connectedSinceMs = millis();
      g_lastIoMs = millis();
      g_lastStateSignature = 0;
      g_pingSentMs = 0;
      setError("");
      if (g_ctx) {
        subscribeCommands(g_ctx->config);
        publishRaw(topicFor(g_ctx->config, "status"), "online", true);
        g_discoveryTarget = 0;
        g_snapshotTarget = 0;
        g_discoveryPending = true;
        g_snapshotPending = true;
      }
    } else {
      g_connecting = false;
      setError("MQTT Broker hat CONNECT abgelehnt");
      g_client.stop();
    }
    return;
  }
  if (type == 13) { // PINGRESP
    g_pingSentMs = 0;
    g_lastIoMs = millis();
    return;
  }
  if (type == 3 && g_ctx) { // PUBLISH
    if (len < 2) return;
    const uint16_t topicLen = (uint16_t(data[0]) << 8) | data[1];
    if (2U + topicLen > len) return;
    String topic;
    topic.reserve(topicLen);
    for (uint16_t i = 0; i < topicLen; ++i) topic += char(data[2 + i]);
    size_t payloadPos = 2 + topicLen;
    const uint8_t qos = (header >> 1) & 0x03;
    if (qos > 0) payloadPos += 2;
    if (payloadPos > len) return;
    String payload;
    payload.reserve(len - payloadPos);
    for (size_t i = payloadPos; i < len; ++i) payload += char(data[i]);
    handleCommand(*g_ctx, topic, payload);
  }
}

void readIncoming() {
  while (g_client.available()) {
    const uint8_t b = (uint8_t)g_client.read();
    g_lastIoMs = millis();
    if (g_rxHeader == 0) {
      g_rxHeader = b;
      g_rxReadingRemaining = true;
      g_rxRemaining = 0;
      g_rxMultiplier = 1;
      g_rxPos = 0;
      continue;
    }
    if (g_rxReadingRemaining) {
      g_rxRemaining += (b & 0x7F) * g_rxMultiplier;
      if (b & 0x80) {
        g_rxMultiplier *= 128;
        if (g_rxMultiplier > 128UL * 128UL * 128UL) { resetRx(); }
      } else {
        g_rxReadingRemaining = false;
        if (g_rxRemaining == 0) {
          handlePacket(g_rxHeader, nullptr, 0);
          resetRx();
        } else if (g_rxRemaining > MQTT_RX_MAX) {
          setError("MQTT Eingangspaket zu gross");
          g_client.stop();
          g_connected = false;
          g_connecting = false;
          resetRx();
          return;
        }
      }
      continue;
    }
    if (g_rxPos < MQTT_RX_MAX) g_rxBuffer[g_rxPos++] = b;
    if (g_rxPos >= g_rxRemaining) {
      handlePacket(g_rxHeader, g_rxBuffer, g_rxPos);
      resetRx();
    }
  }
}

void tryConnect(AppContext& ctx) {
  if (!ctx.config.mqttEnabled || !ctx.config.mqttHost[0]) return;
  if (!ctx.networkStaConnected || WiFi.status() != WL_CONNECTED) return;
  const uint32_t now = millis();
  if (g_lastAttemptMs != 0 && (uint32_t)(now - g_lastAttemptMs) < MQTT_RECONNECT_MS) return;
  g_lastAttemptMs = now;
  g_connecting = true;
  setError("Verbinde mit MQTT Broker");
  g_client.stop();
  g_client.setNoDelay(true);
  g_client.setTimeout(50);
  if (!g_client.connect(ctx.config.mqttHost, ctx.config.mqttPort, MQTT_CONNECT_TIMEOUT_MS)) {
    g_connecting = false;
    setError("MQTT Broker nicht erreichbar");
    return;
  }
  resetRx();
  if (!sendConnect(ctx.config)) {
    g_client.stop();
    g_connecting = false;
    setError("MQTT CONNECT konnte nicht gesendet werden");
  } else {
    g_lastIoMs = now;
  }
}

} // namespace

namespace MqttBridge {

void begin(AppContext& ctx) {
  g_ctx = &ctx;
  sendDisconnect();
  g_lastAttemptMs = 0;
  g_connectedSinceMs = 0;
  g_lastPublishMs = 0;
  g_lastStateSignature = 0;
  g_discoveryPending = false;
  g_snapshotPending = false;
  g_discoveryTarget = g_discoveryVisit = 0;
  g_snapshotTarget = g_snapshotVisit = 0;
  g_snapshotEnumerating = false;
  resetRx();
  setError("");
  setConfigError("");
}

void forceReconnect() {
  sendDisconnect();
  g_lastAttemptMs = 0;
}

void process(AppContext& ctx) {
  g_ctx = &ctx;
  if (!ctx.config.mqttEnabled) {
    if (g_client.connected()) sendDisconnect();
    g_connected = false;
    g_connecting = false;
    return;
  }
  if (!ctx.config.mqttHost[0]) {
    if (g_client.connected()) sendDisconnect();
    g_connected = false;
    g_connecting = false;
    setError("MQTT Broker nicht konfiguriert");
    return;
  }
  if (!ctx.networkStaConnected || WiFi.status() != WL_CONNECTED) {
    if (g_client.connected()) g_client.stop();
    g_connected = false;
    g_connecting = false;
    return;
  }

  if (!g_client.connected()) {
    g_connected = false;
    if (g_connecting && (uint32_t)(millis() - g_lastIoMs) > MQTT_HANDSHAKE_TIMEOUT_MS) {
      g_connecting = false;
      setError("MQTT Handshake Timeout");
      g_client.stop();
    }
    tryConnect(ctx);
    return;
  }

  readIncoming();
  if (!g_client.connected()) {
    g_connected = false;
    g_connecting = false;
    return;
  }

  if (!g_connected) {
    if (g_connecting && (uint32_t)(millis() - g_lastIoMs) > MQTT_HANDSHAKE_TIMEOUT_MS) {
      setError("MQTT CONNACK Timeout");
      g_client.stop();
      g_connecting = false;
    }
    return;
  }

  const uint32_t now = millis();
  // Discovery und Status-Snapshot werden absichtlich auf einzelne MQTT-Pakete
  // pro loop() aufgeteilt. So kann Home Assistant viele Entities bekommen, ohne
  // dass MQTT die eigentliche Regelung fuer laengere Zeit blockiert.
  if (g_discoveryPending) {
    g_discoveryVisit = 0;
    g_discoverySent = false;
    publishDiscovery(ctx);
    if (g_discoverySent) {
      g_discoveryTarget++;
    } else {
      g_discoveryPending = false;
      g_discoveryTarget = 0;
    }
    return;
  }

  if (g_snapshotPending) {
    g_snapshotVisit = 0;
    g_snapshotSent = false;
    g_snapshotEnumerating = true;
    publishSnapshotInternal(ctx);
    g_snapshotEnumerating = false;
    if (g_snapshotSent) {
      g_snapshotTarget++;
    } else {
      g_snapshotPending = false;
      g_snapshotTarget = 0;
      g_lastPublishMs = now;
      g_lastStateSignature = runtimeStateSignature(ctx);
    }
    return;
  }

  const uint32_t sig = runtimeStateSignature(ctx);
  if (g_lastStateSignature != 0 && sig != g_lastStateSignature) {
    g_snapshotTarget = 0;
    g_snapshotPending = true;
    return;
  }

  const uint32_t publishInterval = constrain(ctx.config.mqttPublishIntervalMs, 2000UL, 300000UL);
  if (g_lastPublishMs == 0 || (uint32_t)(now - g_lastPublishMs) >= publishInterval) {
    g_snapshotTarget = 0;
    g_snapshotPending = true;
    return;
  }

  if (g_pingSentMs != 0) {
    if ((uint32_t)(now - g_pingSentMs) >= MQTT_PING_TIMEOUT_MS) {
      setError("MQTT Ping Timeout");
      g_client.stop();
      g_connected = false;
      g_connecting = false;
    }
  } else if ((uint32_t)(now - g_lastIoMs) >= MQTT_PING_AFTER_MS) {
    const uint8_t ping[2] = {0xC0, 0x00};
    if (writeAll(ping, 2)) g_pingSentMs = now;
  }
}

void publishSnapshot(AppContext& ctx) {
  (void)ctx;
  if (g_connected) { g_snapshotTarget = 0; g_snapshotPending = true; }
}

bool connected() { return g_connected && g_client.connected(); }
bool connecting() { return g_connecting; }
const char* lastError() { return g_lastError; }
uint32_t lastConnectAttemptMs() { return g_lastAttemptMs; }
uint32_t lastConnectedMs() { return g_connectedSinceMs; }

} // namespace MqttBridge
