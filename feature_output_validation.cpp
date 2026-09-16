#include "feature_output_validation.h"

namespace {

struct OutputSlot {
  bool used = false;
  String owner;
};

String outputLabel(OutputKind kind, uint8_t index) {
  if (kind == OutputKind::NONE || index == PIN_UNUSED) {
    return "kein Ausgang";
  }

  if (kind == OutputKind::RELAY) {
    return "R" + String(index);
  }

  if (kind == OutputKind::PWM_OUTPUT) {
    return "PO-" + String(index);
  }

  return "?";
}

bool validIndexForKind(OutputKind kind, uint8_t index) {
  if (kind == OutputKind::NONE || index == PIN_UNUSED) return false;
  if (kind == OutputKind::RELAY) return index < RELAY_COUNT;
  if (kind == OutputKind::PWM_OUTPUT) return index < PWM_OUTPUT_COUNT;
  return false;
}

bool outputSupportsFunction(const ConfigData& cfg, const OutputRef& ref, RelayFunction requiredFunction) {
  if (ref.kind == OutputKind::NONE || ref.index == PIN_UNUSED) return false;

  if (ref.kind == OutputKind::RELAY) {
    if (ref.index >= RELAY_COUNT) return false;
    const RelayOutputConfig& out = cfg.relays[ref.index];
    return out.enabled && out.function == requiredFunction;
  }

  if (ref.kind == OutputKind::PWM_OUTPUT) {
    if (ref.index >= PWM_OUTPUT_COUNT) return false;
    const PwmOutputConfig& out = cfg.pwmOutputs[ref.index];
    return out.enabled && out.mode == PwmOutputMode::SWITCH && out.function == requiredFunction;
  }

  return false;
}

bool addOutput(
    OutputSlot relaySlots[RELAY_COUNT],
    OutputSlot pwmSlots[PWM_OUTPUT_COUNT],
    OutputKind kind,
    uint8_t index,
    const String& owner,
    String& error
) {
  if (kind == OutputKind::NONE || index == PIN_UNUSED) {
    return true;
  }

  if (!validIndexForKind(kind, index)) {
    error = owner;
    error += ": ungueltiger Ausgang ";
    error += outputLabel(kind, index);
    return false;
  }

  OutputSlot* slot = nullptr;

  if (kind == OutputKind::RELAY) {
    slot = &relaySlots[index];
  } else if (kind == OutputKind::PWM_OUTPUT) {
    slot = &pwmSlots[index];
  } else {
    return true;
  }

  if (slot->used) {
    error = outputLabel(kind, index);
    error += " ist doppelt belegt: ";
    error += slot->owner;
    error += " und ";
    error += owner;
    return false;
  }

  slot->used = true;
  slot->owner = owner;
  return true;
}

bool addOutputRef(
    OutputSlot relaySlots[RELAY_COUNT],
    OutputSlot pwmSlots[PWM_OUTPUT_COUNT],
    const OutputRef& ref,
    const String& owner,
    String& error
) {
  return addOutput(relaySlots, pwmSlots, ref.kind, ref.index, owner, error);
}

bool outputRefAssigned(const OutputRef& ref) {
  return ref.kind != OutputKind::NONE && ref.index != PIN_UNUSED;
}

bool sameOutputRef(const OutputRef& a, const OutputRef& b) {
  if (!outputRefAssigned(a) || !outputRefAssigned(b)) return false;
  return a.kind == b.kind && a.index == b.index;
}

Ds18Role heatingCircuitFlowRole(uint8_t circuitIndex) {
  switch (circuitIndex) {
    case 0: return Ds18Role::HK1_FLOW;
    case 1: return Ds18Role::HK2_FLOW;
    case 2: return Ds18Role::HK3_FLOW;
    case 3: return Ds18Role::HK4_FLOW;
    default: return Ds18Role::NONE;
  }
}

Ds18Role heatingCircuitReturnRole(uint8_t circuitIndex) {
  switch (circuitIndex) {
    case 0: return Ds18Role::HK1_RETURN;
    case 1: return Ds18Role::HK2_RETURN;
    case 2: return Ds18Role::HK3_RETURN;
    case 3: return Ds18Role::HK4_RETURN;
    default: return Ds18Role::NONE;
  }
}

int8_t heatingCircuitPumpRouteIndex(const PumpConfig& pump) {
  if (pump.sourceType != PumpSourceType::SENSOR_ROLE) return -1;
  for (uint8_t i = 0; i < MAX_HEATING_CIRCUITS; i++) {
    if (pump.sourceSensorRole == heatingCircuitFlowRole(i) &&
        pump.sinkRole == heatingCircuitReturnRole(i)) {
      return (int8_t)i;
    }
  }
  return -1;
}

int8_t partialHeatingCircuitRouteIndex(const PumpConfig& pump) {
  if (pump.sourceType != PumpSourceType::SENSOR_ROLE) return -1;
  for (uint8_t i = 0; i < MAX_HEATING_CIRCUITS; i++) {
    const bool sourceIsFlow = pump.sourceSensorRole == heatingCircuitFlowRole(i);
    const bool sinkIsReturn = pump.sinkRole == heatingCircuitReturnRole(i);
    if (sourceIsFlow || sinkIsReturn) return (int8_t)i;
  }
  return -1;
}

bool validateHeatingCircuitInternal(
    const HeatingCircuitConfig& hk,
    uint8_t index,
    String& error
) {
  if (!hk.enabled) return true;

  if (sameOutputRef(hk.mixerOpenOutput, hk.mixerCloseOutput)) {
    error = "HK";
    error += String(index + 1);
    error += ": Mischer AUF und Mischer ZU verwenden denselben Ausgang ";
    error += outputLabel(hk.mixerOpenOutput.kind, hk.mixerOpenOutput.index);
    return false;
  }

  return true;
}

bool validatePwmOutputModes(const ConfigData& cfg, String& error) {
  for (uint8_t i = 0; i < MAX_PUMPS; i++) {
    const PumpConfig& pump = cfg.pumps[i];

    if (!pump.enabled) continue;
    if (pump.mode != PumpMode::PWM) continue;
    if (pump.pwmChannel == PIN_UNUSED) continue;
    if (pump.pwmChannel >= PWM_OUTPUT_COUNT) continue;

    const PwmOutputConfig& out = cfg.pwmOutputs[pump.pwmChannel];

    if (out.mode != PwmOutputMode::PWM) {
      error = "Pumpe ";
      error += String(i + 1);
      error += ": ";
      error += outputLabel(OutputKind::PWM_OUTPUT, pump.pwmChannel);
      error += " ist als PWM-Pumpenausgang verwendet, aber nicht im Modus PWM konfiguriert";
      return false;
    }
  }

  return true;
}

} // namespace

namespace OutputValidation {

bool validateConfig(const ConfigData& cfg, String& error) {
  error = "";

  OutputSlot relaySlots[RELAY_COUNT];
  OutputSlot pwmSlots[PWM_OUTPUT_COUNT];

  if (!validatePwmOutputModes(cfg, error)) {
    return false;
  }

  // Pumpen
  for (uint8_t i = 0; i < MAX_PUMPS; i++) {
    const PumpConfig& pump = cfg.pumps[i];

    if (!pump.enabled) continue;

    if ((pump.mode == PumpMode::RELAY || pump.mode == PumpMode::PWM) &&
        pump.relayIndex != PIN_UNUSED) {
      if (!addOutput(
              relaySlots,
              pwmSlots,
              OutputKind::RELAY,
              pump.relayIndex,
              "Pumpe " + String(i + 1) + " Pump Enable",
              error)) {
        return false;
      }
    }

    if (pump.mode == PumpMode::PWM && pump.pwmChannel != PIN_UNUSED) {
      if (!addOutput(
              relaySlots,
              pwmSlots,
              OutputKind::PWM_OUTPUT,
              pump.pwmChannel,
              "Pumpe " + String(i + 1) + " PWM",
              error)) {
        return false;
      }
    }

    if (pump.valveIndex != PIN_UNUSED) {
      if (pump.valveIndex >= MAX_VALVES || !cfg.valves[pump.valveIndex].enabled) {
        error = "Pumpe ";
        error += String(i + 1);
        error += ": Ventilzuordnung ungueltig oder Ventil nicht aktiviert";
        return false;
      }
    }
  }

  // Heizkreispumpen: Hardware ist zentral im Pumpenmenue.
  // Gueltig ist nur Quelle HKx Vorlauf -> Ziel HKx Ruecklauf, beide fuer denselben Heizkreis.
  uint8_t hkPumpCount[MAX_HEATING_CIRCUITS] = {0};
  for (uint8_t i = 0; i < MAX_PUMPS; i++) {
    const PumpConfig& pump = cfg.pumps[i];
    if (!pump.enabled || pump.mode == PumpMode::OFF) continue;
    if (pump.sourceType != PumpSourceType::SENSOR_ROLE) continue;

    const int8_t fullHk = heatingCircuitPumpRouteIndex(pump);
    const int8_t partialHk = partialHeatingCircuitRouteIndex(pump);

    if (partialHk >= 0 && fullHk < 0) {
      error = "Pumpe ";
      error += String(i + 1);
      error += ": Heizkreis-Pumpe muss Quelle HKx Vorlauf und Ziel HKx Ruecklauf desselben Heizkreises verwenden";
      return false;
    }

    if (fullHk >= 0) {
      hkPumpCount[(uint8_t)fullHk]++;
      if (hkPumpCount[(uint8_t)fullHk] > 1) {
        error = "HK";
        error += String((int)fullHk + 1);
        error += ": Es darf nur eine Pumpe mit Quelle HK";
        error += String((int)fullHk + 1);
        error += " Vorlauf und Ziel HK";
        error += String((int)fullHk + 1);
        error += " Ruecklauf geben";
        return false;
      }
    }
  }

  // Zusatzheizung / E-Kessel
  const AuxHeaterConfig& aux = cfg.auxHeater;

  if (aux.enabled) {
    if (!addOutputRef(relaySlots, pwmSlots, aux.pumpOutput, "Zusatzheizung Pumpe", error)) return false;
    if (!addOutputRef(relaySlots, pwmSlots, aux.heaterOutput1, "Zusatzheizung Heizstab Stufe 1", error)) return false;
    if (!addOutputRef(relaySlots, pwmSlots, aux.heaterOutput2, "Zusatzheizung Heizstab Stufe 2", error)) return false;
    if (!addOutputRef(relaySlots, pwmSlots, aux.heaterOutput3, "Zusatzheizung Heizstab Stufe 3", error)) return false;
  }

  // Ofenpumpe: Die Hardware wird zentral im Pumpenmenue konfiguriert.
  // Das Ofenmodul darf keinen eigenen Ausgang mehr besitzen, sondern fordert
  // die Pumpe mit Quelle ALT_SOURCE_OVEN an.
  const OvenConfig& oven = cfg.oven;
  uint8_t ovenPumpCount = 0;
  for (uint8_t i = 0; i < MAX_PUMPS; i++) {
    const PumpConfig& pump = cfg.pumps[i];
    if (!pump.enabled || pump.mode == PumpMode::OFF) continue;
    if (pump.sourceType == PumpSourceType::HEAT_SOURCE_ROLE &&
        pump.sourceRole == HeatSourceRole::ALT_SOURCE_OVEN) {
      ovenPumpCount++;
    }
  }

  if (oven.enabled && ovenPumpCount == 0) {
    error = "Ofensteuerung aktiviert, aber keine Pumpe mit Quelle Ofen im Pumpenmenue konfiguriert";
    return false;
  }

  if (ovenPumpCount > 1) {
    error = "Es darf nur eine Pumpe mit Quelle Ofen konfiguriert sein";
    return false;
  }

  // Heizkreise
  for (uint8_t i = 0; i < HEATING_CIRCUIT_COUNT; i++) {
    const HeatingCircuitConfig& hk = cfg.heatingCircuits[i];

    if (!hk.enabled) continue;

    if (!validateHeatingCircuitInternal(hk, i, error)) {
      return false;
    }

    if (!addOutputRef(relaySlots, pwmSlots, hk.mixerOpenOutput, "HK" + String(i + 1) + " Mischer AUF", error)) return false;
    if (!addOutputRef(relaySlots, pwmSlots, hk.mixerCloseOutput, "HK" + String(i + 1) + " Mischer ZU", error)) return false;
  }

  // Valve V2
  for (uint8_t i = 0; i < MAX_VALVES; i++) {
    const ValveConfig& valve = cfg.valves[i];
    if (!valve.enabled) continue;
    if (!outputRefAssigned(valve.output)) {
      error = "Ventil ";
      error += String(i + 1);
      error += ": Ausgang fehlt";
      return false;
    }
    if (!outputSupportsFunction(cfg, valve.output, RelayFunction::ZONE_VALVE)) {
      error = "Ventil ";
      error += String(i + 1);
      error += ": Ausgang muss als Zonenventil konfiguriert sein";
      return false;
    }

    if (!addOutputRef(relaySlots, pwmSlots, valve.output, "Ventil " + String(i + 1), error)) return false;
  }


  // Summer / Alarm-Ausgang: exakt ein Ausgang darf als Alarm-Summer markiert sein.
  bool buzzerSeen = false;
  for (uint8_t i = 0; i < RELAY_COUNT; i++) {
    const RelayOutputConfig& r = cfg.relays[i];
    if (!r.enabled || r.function != RelayFunction::ALARM_BUZZER) continue;
    if (buzzerSeen) {
      error = "Mehrere Summer-/Alarm-Ausgaenge konfiguriert";
      return false;
    }
    buzzerSeen = true;
    if (!addOutput(relaySlots, pwmSlots, OutputKind::RELAY, i, "Summer / Alarm", error)) return false;
  }

  for (uint8_t i = 0; i < PWM_OUTPUT_COUNT; i++) {
    const PwmOutputConfig& po = cfg.pwmOutputs[i];
    if (!po.enabled || po.function != RelayFunction::ALARM_BUZZER) continue;
    if (po.mode != PwmOutputMode::SWITCH) {
      error = "PO-" + String(i) + ": Summer / Alarm muss im Modus Schaltausgang stehen";
      return false;
    }
    if (buzzerSeen) {
      error = "Mehrere Summer-/Alarm-Ausgaenge konfiguriert";
      return false;
    }
    buzzerSeen = true;
    if (!addOutput(relaySlots, pwmSlots, OutputKind::PWM_OUTPUT, i, "Summer / Alarm", error)) return false;
  }

  return true;
}

bool validateAll(const AppContext& ctx, String& error) {
  return validateConfig(ctx.config, error);
}

} // namespace OutputValidation
