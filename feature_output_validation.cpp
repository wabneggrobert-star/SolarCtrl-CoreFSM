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

bool addOutput(
    OutputSlot relaySlots[RELAY_COUNT],
    OutputSlot pwmSlots[PWM_OUTPUT_COUNT],
    OutputKind kind,
    uint8_t index,
    const String& owner,
    String& error
) {
  if (!validIndexForKind(kind, index)) {
    return true;
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

  if (sameOutputRef(hk.mixerOpenOutput, hk.pumpOutput)) {
    error = "HK";
    error += String(index + 1);
    error += ": Mischer AUF und Pumpe verwenden denselben Ausgang ";
    error += outputLabel(hk.mixerOpenOutput.kind, hk.mixerOpenOutput.index);
    return false;
  }

  if (sameOutputRef(hk.mixerCloseOutput, hk.pumpOutput)) {
    error = "HK";
    error += String(index + 1);
    error += ": Mischer ZU und Pumpe verwenden denselben Ausgang ";
    error += outputLabel(hk.mixerCloseOutput.kind, hk.mixerCloseOutput.index);
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

    if (pump.switchValveEnabled && pump.switchValveRelayIndex != PIN_UNUSED) {
      if (!addOutput(
              relaySlots,
              pwmSlots,
              OutputKind::RELAY,
              pump.switchValveRelayIndex,
              "Pumpe " + String(i + 1) + " Umschaltventil",
              error)) {
        return false;
      }
    }
  }

  // Zusatzheizung / E-Kessel
  const AuxHeaterConfig& aux = cfg.auxHeater;

  if (aux.enabled) {
    if (!addOutput(relaySlots, pwmSlots, OutputKind::RELAY, aux.pumpRelay, "Zusatzheizung Pumpe", error)) return false;
    if (!addOutput(relaySlots, pwmSlots, OutputKind::RELAY, aux.heaterRelay1, "Zusatzheizung Heizstab Stufe 1", error)) return false;
    if (!addOutput(relaySlots, pwmSlots, OutputKind::RELAY, aux.heaterRelay2, "Zusatzheizung Heizstab Stufe 2", error)) return false;
    if (!addOutput(relaySlots, pwmSlots, OutputKind::RELAY, aux.heaterRelay3, "Zusatzheizung Heizstab Stufe 3", error)) return false;
  }

  // Ofen
  const OvenConfig& oven = cfg.oven;

  if (oven.enabled) {
    if (!addOutput(relaySlots, pwmSlots, OutputKind::RELAY, oven.pumpRelay, "Ofenpumpe", error)) return false;
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
    if (!addOutputRef(relaySlots, pwmSlots, hk.pumpOutput, "HK" + String(i + 1) + " Pumpe", error)) return false;
  }

  // Ventile: aktivieren, sobald ConfigData.valves[] produktiv verwendet wird.
  /*
  for (uint8_t i = 0; i < MAX_VALVES; i++) {
    const ValveConfig& valve = cfg.valves[i];

    if (!valve.enabled) continue;

    if (!addOutputRef(
            relaySlots,
            pwmSlots,
            valve.output,
            "Ventil " + String(i + 1),
            error)) {
      return false;
    }
  }
  */

  return true;
}

bool validateAll(const AppContext& ctx, String& error) {
  return validateConfig(ctx.config, error);
}

} // namespace OutputValidation
