#include "feature_energy_conflicts.h"

#include <Arduino.h>

namespace {

  bool validPumpIndex(uint8_t pumpIndex) {
    return pumpIndex < MAX_PUMPS;
  }

  bool outputRefAssigned(const OutputRef& ref) {
    return ref.kind != OutputKind::NONE && ref.index != PIN_UNUSED;
  }

  bool sameOutputRef(const OutputRef& a, const OutputRef& b) {
    return outputRefAssigned(a) &&
           outputRefAssigned(b) &&
           a.kind == b.kind &&
           a.index == b.index;
  }

  String outputRefLabel(const OutputRef& ref) {
    if (!outputRefAssigned(ref)) return "kein Ventilausgang";
    if (ref.kind == OutputKind::RELAY) return "R" + String(ref.index);
    if (ref.kind == OutputKind::PWM_OUTPUT) return "PO-" + String(ref.index);
    return "unbekannter Ausgang";
  }

  uint8_t legacyRelayIndex(const OutputRef& ref) {
    if (ref.kind == OutputKind::RELAY) return ref.index;
    return PIN_UNUSED;
  }

  OutputRef reservationValveOutput(const EnergyRouteReservation& r) {
    OutputRef ref;
    ref.kind = (OutputKind)r.valveOutputKind;
    ref.index = r.valveOutputIndex;
    return ref;
  }

  void resetReservation(EnergyRouteReservation& r) {
    r.active = false;
    r.pumpIndex = PIN_UNUSED;
    r.sourceRole = HeatSourceRole::NONE;
    r.sinkRole = Ds18Role::NONE;
    r.pumpRelayIndex = PIN_UNUSED;
    r.valveOutputKind = (uint8_t)OutputKind::NONE;
    r.valveOutputIndex = PIN_UNUSED;
    r.valveRelayIndex = PIN_UNUSED;
  }

  void printConflict(
    const char* reason,
    uint8_t requestedPump,
    const EnergyRouteReservation& existing
  ) {
    Serial.print("ENERGIE KONFLIKT: ");
    Serial.print(reason);
    Serial.print(" | Anfrage P");
    Serial.print(requestedPump + 1);
    Serial.print(" kollidiert mit P");
    Serial.print(existing.pumpIndex + 1);
    Serial.print(" | Sink=");
    Serial.print((int)existing.sinkRole);
    Serial.print(" | PumpRelais=");
    Serial.print(existing.pumpRelayIndex);
    Serial.print(" | VentilAusgang=");
    Serial.println(outputRefLabel(reservationValveOutput(existing)));
    Serial.flush();
  }

}

namespace EnergyConflicts {

void begin(AppContext& ctx) {
  clear(ctx);
}

void clear(AppContext& ctx) {
  for (uint8_t i = 0; i < MAX_PUMPS; i++) {
    resetReservation(ctx.reservations[i]);
  }
}

bool canActivateRoute(
  AppContext& ctx,
  uint8_t pumpIndex,
  HeatSourceRole sourceRole,
  Ds18Role sinkRole,
  uint8_t pumpRelayIndex,
  const OutputRef& valveOutput
) {
  if (!validPumpIndex(pumpIndex)) return false;
  if (sinkRole == Ds18Role::NONE) return false;
  if (pumpRelayIndex == PIN_UNUSED) return false;

  for (uint8_t i = 0; i < MAX_PUMPS; i++) {
    const EnergyRouteReservation& r = ctx.reservations[i];
    if (!r.active) continue;
    if (r.pumpIndex == pumpIndex) continue;

    // Ein Ziel/Sink darf aktuell nur von einer aktiven Route geladen werden.
    if (r.sinkRole == sinkRole) {
      printConflict("Sink bereits belegt", pumpIndex, r);
      return false;
    }

    // Ein Pumpenrelais darf nicht von zwei Pumpen gleichzeitig verwendet werden.
    if (r.pumpRelayIndex != PIN_UNUSED && r.pumpRelayIndex == pumpRelayIndex) {
      printConflict("Pumpenrelais bereits belegt", pumpIndex, r);
      return false;
    }

    // Valve V2: Ein Ventilausgang darf nicht von zwei aktiven Routen gleichzeitig angefordert werden.
    // Das gilt fuer Relaisausgaenge R0-R7 und PWM-Schaltausgaenge PO-0 bis PO-7.
    if (sameOutputRef(reservationValveOutput(r), valveOutput)) {
      printConflict("Ventilausgang bereits belegt", pumpIndex, r);
      return false;
    }
  }

  (void)sourceRole;
  return true;
}

void reserveRoute(
  AppContext& ctx,
  uint8_t pumpIndex,
  HeatSourceRole sourceRole,
  Ds18Role sinkRole,
  uint8_t pumpRelayIndex,
  const OutputRef& valveOutput
) {
  if (!validPumpIndex(pumpIndex)) return;

  EnergyRouteReservation& r = ctx.reservations[pumpIndex];
  r.active = true;
  r.pumpIndex = pumpIndex;
  r.sourceRole = sourceRole;
  r.sinkRole = sinkRole;
  r.pumpRelayIndex = pumpRelayIndex;
  r.valveOutputKind = (uint8_t)valveOutput.kind;
  r.valveOutputIndex = valveOutput.index;
  r.valveRelayIndex = legacyRelayIndex(valveOutput);

  Serial.print("ENERGIE ROUTE RESERVIERT: P");
  Serial.print(pumpIndex + 1);
  Serial.print(" Quelle=");
  Serial.print((int)sourceRole);
  Serial.print(" Sink=");
  Serial.print((int)sinkRole);
  Serial.print(" PumpRelais=");
  Serial.print(pumpRelayIndex);
  Serial.print(" VentilAusgang=");
  Serial.println(outputRefLabel(valveOutput));
  Serial.flush();
}

}
