#include "feature_energy_conflicts.h"

#include <Arduino.h>

#include "feature_build_flags.h"
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
    DBG_PRINT("ENERGIE KONFLIKT: ");
    DBG_PRINT(reason);
    DBG_PRINT(" | Anfrage P");
    DBG_PRINT(requestedPump + 1);
    DBG_PRINT(" kollidiert mit P");
    DBG_PRINT(existing.pumpIndex + 1);
    DBG_PRINT(" | Sink=");
    DBG_PRINT((int)existing.sinkRole);
    DBG_PRINT(" | PumpRelais=");
    DBG_PRINT(existing.pumpRelayIndex);
    DBG_PRINT(" | VentilAusgang=");
    DBG_PRINTLN(outputRefLabel(reservationValveOutput(existing)));
    DBG_FLUSH();
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

  DBG_PRINT("ENERGIE ROUTE RESERVIERT: P");
  DBG_PRINT(pumpIndex + 1);
  DBG_PRINT(" Quelle=");
  DBG_PRINT((int)sourceRole);
  DBG_PRINT(" Sink=");
  DBG_PRINT((int)sinkRole);
  DBG_PRINT(" PumpRelais=");
  DBG_PRINT(pumpRelayIndex);
  DBG_PRINT(" VentilAusgang=");
  DBG_PRINTLN(outputRefLabel(valveOutput));
  DBG_FLUSH();
}

}
