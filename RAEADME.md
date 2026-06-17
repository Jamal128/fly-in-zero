"""Time-Expanded Graph (TEG) for drone routing.

QUÉ ES UN TEG Y POR QUÉ LO NECESITAS
======================================

Tu grafo original tiene zonas (A, B, C...) conectadas entre sí.
El problema: dos drones no pueden estar en la misma zona al mismo turno.
Eso es una restricción en el TIEMPO, no solo en el espacio.

La solución: duplicar el grafo para cada turno.

Grafo original:    A ── B ── C

TEG (3 turnos):
  turno 0:   A₀ ── B₀ ── C₀
  turno 1:   A₁ ── B₁ ── C₁     ← aristas de movimiento conectan t→t+1
  turno 2:   A₂ ── B₂ ── C₂

Tipos de aristas:
  WAIT:  Aₜ → Aₜ₊₁  (coste 0,  el dron espera en la zona)
  MOVE:  Aₜ → Bₜ₊₁  (coste 1,  zona normal/priority)
  MOVE:  Aₜ → Bₜ₊₂  (coste 2,  zona restricted)

Cada arista tiene una capacidad = cuántos drones pueden usarla a la vez.
El MCF se encarga de respetar esas capacidades automáticamente.
"""s