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
"""

from __future__ import annotations
from dataclasses import dataclass
from graph import Graph
from models import ZoneType, Zone
from min_cost_flow import MinCostFlow


# ─────────────────────────────────────────────────────────────────────────────
# TIME NODE
# ─────────────────────────────────────────────────────────────────────────────

@dataclass(frozen=True)
class TimeNode:
    """Un nodo en el TEG = una zona en un turno concreto.

    frozen=True lo hace hashable → puede usarse como clave de dict.

    Ejemplos:
        TimeNode("hub", 0)   → zona "hub" en el turno 0
        TimeNode("roof1", 3) → zona "roof1" en el turno 3
    """
    zone_name: str
    turn: int


# ─────────────────────────────────────────────────────────────────────────────
# TIME EXPANDED GRAPH
# ─────────────────────────────────────────────────────────────────────────────

class TimeExpandedGraph:
    """Construye el grafo expandido en tiempo y llama al MCF.

    FLUJO DE USO:
        teg = TimeExpandedGraph(graph, max_turns=20, nb_drones=5)
        mcf = teg.build()               # construye el TEG
        flow, cost = mcf.run(nb_drones) # corre el algoritmo
        paths = teg.extract_paths(mcf)  # extrae rutas por dron
    """

    def __init__(self, graph: Graph, max_turns: int, nb_drones: int) -> None:
        self.graph = graph
        self.max_turns = max_turns
        self.nb_drones = nb_drones

        # Mapeo bidireccional: TimeNode ↔ entero
        # Los enteros son los índices que usa MinCostFlow internamente.
        self.node_to_id: dict[TimeNode, int] = {}
        self.id_to_node: dict[int, TimeNode] = {}

        # 0 y 1 están reservados para SOURCE y SINK
        self.source_id = 0
        self.sink_id = 1
        self.next_id = 2  # los TimeNodes empiezan desde 2

    # ─────────────────────────────────────────────────────────────────────────
    # GET OR CREATE NODE ID
    # ─────────────────────────────────────────────────────────────────────────

    def get_id(self, node: TimeNode) -> int:
        """Devuelve el entero asociado a este TimeNode.

        Si el nodo no existe aún, lo registra y le asigna el siguiente id.
        Esto evita tener que pre-calcular cuántos nodos habrá.
        """
        if node not in self.node_to_id:
            node_id = self.next_id
            self.node_to_id[node] = node_id
            self.id_to_node[node_id] = node
            self.next_id += 1
        return self.node_to_id[node]

    # ─────────────────────────────────────────────────────────────────────────
    # BUILD
    # ─────────────────────────────────────────────────────────────────────────

    def build(self) -> MinCostFlow:
        """Construye el TEG completo y devuelve el objeto MinCostFlow listo.

        Orden de construcción:
            1. Registrar todos los TimeNodes (así next_id queda fijo)
            2. Crear MinCostFlow con el número exacto de nodos
            3. Añadir aristas de espera (wait edges)
            4. Añadir aristas de movimiento (move edges)
            5. Conectar SOURCE → start en t=0
            6. Conectar end en cada turno → SINK
        """
        # ── 1. Registrar todos los nodos ─────────────────────────────────────
        # Es importante hacerlo antes de crear MinCostFlow, porque
        # MinCostFlow necesita saber el número total de nodos en el constructor.
        for turn in range(self.max_turns + 1):
            for zone_name, zone in self.graph.zones.items():
                if zone.zone_type == ZoneType.BLOCKED:
                    continue  # zonas bloqueadas no existen en el TEG
                self.get_id(TimeNode(zone_name, turn))

        # ── 2. Crear MinCostFlow con el tamaño correcto ───────────────────────
        n_nodes = self.next_id  # total de nodos: SOURCE + SINK + todos los TimeNodes
        mcf = MinCostFlow(n_nodes)

        # ── 3-6. Añadir aristas ───────────────────────────────────────────────
        self._add_wait_edges(mcf)
        self._add_move_edges(mcf)
        self._add_source_edges(mcf)
        self._add_sink_edges(mcf)

        return mcf

    # ─────────────────────────────────────────────────────────────────────────
    # WAIT EDGES
    # ─────────────────────────────────────────────────────────────────────────

    def _add_wait_edges(self, mcf: MinCostFlow) -> None:
        """Aristas de espera: zona Zₜ → Zₜ₊₁, coste 0.

        Un dron puede quedarse quieto en una zona durante un turno.
        La capacidad = cuántos drones pueden estar en esa zona a la vez.
        """
        for turn in range(self.max_turns):
            for zone_name, zone in self.graph.zones.items():
                if zone.zone_type == ZoneType.BLOCKED:
                    continue

                u = self.get_id(TimeNode(zone_name, turn))
                v = self.get_id(TimeNode(zone_name, turn + 1))
                cap = self._zone_capacity(zone_name, zone)

                mcf.add_edge(u, v, cap, cost=0)

    # ─────────────────────────────────────────────────────────────────────────
    # MOVE EDGES
    # ─────────────────────────────────────────────────────────────────────────

    def _add_move_edges(self, mcf: MinCostFlow) -> None:
        """Aristas de movimiento: zona Aₜ → zona Bₜ₊cost, coste = movement_cost.

        El coste de la arista es el movement_cost() del DESTINO:
            normal/priority → 1 turno
            restricted      → 2 turnos (el dron llega 2 turnos después)

        La capacidad de la arista es el mínimo entre:
            - max_link_capacity del connection (cuántos drones pueden cruzar a la vez)
            - max_drones del DESTINO (cuántos drones pueden llegar a la vez)
        """
        for turn in range(self.max_turns):
            for zone_name, zone in self.graph.zones.items():
                if zone.zone_type == ZoneType.BLOCKED:
                    continue

                for neighbor, conn in self.graph.neighbors(zone_name):
                    # movement_cost() del destino: 1 (normal/priority) o 2 (restricted)
                    cost = neighbor.movement_cost()
                    arrival_turn = turn + cost

                    # Si el dron llegaría después del horizonte temporal → ignorar
                    if arrival_turn > self.max_turns:
                        continue

                    u = self.get_id(TimeNode(zone_name, turn))
                    v = self.get_id(TimeNode(neighbor.name, arrival_turn))

                    # Capacidad: lo más restrictivo entre el link y el destino
                    dest_cap = self._zone_capacity(neighbor.name, neighbor)
                    cap = min(conn.max_link_capacity, dest_cap)

                    mcf.add_edge(u, v, cap, cost)

    # ─────────────────────────────────────────────────────────────────────────
    # SOURCE EDGE
    # ─────────────────────────────────────────────────────────────────────────

    def _add_source_edges(self, mcf: MinCostFlow) -> None:
        """Conecta el super-SOURCE con la zona de inicio en t=0.

        SOURCE → start_zone₀, capacidad = nb_drones, coste = 0

        Solo en t=0 porque todos los drones salen al mismo tiempo
        desde la zona de inicio. El MCF "liberará" los drones de a
        uno por uno según el pathfinding encuentre caminos.
        """
        start_name = self.graph.start_zone.name  # type: ignore[union-attr]
        v = self.get_id(TimeNode(start_name, 0))
        mcf.add_edge(self.source_id, v, self.nb_drones, cost=0)

    # ─────────────────────────────────────────────────────────────────────────
    # SINK EDGES
    # ─────────────────────────────────────────────────────────────────────────

    def _add_sink_edges(self, mcf: MinCostFlow) -> None:
        """Conecta la zona de destino en CADA turno con el super-SINK.

        end_zoneₜ → SINK, para t en [0 .. max_turns]
        """
        end_name = self.graph.end_zone.name  # type: ignore[union-attr]
        for turn in range(self.max_turns + 1):
            u = self.get_id(TimeNode(end_name, turn))
            mcf.add_edge(u, self.sink_id, self.nb_drones, cost=0)

    # ─────────────────────────────────────────────────────────────────────────
    # CAPACITY HELPER
    # ─────────────────────────────────────────────────────────────────────────

    def _zone_capacity(self, zone_name: str, zone: Zone) -> int:
        """Cuántos drones pueden estar en esta zona simultáneamente.

        Reglas especiales (según el spec):
            - start_zone: todos los drones empiezan aquí → capacidad = nb_drones
            - end_zone:   múltiples drones pueden llegar → capacidad = nb_drones
            - resto:      zone.max_drones (por defecto 1, configurable en el mapa)
        """
        if self.graph.is_start(zone_name) or self.graph.is_end(zone_name):
            return self.nb_drones
        return zone.max_drones

    # ─────────────────────────────────────────────────────────────────────────
    # EXTRACT PATHS
    # ─────────────────────────────────────────────────────────────────────────

    def extract_paths(self, mcf: MinCostFlow) -> list[list[str]]:
        """Convierte el flujo del MCF en rutas individuales por dron.

        El MCF nos da flujo en las aristas, no rutas individuales.
        Por ejemplo: flow=2 en la arista hub₀→roof1₁ significa que
        2 drones hicieron ese movimiento, pero no CUÁLES.

        Algoritmo de extracción:
            Para cada dron (nb_drones veces):
                1. Empezar en SOURCE
                2. En cada nodo, elegir cualquier arista con flow > 0
                3. Decrementar ese flow en 1 (este dron "consume" esa unidad)
                4. Avanzar al nodo siguiente
                5. Parar cuando llegamos al SINK
                6. Guardar la secuencia de zone_names como ruta del dron

        Por qué funciona:
            El MCF garantiza conservación de flujo en cada nodo.
            Si entran K unidades a un nodo, salen K unidades.
            Siguiendo aristas con flow > 0 siempre llegamos al SINK.

        Returns:
            Lista de rutas. Cada ruta es una lista de zone_names por turno.
            Ejemplo: ["hub", "roof1", "roof2", "goal"]
        """
        adj = mcf.adj
        paths: list[list[str]] = []

        for _ in range(self.nb_drones):
            path: list[str] = []
            current = self.source_id

            while current != self.sink_id:
                moved = False
                for edge in adj[current]:
                    # Buscar una arista con flujo real (flow > 0)
                    # y que sea arista forward (cost >= 0, las back-edges tienen cost < 0)
                    # Excepción: cost == 0 puede ser wait edge o source/sink edge,
                    # que también son válidas.
                    if edge.flow > 0 and edge.cost >= 0:
                        edge.flow -= 1  # este dron consume una unidad de flujo

                        # Actualizar la back-edge correspondiente
                        back = adj[edge.to][edge.rev_idx]
                        back.flow += 1

                        # Si el nodo destino es un TimeNode (no SINK), guardar la zona
                        if edge.to in self.id_to_node:
                            tnode = self.id_to_node[edge.to]
                            path.append(tnode.zone_name)
                            path.append(tnode.turn)

                        current = edge.to
                        moved = True
                        break

                if not moved:
                    # No debería ocurrir si el MCF es correcto
                    break

            paths.append(path)

        return paths
