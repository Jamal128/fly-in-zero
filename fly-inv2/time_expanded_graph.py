from __future__ import annotations
from dataclasses import dataclass
from graph import Graph
from models import ZoneType
from min_cost_flow import MinCostFlow


@dataclass(frozen=True)
class TimeNode:
    """A zone at a specific turn in the Time-Expanded Graph (TEG)."""
    zone_name: str
    turn: int


class TimeExpandedGraph:
    """Builds the TEG and calls Min-Cost Flow to route drones.
    The Time expanded graph has max_turn number of wait edges,
    move edges and sink edges for each zone in the map,
    except the blocked zones.
    This allows the algorithm to make the decision wether to move or wait.
    there is only one source edge.

    Usage:
        teg = TimeExpandedGraph(graph, max_turns, nb_drones)
        mcf = teg.build()
        flow_sent, cost = mcf.run(teg.source_id, teg.sink_id, nb_drones)
        paths = teg.extract_paths(mcf)
    """

    def __init__(self, graph: Graph, max_turns: int, nb_drones: int) -> None:
        self.graph = graph
        self.max_turns = max_turns
        self.nb_drones = nb_drones

        self.node_to_id: dict[TimeNode, int] = {}
        self.id_to_node: dict[int, TimeNode] = {}

        # 0 = SOURCE, 1 = SINK, TimeNodes start at 2
        self.source_id = 0
        self.sink_id = 1
        self.next_id = 2

    def _get_id(self, node: TimeNode) -> int:
        '''Assigns id starting from 2 to each node'''
        if node not in self.node_to_id:
            nid = self.next_id
            self.node_to_id[node] = nid
            self.id_to_node[nid] = node
            self.next_id += 1
        return self.node_to_id[node]

    def build(self) -> MinCostFlow:
        """Register all TimeNodes, construct the MCF graph, return it.
        registers all nodes as ids and get the value of the total n TimeNodes
        which we give to the MCF to construct the flow network for all edges.
        """
        for turn in range(self.max_turns + 1):
            for zone_name, zone in self.graph.zones.items():
                if zone.zone_type != ZoneType.BLOCKED:
                    self._get_id(TimeNode(zone_name, turn))

        # 2. Create MCF with exact node count
        print("next_id val:", self.next_id)
        mcf = MinCostFlow(self.next_id)

        # 3. Add all edge types
        self._add_wait_edges(mcf)
        self._add_move_edges(mcf)
        self._add_source_edges(mcf)
        self._add_sink_edges(mcf)

        return mcf

    def _zone_capacity(self, zone_name: str) -> int:
        """How many drones can occupy this zone simultaneously.
        start and end have nb_drones capacity
        else zone.max_drones
        """
        if self.graph.is_start(zone_name) or self.graph.is_end(zone_name):
            return self.nb_drones
        return self.graph.zones[zone_name].max_drones

    def _add_wait_edges(self, mcf: MinCostFlow) -> None:
        """Stay edges: zone_t → zone_{t+1}, capacity=max_drones, cost=0."""
        for turn in range(self.max_turns):
            for zone_name, zone in self.graph.zones.items():
                if zone.zone_type == ZoneType.BLOCKED:
                    continue
                u = self._get_id(TimeNode(zone_name, turn))
                v = self._get_id(TimeNode(zone_name, turn + 1))
                cap = self._zone_capacity(zone_name)
                mcf.add_edge(u, v, cap, cost=0)

    def _add_move_edges(self, mcf: MinCostFlow) -> None:
        """Move edges: zone_a_t → zone_b_{t+cost}.

        Cost for entering destination zone:
          - normal/priority: 1 turn
          - restricted: 2 turns
          - blocked: never added

        Edge capacity = min(link_capacity, dest_zone_capacity).
        """
        for turn in range(self.max_turns):
            for zone_name, zone in self.graph.zones.items():
                if zone.zone_type == ZoneType.BLOCKED:
                    continue

                for neighbor, conn in self.graph.neighbors(zone_name):
                    cost = neighbor.movement_cost()
                    arrival = turn + cost
                    if arrival > self.max_turns:
                        continue

                    u = self._get_id(TimeNode(zone_name, turn))
                    v = self._get_id(TimeNode(neighbor.name, arrival))
                    cap = min(conn.max_link_capacity,
                              self._zone_capacity(neighbor.name))
                    mcf.add_edge(u, v, cap, cost)

    def _add_source_edges(self, mcf: MinCostFlow) -> None:
        """SOURCE → start_zone at t=0, capacity = nb_drones, cost = 0."""
        start_name = self.graph.start_zone.name  # type: ignore[union-attr]
        v = self._get_id(TimeNode(start_name, 0))
        mcf.add_edge(self.source_id, v, self.nb_drones, cost=0)

    def _add_sink_edges(self, mcf: MinCostFlow) -> None:
        """end_zone_t → SINK for every t, capacity = nb_drones, cost = 0."""
        end_name = self.graph.end_zone.name  # type: ignore[union-attr]
        for turn in range(self.max_turns + 1):
            u = self._get_id(TimeNode(end_name, turn))
            mcf.add_edge(u, self.sink_id, self.nb_drones, cost=0)

    def extract_paths(self, mcf: MinCostFlow) -> list[list[tuple[str, int]]]:
        """Convert MCF flow into individual drone paths.

        Greedy path extraction: for each drone, follow edges with flow > 0
        from SOURCE to SINK, consuming one unit of flow per edge.

        Returns:
            List of paths. Each path is a list of (zone_name, turn) pairs
            representing where the drone is at each step. The start node
            at turn 0 is excluded (it's implicit).
        """
        adj = mcf.adj
        paths: list[list[tuple[str, int]]] = []

        for _ in range(self.nb_drones):
            path: list[tuple[str, int]] = []
            current = self.source_id

            while current != self.sink_id:
                moved = False
                for edge in adj[current]:
                    if edge.flow > 0 and edge.cost >= 0:
                        edge.flow -= 1

                        # Update the back-edge
                        back = adj[edge.to][edge.rev_idx]
                        back.flow += 1

                        if edge.to in self.id_to_node:
                            tnode = self.id_to_node[edge.to]
                            path.append((tnode.zone_name, tnode.turn))

                        current = edge.to
                        moved = True
                        break

                if not moved:
                    break  # shouldn't happen with a valid MCF solution

            paths.append(path)

        return paths
