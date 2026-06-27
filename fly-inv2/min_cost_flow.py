from __future__ import annotations

import heapq
import time
from dataclasses import dataclass, field

INF = 10**9


@dataclass
class Edge:
    """One directed edge in the flow network.

    Every real edge u→v is paired with a back-edge v→u (cap=0, cost=-cost).
    The back-edge lets the algorithm undo a previous routing decision.

    After sending k units through u→v (cap=C, cost=c):
        fwd:  u→v  cap=C  flow=k    residual=C-k
        back: v→u  cap=0  flow=-k   residual=k
    """

    to: int        # destination node index
    cap: int       # maximum flow capacity
    cost: int      # cost per unit of flow (turns)
    rev_idx: int   # index of the paired back-edge in adj[to]
    flow: int = 0

    @property
    def residual(self) -> int:
        """Remaining capacity: how many more units can flow through."""
        return self.cap - self.flow


@dataclass
class Metrics:
    '''
    Calculates the following metrics:

    flow_sent:    Drones successfully routed.
    total_cost:   Sum of turn-costs across all drones.

    ssp_iterations:         Number of shortest-path calls made.
    elapsed_ms:             Wall-clock time for the entire run().
    path_costs:            Cost of each SSP path found (one per iteration).
    path_flows:             Flow pushed on each SSP path.
    dijkstra_nodes_settled: Total heap-pop operations across all calls.

    '''
    flow_sent: int = 0
    total_cost: int = 0
    ssp_iterations: int = 0
    elapsed_ms: float = 0.0
    path_costs: list[int] = field(default_factory=list)
    path_flows: list[int] = field(default_factory=list)
    dijkstra_nodes_settled: int = 0


class MinCostFlow:
    """Successive Shortest Paths min-cost flow.

    Based on Dijkstra with potentials
    """

    def __init__(self, n: int) -> None:
        """Create an empty flow network with n nodes (indexed 0..n-1)."""
        self.n = n
        self.adj: list[list[Edge]] = [[] for _ in range(n)]

    def add_edge(self, u: int, v: int, cap: int, cost: int) -> None:
        """Add directed edge u→v and its back-edge v→u.

        Always call this — never append edges manually.
        The two edges reference each other via rev_idx and must be
        created together or the residual graph breaks.

        Args:
            u:    Source node index.
            v:    Destination node index.
            cap:  Max drones that can use this edge simultaneously.
            cost: Turn cost (0 = wait/source/sink, 1 = normal, 2 = restricted).
        """
        fwd_idx = len(self.adj[u])
        bck_idx = len(self.adj[v])
        self.adj[u].append(Edge(to=v, cap=cap,  cost=cost,
                                flow=0, rev_idx=bck_idx))
        self.adj[v].append(Edge(to=u, cap=0,    cost=-cost,
                                flow=0, rev_idx=fwd_idx))

    def dijkstra_reduced_costs(
            self, source: int, potentials: list[int]
            ) -> tuple[list[int], list[int], list[int], int]:
        '''
        Cada vértice tiene un potencial π(v).
        En lugar de usar el coste original c(u,v),
        Dijkstra usa el coste reducido: c(u,v)=c(u,v)+π(u)−π(v)
        Si los potenciales son válidos, todos los costes reducidos
        son no negativos y Dijkstra funciona aunque existan aristas
        de coste negativo en el grafo residual.

        '''
        dist: list[int] = [INF] * self.n
        prev_node: list[int] = [-1] * self.n
        prev_edge: list[int] = [-1] * self.n
        dist[source] = 0
        nodes_settled = 0

        heap: list[tuple[int, int]] = [(0, source)]

        while heap:
            distance, u = heapq.heappop(heap)
            nodes_settled += 1

            # Es una optimizacion, dist[u] siempre guarda la mejor distancia
            # Si distance > dsit[u] entonces ya tenemos un mejor camino
            if distance > dist[u]:
                continue

            for edge_idx, edge in enumerate(self.adj[u]):
                if edge.residual <= 0:
                    continue

                reduced = edge.cost + potentials[u] - potentials[edge.to]
                new_dist = dist[u] + reduced

                if new_dist < dist[edge.to]:
                    dist[edge.to] = new_dist
                    prev_node[edge.to] = u
                    prev_edge[edge.to] = edge_idx
                    heapq.heappush(heap, (new_dist, edge.to))

        return dist, prev_node, prev_edge, nodes_settled

    def push_flow(
            self, prev_node: list[int], prev_edge: list[int],
            source: int, sink: int, limit: int) -> int:
        '''Send the flow on the path we found with dijkstra using prev_node
        and prev_edge.

        First iteration will find the bottleneck in the path, which is the
        minimum flow in any edge of the path.
        Walk from sink to source, find min residual capacity.

        Second iteration will commit the flow changes
        Args:

        Returns: Units of flows
        '''
        # Find bottleneck
        bottleneck = limit
        v = sink
        while v != source:
            u = prev_node[v]
            edge = self.adj[u][prev_edge[v]]
            bottleneck = min(bottleneck, edge.residual)
            v = u

        if bottleneck <= 0:
            return 0

        v = sink
        while v != source:
            u = prev_node[v]
            fwd = self.adj[u][prev_edge[v]]
            bck = self.adj[v][fwd.rev_idx]
            fwd.flow += bottleneck
            bck.flow -= bottleneck
            v = u

        return bottleneck

    def run(
            self, source: int, sink: int, required_flow: int
            ) -> Metrics:

        result = Metrics()
        potentials = list[int] = [0] * self.n
        time_start = time.perf_counter()

        while result.flow_sent < required_flow:

            # 1 Shortest path
            dist, prev_node, prev_edge, nodes_s = self.dijkstra_reduced_costs(
                source, potentials
                )

            result.dijkstra_nodes_settled += nodes_s

            if dist[sink] == INF:
                break

            # Recover real path cost from reduced distances + potentials
            real_cost = dist[sink] + potentials[sink] - potentials[source]

            for i in range(self.n):
                if dist[i] < INF:
                    potentials[i] += dist[i]

            # 2 Push flow

            pushed = self.push_flow(
                prev_node, prev_edge, source, sink,
                limit=required_flow - result.flow_sent
            )

            # 3 get metrics
            result.flow_sent += pushed
            result.total_cost += pushed * real_cost
            result.ssp_iterations += 1
            result.path_costs.append(real_cost)
            result.path_flows.append(pushed)

        result.elapsed_ms = (time.perf_counter() - time_start) * 1000
        return result
