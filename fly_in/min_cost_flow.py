"""Minimum Cost Flow — Successive Shortest Paths.

ALGORITHMS
==========
Both implement the same SSP loop. They differ only in how they find the
shortest path in the residual graph each iteration.

  Bellman-Ford  O(V x E) per iteration
  ────────────────────────────────────────────────────────────────
  Relaxes every edge V-1 times. Works directly on real costs.
  Safe with negative back-edges (no negative cycles in a TEG).
  Simple to understand, but slower on large or dense graphs.
  Early-exit optimisation: stops as soon as no edge was relaxed.

  Dijkstra + Johnson potentials  O((V + E) log V) per iteration
  ────────────────────────────────────────────────────────────────
  Uses a min-heap. Cannot handle negative edge weights directly,
  so we transform costs with potentials (Johnson's trick):

      reduced_cost(u→v) = real_cost + pot[u] - pot[v]   ≥ 0 always

  After each Dijkstra run we update:  pot[i] += dist[i]
  This keeps all reduced costs non-negative for the next call.
  ~2-3× faster than Bellman-Ford on realistic TEG sizes.

WHICH TO USE
============
  Pass algorithm="bellman_ford"  or  algorithm="dijkstra"  to run().
  Both return identical results. Use dijkstra for production runs,
  bellman_ford for step-by-step debugging (simpler trace).

METRICS
=======
  run() returns a RunResult dataclass with:
    - flow_sent, total_cost          (mandatory spec output)
    - ssp_iterations                 (how many shortest-path calls)
    - dijkstra_nodes_settled         (heap pops — work done per call)
    - bellman_ford_relaxations       (edge relaxations per call)
    - elapsed_ms                     (wall-clock time in ms)
    - path_costs                     (cost of each SSP path found)
    - path_flows                     (flow pushed on each SSP path)
"""

from __future__ import annotations

import heapq
import time
from dataclasses import dataclass, field

INF = 10**9


# ─────────────────────────────────────────────────────────────────────────────
# EDGE
# ─────────────────────────────────────────────────────────────────────────────

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


# ─────────────────────────────────────────────────────────────────────────────
# RUN RESULT  (metrics + mandatory output bundled together)
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class RunResult:
    """All outputs from a single MCF run() call.

    Mandatory (spec output):
        flow_sent:    Drones successfully routed.
        total_cost:   Sum of turn-costs across all drones.

    Algorithm metrics (for evaluation / benchmarking):
        algorithm:              "dijkstra" or "bellman_ford"
        ssp_iterations:         Number of shortest-path calls made.
        elapsed_ms:             Wall-clock time for the entire run().
        path_costs:            Cost of each SSP path found (one per iteration).
        path_flows:             Flow pushed on each SSP path.
        avg_path_cost:          Arithmetic mean of path_costs.

    Dijkstra-specific:
        dijkstra_nodes_settled: Total heap-pop operations across all calls.
                                Lower = fewer nodes explored = more efficient.

    Bellman-Ford-specific:
        bellman_ford_relaxations: Total edge relaxations across all calls.
                                  Lower = earlier termination = more efficient.
    """

    # Mandatory
    flow_sent: int = 0
    total_cost: int = 0

    # Common metrics
    algorithm: str = ""
    ssp_iterations: int = 0
    elapsed_ms: float = 0.0
    path_costs: list[int] = field(default_factory=list)
    path_flows: list[int] = field(default_factory=list)

    # Dijkstra-specific
    dijkstra_nodes_settled: int = 0

    # Bellman-Ford-specific
    bellman_ford_relaxations: int = 0

    @property
    def avg_path_cost(self) -> float:
        return (sum(self.path_costs) / len(self.path_costs)
                if self.path_costs else 0.0)

    @property
    def avg_drones_per_iteration(self) -> float:
        return (sum(self.path_flows) / len(self.path_flows)
                if self.path_flows else 0.0)


# ─────────────────────────────────────────────────────────────────────────────
# MIN COST FLOW
# ─────────────────────────────────────────────────────────────────────────────

class MinCostFlow:
    """Successive Shortest Paths min-cost flow.

    Supports two backends: Dijkstra (default, faster) and Bellman-Ford.
    """

    def __init__(self, n: int) -> None:
        """Create an empty flow network with n nodes (indexed 0..n-1)."""
        self.n = n
        self.adj: list[list[Edge]] = [[] for _ in range(n)]

    # ─────────────────────────────────────────────────────────────────────────
    # BUILD
    # ─────────────────────────────────────────────────────────────────────────

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

    # ─────────────────────────────────────────────────────────────────────────
    # DIJKSTRA  (with Johnson's potentials)
    # ─────────────────────────────────────────────────────────────────────────

    def _dijkstra(
        self,
        source: int,
        potentials: list[int],
    ) -> tuple[list[int], list[int], list[int], int]:
        """Shortest paths from source using reduced costs + a min-heap.

        Reduced cost of u→v  =  real_cost + pot[u] - pot[v]
        This is ≥ 0 when potentials are up to date, so Dijkstra is valid.

        Returns:
            dist: dist[v] = min reduced-cost distance to v from source.
            prev_node: predecessor node on cheapest path (-1 if unreachable).
            prev_edge:      index in adj[prev_node[v]] of the edge used.
            nodes_settled:  number of heap-pop operations (work metric).
        """
        dist: list[int] = [INF] * self.n
        prev_node: list[int] = [-1] * self.n
        prev_edge: list[int] = [-1] * self.n
        dist[source] = 0
        nodes_settled = 0

        heap: list[tuple[int, int]] = [(0, source)]

        while heap:
            d, u = heapq.heappop(heap)
            nodes_settled += 1

            if d > dist[u]:        # stale heap entry
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

    # ─────────────────────────────────────────────────────────────────────────
    # BELLMAN-FORD
    # ─────────────────────────────────────────────────────────────────────────

    def _bellman_ford(
        self,
        source: int,
    ) -> tuple[list[int], list[int], list[int], int]:
        """Shortest paths from source via Bellman-Ford relaxation.

        Iterates up to V-1 rounds. Each round scans every edge.
        Early-exits when no edge was relaxed (already optimal).
        Handles negative back-edge costs correctly.

        Returns:
            dist:         dist[v] = min cost to reach v from source.
            prev_node:    predecessor node on cheapest path.
            prev_edge:    index in adj[prev_node[v]] of the edge used.
            relaxations:  total edge relaxations performed (work metric).
        """
        dist: list[int] = [INF] * self.n
        prev_node: list[int] = [-1] * self.n
        prev_edge: list[int] = [-1] * self.n
        dist[source] = 0
        relaxations = 0

        for _ in range(self.n - 1):
            updated = False
            for u in range(self.n):
                if dist[u] == INF:
                    continue
                for edge_idx, edge in enumerate(self.adj[u]):
                    if edge.residual <= 0:
                        continue
                    new_dist = dist[u] + edge.cost
                    if new_dist < dist[edge.to]:
                        dist[edge.to] = new_dist
                        prev_node[edge.to] = u
                        prev_edge[edge.to] = edge_idx
                        updated = True
                    relaxations += 1
            if not updated:        # early exit
                break

        return dist, prev_node, prev_edge, relaxations

    # ─────────────────────────────────────────────────────────────────────────
    # PUSH FLOW
    # ─────────────────────────────────────────────────────────────────────────

    def _push_flow(
        self,
        prev_node: list[int],
        prev_edge: list[int],
        source: int,
        sink: int,
        limit: int,
    ) -> int:
        """Send flow along the path recorded in prev_node / prev_edge.

        Phase 1 — bottleneck: walk sink→source, find min residual capacity.
        Phase 2 — commit:     walk sink→source again, apply the flow change.

        Args:
            limit: Maximum units to push (= remaining drones still needed).

        Returns:
            Units of flow actually pushed.
        """
        # Phase 1: find bottleneck
        bottleneck = limit
        v = sink
        while v != source:
            u = prev_node[v]
            edge = self.adj[u][prev_edge[v]]
            bottleneck = min(bottleneck, edge.residual)
            v = u

        if bottleneck <= 0:
            return 0

        # Phase 2: commit flow
        v = sink
        while v != source:
            u = prev_node[v]
            fwd = self.adj[u][prev_edge[v]]
            bck = self.adj[v][fwd.rev_idx]
            fwd.flow += bottleneck
            bck.flow -= bottleneck    # back-edge gains residual for future
            v = u

        return bottleneck

    # ─────────────────────────────────────────────────────────────────────────
    # MAIN SSP LOOP
    # ─────────────────────────────────────────────────────────────────────────

    def run(
        self,
        source: int,
        sink: int,
        required_flow: int,
        algorithm: str = "dijkstra",
    ) -> RunResult:
        """Route required_flow drones from source to sink at minimum cost.

        SSP loop:
          1. Find cheapest SOURCE→SINK path in residual graph.
          2. Push as many drones as possible along it.
          3. Update potentials (Dijkstra only).
          4. Repeat until all drones routed or no path exists.

        Args:
            source:        TEG super-source node id.
            sink:          TEG super-sink node id.
            required_flow: Total drones to route.
            algorithm:     "dijkstra" (default, faster) or "bellman_ford".

        Returns:
            RunResult with mandatory outputs and full algorithm metrics.
        """
        if algorithm not in ("dijkstra", "bellman_ford"):
            raise ValueError(f"Unknown algorithm: {algorithm!r}. "
                             f"Use 'dijkstra' or 'bellman_ford'.")

        result = RunResult(algorithm=algorithm)
        potentials: list[int] = [0] * self.n
        t_start = time.perf_counter()

        while result.flow_sent < required_flow:

            # ── 1. Shortest path ─────────────────────────────────────────────
            if algorithm == "dijkstra":
                dist, prev_node, prev_edge, work = self._dijkstra(source,
                                                                  potentials)
                result.dijkstra_nodes_settled += work

                if dist[sink] == INF:
                    break

                # Recover real path cost from reduced distances + potentials
                real_cost = dist[sink] + potentials[sink] - potentials[source]

                # Update potentials for next iteration
                for i in range(self.n):
                    if dist[i] < INF:
                        potentials[i] += dist[i]

            else:  # bellman_ford
                dist, prev_node, prev_edge, work = self._bellman_ford(source)
                result.bellman_ford_relaxations += work

                if prev_node[sink] == -1:
                    break

                real_cost = dist[sink]

            # ── 2. Push flow ─────────────────────────────────────────────────
            pushed = self._push_flow(
                prev_node, prev_edge, source, sink,
                limit=required_flow - result.flow_sent,
            )

            # ── 3. Record metrics ────────────────────────────────────────────
            result.flow_sent += pushed
            result.total_cost += pushed * real_cost
            result.ssp_iterations += 1
            result.path_costs.append(real_cost)
            result.path_flows.append(pushed)

        result.elapsed_ms = (time.perf_counter() - t_start) * 1000
        return result
