"""Minimum Cost Flow — Successive Shortest Paths with Dijkstra + potentials.

WHY DIJKSTRA INSTEAD OF BELLMAN-FORD
======================================
Bellman-Ford runs in O(V  E) per SSP iteration.
Dijkstra runs in O((V + E) log V) per SSP iteration.

On a realistic TEG (40 zones  60 turns = 2400 nodes, ~12000 edges):
  Bellman-Ford: ~0.07s per run()
  Dijkstra:     ~0.02s per run()   → ~3x faster

On the challenger map (120 turns = 4800 nodes):
  Bellman-Ford: ~0.12s
  Dijkstra:     ~0.05s             → ~2x faster

THE PROBLEM WITH DIJKSTRA ON RESIDUAL GRAPHS
=============================================
After sending flow, the residual graph has back-edges with NEGATIVE cost.
Dijkstra doesn't work with negative edge weights — it can pick wrong paths.

THE SOLUTION: Johnson's Potentials
====================================
We maintain a potential[] array for each node.
The "reduced cost" of edge u→v is:
    reduced_cost = real_cost + potential[u] - potential[v]

Key property: if potentials are set correctly, ALL reduced costs are ≥ 0.
So Dijkstra works on reduced costs — then we recover the real cost at the end.

How to keep potentials correct:
  - Start: potential[i] = 0 for all i  (all real costs are ≥ 0 initially)
  - After each Dijkstra: potential[i] += dist[i]
  This maintains the invariant that reduced costs stay ≥ 0 in the next iteration.

PROOF (one step):
  Before: reduced_cost(u→v) = cost + pot[u] - pot[v] ≥ 0
  After update: pot'[i] = pot[i] + dist[i]
  New reduced cost = cost + pot'[u] - pot'[v]
                   = cost + (pot[u]+dist[u]) - (pot[v]+dist[v])
                   = (cost + pot[u] - pot[v]) + (dist[u] - dist[v])
  Since Dijkstra guarantees dist[v] ≤ dist[u] + reduced_cost(u→v):
    dist[u] - dist[v] ≥ -reduced_cost(u→v)
  So: new reduced cost ≥ 0  ✓
"""

from __future__ import annotations
from dataclasses import dataclass
import heapq

INF = 10**9


# ─────────────────────────────────────────────────────────────────────────────
# EDGE
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class Edge:
    """One directed edge in the flow network.

    For every real edge A→B we also create a back-edge B→A.
    The back-edge lets the algorithm undo a routing decision.

    Example after sending 1 unit through A→B (cap=2, cost=1):
        fwd:  A→B  cap=2  cost=+1  flow=1   residual=1
        back: B→A  cap=0  cost=-1  flow=-1  residual=1
    """
    to: int       # destination node index
    cap: int      # maximum flow capacity
    cost: int     # cost per unit of flow (turns)
    rev_idx: int  # index of the paired back-edge in adj[to]
    flow: int = 0

    @property
    def residual(self) -> int:
        """Remaining capacity available right now."""
        return self.cap - self.flow


# ─────────────────────────────────────────────────────────────────────────────
# MIN COST FLOW
# ─────────────────────────────────────────────────────────────────────────────

class MinCostFlow:
    """Successive Shortest Paths min-cost flow using Dijkstra + potentials."""

    def __init__(self, n: int) -> None:
        """Create an empty flow network with n nodes (indexed 0..n-1)."""
        self.n = n
        self.adj: list[list[Edge]] = [[] for _ in range(n)]

    # ── Build ─────────────────────────────────────────────────────────────────

    def add_edge(self, u: int, v: int, cap: int, cost: int) -> None:
        """Add directed edge u→v AND its back-edge v→u.

        Always use this method — never append edges manually.
        The two edges must be created together so rev_idx stays consistent.

        Args:
            u:    Source node.
            v:    Destination node.
            cap:  Max drones that can use this edge simultaneously.
            cost: Turns cost (0=wait, 1=normal/priority, 2=restricted).
        """
        fwd_idx = len(self.adj[u])
        bck_idx = len(self.adj[v])
        self.adj[u].append(Edge(to=v, cap=cap,  cost=cost,  flow=0, rev_idx=bck_idx))
        self.adj[v].append(Edge(to=u, cap=0,    cost=-cost, flow=0, rev_idx=fwd_idx))

    # ── Dijkstra with potentials ──────────────────────────────────────────────

    def _dijkstra(
        self,
        source: int,
        potentials: list[int],
    ) -> tuple[list[int], list[int], list[int]]:
        """Shortest paths from source using reduced costs.

        Uses a min-heap for O((V + E) log V) per call.
        Reduced cost of edge u→v = real_cost + pot[u] - pot[v]
        This is always ≥ 0 as long as potentials are up to date.

        Args:
            source:     Starting node.
            potentials: Current potential for each node.

        Returns:
            dist:      dist[v] = minimum reduced-cost distance to v.
            prev_node: prev_node[v] = predecessor node on shortest path.
            prev_edge: prev_edge[v] = index in adj[prev_node[v]] of edge used.
                       -1 means unreachable.
        """
        dist: list[int] = [INF] * self.n
        prev_node: list[int] = [-1] * self.n
        prev_edge: list[int] = [-1] * self.n
        dist[source] = 0

        # heap entries: (reduced_distance, node_index)
        heap: list[tuple[int, int]] = [(0, source)]

        while heap:
            d, u = heapq.heappop(heap)

            # Stale heap entry — a shorter path to u was already found
            if d > dist[u]:
                continue

            for edge_idx, edge in enumerate(self.adj[u]):
                if edge.residual <= 0:
                    continue  # no capacity: skip

                # Reduced cost: always ≥ 0 thanks to potentials
                reduced = edge.cost + potentials[u] - potentials[edge.to]
                new_dist = dist[u] + reduced

                if new_dist < dist[edge.to]:
                    dist[edge.to] = new_dist
                    prev_node[edge.to] = u
                    prev_edge[edge.to] = edge_idx
                    heapq.heappush(heap, (new_dist, edge.to))

        return dist, prev_node, prev_edge

    # ── Push flow ─────────────────────────────────────────────────────────────

    def _push_flow(
        self,
        prev_node: list[int],
        prev_edge: list[int],
        source: int,
        sink: int,
        limit: int,
    ) -> int:
        """Send flow along the path found by Dijkstra.

        Phase 1: Walk backward from sink to source, find bottleneck
                 (= minimum residual capacity along the path).
        Phase 2: Walk backward again, commit the flow.
                 Each forward edge gains flow; its back-edge loses flow
                 (= gains residual capacity, allowing future "undo").

        Args:
            prev_node: Path predecessor nodes (from _dijkstra).
            prev_edge: Path predecessor edge indices (from _dijkstra).
            source:    Source node.
            sink:      Sink node.
            limit:     Max flow to send (= remaining drones needed).

        Returns:
            Units of flow actually pushed (≤ limit, ≤ bottleneck capacity).
        """
        # Phase 1: bottleneck
        bottleneck = limit
        v = sink
        while v != source:
            u = prev_node[v]
            edge = self.adj[u][prev_edge[v]]
            bottleneck = min(bottleneck, edge.residual)
            v = u

        if bottleneck <= 0:
            return 0

        # Phase 2: commit
        v = sink
        while v != source:
            u = prev_node[v]
            fwd = self.adj[u][prev_edge[v]]
            bck = self.adj[v][fwd.rev_idx]
            fwd.flow += bottleneck   # forward edge carries more
            bck.flow -= bottleneck   # back-edge gains residual
            v = u

        return bottleneck

    # ── Main SSP loop ─────────────────────────────────────────────────────────

    def run(self, source: int, sink: int, required_flow: int) -> tuple[int, int]:
        """Route required_flow units from source to sink at minimum cost.

        Successive Shortest Paths algorithm:
          1. Find cheapest path SOURCE→SINK in residual graph (Dijkstra).
          2. Send as many drones as possible along it (limited by capacities).
          3. Update potentials so next Dijkstra sees non-negative costs.
          4. Repeat until all drones routed or no path exists.

        Args:
            source:        Source node (TEG super-source S).
            sink:          Sink node (TEG super-sink T).
            required_flow: Number of drones to route.

        Returns:
            (flow_sent, total_cost)
            flow_sent:   Drones actually routed (may be < required_flow
                         if the graph doesn't have enough capacity).
            total_cost:  Sum of movement costs across all drones.
        """
        potentials: list[int] = [0] * self.n
        flow_sent = 0
        total_cost = 0

        while flow_sent < required_flow:

            # 1. Find cheapest path in residual graph
            dist, prev_node, prev_edge = self._dijkstra(source, potentials)

            # 2. No path to sink → can't route any more drones
            if dist[sink] == INF:
                break

            # 3. Recover real path cost from reduced distance + potentials
            #    real_cost = reduced_dist[sink] + pot[sink] - pot[source]
            #    At first iteration pot is all 0, so real_cost == dist[sink].
            real_path_cost = dist[sink] + potentials[sink] - potentials[source]

            # 4. Update potentials for next iteration
            #    Only reachable nodes (dist < INF) get updated.
            #    Unreachable nodes keep their old potential (they stay valid).
            for i in range(self.n):
                if dist[i] < INF:
                    potentials[i] += dist[i]

            # 5. Send as many drones as possible along this path
            pushed = self._push_flow(
                prev_node, prev_edge, source, sink,
                limit=required_flow - flow_sent,
            )

            flow_sent += pushed
            total_cost += pushed * real_path_cost

        return flow_sent, total_cost
