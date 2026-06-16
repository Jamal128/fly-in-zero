from dataclasses import dataclass


@dataclass
class Edge:
    to: int    # nodo destino
    cap: int    # capacidad residual (cap - flow)
    cost: int    # coste por unidad de flujo
    rev_idx: int    # índice de la back-edge en adj[to]
    flow: int = 0    # flujo actual (empieza en 0)

    @property
    def residual(self) -> int:
        return self.cap - self.flow


class MinCostFlow:
    def __init__(self, n: int):
        self.n = n
        self.adj: list[list[Edge]] = [[] for _ in range(n)]

    # ─────────────────────────────────────────────
    # AÑADIR ARISTA (siempre en pares fwd + back)
    # ─────────────────────────────────────────────

    def add_edge(self, u: int, v: int, cap: int, cost: int) -> None:
        """Add a directed edge u->v with given capacity and cost.

        Also adds the back-edge v->u with 0 capacity and negative cost.

        Args:
            u: Source node index.
            v: Destination node index.
            cap: Capacity of the edge.
            cost: Cost per unit of flow for the edge.
        """
        # Forward edge
        fwd = Edge(to=v, cap=cap, cost=cost, flow=0, rev_idx=len(self.adj[v]))
        # Backward edge
        back = Edge(to=u, cap=0, cost=-cost, flow=0, rev_idx=len(self.adj[u]))
        self.adj[u].append(fwd)
        self.adj[v].append(back)

    def shortest_path(self, source: int):
        '''We search the shortest path in the graph
        return the distances, parent node and edge.'''
        INF = 10**9

        dist = [INF] * self.n
        parent_node = [-1] * self.n
        parent_edge = [-1] * self.n

        dist[source] = 0

        for _ in range(self.n - 1):
            updated = False

            for u in range(self.n):

                if dist[u] == INF:
                    continue

                for edge_idx, edge in enumerate(self.adj[u]):

                    if edge.residual <= 0:
                        continue

                    v = edge.to
                    nd = dist[u] + edge.cost

                    if nd < dist[v]:
                        dist[v] = nd
                        parent_node[v] = u
                        parent_edge[v] = edge_idx

                        updated = True

            if not updated:
                break
        return dist, parent_node, parent_edge

    def run(self, source: int, sink: int, required_flow: int):

        flow_sent = 0
        total_cost = 0

        while flow_sent < required_flow:

            dist, parent_node, parent_edge = (
                self.shortest_path(source=source)
            )
            if parent_node[sink] == -1:
                break

            path_flow = required_flow - flow_sent

            v = sink
            while v != source:

                u = parent_node[v]
                edge_idx = parent_edge[v]

                edge = self.adj[u][edge_idx]

                path_flow = min(path_flow, edge.residual)
                v = u

            v = sink

            while v != source:

                u = parent_node[v]
                edge_idx = parent_edge[v]

                edge = self.adj[u][edge_idx]
                edge.flow += path_flow

                back = self.adj[v][edge.rev_idx]
                back.flow -= path_flow

                v = u

            flow_sent += path_flow
            total_cost += path_flow * dist[sink]

        # if flow_sent < required_flow:
        #     raise ValueError(
        #         f"Only {flow_sent}/{required_flow} units of flow could be sent"
        #     )
        return flow_sent, total_cost
