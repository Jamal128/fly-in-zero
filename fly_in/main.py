from __future__ import annotations

import argparse
import sys
from collections import deque

import arcade

from graph import Graph
from min_cost_flow import RunResult
from parser import Parser
from simulation import Simulator
from time_expanded_graph import TimeExpandedGraph
from visualizer import DroneVisualizer

# ─────────────────────────────────────────────────────────────────────────────
# HELPER FUNCTIONS
# ─────────────────────────────────────────────────────────────────────────────


def compute_max_turns(graph: Graph, nb_drones: int) -> int:
    """Estimate a safe upper bound for max_turns using an unweighted BFS."""
    start = graph.start_zone
    end = graph.end_zone

    if start is None or end is None:
        return max(10, nb_drones * 5)

    visited: set[str] = {start.name}
    queue: deque[tuple[str, int]] = deque([(start.name, 0)])
    shortest = 0

    while queue:
        zone_name, dist = queue.popleft()
        if zone_name == end.name:
            shortest = dist
            break
        for neighbor, _ in graph.neighbors(zone_name):
            if neighbor.name not in visited:
                visited.add(neighbor.name)
                queue.append((neighbor.name, dist + 1))

    if shortest == 0:
        shortest = len(graph.zones)  # Fallback

    # Buffer: shortest path × 2 + drone queuing overhead
    max_turns = shortest * 2 + nb_drones + 5
    return max(max_turns, 10)


def route(
    graph: Graph,
    nb_drones: int,
    max_turns: int,
    algorithm: str,
) -> tuple[list[list[tuple[str, int]]], RunResult]:
    """Build the Time-Expanded Graph, run MCF, and extract drone paths."""
    teg = TimeExpandedGraph(graph, max_turns=max_turns, nb_drones=nb_drones)
    mcf = teg.build()

    result = mcf.run(
        source=teg.source_id,
        sink=teg.sink_id,
        required_flow=nb_drones,
        algorithm=algorithm,
    )

    if result.flow_sent < nb_drones:
        print(
            f"Warning: only {result.flow_sent}/{nb_drones} drones reached goal"
            f"Consider increasing --turns (current horizon: {max_turns}).",
            file=sys.stderr,
        )

    return teg.extract_paths(mcf), result


# ─────────────────────────────────────────────────────────────────────────────
# CLI ARGUMENTS
# ─────────────────────────────────────────────────────────────────────────────

def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="python main.py",
        description="Drone routing simulation — MCF on a Time-Expanded Graph.",
    )
    p.add_argument("map_file", help="Path to the map file (maps/easy1.txt).")
    p.add_argument(
        "--algo",
        choices=["dijkstra", "bellman_ford", "both"],
        default="dijkstra",
        help="Pathfinding backend algorithm (default: dijkstra).",
    )
    p.add_argument("--turns", type=int, default=None, help="Override max_turn")
    p.add_argument("--metrics", action="store_true", help="Print the metrics.")
    p.add_argument("--compare", action="store_true", help="Run both & compare")
    return p


# ─────────────────────────────────────────────────────────────────────────────
# MAIN EXECUTION
# ─────────────────────────────────────────────────────────────────────────────

def main() -> int:
    args = build_arg_parser().parse_args()
    show_metrics = args.metrics or args.compare

    # 1. Parse Map File
    try:
        parser = Parser()
        graph, nb_drones = parser.parse_file(args.map_file)
    except FileNotFoundError:
        print(f"Error: file not found: {args.map_file!r}", file=sys.stderr)
        return 1
    except Exception as exc:
        print(f"Parse error: {exc}", file=sys.stderr)
        return 1

    # 2. Horizon Calculation
    max_turns = (args.turns if args.turns is not None
                 else compute_max_turns(graph, nb_drones))

    # 3. Pathfinding & MCF Routing
    if args.algo == "both" or args.compare:
        paths, result_dj = route(graph, nb_drones, max_turns, "dijkstra")
        _, result_bf = route(graph, nb_drones, max_turns, "bellman_ford")
        primary, compare = result_dj, result_bf
    else:
        paths, primary = route(graph, nb_drones, max_turns, args.algo)
        compare = None

    # 4. Text-Based Simulation Log
    sim = Simulator(graph, paths, nb_drones)
    sim.run(
        show_metrics=show_metrics,
        mcf_result=primary,
        compare_result=compare,
    )

    # 5. Graphical Analytics Window
    print(f"\nLaunching Visualizer (Horizon: {max_turns} turns)...")
    _window = DroneVisualizer(graph, paths, nb_drones)
    _ = _window
    arcade.run()

    return 0


if __name__ == "__main__":
    sys.exit(main())