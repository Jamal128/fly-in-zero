"""Entry point for the drone routing simulation.

USAGE
=====
  python main.py <map_file> [OPTIONS]

OPTIONS
=======
  --algo   dijkstra | bellman_ford | both
             Pathfinding backend.  Default: dijkstra
  --turns  <int>
             Override the automatic max_turns heuristic.
  --metrics
             Print the optional metrics panel after the simulation log.
  --compare
             Run both algorithms and print a side-by-side comparison.
             Implies --metrics.

MAKEFILE TARGETS (convenience wrappers)
========================================
  make run       MAP=maps/easy1.txt
  make metrics   MAP=maps/medium1.txt
  make compare   MAP=maps/hard1.txt
  make bf        MAP=maps/easy1.txt   (Bellman-Ford only)
  make debug     MAP=maps/easy1.txt   (pdb)

All options can be combined:
  make run MAP=maps/hard1.txt ALGO=dijkstra TURNS=80
"""

from __future__ import annotations

import argparse
import sys
from typing import Optional

from graph import Graph
from parser import Parser          # your existing parser
from time_expanded_graph import TimeExpandedGraph
from min_cost_flow import RunResult
from simulation import Simulator


# ─────────────────────────────────────────────────────────────────────────────
# MAX TURNS HEURISTIC
# ─────────────────────────────────────────────────────────────────────────────

def compute_max_turns(graph: Graph, nb_drones: int) -> int:
    """Estimate a safe upper bound for max_turns before running the TEG.

    Strategy: BFS from start_zone to find the shortest path length,
    then multiply by (nb_drones + 1) to allow for queuing delays.
    Minimum of 10 to handle trivially small maps.

    This avoids building an oversized TEG while still guaranteeing
    the optimal solution fits within the horizon.
    """
    start = graph.start_zone
    end   = graph.end_zone

    if start is None or end is None:
        return max(10, nb_drones * 5)

    # BFS — unweighted shortest path (ignores restricted cost, conservative)
    from collections import deque
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
        shortest = len(graph.zones)   # fallback: full graph diameter

    # Buffer: shortest path × 2 (restricted zones) + drone queuing
    max_turns = shortest * 2 + nb_drones + 5
    return max(max_turns, 10)


# ─────────────────────────────────────────────────────────────────────────────
# CORE PIPELINE
# ─────────────────────────────────────────────────────────────────────────────

def route(
    graph: Graph,
    nb_drones: int,
    max_turns: int,
    algorithm: str,
) -> tuple[list[list[tuple[str, int]]], RunResult]:
    """Build TEG, run MCF, extract paths. Returns (paths, result).

    paths: one list per drone, each entry is (zone_name, turn).
           turn comes directly from the TimeNode — no reconstruction needed.
    """
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
            f"Warning: only {result.flow_sent}/{nb_drones} drones could be routed. "
            f"Try increasing --turns (current: {max_turns}).",
            file=sys.stderr,
        )

    return teg.extract_paths(mcf), result


# ─────────────────────────────────────────────────────────────────────────────
# CLI ARGUMENT PARSER
# ─────────────────────────────────────────────────────────────────────────────

def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="python main.py",
        description="Drone routing simulation — Min-Cost Flow on a Time-Expanded Graph.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    p.add_argument(
        "map_file",
        help="Path to the map input file (e.g. maps/easy1.txt).",
    )
    p.add_argument(
        "--algo",
        choices=["dijkstra", "bellman_ford", "both"],
        default="dijkstra",
        metavar="ALGO",
        help="Algorithm: dijkstra (default), bellman_ford, or both.",
    )
    p.add_argument(
        "--turns",
        type=int,
        default=None,
        metavar="N",
        help="Override automatic max_turns heuristic.",
    )
    p.add_argument(
        "--metrics",
        action="store_true",
        help="Print the optional metrics panel after the simulation log.",
    )
    p.add_argument(
        "--compare",
        action="store_true",
        help="Run both algorithms and print a comparison. Implies --metrics.",
    )
    return p


# ─────────────────────────────────────────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────────────────────────────────────────

def main() -> int:
    """Parse arguments, run simulation, return exit code."""
    args = build_arg_parser().parse_args()

    # --compare implies --metrics
    show_metrics = args.metrics or args.compare

    # ── 1. Parse map ─────────────────────────────────────────────────────────
    try:
        parser = Parser()
        graph, nb_drones = parser.parse_file(args.map_file)
    except FileNotFoundError:
        print(f"Error: file not found: {args.map_file!r}", file=sys.stderr)
        return 1
    except Exception as exc:
        print(f"Parse error: {exc}", file=sys.stderr)
        return 1

    # ── 2. Determine max_turns ────────────────────────────────────────────────
    max_turns = args.turns if args.turns is not None else compute_max_turns(graph, nb_drones)

    # ── 3. Route ──────────────────────────────────────────────────────────────
    algo = args.algo

    if algo == "both" or args.compare:
        # Run Dijkstra first (canonical result for the simulator)
        paths_dj, result_dj = route(graph, nb_drones, max_turns, "dijkstra")
        # Run Bellman-Ford separately (paths may differ but cost must match)
        _, result_bf = route(graph, nb_drones, max_turns, "bellman_ford")
        paths    = paths_dj
        primary  = result_dj
        compare  = result_bf
    else:
        paths, primary = route(graph, nb_drones, max_turns, algo)
        compare = None

    # ── 4. Simulate and print ─────────────────────────────────────────────────
    sim = Simulator(graph, paths, nb_drones)
    sim.run(
        show_metrics=show_metrics,
        mcf_result=primary,
        compare_result=compare,
    )

    return 0


if __name__ == "__main__":
    sys.exit(main())