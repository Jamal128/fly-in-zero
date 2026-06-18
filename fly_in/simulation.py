"""Simulation output engine.

Replays MCF paths turn by turn and prints the mandatory spec output,
followed by an optional metrics panel.

MANDATORY OUTPUT FORMAT (spec §VII.5)
======================================
  One line per active turn.
  Per drone per turn, one of:
    D<N>-<zone>              drone arrives at zone
    D<N>-<origin>-<dest>   drone in transit to a restricted zone (turn 1 of 2)
  Drones that don't move: omitted from the line.
  Delivered drones: not tracked after reaching end zone.

OPTIONAL METRICS PANEL (--verbose / show_metrics=True)
=======================================================
  Shown after the mandatory output. Contains:
    • Turn-by-turn breakdown (drones moved per turn, active vs delivered)
    • Per-drone stats (path length, zones visited, wait turns, total cost)
    • Algorithm comparison (if both BF and Dijkstra results are passed)
    • Secondary spec metrics: avg turns/drone, total weighted cost
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional
from graph import Graph
from models import ZoneType
from min_cost_flow import RunResult


# ─────────────────────────────────────────────────────────────────────────────
# ANSI COLOUR HELPERS
# ─────────────────────────────────────────────────────────────────────────────

_ANSI: dict[str, str] = {
    "red":     "\033[91m",
    "green":   "\033[92m",
    "yellow":  "\033[93m",
    "blue":    "\033[94m",
    "magenta": "\033[95m",
    "cyan":    "\033[96m",
    "white":   "\033[97m",
    "gray":    "\033[90m",
    "grey":    "\033[90m",
    "orange":  "\033[33m",
    "bold":    "\033[1m",
    "dim":     "\033[2m",
}
_RST = "\033[0m"


def _c(text: str, *styles: str) -> str:
    """Wrap text with one or more ANSI styles."""
    codes = "".join(_ANSI.get(s.lower(), "") for s in styles)
    return f"{codes}{text}{_RST}" if codes else text


def _zone_color(text: str, color: str | None) -> str:
    """Colorize a zone name using the zone's own map color."""
    if not color:
        return text
    return _c(text, color)


# ─────────────────────────────────────────────────────────────────────────────
# PER-DRONE STATS  (collected during replay)
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class DroneStats:
    """Statistics collected for one drone during simulation replay."""
    drone_id: int
    zones_visited: list[str] = field(default_factory=list)
    wait_turns: int = 0
    transit_turns: int = 0
    move_turns: int = 0
    total_turns: int = 0
    weighted_cost: int = 0
    delivered_at_turn: Optional[int] = None

    @property
    def path_length(self) -> int:
        """Unique zones visited (excluding start)."""
        seen: set[str] = set()
        return sum(1 for z in self.zones_visited
                   if not (z in seen or seen.add(z)))

    @property
    def efficiency(self) -> float:
        """Fraction of turns spent actually moving (not waiting)."""
        if self.total_turns == 0:
            return 0.0
        return self.move_turns / self.total_turns


# ─────────────────────────────────────────────────────────────────────────────
# TURN RECORD  (per-turn snapshot for the metrics table)
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class TurnRecord:
    turn: int
    moves: list[str] = field(default_factory=list)   # raw spec tokens
    drones_moved: int = 0
    drones_in_transit: int = 0
    drones_waiting: int = 0
    drones_delivered_this_turn: int = 0


# ─────────────────────────────────────────────────────────────────────────────
# SIMULATOR
# ─────────────────────────────────────────────────────────────────────────────

class Simulator:
    """Replay MCF paths and produce spec output + optional metrics panel.

    Usage:
        sim = Simulator(graph, paths, nb_drones)
        total_turns = sim.run(show_metrics=True, mcf_result=result)
    """

    def __init__(
        self,
        graph: Graph,
        paths: list[list[tuple[str, int]]],
        nb_drones: int,
    ) -> None:
        self.graph = graph
        self.nb_drones = nb_drones

        # Convert raw (zone_name, turn) path lists to sorted (turn, zone).
        self.schedules: list[list[tuple[int, str]]] = []
        for path in paths:
            by_turn: dict[int, str] = {}
            for zone_name, turn in path:
                by_turn[turn] = zone_name
            self.schedules.append(sorted(by_turn.items()))

    # ─────────────────────────────────────────────────────────────────────────
    # PUBLIC ENTRY POINT
    # ─────────────────────────────────────────────────────────────────────────

    def run(
        self,
        show_metrics: bool = False,
        mcf_result: Optional[RunResult] = None,
        compare_result: Optional[RunResult] = None,
    ) -> int:
        """Execute simulation, print output, return total turns.

        Args:
            show_metrics:    Print the optional metrics panel after the log.
            mcf_result:      RunResult from the MCF run (for algo metrics).
            compare_result:  Second RunResult (different algo) for comparison.

        Returns:
            Total simulation turns (last turn any drone moved).
        """
        total_turns, turn_records, drone_stats = self._replay()

        self._print_simulation_log(turn_records)

        if show_metrics:
            self._print_metrics(total_turns, turn_records, drone_stats,
                                mcf_result, compare_result)

        return total_turns

    # ─────────────────────────────────────────────────────────────────────────
    # CORE REPLAY  (builds turn_records and drone_stats)
    # ─────────────────────────────────────────────────────────────────────────

    def _replay(
        self,
    ) -> tuple[int, list[TurnRecord], list[DroneStats]]:
        """Simulate all drones turn by turn. Returns data, does not print."""
        end_name = self.graph.end_zone.name    # type: ignore[union-attr]
        start_name = self.graph.start_zone.name  # type: ignore[union-attr]

        if not any(self.schedules):
            return 0, [], [DroneStats(i + 1) for i in range(self.nb_drones)]

        max_turn = max(
            (sched[-1][0] for sched in self.schedules if sched),
            default=0,
        )

        drone_pos:  list[str] = [start_name] * self.nb_drones
        sched_ptr:  list[int] = [0] * self.nb_drones
        delivered:  list[bool] = [False] * self.nb_drones

        stats = [DroneStats(drone_id=i + 1) for i in range(self.nb_drones)]

        # Skip implicit t=0 (all drones start at start_zone, no output)
        for i, sched in enumerate(self.schedules):
            while sched_ptr[i] < len(sched) and sched[sched_ptr[i]][0] == 0:
                sched_ptr[i] += 1

        turn_records: list[TurnRecord] = []
        total_turns = 0

        for turn in range(1, max_turn + 1):
            rec = TurnRecord(turn=turn)

            for i in range(self.nb_drones):
                if delivered[i]:
                    continue

                sched = self.schedules[i]
                ptr = sched_ptr[i]

                if ptr >= len(sched):
                    rec.drones_waiting += 1
                    stats[i].wait_turns += 1
                    continue

                next_turn, next_zone = sched[ptr]

                # ── Drone arrives this turn
                if next_turn == turn:
                    prev_zone = drone_pos[i]

                    if next_zone == prev_zone:
                        # Wait edge — drone stays in place, no spec output
                        sched_ptr[i] += 1
                        rec.drones_waiting += 1
                        stats[i].wait_turns += 1
                        continue

                    dest = self.graph.zones[next_zone]
                    colored = _zone_color(next_zone, dest.color)
                    token = f"D{i + 1}-{colored}"
                    rec.moves.append(token)

                    drone_pos[i] = next_zone
                    sched_ptr[i] += 1
                    rec.drones_moved += 1
                    stats[i].move_turns += 1
                    stats[i].total_turns += 1
                    stats[i].weighted_cost += dest.movement_cost()
                    stats[i].zones_visited.append(next_zone)

                    if next_zone == end_name:
                        delivered[i] = True
                        stats[i].delivered_at_turn = turn
                        rec.drones_delivered_this_turn += 1

                # ── First turn of a 2-turn restricted move
                # Drone will arrive next turn. Emit the in-transit label.
                # Order: origin (current) - destination (next zone)
                elif next_turn == turn + 1:
                    dest = self.graph.zones[next_zone]
                    if dest.zone_type == ZoneType.RESTRICTED:
                        origin = drone_pos[i]           # current zone
                        token = f"D{i + 1}-{origin}-{next_zone}"
                        rec.moves.append(token)
                        rec.drones_in_transit += 1
                        stats[i].transit_turns += 1
                    # ptr NOT advanced — drone hasn't arrived yet

                # ── next_turn > turn + 1: drone waiting silently
                else:
                    rec.drones_waiting += 1
                    stats[i].wait_turns += 1

            if rec.moves:
                total_turns = turn
                turn_records.append(rec)
            elif any(not delivered[i] for i in range(self.nb_drones)):
                # Silent turn — no moves, but simulation still active
                turn_records.append(rec)

            if all(delivered):
                break

        return total_turns, turn_records, stats

    # ─────────────────────────────────────────────────────────────────────────
    # MANDATORY OUTPUT
    # ─────────────────────────────────────────────────────────────────────────

    def _print_simulation_log(self, turn_records: list[TurnRecord]) -> None:
        """Print the spec-required turn-by-turn drone movement log."""
        for rec in turn_records:
            if rec.moves:
                print(" ".join(rec.moves))

    # ─────────────────────────────────────────────────────────────────────────
    # OPTIONAL METRICS PANEL
    # ─────────────────────────────────────────────────────────────────────────

    def _print_metrics(
        self,
        total_turns: int,
        turn_records: list[TurnRecord],
        drone_stats: list[DroneStats],
        mcf_result:     Optional[RunResult],
        compare_result: Optional[RunResult],
    ) -> None:
        """Print the full metrics panel below the simulation log."""
        W = 62   # panel width

        def bar(label: str, value: str) -> None:
            """Print one key/value row."""
            dots = "." * max(1, W - len(label) - len(value) - 4)
            print(f"  {_c(label, 'cyan')} {_c(dots, 'dim')} "
                  "{_c(value, 'white', 'bold')}")

        def section(title: str) -> None:
            print()
            print(_c(f"  {'─' * (W - 2)}", "dim"))
            print(f"  {_c(title.upper(), 'yellow', 'bold')}")

        def divider() -> None:
            print(_c("  " + "─" * (W - 2), "dim"))

        # ── Header ───────────────────────────────────────────────────────────
        print()
        print(_c("  " + "═" * (W - 2), "cyan"))
        print(f"  {_c('DRONE ROUTING  —  SIMULATION REPORT', 'cyan', 'bold')}")
        print(_c("  " + "═" * (W - 2), "cyan"))

        # ── Simulation summary
        section("Simulation")
        bar("Total turns",        str(total_turns))
        bar("Drones routed",      f"{self.nb_drones}/{self.nb_drones}")
        bar("Start zone",         self.graph.start_zone.name)
        bar("End zone",           self.graph.end_zone.name)
        bar("Zones in graph",     str(len(self.graph.zones)))

        # ── Turn-by-turn breakdown
        section("Turn breakdown")
        active_turns = sum(1 for r in turn_records if r.moves)
        silent_turns = sum(1 for r in turn_records if not r.moves)
        max_moved = max((r.drones_moved for r in turn_records), default=0)
        avg_moved = (
            sum(r.drones_moved for r in turn_records) / active_turns
            if active_turns else 0.0
        )
        bar("Active turns (with movement)",  str(active_turns))
        bar("Silent turns (all drones wait)", str(silent_turns))
        bar("Peak drones moved in one turn",  str(max_moved))
        bar("Avg drones moved / active turn", f"{avg_moved:.2f}")

        # ── Per-drone table
        section("Per-drone stats")
        col_w = [6, 10, 6, 8, 7, 12]
        headers = ["Drone", "Delivered", "Moves",
                   "Waits", "Cost", "Efficiency"]
        header_row = "  " + "  ".join(
            _c(h.ljust(col_w[j]), "yellow") for j, h in enumerate(headers)
        )
        print(header_row)
        divider()
        for st in drone_stats:
            delivered_str = (
                f"turn {st.delivered_at_turn}"
                if st.delivered_at_turn else _c("—", "red")
            )
            eff_pct = f"{st.efficiency * 100:.0f}%"
            row = "  " + "  ".join([
                _c(f"D{st.drone_id}".ljust(col_w[0]),   "white"),
                delivered_str.ljust(col_w[1] + 9),      # +9 for ANSI escape
                str(st.move_turns).ljust(col_w[2]),
                str(st.wait_turns).ljust(col_w[3]),
                str(st.weighted_cost).ljust(col_w[4]),
                _c(eff_pct, "green" if st.efficiency >= 0.7 else "orange"),
            ])
            print(row)

        # ── Secondary spec metrics
        section("Secondary metrics  (spec §VII.6)")
        total_weighted = sum(st.weighted_cost for st in drone_stats)
        avg_turns = (
            sum(st.delivered_at_turn for st in drone_stats
                if st.delivered_at_turn)
            / self.nb_drones
        )
        bar("Total weighted path cost",   str(total_weighted))
        bar("Avg turns per drone",        f"{avg_turns:.2f}")
        bar("Total wait turns (all drones)",
            str(sum(st.wait_turns for st in drone_stats)))

        # ── MCF algorithm metrics
        if mcf_result:
            section(f"Algorithm  —  {mcf_result.algorithm}")
            bar("SSP iterations",      str(mcf_result.ssp_iterations))
            bar("MCF total cost",      str(mcf_result.total_cost))
            bar("Avg cost per path",   f"{mcf_result.avg_path_cost:.2f}")
            bar("Avg drones per path",
                f"{mcf_result.avg_drones_per_iteration:.2f}")
            bar("Wall-clock time",     f"{mcf_result.elapsed_ms:.2f} ms")

            if mcf_result.algorithm == "dijkstra":
                bar("Heap pops (nodes settled)",
                    str(mcf_result.dijkstra_nodes_settled))
            else:
                bar("Edge relaxations",
                    str(mcf_result.bellman_ford_relaxations))

            # Path-by-path breakdown
            print()
            print(f"  {_c('SSP path breakdown:', 'cyan')}")
            for idx, (pcost, pflow) in enumerate(
                zip(mcf_result.path_costs, mcf_result.path_flows), 1
            ):
                bar(f"  Path {idx}  ({pflow} drone{'s' if pflow > 1 else ''})",
                    f"cost {pcost}")

        # ── Algorithm comparison
        if mcf_result and compare_result:
            section("Algorithm comparison")
            a, b = mcf_result, compare_result
            faster = a if a.elapsed_ms <= b.elapsed_ms else b
            slower = b if faster is a else a
            speedup = slower.elapsed_ms / max(faster.elapsed_ms, 0.001)

            bar(f"{a.algorithm} time",   f"{a.elapsed_ms:.2f} ms")
            bar(f"{b.algorithm} time",   f"{b.elapsed_ms:.2f} ms")
            bar("Faster algorithm",
                _c(faster.algorithm, "green"))
            bar("Speedup",               f"{speedup:.2f}×")
            bar("Same result?",
                _c("yes" if a.total_cost == b.total_cost else "NO — mismatch!",
                   "green" if a.total_cost == b.total_cost else "red"))

        # ── Footer ───────────────────────────────────────────────────────────
        print()
        print(_c("  " + "═" * (W - 2), "cyan"))
        status = _c(f"  ✓  All {self.nb_drones}"
                    f"drones delivered in {total_turns} turns",
                    "green", "bold")
        print(status)
        print(_c("  " + "═" * (W - 2), "cyan"))
        print()
