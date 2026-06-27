
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

from graph import Graph
from models import Zone, ZoneType
from min_cost_flow import RunResult


# ─────────────────────────────────────────────────────────────────────────────
# ANSI COLOUR HELPERS
# ─────────────────────────────────────────────────────────────────────────────

_ANSI: dict[str, str] = {
    # named colours (foreground)
    "black":   "\033[30m",
    "red":     "\033[91m",
    "green":   "\033[92m",
    "yellow":  "\033[93m",
    "blue":    "\033[94m",
    "magenta": "\033[95m",
    "cyan":    "\033[96m",
    "white":   "\033[97m",
    "gray":    "\033[90m",
    "grey":    "\033[90m",

    # extended colors (8-bit / bright variants approximated)
    "dark_red":    "\033[31m",
    "dark_green":  "\033[32m",
    "dark_yellow": "\033[33m",
    "dark_blue":   "\033[34m",
    "dark_magenta": "\033[35m",
    "dark_cyan":   "\033[36m",

    "light_red":    "\033[91m",
    "light_green":  "\033[92m",
    "light_yellow": "\033[93m",
    "light_blue":   "\033[94m",
    "light_cyan":   "\033[96m",
    "light_white":  "\033[97m",

    # purples / violet tones
    "purple":      "\033[35m",
    "violet":      "\033[35m",
    "dark_purple":  "\033[35;2m",
    "deep_purple":  "\033[38;5;54m",
    "light_purple": "\033[38;5;141m",
    "lavender":     "\033[38;5;183m",
    "orchid":       "\033[38;5;170m",

    # orange / extended warm tones
    "orange":      "\033[33m",
    "dark_orange": "\033[38;5;208m",
    "light_orange": "\033[38;5;214m",
    "gold":        "\033[38;5;220m",

    # styles
    "bold":      "\033[1m",
    "dim":       "\033[2m",
    "italic":    "\033[3m",
    "underline": "\033[4m",
    "blink":     "\033[5m",
    "reverse":   "\033[7m",
    "hidden":    "\033[8m",
    "reset":     "\033[0m",
    "crimson": "\033[38;5;196m",
}
_RST = "\033[0m"

_DRONE_COLOURS: list[str] = [
    "black", "red", "green", "yellow", "blue", "magenta", "cyan", "white",
    "gray", "grey", "dark_red", "dark_green", "dark_yellow", "dark_blue",
    "dark_magenta", "dark_cyan", "light_red", "light_green", "light_yellow",
    "light_blue", "light_cyan", "light_white", "purple", "violet",
    "dark_purple", "deep_purple", "light_purple", "lavender", "orchid",
    "orange", "dark_orange", "light_orange", "gold"
]


def style_wrapper(text: str, *styles: str) -> str:
    ''' Wraps a str with ANSI styles '''

    codes = "".join(_ANSI.get(s.lower(), "") for s in styles)
    return f"{codes}{text}{_RST}" if codes else text


def drone_label(drone_id: int) -> str:
    '''Returns D<n> coloured with _DRONE_COLOURS ANSI'''
    colour = _DRONE_COLOURS[(drone_id-1) % len(_DRONE_COLOURS)]
    return style_wrapper(f"D{drone_id}", colour, "bold")


def zone_label(zone: Zone, is_end: bool = False) -> str:
    """Return the zone name styled by map colour + zone type.

    Priority order:
      1. Goal arrival  → bold green (always, overrides map colour)
      2. Map colour    → base colour from `color=` attribute
      3. Zone type     → dim for priority, bold for restricted, plain otherwise
    """
    if is_end:
        return style_wrapper(zone.name, "green", "bold")

    colour = zone.color or "light_cyan"

    if zone.zone_type == ZoneType.RESTRICTED:
        return style_wrapper(zone.name, colour, "bold")

    if zone.zone_type == ZoneType.PRIORITY:
        return style_wrapper(zone.name, colour, "dim")

    return style_wrapper(zone.name, colour)

@dataclass
class DroneMetrics:
    delivered_at_turn: Optional[int] = None
    weighted_cost: int = 0


@dataclass
class TurnRecord:
    moves: list[str] = field(default_factory=list)
    drones_at_goal: int = 0

class Simulator:
        """Replay MCF paths and produce spec output and metrics.

    Args:
        graph:     Parsed zone graph.
        paths:     List of per-drone schedules: [(zone_name, turn), …].
        nb_drones: Total number of drones.

    Usage::

        sim = Simulator(graph, paths, nb_drones)
        total_turns = sim.run(show_metrics=True)
    """
        def __init__(self, graph: Graph, paths: list[list[tuple[str, int]]],
                     nb_drones: int) -> None:
            self.graph = graph
            self.nb_drones = nb_drones


