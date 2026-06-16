"""Core domain models: Zone, Connection, Drone."""

from __future__ import annotations
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional


class ZoneType(Enum):
    """Movement cost and accessibility of a zone."""

    NORMAL = "normal"
    BLOCKED = "blocked"
    RESTRICTED = "restricted"
    PRIORITY = "priority"

    def movement_cost(self) -> int:
        """Return the number of turns required to enter this zone type."""
        if self == ZoneType.RESTRICTED:
            return 2
        if self == ZoneType.BLOCKED:
            return 999
        return 1


@dataclass
class Zone:
    """A node in the drone network graph.

    Attributes:
        name: Unique identifier for the zone.
        x: X coordinate (for visual layout).
        y: Y coordinate (for visual layout).
        zone_type: Movement cost / accessibility type.
        color: Optional display color for terminal/graphical output.
        max_drones: Maximum drones that may occupy this zone simultaneously.
    """

    name: str
    x: int
    y: int
    zone_type: ZoneType = ZoneType.NORMAL
    color: Optional[str] = None
    max_drones: int = 1

    def movement_cost(self) -> int:
        """Turns required for a drone to enter this zone."""
        return self.zone_type.movement_cost()

    def is_accessible(self) -> bool:
        """Return True if drones can ever enter this zone."""
        return self.zone_type != ZoneType.BLOCKED

    def __hash__(self) -> int:
        return hash(self.name)

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, Zone):
            return NotImplemented
        return self.name == other.name

    def __repr__(self) -> str:
        return f"Zone({self.name!r}, type={self.zone_type.value})"


@dataclass
class Connection:
    """A bidirectional edge between two zones.

    Attributes:
        zone_a: First endpoint zone.
        zone_b: Second endpoint zone.
        max_link_capacity: Max drones traversing simultaneously.
    """

    zone_a: Zone
    zone_b: Zone
    max_link_capacity: int = 1

    def connects(self, zone_name: str) -> bool:
        """Return True if this connection involves the named zone."""
        return zone_name in (self.zone_a.name, self.zone_b.name)

    def other(self, zone_name: str) -> Zone:
        """Return the zone on the other end of this connection."""
        if zone_name == self.zone_a.name:
            return self.zone_b
        if zone_name == self.zone_b.name:
            return self.zone_a
        raise ValueError(f"Zone {zone_name!r} not part of this connection")

    def key(self) -> tuple[str, str]:
        """Canonical (sorted) key for deduplication."""
        names = sorted([self.zone_a.name, self.zone_b.name])
        return (names[0], names[1])

    def __hash__(self) -> int:
        return hash(self.key())

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, Connection):
            return NotImplemented
        return self.key() == other.key()

    def __repr__(self) -> str:
        return (
            f"Connection({self.zone_a.name!r}-{self.zone_b.name!r}, "
            f"cap={self.max_link_capacity})"
        )


@dataclass
class DroneState:
    """Mutable runtime state of a single drone.

    Attributes:
        drone_id: Unique identifier (e.g. 1 for D1).
        current_zone: Zone the drone currently occupies.
        in_transit_to: Destination zone if mid-flight (restricted, cost=2).
        transit_turns_left: Turns remaining until transit completes.
        delivered: True once the drone has reached the end zone.
        path: Planned sequence of zone names (from pathfinder).
        path_index: Current position in `path`.
    """

    drone_id: int
    current_zone: Zone
    in_transit_to: Optional[Zone] = None
    transit_turns_left: int = 0
    delivered: bool = False
    path: list[str] = field(default_factory=list)
    path_index: int = 0

    @property
    def label(self) -> str:
        """Human-readable drone label, e.g. 'D3'."""
        return f"D{self.drone_id}"

    def is_in_transit(self) -> bool:
        """Return True if the drone is mid-flight toward a restricted zone."""
        return self.in_transit_to is not None and self.transit_turns_left > 0

    def next_planned_zone(self) -> Optional[str]:
        """Return the next zone name in the planned path, or None if done."""
        next_idx = self.path_index + 1
        if next_idx < len(self.path):
            return self.path[next_idx]
        return None

    def __hash__(self) -> int:
        return hash(self.drone_id)

    def __repr__(self) -> str:
        loc = self.current_zone.name
        if self.is_in_transit():
            dest = self.in_transit_to.name  # type: ignore[union-attr]
            loc = f"{loc}→{dest}({self.transit_turns_left})"
        return f"Drone(D{self.drone_id} @{loc})"
