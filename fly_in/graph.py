from __future__ import annotations
from typing import Optional
from models import Zone, Connection, ZoneType


class Graph:
    """Undirected weighted graph of zones and connections.

    No external graph libraries are used (networkx etc. are forbidden).

    Attributes:
        zones: Mapping from zone name to Zone object.
        connections: Mapping from zone name to its list of Connections.
        start_zone: The starting hub zone.
        end_zone: The ending hub zone.
    """

    def __init__(self) -> None:
        """Initialise an empty graph."""
        self.zones: dict[str, Zone] = {}
        self.connections: dict[str, list[Connection]] = {}
        self.start_zone: Optional[Zone] = None
        self.end_zone: Optional[Zone] = None
        self._connection_set: set[tuple[str, str]] = set()

    # ------------------------------------------------------------------
    # Mutators
    # ------------------------------------------------------------------

    def add_zone(self, zone: Zone) -> None:
        """Register a zone in the graph.

        Args:
            zone: Zone to add.

        Raises:
            ValueError: If a zone with the same name already exists.
        """
        if zone.name in self.zones:
            raise ValueError(f"Duplicate zone name: {zone.name!r}")
        self.zones[zone.name] = zone
        self.connections[zone.name] = []

    def add_connection(self, connection: Connection) -> None:
        """Register a bidirectional connection between two zones.

        Args:
            connection: Connection to add.

        Raises:
            ValueError: If the connection is a duplicate or references
                unknown zones.
        """
        for zone in (connection.zone_a, connection.zone_b):
            if zone.name not in self.zones:
                raise ValueError(f"Unknown zone in connection: {zone.name!r}")

        key = connection.key()
        if key in self._connection_set:
            raise ValueError(
                f"Duplicate connection: {key[0]!r}-{key[1]!r}"
            )
        self._connection_set.add(key)
        self.connections[connection.zone_a.name].append(connection)
        self.connections[connection.zone_b.name].append(connection)

    def set_start(self, zone: Zone) -> None:
        """Designate a zone as the start hub."""
        self.start_zone = zone

    def set_end(self, zone: Zone) -> None:
        """Designate a zone as the end hub."""
        self.end_zone = zone

    def is_start(self, zone_name: str) -> bool:
        return (
            self.start_zone is not None
            and self.start_zone.name == zone_name
        )

    def is_end(self, zone_name: str) -> bool:
        return (
            self.end_zone is not None
            and self.end_zone.name == zone_name
        )

    # ------------------------------------------------------------------
    # Accessors
    # ------------------------------------------------------------------

    def get_zone(self, name: str) -> Zone:
        """Return the zone by name, or raise KeyError."""
        return self.zones[name]

    def neighbors(self, zone_name: str) -> list[tuple[Zone, Connection]]:
        """Return accessible (neighbor, connection) pairs for a zone.

        Blocked neighbors are excluded.

        Args:
            zone_name: Name of the source zone.

        Returns:
            List of (Zone, Connection) tuples for each accessible neighbor.
        """
        result: list[tuple[Zone, Connection]] = []
        for conn in self.connections.get(zone_name, []):
            neighbor = conn.other(zone_name)
            if neighbor.is_accessible():
                result.append((neighbor, conn))
        return result

    def connection_between(
        self, a_name: str, b_name: str
    ) -> Optional[Connection]:
        """Return the Connection between zones a and b, or None."""
        for conn in self.connections.get(a_name, []):
            if conn.connects(b_name):
                return conn
        return None

    def has_zone(self, name: str) -> bool:
        """Return True if a zone with this name exists."""
        return name in self.zones

    def all_zone_names(self) -> list[str]:
        """Return list of all zone names."""
        return list(self.zones.keys())

    def zone_count(self) -> int:
        """Return number of zones in the graph."""
        return len(self.zones)

    def connection_count(self) -> int:
        """Return number of unique connections."""
        return len(self._connection_set)

    def __repr__(self) -> str:
        return (
            f"Graph(zones={self.zone_count()}, "
            f"connections={self.connection_count()}, "
            f"start={self.start_zone}, end={self.end_zone})"
        )

    # ------------------------------------------------------------------
    # Heuristic helpers for pathfinding
    # ------------------------------------------------------------------

    def heuristic(self, zone_name: str, target_name: str) -> float:
        """Manhattan distance heuristic between two zones.

        Used by A* / Dijkstra with spatial hint.

        Args:
            zone_name: Source zone name.
            target_name: Target zone name.

        Returns:
            Manhattan distance as float.
        """
        a = self.zones[zone_name]
        b = self.zones[target_name]
        return float(abs(a.x - b.x) + abs(a.y - b.y))

    def priority_multiplier(self, zone_type: ZoneType) -> float:
        """Return a cost multiplier to prefer priority zones.

        Priority zones get a slight discount to encourage pathfinder
        to prefer them when costs are otherwise equal.

        Args:
            zone_type: The ZoneType of the destination zone.

        Returns:
            Multiplier applied to the edge cost.
        """
        if zone_type == ZoneType.PRIORITY:
            return 0.9
        return 1.0
