from __future__ import annotations
from typing import Optional
from models import Zone, Connection


class Graph:
    """Undirected weighted graph of zones and connections.

    Attributes:
        zones: Dict of zone name to Zone class.
        connections: Dict from zone name to its list of Connections.
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
