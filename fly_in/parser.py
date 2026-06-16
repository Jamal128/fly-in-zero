from __future__ import annotations
import re
from typing import Optional
from models import Zone, Connection, ZoneType
from graph import Graph


class ParseError(Exception):
    """Raised when the map file contains a syntax or semantic error."""

    def __init__(self, line_number: int, line: str, reason: str) -> None:
        """Initialise with location and reason.

        Args:
            line_number: 1-based line number where the error occurred.
            line: Raw text of the offending line.
            reason: Human-readable description of the error.
        """
        super().__init__(
            f"Parse error at line {line_number}: {reason}\n  >> {line!r}"
        )
        self.line_number = line_number
        self.line = line
        self.reason = reason


class Parser:
    """Parse a drone-network map file into a Graph and drone count.

    File format (BNF sketch)::

        nb_drones: <positive_int>
        start_hub: <name> <x> <y> [metadata]
        end_hub:   <name> <x> <y> [metadata]
        hub:       <name> <x> <y> [metadata]
        connection: <name1>-<name2> [metadata]
        # comments are ignored

    Metadata syntax::

        [key=value key=value ...]

    Zone metadata keys: zone, color, max_drones
    Connection metadata keys: max_link_capacity
    """

    _ZONE_TYPES: dict[str, ZoneType] = {
        "normal": ZoneType.NORMAL,
        "blocked": ZoneType.BLOCKED,
        "restricted": ZoneType.RESTRICTED,
        "priority": ZoneType.PRIORITY,
    }

    def __init__(self) -> None:
        """Initialise parser state."""
        self._graph: Graph = Graph()
        self._nb_drones: int = 0
        self._has_nb_drones: bool = False
        self._has_start: bool = False
        self._has_end: bool = False
        self._defined_zones: set[str] = set()
        self._defined_connections: set[tuple[str, str]] = set()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def parse_file(self, path: str) -> tuple[Graph, int]:
        """Parse a map file and return the graph and drone count.

        Args:
            path: Filesystem path to the map file.

        Returns:
            Tuple of (Graph, nb_drones).

        Raises:
            ParseError: On any syntax or semantic error.
            FileNotFoundError: If the file does not exist.
        """
        with open(path, "r", encoding="utf-8") as fh:
            lines = fh.readlines()
        return self._parse_lines(lines)

    def parse_string(self, text: str) -> tuple[Graph, int]:
        """Parse map text from a string (useful for testing).

        Args:
            text: Full map file content as a string.

        Returns:
            Tuple of (Graph, nb_drones).
        """
        lines = text.splitlines(keepends=True)
        return self._parse_lines(lines)

    # ------------------------------------------------------------------
    # Internal parsing
    # ------------------------------------------------------------------

    def _parse_lines(
        self, lines: list[str]
    ) -> tuple[Graph, int]:
        """Process each line and validate the complete graph.

        Args:
            lines: Raw lines from the input file.

        Returns:
            Tuple of (Graph, nb_drones).
        """
        for lineno, raw in enumerate(lines, start=1):
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            self._parse_line(lineno, line)

        self._validate_complete()
        return self._graph, self._nb_drones

    def _parse_line(self, lineno: int, line: str) -> None:
        """Dispatch a single non-empty, non-comment line.

        Args:
            lineno: 1-based line number.
            line: Stripped line content.
        """
        if line.startswith("nb_drones:"):
            self._parse_nb_drones(lineno, line)
        elif line.startswith("start_hub:"):
            self._parse_zone(lineno, line, kind="start")
        elif line.startswith("end_hub:"):
            self._parse_zone(lineno, line, kind="end")
        elif line.startswith("hub:"):
            self._parse_zone(lineno, line, kind="hub")
        elif line.startswith("connection:"):
            self._parse_connection(lineno, line)
        else:
            raise ParseError(lineno, line, "Unrecognised line prefix")

    # ------------------------------------------------------------------
    # nb_drones
    # ------------------------------------------------------------------

    def _parse_nb_drones(self, lineno: int, line: str) -> None:
        """Parse 'nb_drones: <int>' line."""
        if self._has_nb_drones:
            raise ParseError(lineno, line, "nb_drones defined more than once")
        _, _, rest = line.partition(":")
        rest = rest.strip()
        if not rest.isdigit() or int(rest) <= 0:
            raise ParseError(
                lineno, line,
                f"nb_drones must be a positive integer, got {rest!r}"
            )
        self._nb_drones = int(rest)
        self._has_nb_drones = True

    # ------------------------------------------------------------------
    # Zone parsing
    # ------------------------------------------------------------------

    def _parse_zone(
        self, lineno: int, line: str, kind: str
    ) -> None:
        """Parse a hub / start_hub / end_hub line.

        Args:
            lineno: Line number.
            line: Full line text.
            kind: 'start', 'end', or 'hub'.
        """
        prefix_map = {"start": "start_hub:", "end": "end_hub:", "hub": "hub:"}
        prefix = prefix_map[kind]
        rest = line[len(prefix):].strip()

        name, x, y, meta = self._parse_zone_fields(lineno, line, rest)

        zone_type_str = meta.get("zone", "normal")
        if zone_type_str not in self._ZONE_TYPES:
            raise ParseError(
                lineno, line,
                f"Invalid zone type {zone_type_str!r}. "
                f"Expected one of: {list(self._ZONE_TYPES)}"
            )
        zone_type = self._ZONE_TYPES[zone_type_str]
        if "zone" not in meta and "dead" in name:
            zone_type = ZoneType.BLOCKED
        color: Optional[str] = meta.get("color", None)

        max_drones_raw = meta.get("max_drones", "1")
        if not max_drones_raw.isdigit() or int(max_drones_raw) <= 0:
            raise ParseError(
                lineno, line,
                f"max_drones must be positive integer, got {max_drones_raw!r}"
            )
        max_drones = int(max_drones_raw)

        if name in self._defined_zones:
            raise ParseError(lineno, line, f"Duplicate zone name {name!r}")

        zone = Zone(
            name=name,
            x=x,
            y=y,
            zone_type=zone_type,
            color=color,
            max_drones=max_drones,
        )
        self._graph.add_zone(zone)
        self._defined_zones.add(name)

        if kind == "start":
            if self._has_start:
                raise ParseError(lineno, line, "Multiple start_hub definition")
            self._graph.set_start(zone)
            self._has_start = True
        elif kind == "end":
            if self._has_end:
                raise ParseError(lineno, line, "Multiple end_hub definition")
            self._graph.set_end(zone)
            self._has_end = True

    def _parse_zone_fields(
        self, lineno: int, line: str, rest: str
    ) -> tuple[str, int, int, dict[str, str]]:
        """Extract name, x, y, and metadata from the rest of a zone line.

        Args:
            lineno: Line number for error reporting.
            line: Original line for error reporting.
            rest: Everything after the 'hub:' / 'start_hub:' prefix.

        Returns:
            Tuple of (name, x, y, metadata_dict).
        """
        meta: dict[str, str] = {}
        meta_str = ""
        bracket_match = re.search(r"\[([^\]]*)\]", rest)
        if bracket_match:
            meta_str = bracket_match.group(1)
            rest = rest[: bracket_match.start()].strip()
            meta = self._parse_metadata(lineno, line, meta_str)

        parts = rest.split()
        if len(parts) != 3:
            raise ParseError(
                lineno, line,
                f"Expected '<name> <x> <y>' before metadata, got {rest!r}"
            )
        name, x_str, y_str = parts

        if "-" in name or " " in name:
            raise ParseError(
                lineno, line,
                f"Zone name {name!r} must not contain dashes or spaces"
            )

        if not self._is_int(x_str) or not self._is_int(y_str):
            raise ParseError(
                lineno, line,
                f"Zone coordinates must be int, got ({x_str!r}, {y_str!r})"
            )
        return name, int(x_str), int(y_str), meta

    # ------------------------------------------------------------------
    # Connection parsing
    # ------------------------------------------------------------------

    def _parse_connection(self, lineno: int, line: str) -> None:
        """Parse a 'connection: <a>-<b> [metadata]' line."""
        rest = line[len("connection:"):].strip()

        meta: dict[str, str] = {}
        bracket_match = re.search(r"\[([^\]]*)\]", rest)
        if bracket_match:
            meta = self._parse_metadata(
                lineno, line, bracket_match.group(1)
            )
            rest = rest[: bracket_match.start()].strip()

        if rest.count("-") != 1:
            raise ParseError(
                lineno, line,
                "Connection must follow '<zone1>-<zone2>' format "
                "(exactly one dash, no dashes in zone names)"
            )
        a_name, _, b_name = rest.partition("-")
        a_name = a_name.strip()
        b_name = b_name.strip()

        for name in (a_name, b_name):
            if name not in self._defined_zones:
                raise ParseError(
                    lineno, line,
                    f"Connection references unknown zone {name!r}"
                )

        canon = tuple(sorted([a_name, b_name]))
        if canon in self._defined_connections:
            raise ParseError(
                lineno, line,
                f"Duplicate connection {a_name!r}-{b_name!r}"
            )
        self._defined_connections.add(canon)  # type: ignore[arg-type]

        cap_raw = meta.get("max_link_capacity", "1")
        if not cap_raw.isdigit() or int(cap_raw) <= 0:
            raise ParseError(
                lineno, line,
                f"max_link_capacity must be a positive int, got {cap_raw!r}"
            )
        capacity = int(cap_raw)

        conn = Connection(
            zone_a=self._graph.get_zone(a_name),
            zone_b=self._graph.get_zone(b_name),
            max_link_capacity=capacity,
        )
        self._graph.add_connection(conn)

    # ------------------------------------------------------------------
    # Metadata helpers
    # ------------------------------------------------------------------

    def _parse_metadata(
        self, lineno: int, line: str, meta_str: str
    ) -> dict[str, str]:
        """Parse 'key=value key=value ...' into a dict.

        Args:
            lineno: Line number for error reporting.
            line: Original line for error reporting.
            meta_str: Content inside brackets.

        Returns:
            Dict of parsed metadata key-value pairs.
        """
        meta: dict[str, str] = {}
        for token in meta_str.split():
            if "=" not in token:
                raise ParseError(
                    lineno, line,
                    f"Invalid metadata token {token!r} (expected key=value)"
                )
            key, _, value = token.partition("=")
            if not key or not value:
                raise ParseError(
                    lineno, line,
                    f"Malformed metadata token {token!r}"
                )
            meta[key] = value
        return meta

    # ------------------------------------------------------------------
    # Validation
    # ------------------------------------------------------------------

    def _validate_complete(self) -> None:
        """Validate that the fully-parsed graph is consistent.

        Raises:
            ParseError: If mandatory elements are missing.
        """
        if not self._has_nb_drones:
            raise ParseError(0, "", "Missing nb_drones definition")
        if not self._has_start:
            raise ParseError(0, "", "Missing start_hub definition")
        if not self._has_end:
            raise ParseError(0, "", "Missing end_hub definition")

    # ------------------------------------------------------------------
    # Utility
    # ------------------------------------------------------------------

    @staticmethod
    def _is_int(s: str) -> bool:
        """Return True if string represents a (possibly negative) integer."""
        return bool(re.fullmatch(r"-?\d+", s))
